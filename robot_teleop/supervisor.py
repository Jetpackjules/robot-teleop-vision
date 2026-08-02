from __future__ import annotations

import json
import os
import re
import signal
import socket
import ssl
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

from robot_teleop.config import AppConfig, REPO_ROOT
from robot_teleop.doctor import find_cloudflared, find_godot
from robot_teleop.interfaces import LaunchSpec
from robot_teleop.registry import create


RUN_DIR = REPO_ROOT / ".teleop"
STATE_PATH = RUN_DIR / "run.json"
QUICK_URL_PATTERN = re.compile(r"https://[a-z0-9-]+\.trycloudflare\.com", re.I)


def _port_available(port: int) -> bool:
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def _write_state(payload: dict) -> None:
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    temporary = STATE_PATH.with_suffix(".tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(STATE_PATH)


def read_state() -> dict:
    try:
        return json.loads(STATE_PATH.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {"running": False}


class Supervisor:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.processes: list[tuple[str, subprocess.Popen]] = []
        self.stopping = threading.Event()
        self.public_url = ""
        self.local_url = f"https://127.0.0.1:{config.stack.https_port}/controller.html"
        self.robot = create("robot", config.robot.adapter, config=config.robot)

    def _spawn(self, spec: LaunchSpec) -> subprocess.Popen:
        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
        process = subprocess.Popen(
            spec.command,
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            creationflags=creationflags,
        )
        self.processes.append((spec.name, process))
        threading.Thread(target=self._relay_output, args=(spec.name, process), daemon=True).start()
        return process

    def _relay_output(self, name: str, process: subprocess.Popen) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            print(f"[{name}] {line}", end="", flush=True)
            match = QUICK_URL_PATTERN.search(line)
            if match and not self.public_url:
                self.public_url = f"{match.group(0)}/controller.html"
                self._publish_state("running")
                threading.Thread(target=self._verify_public_url, daemon=True).start()

    def _publish_state(self, status: str, detail: str = "") -> None:
        _write_state(
            {
                "running": status in {"starting", "running"},
                "status": status,
                "detail": detail,
                "supervisor_pid": os.getpid(),
                "local_url": self.local_url,
                "public_url": self.public_url,
                "processes": {name: process.pid for name, process in self.processes},
                "updated_unix_ms": int(time.time() * 1000),
            }
        )

    def _verify_local_url(self) -> None:
        context = ssl._create_unverified_context()
        deadline = time.monotonic() + 25
        last_error: Exception | None = None
        while time.monotonic() < deadline and not self.stopping.is_set():
            try:
                with urllib.request.urlopen(self.local_url, context=context, timeout=2) as response:
                    if response.status == 200 and b"hybrid-canvas" in response.read(200_000):
                        return
            except Exception as exc:
                last_error = exc
            time.sleep(0.25)
        raise RuntimeError(f"operator UI health check failed: {last_error}")

    def _verify_public_url(self) -> None:
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not self.stopping.is_set():
            try:
                request = urllib.request.Request(self.public_url, method="GET")
                with urllib.request.urlopen(request, timeout=5) as response:
                    if response.status in {200, 302, 303}:
                        print(f"Verified public operator URL: {self.public_url}", flush=True)
                        return
            except Exception:
                time.sleep(0.5)
        print(f"WARNING: public URL did not pass its health check: {self.public_url}", flush=True)

    def start(self) -> None:
        for port, label in (
            (self.config.stack.https_port, "HTTPS UI"),
            (self.config.stack.stream_port, "Godot RGB-D stream"),
            (self.config.stack.calibration_status_port, "calibration status"),
        ):
            if not _port_available(port):
                raise RuntimeError(f"{label} port {port} is already in use")
        self._publish_state("starting")
        robot_spec = self.robot.launch_spec()
        if robot_spec:
            self._spawn(robot_spec)
        if self.config.godot.launch_runtime:
            godot = find_godot(self.config.godot.executable)
            if not godot:
                raise RuntimeError("Godot 4.6+ was not found; set godot.executable in config/local.toml")
            godot_command = [str(godot), "--headless"]
            if not self.config.godot.packaged_runtime:
                godot_command.extend(("--path", str(self.config.project_root)))
            self._spawn(
                LaunchSpec(
                    "Godot runtime",
                    tuple(godot_command),
                )
            )
        password = os.environ.get(
            self.config.stack.password_env,
            self.config.stack.password_default,
        )
        server = [
            sys.executable,
            str(REPO_ROOT / "tools" / "lan_remote_view_server.py"),
            "--host", self.config.stack.host,
            "--port", str(self.config.stack.https_port),
            "--root", str(REPO_ROOT / "web"),
            "--udp-port", str(self.config.stack.tracking_port),
            "--stream-port", str(self.config.stack.stream_port),
            "--robot-calibration-status-port", str(self.config.stack.calibration_status_port),
            "--password", password,
        ]
        if self.config.stack.public_mode == "quick":
            if not password or password == "change-me":
                raise RuntimeError("quick public mode requires a strong GODOT_REMOTE_PASSWORD")
            server.append("--allow-quick-tunnel-arm")
        self._spawn(LaunchSpec("operator server", tuple(server)))
        self._verify_local_url()
        if self.config.stack.public_mode == "quick":
            cloudflared = find_cloudflared()
            if not cloudflared:
                raise RuntimeError("cloudflared is required for public_mode='quick'")
            self._spawn(
                LaunchSpec(
                    "Cloudflare tunnel",
                    (
                        str(cloudflared), "tunnel", "--url",
                        f"https://127.0.0.1:{self.config.stack.https_port}",
                        "--no-tls-verify",
                    ),
                )
            )
        self._publish_state("running")
        print(f"Operator UI verified: {self.local_url}", flush=True)

    def run(self) -> int:
        def stop_handler(_signum, _frame):
            self.stopping.set()

        signal.signal(signal.SIGINT, stop_handler)
        if hasattr(signal, "SIGTERM"):
            signal.signal(signal.SIGTERM, stop_handler)
        try:
            self.start()
            while not self.stopping.wait(0.25):
                for name, process in self.processes:
                    code = process.poll()
                    if code is not None:
                        raise RuntimeError(f"{name} exited with status {code}")
            return 0
        except Exception as exc:
            self._publish_state("failed", str(exc))
            print(f"robot-teleop: {exc}", file=sys.stderr)
            return 1
        finally:
            self.stop()

    def stop(self) -> None:
        self.stopping.set()
        try:
            self.robot.hold()
            time.sleep(0.1)
        except Exception as exc:
            print(f"WARNING: robot Hold before shutdown failed: {exc}", file=sys.stderr)
        for _name, process in reversed(self.processes):
            if process.poll() is None:
                process.terminate()
        deadline = time.monotonic() + 4
        for _name, process in reversed(self.processes):
            if process.poll() is not None:
                continue
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                process.kill()
        self._publish_state("stopped")


def stop_running_supervisor() -> bool:
    state = read_state()
    pid = int(state.get("supervisor_pid", 0) or 0)
    if not state.get("running") or pid <= 0:
        return False
    try:
        os.kill(pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        return False
    return True
