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
from urllib.parse import urlparse

from robot_teleop.config import REPO_ROOT, AppConfig
from robot_teleop.doctor import (
    ensure_godot_extension_index,
    find_cloudflared,
    find_godot,
    godot_extension_smoke_status,
)
from robot_teleop.interfaces import LaunchSpec
from robot_teleop.journal import JournalSink
from robot_teleop.modules import public_robot_module
from robot_teleop.registry import create

RUN_DIR = REPO_ROOT / ".teleop"
STATE_PATH = RUN_DIR / "run.json"
QUICK_URL_PATTERN = re.compile(r"https://[a-z0-9-]+\.trycloudflare\.com", re.IGNORECASE)


def operator_ui_response_ready(response_url: str, status: int, body: bytes) -> bool:
    """Accept either the operator shell or its expected password gate."""

    if status != 200:
        return False
    if b"hybrid-canvas" in body:
        return True
    return (
        urlparse(response_url).path == "/login"
        and b'<form method="post" action="/login">' in body
        and b'name="password"' in body
    )


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


def write_runtime_configuration(project_root: Path, payload: dict) -> None:
    """Publish the same non-secret launch settings for a standalone editor."""
    destination = project_root / ".teleop" / "runtime_config.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".runtime_config.{os.getpid()}.tmp")
    try:
        temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)


class Supervisor:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.processes: list[tuple[str, subprocess.Popen]] = []
        self.stopping = threading.Event()
        self.public_url = ""
        self._pending_public_url = ""
        self.local_url = f"https://127.0.0.1:{config.stack.https_port}/controller.html"
        self.robot = create("robot", config.robot.adapter, config=config.robot)
        self.robot_manifest = public_robot_module(
            config.robot.adapter,
            config.robot.module_paths,
        )
        self.journal = JournalSink()

    def _log(self, message: str, *, component: str = "supervisor", priority: int = 6) -> None:
        rendered = f"[{component}] {message}" if component != "supervisor" else message
        print(rendered, flush=True, file=sys.stderr if priority <= 3 else sys.stdout)
        self.journal.emit(rendered, priority=priority, component=component)

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
            env={**os.environ, **spec.environment},
        )
        self.processes.append((spec.name, process))
        threading.Thread(target=self._relay_output, args=(spec.name, process), daemon=True).start()
        return process

    def _relay_output(self, name: str, process: subprocess.Popen) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            self._log(line.rstrip("\r\n"), component=name)
            match = QUICK_URL_PATTERN.search(line)
            if match and not self.public_url and not self._pending_public_url:
                self._pending_public_url = f"{match.group(0)}/controller.html"
                threading.Thread(
                    target=self._verify_public_url,
                    args=(self._pending_public_url,),
                    daemon=True,
                ).start()

    def _publish_state(self, status: str, detail: str = "") -> None:
        _write_state(
            {
                "running": status in {"starting", "running"},
                "status": status,
                "detail": detail,
                "supervisor_pid": os.getpid(),
                "local_url": self.local_url,
                "public_url": self.public_url,
                "robot_module": self.robot_manifest,
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
                    body = response.read(200_000)
                    if operator_ui_response_ready(response.geturl(), response.status, body):
                        return
            except Exception as exc:
                last_error = exc
            time.sleep(0.25)
        raise RuntimeError(f"operator UI health check failed: {last_error}")

    def _verify_public_url(self, candidate_url: str) -> None:
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not self.stopping.is_set():
            try:
                request = urllib.request.Request(candidate_url, method="GET")
                with urllib.request.urlopen(request, timeout=5) as response:
                    body = response.read(200_000)
                    if operator_ui_response_ready(response.geturl(), response.status, body):
                        self.public_url = candidate_url
                        self._pending_public_url = ""
                        self._publish_state("running")
                        self._log(f"Verified public operator URL: {candidate_url}")
                        return
            except Exception:
                time.sleep(0.5)
        self._pending_public_url = ""
        self._log(
            f"WARNING: public URL did not pass its health check: {candidate_url}",
            priority=4,
        )

    def start(self) -> None:
        calibration_port = self.config.stack.calibration_status_port
        if (
            isinstance(calibration_port, bool)
            or not isinstance(calibration_port, int)
            or not 1 <= calibration_port <= 65535
        ):
            raise ValueError("stack.calibration_status_port must be an integer from 1 to 65535")
        module_configuration = getattr(
            self.robot,
            "godot_configuration",
            lambda **_kwargs: {},
        )(
            calibration_status_port=calibration_port,
            reserved_udp_ports={
                "stack.tracking_port": self.config.stack.tracking_port,
                # The shared Godot gateway still uses its scene default. Do
                # not let a custom module endpoint steal that listener even
                # when the operator's configured tracking target differs.
                "Godot remote-control gateway": 4247,
            },
        )
        runtime_configuration = {
            "schema_version": 1,
            "module": self.config.robot.adapter,
            "enabled": self.config.robot.enabled,
            "calibration_status_port": calibration_port,
            "module_settings": module_configuration,
        }
        # Resolve everything before starting hardware so a bad port/profile
        # cannot leave a partially started follower behind.
        runtime_configuration_json = json.dumps(runtime_configuration)
        robot_spec = self.robot.launch_spec()
        operator_environment = getattr(
            self.robot,
            "operator_environment",
            dict,
        )()
        for port, label in (
            (self.config.stack.https_port, "HTTPS UI"),
            (self.config.stack.stream_port, "Godot RGB-D stream"),
            (self.config.stack.calibration_status_port, "calibration status"),
        ):
            if not _port_available(port):
                raise RuntimeError(f"{label} port {port} is already in use")
        self._publish_state("starting")
        write_runtime_configuration(self.config.project_root, runtime_configuration)
        if robot_spec:
            self._spawn(robot_spec)
        if self.config.godot.launch_runtime:
            godot = find_godot(self.config.godot.executable)
            if not godot:
                raise RuntimeError("Godot 4.6+ was not found; set godot.executable in config/local.toml")
            godot_command = [str(godot), "--headless"]
            if not self.config.godot.packaged_runtime:
                ensure_godot_extension_index(godot, self.config.project_root)
                extension_ok, extension_detail = godot_extension_smoke_status(
                    godot,
                    self.config.project_root,
                )
                if not extension_ok:
                    raise RuntimeError(
                        f"Godot native RGB-D extension check failed: {extension_detail}"
                    )
                godot_command.extend(("--path", str(self.config.project_root)))
            self._spawn(
                LaunchSpec(
                    "Godot runtime",
                    tuple(godot_command),
                    {
                        "ROBOT_TELEOP_MODULE": self.config.robot.adapter,
                        "ROBOT_TELEOP_RUNTIME_CONFIG": runtime_configuration_json,
                    },
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
            f"--password={password}",
        ]
        if self.config.stack.public_mode == "quick":
            if not password or password == "change-me":
                raise RuntimeError("quick public mode requires a strong GODOT_REMOTE_PASSWORD")
            server.append("--allow-quick-tunnel-robot")
        module_paths = os.pathsep.join(self.config.robot.module_paths)
        self._spawn(
            LaunchSpec(
                "operator server",
                tuple(server),
                {
                    "ROBOT_TELEOP_MODULE": self.config.robot.adapter,
                    "ROBOT_TELEOP_MODULE_MANIFEST": json.dumps(self.robot_manifest),
                    "ROBOT_TELEOP_MODULE_PATH": module_paths,
                    **operator_environment,
                },
            )
        )
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
        self._log(f"Operator UI verified: {self.local_url}")

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
            self._log(f"robot-teleop: {exc}", priority=3)
            return 1
        finally:
            self.stop()

    def stop(self) -> None:
        self.stopping.set()
        try:
            self.robot.hold()
            time.sleep(0.1)
        except Exception as exc:
            self._log(f"WARNING: robot Hold before shutdown failed: {exc}", priority=4)
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
        self.journal.close()


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
