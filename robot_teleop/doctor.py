from __future__ import annotations

import importlib.util
import json
import shutil
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path

from robot_teleop.config import AppConfig, REPO_ROOT
from robot_teleop.registry import create


@dataclass(frozen=True)
class Check:
    name: str
    ok: bool
    detail: str


def find_godot(configured: str = "") -> Path | None:
    if configured:
        candidate = Path(configured).expanduser()
        return candidate.resolve() if candidate.is_file() else None
    for command in ("godot", "godot4"):
        resolved = shutil.which(command)
        if resolved:
            return Path(resolved).resolve()
    downloads = Path.home() / "Downloads"
    candidates = sorted(downloads.glob("Godot_v4.*_linux.x86_64"), reverse=True)
    return candidates[0].resolve() if candidates else None


def find_cloudflared() -> Path | None:
    local = REPO_ROOT / "tools" / "bin" / "cloudflared"
    if local.is_file():
        return local
    resolved = shutil.which("cloudflared")
    return Path(resolved).resolve() if resolved else None


def run_checks(config: AppConfig) -> tuple[list[Check], list[dict]]:
    checks = [
        Check("project.godot", (REPO_ROOT / "project.godot").is_file(), str(REPO_ROOT)),
        Check("web UI", (REPO_ROOT / "web" / "controller.html").is_file(), "web/controller.html"),
        Check(
            "native extension",
            (REPO_ROOT / "native" / "realsense_shared_memory" / "realsense_shared_memory.gdextension").is_file(),
            "native/realsense_shared_memory",
        ),
    ]
    godot = find_godot(config.godot.executable)
    version = "not found"
    godot_supported = False
    if godot:
        result = subprocess.run([str(godot), "--version"], capture_output=True, text=True)
        version = (result.stdout or result.stderr).strip()
        try:
            major, minor = (int(value) for value in version.split(".", 2)[:2])
            godot_supported = (major, minor) >= (4, 6)
        except (TypeError, ValueError):
            godot_supported = False
    checks.append(Check("Godot 4.6+", godot is not None and godot_supported, version))
    for module in ("numpy", "av", "aiortc", "jwt"):
        checks.append(Check(f"Python {module}", importlib.util.find_spec(module) is not None, module))
    if "realsense" in config.cameras.adapters:
        checks.append(
            Check(
                "Python pyrealsense2",
                importlib.util.find_spec("pyrealsense2") is not None,
                "pyrealsense2",
            )
        )
    if config.stack.public_mode == "quick":
        tunnel = find_cloudflared()
        checks.append(Check("cloudflared", tunnel is not None, str(tunnel or "not found")))
    devices: list[dict] = []
    for name in config.cameras.adapters:
        try:
            adapter = create("camera", name)
            found = [asdict(device) for device in adapter.discover()]
            devices.extend(found)
            checks.append(Check(f"camera adapter: {name}", True, f"{len(found)} device(s)"))
        except Exception as exc:
            checks.append(Check(f"camera adapter: {name}", False, str(exc)))
    try:
        robot = create("robot", config.robot.adapter, config=config.robot)
        spec = robot.launch_spec()
        checks.append(Check("robot adapter", True, spec.name if spec else "disabled"))
    except Exception as exc:
        checks.append(Check("robot adapter", False, str(exc)))
    return checks, devices


def format_report(checks: list[Check], devices: list[dict], *, as_json: bool = False) -> str:
    if as_json:
        return json.dumps(
            {"ok": all(check.ok for check in checks), "checks": [asdict(c) for c in checks], "devices": devices},
            indent=2,
        )
    lines = [f"{'OK' if check.ok else 'FAIL':4}  {check.name}: {check.detail}" for check in checks]
    if devices:
        lines.append("\nDetected cameras:")
        lines.extend(f"  - {device['label']}" for device in devices)
    else:
        lines.append("\nDetected cameras: none")
    return "\n".join(lines)
