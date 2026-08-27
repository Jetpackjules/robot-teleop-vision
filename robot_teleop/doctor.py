from __future__ import annotations

import importlib.util
import json
import shutil
import subprocess
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

from robot_teleop.config import REPO_ROOT, AppConfig
from robot_teleop.modules import public_robot_module
from robot_teleop.registry import create

GODOT_EXTENSION_RESOURCE = (
    "res://native/realsense_shared_memory/realsense_shared_memory.gdextension"
)
GODOT_EXTENSION_CLASSES = (
    "RealSenseDirectFrameSource",
    "RealSensePairCalibrator",
    "RealSenseSharedMemoryReader",
    "RealSenseSharedMemoryPointCloud",
)
GODOT_EXTENSION_CLASS = "RealSenseSharedMemoryPointCloud"
GODOT_EXTENSION_METHODS = (
    "get_depth_u16_frame",
    "get_color_image",
    "request_rgbd_encode",
    "take_rgbd_encoded_frame",
)
GODOT_SMOKE_MARKER = "ROBOT_TELEOP_GDEXTENSION_SMOKE="


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


def godot_console_executable(godot: Path) -> Path:
    """Prefer Godot's console wrapper for bounded diagnostic subprocesses on Windows."""

    if godot.suffix.lower() != ".exe" or godot.stem.lower().endswith("_console"):
        return godot
    console = godot.with_name(f"{godot.stem}_console{godot.suffix}")
    return console if console.is_file() else godot


def godot_extension_index_status(project_root: Path) -> tuple[bool, str]:
    """Report whether Godot's ignored source-project cache indexes our extension."""

    index = project_root / ".godot" / "extension_list.cfg"
    try:
        lines = index.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return False, f"missing {index}; run a Godot editor import scan"
    except OSError as exc:
        return False, f"could not read {index}: {exc}"
    resources = {line.strip().strip('"') for line in lines if line.strip()}
    if GODOT_EXTENSION_RESOURCE not in resources:
        return False, f"{index} does not list {GODOT_EXTENSION_RESOURCE}"
    return True, f"{index} lists {GODOT_EXTENSION_RESOURCE}"


def _godot_output_detail(result: subprocess.CompletedProcess[str]) -> str:
    combined = "\n".join(part for part in (result.stdout, result.stderr) if part)
    lines = [line.strip() for line in combined.splitlines() if line.strip()]
    if not lines:
        return f"Godot exited with status {result.returncode} and no output"
    return " | ".join(lines[-6:])[-1200:]


def ensure_godot_extension_index(godot: Path, project_root: Path) -> None:
    """Create Godot's ignored extension index before a source-mode launch."""

    indexed, _detail = godot_extension_index_status(project_root)
    if indexed:
        return
    try:
        result = subprocess.run(
            [
                str(godot_console_executable(godot)),
                "--headless",
                "--editor",
                "--path",
                str(project_root),
                "--quit",
            ],
            capture_output=True,
            check=False,
            encoding="utf-8",
            errors="replace",
            timeout=120,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"Godot editor import scan failed: {exc}") from exc
    if result.returncode != 0:
        raise RuntimeError(f"Godot editor import scan failed: {_godot_output_detail(result)}")
    indexed, detail = godot_extension_index_status(project_root)
    if not indexed:
        raise RuntimeError(f"Godot editor import scan did not index the native extension: {detail}")


def _godot_smoke_script() -> str:
    methods = json.dumps(list(GODOT_EXTENSION_METHODS))
    classes = json.dumps(list(GODOT_EXTENSION_CLASSES))
    class_name = json.dumps(GODOT_EXTENSION_CLASS)
    marker = json.dumps(GODOT_SMOKE_MARKER)
    return f"""extends SceneTree

const REQUIRED_CLASSES := {classes}
const POINT_CLOUD_CLASS := {class_name}
const REQUIRED_METHODS := {methods}
const RESULT_MARKER := {marker}

func _initialize() -> void:
    var missing_classes: Array[String] = []
    for required_class in REQUIRED_CLASSES:
        if not ClassDB.class_exists(required_class):
            missing_classes.append(required_class)
    var missing_methods: Array[String] = []
    if ClassDB.class_exists(POINT_CLOUD_CLASS):
        var available_methods := {{}}
        for method in ClassDB.class_get_method_list(POINT_CLOUD_CLASS):
            available_methods[String(method.get("name", ""))] = true
        for method_name in REQUIRED_METHODS:
            if not available_methods.has(method_name):
                missing_methods.append(method_name)
    var payload := {{
        "missing_classes": missing_classes,
        "missing_methods": missing_methods,
    }}
    print(RESULT_MARKER + JSON.stringify(payload))
    quit(0 if missing_classes.is_empty() and missing_methods.is_empty() else 3)
"""


def godot_extension_smoke_status(godot: Path, project_root: Path) -> tuple[bool, str]:
    """Load the extension in Godot and verify the RGB-D API used by the web streamer."""

    indexed, detail = godot_extension_index_status(project_root)
    if not indexed:
        return False, detail
    try:
        with tempfile.TemporaryDirectory(prefix="robot-teleop-godot-smoke-") as temporary:
            script = Path(temporary) / "extension_smoke.gd"
            script.write_text(_godot_smoke_script(), encoding="utf-8")
            result = subprocess.run(
                [
                    str(godot_console_executable(godot)),
                    "--headless",
                    "--path",
                    str(project_root),
                    "--quit-after",
                    "120",
                    "--script",
                    str(script),
                ],
                capture_output=True,
                check=False,
                encoding="utf-8",
                errors="replace",
                timeout=45,
            )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"Godot native-extension smoke check failed: {exc}"
    output = "\n".join(part for part in (result.stdout, result.stderr) if part)
    payload = None
    for line in output.splitlines():
        marker_at = line.find(GODOT_SMOKE_MARKER)
        if marker_at < 0:
            continue
        try:
            payload = json.loads(line[marker_at + len(GODOT_SMOKE_MARKER) :].strip())
        except json.JSONDecodeError:
            payload = None
    if not isinstance(payload, dict):
        return False, f"Godot smoke check returned no result: {_godot_output_detail(result)}"
    missing_classes = payload.get("missing_classes", [])
    if isinstance(missing_classes, list) and missing_classes:
        return False, "native extension is missing required classes: " + ", ".join(
            str(class_name) for class_name in missing_classes
        )
    missing = payload.get("missing_methods", [])
    if isinstance(missing, list) and missing:
        return False, (
            "loaded native extension is incompatible or stale; missing required methods: "
            + ", ".join(str(method) for method in missing)
        )
    if result.returncode != 0:
        return False, f"Godot native-extension smoke check failed: {_godot_output_detail(result)}"
    return True, (
        f"{len(GODOT_EXTENSION_CLASSES)} native classes loaded with "
        f"{len(GODOT_EXTENSION_METHODS)} required RGB-D methods"
    )


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
        result = subprocess.run(
            [str(godot_console_executable(godot)), "--version"],
            capture_output=True,
            check=False,
            text=True,
        )
        version = (result.stdout or result.stderr).strip()
        try:
            major, minor = (int(value) for value in version.split(".", 2)[:2])
            godot_supported = (major, minor) >= (4, 6)
        except (TypeError, ValueError):
            godot_supported = False
    checks.append(Check("Godot 4.6+", godot is not None and godot_supported, version))
    if not config.godot.packaged_runtime:
        extension_indexed, extension_index_detail = godot_extension_index_status(
            config.project_root
        )
        checks.append(Check("Godot extension index", extension_indexed, extension_index_detail))
        if godot is not None and godot_supported and extension_indexed:
            extension_loaded, extension_detail = godot_extension_smoke_status(
                godot,
                config.project_root,
            )
        elif not extension_indexed:
            extension_loaded, extension_detail = False, (
                "not checked because the Godot extension index is missing"
            )
        else:
            extension_loaded, extension_detail = False, (
                "not checked because Godot 4.6+ is unavailable"
            )
        checks.append(Check("Godot native RGB-D API", extension_loaded, extension_detail))
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
        manifest = public_robot_module(config.robot.adapter, config.robot.module_paths)
        checks.append(
            Check(
                "robot module",
                True,
                f"{manifest['label']} ({spec.name if spec else 'motion disabled'})",
            )
        )
    except Exception as exc:
        checks.append(Check("robot module", False, str(exc)))
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
