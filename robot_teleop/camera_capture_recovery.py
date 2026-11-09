"""Bounded recovery of the configured local capture owner after calibration."""
from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.parse import urlsplit
from urllib.request import urlopen

from robot_teleop.state import godot_app_userdata_root

ROOT = Path(__file__).resolve().parents[1]


def read_policy():
    path = godot_app_userdata_root() / "Robot Teleop Vision/camera_color_policy.json"
    if not path.exists():
        return {}
    value = json.loads(path.read_text())
    if not isinstance(value, dict) or value.get("schema_version") != 1 or not isinstance(value.get("cameras", {}), dict):
        raise ValueError("Invalid camera color policy")
    for serial, settings in value.get("cameras", {}).items():
        if (not isinstance(serial, str) or not serial or not serial.isascii()
                or not all(c.isalnum() or c in "-_" for c in serial)
                or not isinstance(settings, dict)
                or type(settings.get("auto_white_balance")) is not bool):
            raise ValueError("Invalid camera white-balance choice")
    return value


def apply_color_policy(policy):
    if not policy.get("cameras"):
        return
    import pyrealsense2 as rs

    for device in rs.context().query_devices():
        serial = device.get_info(rs.camera_info.serial_number)
        settings = policy["cameras"].get(serial)
        if settings is None:
            continue
        for sensor in device.query_sensors():
            if sensor.supports(rs.option.enable_auto_white_balance):
                desired = float(settings["auto_white_balance"])
                if sensor.get_option(rs.option.enable_auto_white_balance) != desired:
                    sensor.set_option(rs.option.enable_auto_white_balance, desired)


def stale_cameras(ids):
    try:
        with urlopen("http://127.0.0.1:8781/cameras", timeout=3) as response:
            cameras = json.load(response)["cameras"]
    except (OSError, ValueError, KeyError):
        return list(ids)
    fresh = set()
    for camera in cameras:
        age = camera.get("image_levels", {}).get("rgb_frame_age_ms", -1)
        if type(age) in (int, float) and 0 <= age < 1000:
            fresh.add(camera.get("id"))
    return [serial for serial in ids if serial not in fresh]


def wait_ready(ids, seconds):
    deadline = time.monotonic() + seconds
    while True:
        missing = stale_cameras(ids)
        if not missing or time.monotonic() >= deadline:
            return missing
        time.sleep(.5)


def _capture_snapshot():
    configuration = json.loads((ROOT / ".teleop/runtime_config.json").read_text())["capture"]
    pid, prefix = configuration["pid"], configuration["prefix"]
    if type(pid) is not int or pid <= 1 or not isinstance(prefix, str) or not prefix.isidentifier():
        raise ValueError("Invalid capture owner identity")
    process = Path("/proc") / str(pid)
    if process.stat().st_uid != os.getuid() or (process / "cwd").resolve() != ROOT:
        raise ValueError("Capture owner is outside this workspace")
    argv = [a.decode() for a in (process / "cmdline").read_bytes().split(b"\0") if a]
    env = dict(x.split("=", 1) for x in (process / "environ").read_bytes().decode().split("\0") if "=" in x)
    if ("--headless" not in argv or argv[-1] != "res://godot/runtime/streaming/camera_capture_service.gd"
            or env.get("ROBOT_TELEOP_CAPTURE_ONLY") != "1"
            or env.get("ROBOT_TELEOP_MODULE") != "disabled"
            or env.get("ROBOT_TELEOP_CAPTURE_SHM_PREFIX") != prefix
            or configuration.get("port") != 8781):
        raise ValueError("Process is not the configured isolated camera capture owner")
    return pid, argv, env, (process / "stat").read_text().split(") ", 1)[1].split()[19]


def restart_capture(stale, folder):
    import fcntl
    import pyrealsense2 as rs

    with (ROOT / ".teleop/camera-capture-recovery.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        pid, argv, env, started = _capture_snapshot()
        process = Path("/proc") / str(pid)
        if (process / "stat").read_text().split(") ", 1)[1].split()[19] != started:
            raise ValueError("Capture owner changed before recovery")
        os.kill(pid, signal.SIGINT)
        deadline = time.monotonic() + 10
        while process.exists():
            if (process / "stat").read_text().split(") ", 1)[1].startswith("Z"):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("Capture did not stop; no force kill attempted")
            time.sleep(.1)
        reset_errors = []
        # Only the requested cameras that failed fresh-frame readiness are reset.
        # Always relaunch capture, even if a device disappeared during reset.
        try:
            context = rs.context()
            for device in context.query_devices():
                serial = device.get_info(rs.camera_info.serial_number)
                if serial in stale:
                    try:
                        device.hardware_reset()
                    except Exception as error:
                        reset_errors.append(str(error))
            del context
            time.sleep(4)
        except Exception as error:
            reset_errors.append(str(error))
        finally:
            with (folder / "capture-recovery.log").open("ab", buffering=0) as log:
                child = subprocess.Popen(argv, cwd=ROOT, env=env, stdin=subprocess.DEVNULL,
                                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            _record_capture_pid(pid, child.pid)
            (folder / "capture-recovery.json").write_text(json.dumps({
                "previous_pid": pid, "capture_pid": child.pid,
                "reset_serials": stale, "reset_errors": reset_errors,
                "calibration_written": False, "robot_commands_sent": False,
            }, indent=2))


def _record_capture_pid(previous, current):
    paths = [ROOT / ".teleop/runtime_config.json", ROOT / ".teleop/latest-web-ui-session.json",
             ROOT / ".teleop/virtual-godot/session.json"]
    latest = ROOT / ".teleop/latest-web-ui-session.json"
    if latest.exists():
        metadata = json.loads(latest.read_text())
        state = metadata.get("state_directory")
        if state and Path(state).resolve().is_relative_to(ROOT / ".teleop"):
            paths.append(Path(state) / "state.json")
        desktop = metadata.get("desktop_state")
        if desktop and Path(desktop).resolve().is_relative_to(ROOT / ".teleop"):
            paths.append(Path(desktop) / "session.json")
    for path in dict.fromkeys(paths):
        if not path.exists():
            continue
        value = json.loads(path.read_text())
        changed = False
        for mapping, key in [(value.get("capture", {}), "pid"), (value.get("pids", {}), "capture"),
                             (value, "capture_pid")]:
            if mapping.get(key) == previous:
                mapping[key] = current
                changed = True
        if changed:
            temporary = path.with_name(path.name + ".capture-recovery.tmp")
            temporary.write_text(json.dumps(value, indent=2) + "\n")
            temporary.replace(path)


def prepare_calibration_capture(request, folder, progress):
    policy = read_policy()
    if not policy:
        return
    source = urlsplit(request["source_url"])
    # Never use calibration input to control an arbitrary host or service.
    if (source.scheme, source.hostname, source.port, source.path, source.query) != (
            "http", "127.0.0.1", 8781, "/rgbd", ""):
        return
    apply_color_policy(policy)
    if policy.get("recover_capture_after_calibration") is not True or not sys.platform.startswith("linux"):
        return
    progress("Waiting for fresh RGB-D from every camera after calibration")
    missing = wait_ready(request["cameras"], 8)
    if missing:
        progress("Recovering stalled camera capture once before fresh-frame validation")
        restart_capture(missing, folder)
        missing = wait_ready(request["cameras"], 25)
        if missing:
            raise ValueError("Camera capture did not recover: " + ", ".join(missing))
        apply_color_policy(policy)

