from __future__ import annotations

import argparse
import json
import os
import secrets
import sys
import time
import webbrowser
from dataclasses import asdict
from pathlib import Path

import robot_teleop.cameras  # noqa: F401
import robot_teleop.robots  # noqa: F401
import robot_teleop.tracking  # noqa: F401
from robot_teleop.config import DEFAULT_CONFIG, load_config
from robot_teleop.doctor import format_report, run_checks
from robot_teleop.modules import load_robot_modules
from robot_teleop.registry import create
from robot_teleop.state import migrate_project_state
from robot_teleop.supervisor import Supervisor, read_state, stop_running_supervisor


def _initialize(destination: Path, example: str, force: bool) -> int:
    source = Path(__file__).resolve().parents[1] / "config" / "examples" / f"{example}.toml"
    if destination.exists() and not force:
        print(f"Already exists: {destination}", file=sys.stderr)
        return 1
    destination.parent.mkdir(parents=True, exist_ok=True)
    contents = source.read_text(encoding="utf-8")
    contents = contents.replace(
        'password_default = "change-me"',
        f'password_default = "{secrets.token_urlsafe(24)}"',
    )
    destination.write_text(contents, encoding="utf-8")
    try:
        os.chmod(destination, 0o600)
    except OSError:
        pass
    print(f"Created local configuration: {destination}")
    return 0


def _example_names() -> tuple[str, ...]:
    root = Path(__file__).resolve().parents[1] / "config" / "examples"
    return tuple(path.stem for path in sorted(root.glob("*.toml")))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robot-teleop")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    sub = parser.add_subparsers(dest="command", required=True)
    initialize = sub.add_parser("init", help="create an ignored local configuration")
    initialize.add_argument("--example", choices=_example_names(), default="vision_only")
    initialize.add_argument("--force", action="store_true")
    sub.add_parser("doctor", help="check dependencies and discover hardware")
    devices = sub.add_parser("devices", help="list discovered camera devices")
    devices.add_argument("--json", action="store_true")
    modules = sub.add_parser("modules", help="list installed robot modules")
    modules.add_argument("--json", action="store_true")
    sub.add_parser("start", help="start and supervise the local runtime")
    sub.add_parser("stop", help="safely stop the supervised runtime")
    sub.add_parser("status", help="show launcher state")
    sub.add_parser("open", help="open the active operator UI")
    migrate = sub.add_parser("migrate-state", help="copy durable calibration from an older Godot project")
    migrate.add_argument("--from-project", required=True)
    migrate.add_argument("--force", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config_path = Path(args.config).expanduser().resolve()
    if args.command == "init":
        return _initialize(config_path, args.example, args.force)
    if args.command == "status":
        print(json.dumps(read_state(), indent=2))
        return 0
    if args.command == "stop":
        if not stop_running_supervisor():
            print("Runtime is not running.")
            return 0
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and read_state().get("running"):
            time.sleep(0.1)
        print("Stop requested; physical motion was held before shutdown.")
        return 0
    if args.command == "open":
        state = read_state()
        url = state.get("public_url") or state.get("local_url")
        if not url:
            print("Runtime has no active operator URL.", file=sys.stderr)
            return 1
        webbrowser.open(url)
        print(url)
        return 0
    if args.command == "migrate-state":
        module_state_files = tuple(
            name
            for manifest in load_robot_modules()
            for name in manifest.state_files
        )
        copied = migrate_project_state(
            args.from_project,
            force=args.force,
            extra_state_files=module_state_files,
        )
        if copied:
            print("Copied durable state:\n" + "\n".join(f"  - {path.name}" for path in copied))
        else:
            print("No state copied; target files already exist or the source has no durable state.")
        return 0
    if args.command == "modules":
        modules = [manifest.public_dict() for manifest in load_robot_modules()]
        modules.insert(0, create("robot", "disabled").public_manifest())
        if args.json:
            print(json.dumps(modules, indent=2))
        else:
            print("\n".join(f"{item['id']}: {item['label']}" for item in modules))
        return 0
    config = load_config(config_path)
    if args.command == "doctor":
        checks, devices = run_checks(config)
        print(format_report(checks, devices))
        return 0 if all(check.ok for check in checks) else 1
    if args.command == "devices":
        discovered = []
        for name in config.cameras.adapters:
            discovered.extend(asdict(device) for device in create("camera", name).discover())
        if args.json:
            print(json.dumps(discovered, indent=2))
        elif discovered:
            print("\n".join(f"{item['adapter']}: {item['label']}" for item in discovered))
        else:
            print("No configured camera devices detected.")
        return 0
    if args.command == "start":
        return Supervisor(config).run()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
