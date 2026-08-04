from __future__ import annotations

import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import zipfile
from collections.abc import Iterable
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit, urlunsplit

from robot_teleop.config import REPO_ROOT, AppConfig
from robot_teleop.doctor import run_checks
from robot_teleop.supervisor import read_state

SENSITIVE_KEY = re.compile(
    r"password|passwd|token|secret|credential|private.?key|api.?key|access.?key|cookie|authorization",
    re.IGNORECASE,
)
INLINE_SECRET = re.compile(
    r"(?i)\b(password|passwd|token|secret|authorization|cookie)"
    r"(\s*[:=]\s*)([^\s,;]+)",
)
URL_PATTERN = re.compile(r"https?://[^\s\]\[\"'<>]+", re.IGNORECASE)


def _alias(value: str, prefix: str) -> str:
    digest = hashlib.sha256(value.encode("utf-8", errors="replace")).hexdigest()[:10]
    return f"<{prefix}-{digest}>"


def _secret_values(value: Any, *, key: str = "") -> list[str]:
    if isinstance(value, dict):
        result: list[str] = []
        for child_key, child_value in value.items():
            if SENSITIVE_KEY.search(str(child_key)):
                if isinstance(child_value, str) and child_value:
                    result.append(child_value)
            else:
                result.extend(_secret_values(child_value, key=str(child_key)))
        return result
    if isinstance(value, (list, tuple)):
        return [item for child in value for item in _secret_values(child, key=key)]
    if key and SENSITIVE_KEY.search(key) and isinstance(value, str) and value:
        return [value]
    return []


def sanitize_url(value: str) -> str:
    try:
        parts = urlsplit(value)
    except ValueError:
        return "<redacted-url>"
    if not parts.scheme or not parts.netloc:
        return value
    hostname = parts.hostname or "host"
    if hostname.endswith(".trycloudflare.com"):
        netloc = "<temporary-quick-tunnel>"
    elif hostname in {"127.0.0.1", "localhost", "::1"}:
        netloc = hostname
        if parts.port:
            netloc += f":{parts.port}"
    else:
        netloc = _alias(hostname, "host")
        if parts.port:
            netloc += f":{parts.port}"
    return urlunsplit((parts.scheme, netloc, parts.path, "", ""))


def sanitize_text(
    value: str,
    *,
    secrets: Iterable[str] = (),
    aliases: dict[str, str] | None = None,
) -> str:
    sanitized = value
    replacements = dict(aliases or {})
    for secret in secrets:
        if secret and len(secret) >= 4:
            replacements.setdefault(secret, "<redacted>")
    for original in sorted(replacements, key=len, reverse=True):
        sanitized = sanitized.replace(original, replacements[original])
    home = str(Path.home())
    if home and home != "/":
        sanitized = sanitized.replace(home, "<HOME>")
    sanitized = INLINE_SECRET.sub(lambda match: f"{match.group(1)}{match.group(2)}<redacted>", sanitized)
    sanitized = URL_PATTERN.sub(lambda match: sanitize_url(match.group(0)), sanitized)
    return sanitized


def _run(command: list[str], *, timeout: float = 10.0) -> dict[str, Any]:
    try:
        result = subprocess.run(
            command,
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return {
            "command": command,
            "returncode": result.returncode,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"command": command, "returncode": None, "error": str(exc)}


def collect_journal(lines: int) -> dict[str, Any]:
    journalctl = shutil.which("journalctl")
    if not journalctl:
        return {"available": False, "detail": "journalctl is not installed", "text": ""}
    result = _run(
        [
            journalctl,
            "--no-pager",
            "--quiet",
            "--since=-24h",
            "-n",
            str(max(1, min(lines, 5000))),
            "-o",
            "short-iso",
            "SYSLOG_IDENTIFIER=robot-teleop",
        ],
        timeout=15,
    )
    text = "\n".join(part for part in (result.get("stdout", ""), result.get("stderr", "")) if part)
    return {
        "available": result.get("returncode") == 0,
        "detail": "last 24 hours, newest entries capped" if result.get("returncode") == 0 else "journal query failed",
        "text": text,
    }


def _sanitize_devices(devices: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, str]]:
    sanitized: list[dict[str, Any]] = []
    aliases: dict[str, str] = {}
    for device in devices:
        identifier = str(device.get("identifier", ""))
        alias = _alias(identifier, "camera") if identifier else "<camera-unidentified>"
        if identifier:
            aliases[identifier] = alias
        label = str(device.get("label", ""))
        sanitized.append(
            {
                "adapter": str(device.get("adapter", "")),
                "identifier": alias,
                "label": sanitize_text(label, aliases=aliases),
                "metadata": dict(device.get("metadata", {})),
            }
        )
    return sanitized, aliases


def _sanitized_config(config: AppConfig) -> dict[str, Any]:
    return {
        "stack": {
            "host": config.stack.host,
            "https_port": config.stack.https_port,
            "stream_port": config.stack.stream_port,
            "tracking_port": config.stack.tracking_port,
            "calibration_status_port": config.stack.calibration_status_port,
            "public_mode": config.stack.public_mode,
        },
        "godot": {
            "executable": Path(config.godot.executable).name if config.godot.executable else "auto",
            "project": "repository root" if config.project_root == REPO_ROOT else "custom path",
            "launch_runtime": config.godot.launch_runtime,
            "packaged_runtime": config.godot.packaged_runtime,
        },
        "cameras": {"adapters": list(config.cameras.adapters)},
        "robot": {
            "adapter": config.robot.adapter,
            "enabled": config.robot.enabled,
            # Module options may contain ports, paths, identities, or future credentials.
            "option_names": sorted(config.robot.options),
        },
    }


def _sanitized_state(state: dict[str, Any]) -> dict[str, Any]:
    safe = {
        key: value
        for key, value in state.items()
        if key in {
            "running",
            "status",
            "detail",
            "supervisor_pid",
            "robot_module",
            "processes",
            "updated_unix_ms",
        }
    }
    for key in ("local_url", "public_url"):
        if state.get(key):
            safe[key] = sanitize_url(str(state[key]))
    return safe


def create_support_bundle(
    config: AppConfig,
    output: Path | None = None,
    *,
    journal_lines: int = 1000,
) -> Path:
    created = datetime.now(timezone.utc)
    destination = output or Path.cwd() / f"robot-teleop-support-{created.strftime('%Y%m%dT%H%M%SZ')}.zip"
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(f"support bundle already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)

    checks, raw_devices = run_checks(config)
    devices, device_aliases = _sanitize_devices(raw_devices)
    config_values = asdict(config)
    secrets = _secret_values(config_values)
    password_from_environment = os.environ.get(config.stack.password_env, "")
    if password_from_environment:
        secrets.append(password_from_environment)

    git_commit = _run(["git", "rev-parse", "HEAD"])
    git_branch = _run(["git", "branch", "--show-current"])
    git_status = _run(["git", "status", "--short"])
    changed_paths = [line for line in str(git_status.get("stdout", "")).splitlines() if line]
    journal = collect_journal(journal_lines)
    journal_text = sanitize_text(
        str(journal.pop("text", "")),
        secrets=secrets,
        aliases=device_aliases,
    )
    summary = {
        "schema_version": 1,
        "created_utc": created.isoformat(),
        "privacy": {
            "sanitized": True,
            "excluded": [
                "raw configuration and environment",
                "passwords, tokens, and credentials",
                "robot profiles and calibration payloads",
                "captures, RGB/depth frames, and debug images",
            ],
            "pseudonymized": ["camera serials", "home-directory paths", "remote hostnames"],
        },
        "system": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "python": sys.version,
        },
        "repository": {
            "commit": git_commit.get("stdout", "unavailable"),
            "branch": git_branch.get("stdout", "unavailable"),
            "worktree_clean": not changed_paths,
            "changed_path_count": len(changed_paths),
        },
        "configuration": _sanitized_config(config),
        "runtime": _sanitized_state(read_state()),
        "checks": [asdict(check) for check in checks],
        "devices": devices,
        "journal": journal,
    }
    summary_text = sanitize_text(
        json.dumps(summary, indent=2, default=str),
        secrets=secrets,
        aliases=device_aliases,
    )
    notice = (
        "Robot Teleop Vision sanitized support bundle\n\n"
        "Safe to share after a quick human review. It intentionally excludes raw local config, "
        "environment variables, robot profiles, calibration data, and camera imagery.\n"
    )
    try:
        with zipfile.ZipFile(destination, "x", compression=zipfile.ZIP_DEFLATED) as archive:
            archive.writestr("README.txt", notice)
            archive.writestr("summary.json", summary_text + "\n")
            archive.writestr("journal.log", journal_text + ("\n" if journal_text else ""))
    except Exception:
        destination.unlink(missing_ok=True)
        raise
    return destination
