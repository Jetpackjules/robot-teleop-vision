from __future__ import annotations

import json
import os
import re
import sys
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from typing import Any

from robot_teleop.config import REPO_ROOT
from robot_teleop.registry import register

MODULE_ID = re.compile(r"^[a-z][a-z0-9_-]{0,63}$")
MANIFEST_NAME = "robot.json"


@dataclass(frozen=True)
class RobotModuleManifest:
    schema_version: int
    id: str
    label: str
    version: str
    description: str
    capabilities: dict[str, Any]
    entrypoints: dict[str, str]
    state_files: tuple[str, ...]
    path: Path

    @classmethod
    def load(cls, path: Path) -> RobotModuleManifest:
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid robot module manifest {path}: {exc}") from exc
        if not isinstance(raw, dict):
            raise TypeError(f"robot module manifest must be an object: {path}")
        schema_version = int(raw.get("schema_version", 0))
        if schema_version != 1:
            raise ValueError(f"unsupported robot module schema {schema_version}: {path}")
        module_id = str(raw.get("id", ""))
        if not MODULE_ID.fullmatch(module_id):
            raise ValueError(f"invalid robot module id {module_id!r}: {path}")
        capabilities = raw.get("capabilities", {})
        entrypoints = raw.get("entrypoints", {})
        state_files = raw.get("state_files", [])
        if not isinstance(capabilities, dict) or not isinstance(entrypoints, dict):
            raise TypeError(f"capabilities and entrypoints must be objects: {path}")
        if not isinstance(state_files, list) or not all(
            isinstance(item, str)
            and item
            and item not in {".", ".."}
            and Path(item).name == item
            for item in state_files
        ):
            raise ValueError(f"state_files must contain safe file names: {path}")
        for required in ("control_spaces", "inputs", "actions", "views", "features"):
            value = capabilities.get(required, [])
            if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
                raise ValueError(f"capabilities.{required} must be a string list: {path}")
        if "hold" not in capabilities.get("actions", []):
            raise ValueError(f"robot modules must advertise the safety action 'hold': {path}")
        return cls(
            schema_version=schema_version,
            id=module_id,
            label=str(raw.get("label", module_id)),
            version=str(raw.get("version", "0.0.0")),
            description=str(raw.get("description", "")),
            capabilities=dict(capabilities),
            entrypoints={str(key): str(value) for key, value in entrypoints.items()},
            state_files=tuple(state_files),
            path=path.resolve(),
        )

    def public_dict(self) -> dict[str, Any]:
        public_entrypoints = {
            key: value
            for key, value in self.entrypoints.items()
            if key in {"godot", "web"}
        }
        return {
            "schema_version": self.schema_version,
            "id": self.id,
            "label": self.label,
            "version": self.version,
            "description": self.description,
            "capabilities": self.capabilities,
            "entrypoints": public_entrypoints,
        }


_MANIFESTS: dict[str, RobotModuleManifest] = {}
_LOADED_PATHS: set[Path] = set()


def _search_roots(extra_paths: tuple[str, ...] = ()) -> tuple[Path, ...]:
    values: list[str | Path] = [REPO_ROOT / "robot_modules"]
    environment = os.environ.get("ROBOT_TELEOP_MODULE_PATH", "")
    if environment:
        values.extend(item for item in environment.split(os.pathsep) if item)
    values.extend(extra_paths)
    roots: list[Path] = []
    for value in values:
        path = Path(os.path.expandvars(str(value))).expanduser()
        if not path.is_absolute():
            path = REPO_ROOT / path
        resolved = path.resolve()
        if resolved not in roots:
            roots.append(resolved)
    return tuple(roots)


def _load_symbol(reference: str, manifest: RobotModuleManifest) -> Any:
    module_name, separator, symbol_name = reference.partition(":")
    if not separator or not module_name or not symbol_name:
        raise ValueError(
            f"entrypoints.python must use 'package.module:Class' in {manifest.path}"
        )
    module_root = manifest.path.parent.parent
    search_paths = (module_root, module_root.parent)
    added: list[str] = []
    for search_path in reversed(search_paths):
        value = str(search_path)
        if value not in sys.path:
            sys.path.insert(0, value)
            added.append(value)
    try:
        return getattr(import_module(module_name), symbol_name)
    finally:
        for value in added:
            sys.path.remove(value)


def robot_module_entrypoint(
    module_id: str,
    name: str,
    extra_paths: tuple[str, ...] = (),
) -> Any | None:
    """Load one private module entrypoint without exposing it to the browser."""

    if module_id == "disabled":
        return None
    manifest = robot_module_manifest(module_id, extra_paths)
    reference = manifest.entrypoints.get(name, "")
    return _load_symbol(reference, manifest) if reference else None


def load_robot_modules(extra_paths: tuple[str, ...] = ()) -> tuple[RobotModuleManifest, ...]:
    for root in _search_roots(extra_paths):
        if root in _LOADED_PATHS:
            continue
        _LOADED_PATHS.add(root)
        if not root.is_dir():
            continue
        for path in sorted(root.glob(f"*/{MANIFEST_NAME}")):
            manifest = RobotModuleManifest.load(path)
            existing = _MANIFESTS.get(manifest.id)
            if existing and existing.path != manifest.path:
                raise ValueError(
                    f"duplicate robot module id {manifest.id!r}: "
                    f"{existing.path} and {manifest.path}"
                )
            python_entrypoint = manifest.entrypoints.get("python", "")
            if not python_entrypoint:
                raise ValueError(f"robot module {manifest.id!r} has no Python entrypoint")
            factory = _load_symbol(python_entrypoint, manifest)
            register("robot", manifest.id, factory)
            _MANIFESTS[manifest.id] = manifest
    return tuple(_MANIFESTS[key] for key in sorted(_MANIFESTS))


def robot_module_manifest(module_id: str, extra_paths: tuple[str, ...] = ()) -> RobotModuleManifest:
    load_robot_modules(extra_paths)
    try:
        return _MANIFESTS[module_id]
    except KeyError as exc:
        available = ", ".join(sorted(_MANIFESTS)) or "none"
        raise KeyError(f"unknown robot module {module_id!r} (available: {available})") from exc


def public_robot_module(module_id: str, extra_paths: tuple[str, ...] = ()) -> dict[str, Any]:
    if module_id == "disabled":
        from robot_teleop.robots.disabled import DisabledRobotAdapter

        return DisabledRobotAdapter().public_manifest()
    return robot_module_manifest(module_id, extra_paths).public_dict()
