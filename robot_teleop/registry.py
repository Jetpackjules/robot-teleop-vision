from __future__ import annotations

from importlib import import_module
from importlib.metadata import entry_points
from typing import Any, Callable


Factory = Callable[..., Any]
_BUILTINS: dict[str, dict[str, Factory]] = {"camera": {}, "robot": {}}


def _load_builtins(kind: str, *, module_paths: tuple[str, ...] = ()) -> None:
    if kind not in _BUILTINS:
        raise KeyError(f"unknown adapter kind: {kind}")
    import_module(f"robot_teleop.{kind}s")
    if kind == "robot":
        from robot_teleop.modules import load_robot_modules

        load_robot_modules(module_paths)


def register(kind: str, name: str, factory: Factory) -> None:
    if kind not in _BUILTINS:
        raise KeyError(f"unknown adapter kind: {kind}")
    _BUILTINS[kind][name] = factory


def create(kind: str, name: str, **kwargs: Any) -> Any:
    config = kwargs.get("config")
    module_paths = tuple(getattr(config, "module_paths", ())) if kind == "robot" else ()
    _load_builtins(kind, module_paths=module_paths)
    factory = _BUILTINS.get(kind, {}).get(name)
    if factory is not None:
        return factory(**kwargs)
    group = f"robot_teleop.{kind}s"
    for entry in entry_points().select(group=group, name=name):
        return entry.load()(**kwargs)
    available = ", ".join(sorted(_BUILTINS.get(kind, {}))) or "none"
    raise KeyError(f"unknown {kind} adapter '{name}' (built-ins: {available})")


def names(kind: str) -> tuple[str, ...]:
    _load_builtins(kind)
    external = [entry.name for entry in entry_points().select(group=f"robot_teleop.{kind}s")]
    return tuple(sorted({*_BUILTINS.get(kind, {}), *external}))
