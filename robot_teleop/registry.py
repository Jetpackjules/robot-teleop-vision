from __future__ import annotations

from importlib import import_module
from importlib.metadata import entry_points
from typing import Any, Callable


Factory = Callable[..., Any]
_BUILTINS: dict[str, dict[str, Factory]] = {"camera": {}, "robot": {}, "tracking": {}}


def _load_builtins(kind: str) -> None:
    if kind not in _BUILTINS:
        raise KeyError(f"unknown adapter kind: {kind}")
    module = "tracking" if kind == "tracking" else f"{kind}s"
    import_module(f"robot_teleop.{module}")


def register(kind: str, name: str, factory: Factory) -> None:
    if kind not in _BUILTINS:
        raise KeyError(f"unknown adapter kind: {kind}")
    _BUILTINS[kind][name] = factory


def create(kind: str, name: str, **kwargs: Any) -> Any:
    _load_builtins(kind)
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
