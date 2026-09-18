from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "diagnose_so101_motors.py"
SPEC = importlib.util.spec_from_file_location("motor_diagnostics", SCRIPT)
diagnostics = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diagnostics)


class ReadOnlyBus:
    def __init__(self, fail_connect=False):
        self.calls = []
        self.fail_connect = fail_connect
        self.port_handler = SimpleNamespace(closePort=lambda: self.calls.append("close"))

    def connect(self, *, handshake):
        assert handshake is False
        self.calls.append("connect")
        if self.fail_connect:
            raise OSError("port busy")

    def read(self, register, name, *, normalize, num_retry):
        assert normalize is False and num_retry == 0
        self.calls.append((register, name))
        if register == "Status" and name == "elbow_flex":
            raise RuntimeError("Overload error")
        return 123

    def __getattr__(self, name):
        raise AssertionError(f"unexpected motor operation: {name}")


def test_diagnostic_reads_only_and_retains_other_values_after_motor_fault():
    bus = ReadOnlyBus()
    rows = diagnostics.collect_registers(bus, ["shoulder_pan", "shoulder_lift", "elbow_flex"])
    assert [row["id"] for row in rows] == [1, 2, 3]
    assert rows[2]["errors"] == {"Status": "Overload error"}
    assert rows[2]["registers"]["Goal_Position"] == 123
    assert bus.calls[0] == "connect" and bus.calls[-1] == "close"
    assert len(bus.calls) == len(diagnostics.REGISTERS) * 3 + 2


def test_diagnostic_closes_port_after_partial_connection_failure():
    bus = ReadOnlyBus(fail_connect=True)
    with pytest.raises(OSError, match="port busy"):
        diagnostics.collect_registers(bus, ["shoulder_pan"])
    assert bus.calls == ["connect", "close"]


def test_cleanup_error_does_not_hide_original_connection_failure():
    bus = ReadOnlyBus(fail_connect=True)

    def broken_close():
        raise AttributeError("NoneType object has no attribute close")

    bus.port_handler.closePort = broken_close
    with pytest.raises(OSError, match="port busy"):
        diagnostics.collect_registers(bus, ["shoulder_pan"])
