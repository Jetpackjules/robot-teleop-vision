"""Real local UDP wiring, with the service's explicitly fake motor bus."""

import json
import socket
import subprocess
import sys
import time
from contextlib import ExitStack
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_fake_service_uses_all_profile_ports_without_hardware(tmp_path):
    with ExitStack() as sockets:
        receivers = []
        for _ in range(3):
            receiver = sockets.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(8)
            receivers.append(receiver)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            command_port = reservation.getsockname()[1]
        data = json.loads((ROOT / "tests/fixtures/so101_arm_pair.json").read_text())
        data.update(
            command_port=command_port,
            status_port=receivers[0].getsockname()[1],
            godot_status_port=receivers[1].getsockname()[1],
            editor_status_port=receivers[2].getsockname()[1],
        )
        # The nonexistent device is deliberate; --dry-run must not open it.
        data["follower"]["port"] = "NEVER_OPEN_PHYSICAL_HARDWARE_IN_THIS_TEST"
        profile = tmp_path / "fake_profile.json"
        profile.write_text(json.dumps(data), encoding="utf-8")
        process = subprocess.Popen(
            [sys.executable, str(ROOT / "robot_modules/so101/tools/so101_follower_service.py"),
             "--dry-run", "--profile", str(profile)],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            for receiver in receivers:
                packet = json.loads(receiver.recv(16384))
                assert packet["type"] == "arm_status"
                assert packet["follower_connected"] is True
                assert packet["calibration_request_state"] == "idle"
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(b'{"type":"arm_hold"}', ("127.0.0.1", command_port))
            deadline = time.monotonic() + 3
            observed_hold = False
            while time.monotonic() < deadline:
                packet = json.loads(receivers[2].recv(16384))
                if "operator hold" in packet.get("message", "").lower():
                    observed_hold = True
                    break
            assert observed_hold, "custom command port did not reach the fake follower"
        finally:
            process.terminate()
            try:
                output, _ = process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                output, _ = process.communicate(timeout=5)
            assert "dry-run mode" in output, output
