"""Real local UDP wiring, with the service's explicitly fake motor bus."""

import json
import socket
import subprocess
import sys
import time
from contextlib import ExitStack
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_running_service_reads_encoders_and_stops_stalled_calibration_during_travel():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(8)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            command_port = reservation.getsockname()[1]
        # Explicitly construct a fake bus; this process cannot open a serial port.
        program = '''
import sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd() / "robot_modules/so101/tools"))
import so101_follower_service as s
from so101_arm_common import ArmPairProfile
p = ArmPairProfile.load(Path("tests/fixtures/so101_arm_pair.json"))
class Stalled(s.FakeFollowerBus):
    def write_positions(self, positions):
        pass
def plan(initial):
    target = list(initial)
    target[0] += 30
    return [(target, 2.0, "moving test joint"), (target, 2.0, "sampling test joint")]
s.build_calibration_sweep_waypoints = plan
bus = Stalled(p.follower_calibration.normalized_to_raw([0, 130, 38, 78, -8, 40]))
s.run_service(p, bus, int(sys.argv[1]), int(sys.argv[2]), 0, 0, 30, 20)
'''
        process = subprocess.Popen(
            [sys.executable, "-c", program, str(command_port), str(receiver.getsockname()[1])],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        try:
            assert json.loads(receiver.recv(16384))["follower_connected"]
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(json.dumps({
                    "type": "arm_calibration_sweep", "calibration_request_id": "stalled-travel",
                }).encode(), ("127.0.0.1", command_port))
            deadline = time.monotonic() + 6
            saw_fresh_moving_feedback = False
            stopped = None
            while time.monotonic() < deadline:
                packet = json.loads(receiver.recv(16384))
                if packet["calibration_request_id"] != "stalled-travel":
                    continue
                if packet["calibration_sweep_progress"] > 0.1 and packet["state"] == "calibrating":
                    assert packet["last_read_age_ms"] < 150
                    assert not packet["calibration_pose_settled"]
                    saw_fresh_moving_feedback = True
                if packet["calibration_request_state"] == "cancelled":
                    stopped = packet
                    break
            assert saw_fresh_moving_feedback
            assert stopped and stopped["following_error_stop"]
            assert stopped["applied_normalized"] == stopped["follower_normalized"]
        finally:
            process.terminate()
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=5)


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
