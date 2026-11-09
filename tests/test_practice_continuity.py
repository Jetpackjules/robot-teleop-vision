import time

import pytest
from test_so101_arm import profile, FakeFollowerBus, FollowerController
from test_trial_protocol import host


def test_host_does_not_attach_arm_hold_to_recording_deadlines(tmp_path):
    with host(tmp_path) as server:
        assert server.episode_recorder.on_time_limit is None


@pytest.mark.parametrize('moved', [False, True])
def test_explicit_leader_enable_resumes_at_held_target(moved):
    pair = profile()
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw([0.] * 6))
    controller = FollowerController(pair, bus, now=lambda: 10.)
    controller.connect()

    def send_leader(seq, first_joint):
        positions = pair.leader_calibration.normalized_to_raw([first_joint] + [20.] * 5)
        controller.receive({'type': 'arm_command', 'seq': seq,
                            'sent_unix_ms': int(time.time() * 1000), 'positions': positions})

    send_leader(1, 20.)
    controller.receive({'type': 'arm_enable', 'source': 'leader'})
    target = list(controller.applied_normalized)
    controller.hold('operator hold')
    bus.positions = [p + 3 for p in bus.positions]  # Simulate measured sag.
    held_positions = list(bus.positions)
    current_leader = 35. if moved else 20.
    send_leader(2, current_leader)
    assert controller.state == 'hold'
    controller.receive({'type': 'arm_enable', 'source': 'leader'})
    assert controller.state == 'armed'
    assert controller.leader_enable_normalized == controller.leader_normalized
    assert controller.follower_enable_normalized == controller.target_normalized == controller.applied_normalized == target
    assert bus.positions == held_positions  # Enable itself makes no motor write.
    send_leader(3, current_leader)
    assert controller.target_normalized == pytest.approx(target)
    send_leader(4, current_leader + 3.)
    assert controller.target_normalized[0] - target[0] == pytest.approx(
        3. * pair.leader_joint_directions[0], abs=.15)
