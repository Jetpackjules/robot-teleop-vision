from pathlib import Path
import copy
import json
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))
from so101_floor_constraint import MeshFloorGeometry, RuntimeFloorConstraint


class FlatClaw:
    base = np.eye(4)
    normal = np.array([0., 0., 1.])

    def evaluate(self, q, derivatives=False):
        h = q[1]/1000
        if not derivatives:
            return h
        return h, np.array([0., .001, 0., 0., 0.]), np.array([[.001, 0., 0., 0., 0.], [0., 0., .001, 0., 0.]])


STEPS = [4., 4., 4., 4., 4., 4.]


def test_sideways_and_diagonal_motion_keep_tangent_while_downward_component_is_removed():
    floor = RuntimeFloorConstraint(geometry=FlatClaw())
    held = [0., 2., 0., 0., 0., 40.]
    accepted, clipped = floor.limit(held, [1., -2., 1., 0., 0., 41.], STEPS)
    assert clipped
    assert accepted[0] == pytest.approx(1.)
    assert accepted[2] == pytest.approx(1.)
    assert accepted[1] >= 0.5
    assert accepted[5] == 41.  # Safe claw motion survives a blocked arm step.


def test_straight_down_stops_without_creating_sideways_motion_and_up_escapes():
    floor = RuntimeFloorConstraint(geometry=FlatClaw())
    held = [0., 2., 0., 0., 0., 40.]
    accepted, clipped = floor.limit(held, [0., -2., 0., 0., 0., 40.], STEPS, allow_slide=False)
    assert clipped
    assert accepted[0] == 0. and accepted[2:5] == [0., 0., 0.]
    assert .5 <= accepted[1] < .51
    lifted = accepted.copy(); lifted[1] += 1
    up, clipped = floor.limit(accepted, lifted, STEPS, allow_slide=False)
    assert not clipped and up == lifted


def test_fixed_tolerance_cannot_ratchet_down_during_repeated_lateral_input():
    floor = RuntimeFloorConstraint(geometry=FlatClaw())
    held = [0., 2., 0., 0., 0., 40.]
    for i in range(500):
        candidate = held.copy()
        candidate[0] += .2 if i % 2 else -.2
        candidate[1] -= .1
        held, _ = floor.limit(held, candidate, STEPS)
        assert held[1] >= .5-1e-6
    assert floor.reference == .001


def test_below_plane_start_can_escape_and_does_not_reopen_deeper_allowance():
    floor = RuntimeFloorConstraint(geometry=FlatClaw())
    held = [0., -2., 0., 0., 0., 40.]
    q, _ = floor.limit(held, [0., -4., 0., 0., 0., 40.], STEPS, allow_slide=False)
    assert q[1] >= -2.5-1e-6
    q, _ = floor.limit(q, [0., 1.5, 0., 0., 0., 40.], STEPS, allow_slide=False)
    q, _ = floor.limit(q, [0., -2., 0., 0., 0., 40.], STEPS, allow_slide=False)
    assert q[1] >= .5-1e-6


def test_jaw_opening_is_clipped_only_if_the_jaw_itself_crosses_base_level():
    class Jaw(FlatClaw):
        def evaluate(self, q, derivatives=False):
            result = super().evaluate(q, derivatives)
            delta = max(0., q[5]-40)/1000
            return (result[0]-delta, *result[1:]) if derivatives else result-delta
    floor = RuntimeFloorConstraint(geometry=Jaw())
    held = [0., 2., 0., 0., 0., 40.]
    q, clipped = floor.limit(held, [0., 2., 0., 0., 0., 44.], STEPS, allow_slide=False)
    assert clipped and q[:5] == held[:5]
    assert 41.4 < q[5] <= 41.5


def test_exact_claw_support_and_calibrated_fk_match_saved_viewer_geometry():
    samples = json.loads((ROOT/'tests/fixtures/so101_measured_fk.json').read_text())
    for sample in samples:
        geometry = MeshFloorGeometry(sample['robot']['kinematics'])
        q = sample['normalized']
        world, _, _ = geometry.transforms(q)
        heights = []
        for name, expected in sample['robot']['links'].items():
            matrix = np.eye(4)
            for i, key in enumerate(('basis_x', 'basis_y', 'basis_z', 'origin')):
                matrix[:3, i] = expected[key]
            np.testing.assert_allclose(world[name], matrix, atol=3e-7)
            if name in geometry.CLAW_LINKS:
                heights.append(np.min(geometry.hulls[name] @ (matrix[:3, :3].T @ geometry.normal))
                               + geometry.normal @ matrix[:3, 3]-geometry.height)
        assert geometry.evaluate(q) == pytest.approx(min(heights), abs=3e-7)
        height, gradient, _ = geometry.evaluate(q, True)
        numeric = []
        for i in range(5):
            a, b = q.copy(), q.copy(); a[i] += .0001; b[i] -= .0001
            numeric.append((geometry.evaluate(a)-geometry.evaluate(b))/.0002)
        np.testing.assert_allclose(gradient, numeric, atol=2e-8)
        # An elbow/shoulder bounding box cannot create a claw/base alert.
        geometry.hulls = {**geometry.hulls, 'lower_arm_link': np.array([[0., 0., -1000.]])}
        assert geometry.evaluate(q) == height


def test_manual_calibration_updates_automatically_without_table_settings(tmp_path):
    sample = json.loads((ROOT/'tests/fixtures/so101_measured_fk.json').read_text())[0]
    robot = copy.deepcopy(sample['robot'])
    path = tmp_path/'robot_overlay_render_state.json'
    path.write_text(json.dumps({'robot': robot}))
    floor = RuntimeFloorConstraint(state_dir=tmp_path)
    floor.refresh()
    assert 'live calibrated claw' in floor.source
    old = floor.geometry
    robot['kinematics']['manual_active'] = True
    robot['kinematics']['joints'][2]['offset_degrees'] += 5
    robot['kinematics']['base_transform']['origin'][1] += .1
    path.write_text(json.dumps({'robot': robot}))
    floor._next_refresh = 0; floor.refresh()
    assert floor.geometry is not old
    assert floor.geometry.joints[2]['offset_degrees'] == old.joints[2]['offset_degrees']+5
    assert floor.geometry.height == pytest.approx(old.height+.1)
    current = floor.geometry
    path.write_text('{')
    floor._next_refresh = 0; floor.refresh()
    assert floor.geometry is current and floor.warning
