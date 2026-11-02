"""Claw/base-level motion constraint. Reads calibration; never changes it."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

from so101_kinematics import (
    GRIPPER_CLOSED_NORMALIZED, GRIPPER_LIMITS_RAD, GRIPPER_ORIGIN,
    JOINT_DIRECTIONS, JOINT_LIMITS_RAD, JOINT_OFFSETS_DEG, JOINT_ORIGINS,
    TOOL_ORIGIN, _rest_collision_hulls, _transform,
)


def _matrix(value):
    result = np.eye(4)
    for i, key in enumerate(("basis_x", "basis_y", "basis_z", "origin")):
        vector = np.asarray(value[key], dtype=float)
        if vector.shape != (3,) or not np.isfinite(vector).all():
            raise ValueError("Invalid floor geometry transform")
        result[:3, i] = vector
    return result


def _rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    result = np.eye(4)
    result[:3, :3] = np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)
    return result


class MeshFloorGeometry:
    CLAW_LINKS = ("gripper_link", "moving_jaw_so101_v1_link")

    def __init__(self, model=None, plane_height=None):
        self.hulls = _rest_collision_hulls()
        self.model = model
        self.normal = np.array([0., 1., 0.] if model else [0., 0., 1.])
        if model:
            json.dumps(model, allow_nan=False)
            if model.get("schema") != "so101_measured_fk_v1" or len(model["joints"]) != 6:
                raise ValueError("No calibrated claw geometry")
            self.base = _matrix(model["base_transform"])
            self.base_link = model["base_link"]
            self.joints = []
            for joint in model["joints"]:
                axis = np.asarray(joint["axis"], dtype=float)
                translation = np.asarray(joint["parent_translation"], dtype=float)
                if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-8:
                    raise ValueError("Invalid calibrated joint axis")
                if translation.shape != (3,) or not np.isfinite(translation).all():
                    raise ValueError("Invalid calibrated mount translation")
                self.joints.append({**joint, "origin": _matrix(joint["origin"]), "post": _matrix(joint["post"]),
                                    "axis": axis/np.linalg.norm(axis), "parent_translation": translation})
        else:
            self.base = np.eye(4)
            self.base_link = "base_link"
            names = ["shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"]
            self.joints = [{"parent": self.base_link if i == 0 else names[i-1], "child": name,
                            "origin": _transform(*(JOINT_ORIGINS[i] if i < 5 else GRIPPER_ORIGIN)),
                            "post": np.eye(4), "axis": np.array([0., 0., 1.]), "parent_translation": np.zeros(3),
                            "direction": JOINT_DIRECTIONS[i] if i < 5 else 1.,
                            "offset_degrees": JOINT_OFFSETS_DEG[i] if i < 5 else 0.}
                           for i, name in enumerate(names)]
        if not set(self.CLAW_LINKS).issubset({j["child"] for j in self.joints}):
            raise ValueError("Calibrated model omits collision links")
        # Only the base's own underside defines the reference. No table,
        # objects, shoulder, elbow or other arm geometry is an obstacle.
        base_heights = self.hulls["base_link"] @ (self.base[:3, :3].T @ self.normal)
        base_bottom = float(np.min(base_heights) + self.normal @ self.base[:3, 3])
        self.height = base_bottom if plane_height is None else float(plane_height)
        if not math.isfinite(self.height):
            raise ValueError("Invalid base underside plane")
        # Validate the complete descriptor before it can replace a working one.
        if model:
            g = model["gripper"]
            xs = g.get("samples_normalized", [])
            if any(b <= a for a, b in zip(xs, xs[1:])):
                raise ValueError("Invalid saved jaw curve")
            self.evaluate(np.zeros(6))

    def _jaw_angle(self, value):
        if not self.model:
            fraction = np.clip((value-GRIPPER_CLOSED_NORMALIZED)/(100-GRIPPER_CLOSED_NORMALIZED), 0, 1)
            return GRIPPER_LIMITS_RAD[0] + fraction*(GRIPPER_LIMITS_RAD[1]-GRIPPER_LIMITS_RAD[0])
        g = self.model["gripper"]
        xs = g.get("samples_normalized", [])
        ys = g.get("samples_degrees", [])
        if min(len(xs), len(ys)) < 2 and g["moving_jaw"]:
            ys = g.get("fallback_samples_degrees", [])
        if min(len(xs), len(ys)) >= 2:
            index = min(max(0, int(np.searchsorted(xs, value))-1), min(len(xs), len(ys))-2)
            if xs[index+1] <= xs[index]:
                raise ValueError("Invalid saved jaw curve")
            degrees = ys[index] + (ys[index+1]-ys[index])*(value-xs[index])/(xs[index+1]-xs[index])
        else:
            degrees = g["offset"] + g["scale"]*value + g["curvature"]*value*(100-value)/100
        angle = math.radians(degrees)
        return angle if g["moving_jaw"] else float(np.clip(angle, *self.joints[5]["limits"]))

    def transforms(self, q):
        world = {self.base_link: self.base}
        axes, pivots = [], []
        for i, j in enumerate(self.joints):
            angle = self._jaw_angle(q[i]) if i == 5 else math.radians(q[i]*j["direction"]+j["offset_degrees"])
            if i < 5:
                if j.get("force_zero"):
                    angle = 0.
                elif self.model and self.model.get("clamp_limits"):
                    angle = float(np.clip((angle+math.pi)%(2*math.pi)-math.pi, *j["limits"]))
                if i == 4:
                    angle += math.radians(j.get("extra_degrees", 0.))
            parent = world[j["parent"]]
            frame = parent @ j["origin"]
            pivot = frame[:3, 3].copy()
            if i in (3, 4):
                pivot += parent[:3, :3] @ j["parent_translation"]
            axes.append(frame[:3, :3] @ j["axis"])
            pivots.append(pivot)
            local = j["origin"] @ _rotation(j["axis"], angle)
            if i == 4:
                local[:3, 3] += j["parent_translation"]
            local = local @ j["post"]
            if i == 3:
                local[:3, 3] += j["parent_translation"]
            world[j["child"]] = parent @ local
        return world, axes, pivots

    def evaluate(self, q, derivatives=False):
        world, axes, pivots = self.transforms(q)
        best = (math.inf, None, None)
        for index, j in enumerate(self.joints):
            name = j["child"]
            if name not in self.CLAW_LINKS:
                continue
            transform = world[name]
            heights = self.hulls[name] @ (transform[:3, :3].T @ self.normal)
            vertex = int(np.argmin(heights))
            height = float(heights[vertex] + self.normal @ transform[:3, 3] - self.height)
            if height < best[0]:
                best = (height, index, transform[:3, :3] @ self.hulls[name][vertex] + transform[:3, 3])
        if not derivatives:
            return best[0]
        gradient, jacobian = np.zeros(5), np.zeros((3, 5))
        tool_transform = world["gripper_link"]
        tool = tool_transform[:3, :3] @ np.asarray(TOOL_ORIGIN[0]) + tool_transform[:3, 3]
        for i in range(5):
            factor = 0. if self.joints[i].get("force_zero") else math.radians(self.joints[i]["direction"])
            if self.model and self.model.get("clamp_limits"):
                raw = math.radians(q[i]*self.joints[i]["direction"]+self.joints[i]["offset_degrees"])
                wrapped = (raw+math.pi)%(2*math.pi)-math.pi
                lo, hi = self.joints[i]["limits"]
                if wrapped < lo or wrapped > hi:
                    factor = 0.
            jacobian[:, i] = np.cross(axes[i], tool-pivots[i])*factor
            if i <= best[1]:
                gradient[i] = self.normal @ np.cross(axes[i], best[2]-pivots[i])*factor
        tangent = (np.eye(3)-np.outer(self.normal, self.normal)) @ jacobian
        return best[0], gradient, tangent


class RuntimeFloorConstraint:
    MARGIN = 0.001
    TOLERANCE = 0.0005

    def __init__(self, state_dir=None, geometry=None):
        self.geometry = geometry or MeshFloorGeometry()
        self.state_dir = Path(state_dir) if state_dir else None
        self.source = "stock claw / base-model underside"
        self.warning = ""
        self.reference = None
        self._next_refresh = 0.
        self._signature = None

    def reset(self):
        self.reference = None

    def refresh(self):
        if self.state_dir is None or time.monotonic() < self._next_refresh:
            return
        self._next_refresh = time.monotonic()+1
        try:
            robot = json.loads((self.state_dir/"robot_overlay_render_state.json").read_text())["robot"]
            model = robot["kinematics"]
            signature = json.dumps(model, sort_keys=True)
            if signature != self._signature:
                geometry = MeshFloorGeometry(model)
                self.geometry = geometry
                self.reference = None
                self._signature = signature
            self.source = "live calibrated claw / base-model underside"
            self.warning = ""
        except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
            # Retain the last complete model through transient file writes.
            self.warning = str(exc)[:160]

    def limit(self, held, candidate, max_step, *, allow_slide=True):
        self.refresh()
        geometry = self.geometry
        held, candidate = np.asarray(held, dtype=float), np.asarray(candidate, dtype=float)
        candidate = candidate.copy()
        candidate[5] = np.clip(candidate[5], max(0., held[5]-max_step[5]), min(100., held[5]+max_step[5]))
        if self.reference is None:
            self.reference = min(self.MARGIN, geometry.evaluate(held))
        else:
            # Recovery may tighten a below-plane starting allowance, never
            # lower it. Numerical errors cannot accumulate into floor drift.
            self.reference = max(self.reference, min(self.MARGIN, geometry.evaluate(held)))
        minimum = self.reference-self.TOLERANCE
        low, high = [], []
        for i in range(5):
            bounds = sorted((math.degrees(v)-JOINT_OFFSETS_DEG[i])/JOINT_DIRECTIONS[i] for v in JOINT_LIMITS_RAD[i])
            low.append(max(min(bounds[0], held[i]), held[i]-max_step[i]))
            high.append(min(max(bounds[1], held[i]), held[i]+max_step[i]))
        bounded = lambda q: np.r_[np.clip(q[:5], low, high), candidate[5]]
        result = bounded(candidate)
        def worst(q):
            samples = [(geometry.evaluate(held+a*(q-held)), a) for a in (0.5, 1.)]
            return min(samples)
        height, alpha = worst(result)
        if height >= minimum-1e-9:
            return result.tolist(), False
        if allow_slide:
            for _ in range(4):
                height, gradient, tangent = geometry.evaluate(held+alpha*(result-held), True)
                gradient *= alpha
                metric = tangent.T @ tangent + np.eye(5)*1e-8
                direction = np.linalg.solve(metric, gradient)
                denominator = float(gradient @ direction)
                if denominator < 1e-16:
                    break
                result[:5] += direction*((minimum-height+1e-7)/denominator)
                result = bounded(result)
                height, alpha = worst(result)
                if height >= minimum-1e-9:
                    return result.tolist(), True
        # Pure normal input stops at the boundary. If a joint limit prevents
        # sliding, retain the feasible part without rebasing or accumulating sag.
        start, end = 0., 1.
        for _ in range(9):
            mid = (start+end)/2
            if worst(held+mid*(result-held))[0] >= minimum-1e-9:
                start = mid
            else:
                end = mid
        accepted = held+start*(result-held)
        # A blocked arm direction must not cancel an independently safe claw
        # command. Only clip the jaw itself if that opening crosses the plane.
        full_jaw = accepted.copy()
        full_jaw[5] = candidate[5]
        if worst(full_jaw)[0] >= minimum-1e-9:
            accepted = full_jaw
        return accepted.tolist(), True
