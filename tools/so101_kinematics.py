#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


JOINT_ORIGINS = (
    ((0.0388353, -8.97657e-09, 0.0624), (math.pi, 0.0, -math.pi)),
    ((-0.0303992, -0.0182778, -0.0542), (-math.pi / 2.0, -math.pi / 2.0, 0.0)),
    ((-0.11257, -0.028, 0.0), (0.0, 0.0, math.pi / 2.0)),
    ((-0.1349, 0.0052, 0.0), (0.0, 0.0, -math.pi / 2.0)),
    ((0.0, -0.0611, 0.0181), (math.pi / 2.0, 0.0486795, math.pi)),
)
TOOL_ORIGIN = ((-0.0079, -0.000218121, -0.0981274), (0.0, math.pi, 0.0))
JOINT_LIMITS_RAD = (
    (-1.91986, 1.91986),
    (-1.74533, 1.74533),
    (-1.69, 1.69),
    (-math.pi, math.pi),
    (-2.74385, 2.84121),
)
JOINT_DIRECTIONS = (1.0, -1.0, 1.0, 1.0, 1.0)
# The follower encoder reads -40.4296875 degrees when the shoulder pan is
# physically straight ahead. Convert that calibrated servo value into URDF
# joint zero before doing IK or rendering the chain.
BASE_STRAIGHT_NORMALIZED_DEG = -40.4296875
BASE_JOINT_OFFSET_DEG = -BASE_STRAIGHT_NORMALIZED_DEG
JOINT_OFFSETS_DEG = (BASE_JOINT_OFFSET_DEG, 80.0, 0.0, -70.0, 0.0)

# Rendered geometry follows the same normalized base direction as the follower.
OVERLAY_JOINT_DIRECTIONS = (1.0, -1.0, 1.0, 1.0, 1.0)
OVERLAY_JOINT_OFFSETS_DEG = (BASE_JOINT_OFFSET_DEG, 80.0, 0.0, -70.0, 0.0)
GRIPPER_ORIGIN = ((0.0202, 0.0188, -0.0234), (math.pi / 2.0, -0.000000052, 0.0))
# The visual mesh tips meet at -20 degrees. The source URDF's nominal
# -10-degree stop leaves an approximately 10 mm rendered gap.
GRIPPER_LIMITS_RAD = (-0.349066, 1.74533)
GRIPPER_CLOSED_NORMALIZED = 2.5

# Link-local bounds extracted from the imported GLBs by
# tools/dump_so101_link_bounds.gd. They let the safety planner check the whole
# rendered arm, including the fixed jaw and moving jaw, rather than only the
# tool origin.
LINK_BOUNDS = {
    "shoulder_link": ((-0.05059921, -0.02637770, -0.06440005), (0.01220080, 0.02870289, 0.04620009)),
    "upper_arm_link": ((-0.13016999, -0.03820004, -0.01350000), (0.01200000, 0.01200009, 0.05380006)),
    "lower_arm_link": ((-0.14510003, -0.01500008, -0.01200012), (0.01200001, 0.02190007, 0.05239995)),
    "wrist_link": ((-0.02000027, -0.06582844, -0.00900007), (0.01566000, 0.01199776, 0.05330006)),
    "gripper_link": ((-0.03520000, -0.02800000, -0.10442537), (0.03040000, 0.02400028, 0.00100006)),
    "moving_jaw_so101_v1_link": ((-0.01230000, -0.08200000, -0.00510000), (0.00999798, 0.00999466, 0.04290000)),
}


@dataclass(frozen=True)
class ArmPose:
    position: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]


def _rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rz = np.array(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return rz @ ry @ rx


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def _transform(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = _rpy_matrix(rpy)
    result[:3, 3] = xyz
    return result


def _canonical_radians(value_degrees: float, limits: tuple[float, float]) -> float:
    value = math.radians(value_degrees)
    center = (limits[0] + limits[1]) * 0.5
    value += round((center - value) / (2.0 * math.pi)) * 2.0 * math.pi
    return min(limits[1], max(limits[0], value))


def _preserve_turns(
    value_radians: float,
    previous_degrees: float,
    offset_degrees: float = 0.0,
    direction: float = 1.0,
) -> float:
    # Convert the URDF-space angle back to the calibrated servo-space angle
    # before choosing the equivalent turn nearest the previous command.
    base = (math.degrees(value_radians) - offset_degrees) / direction
    return base + round((previous_degrees - base) / 360.0) * 360.0


def normalized_to_joint_radians(normalized: list[float] | tuple[float, ...]) -> np.ndarray:
    return np.array([
        _canonical_radians(
            float(normalized[index]) * JOINT_DIRECTIONS[index] + JOINT_OFFSETS_DEG[index],
            JOINT_LIMITS_RAD[index],
        )
        for index in range(5)
    ])


def overlay_joint_radians(normalized: list[float] | tuple[float, ...]) -> np.ndarray:
    if len(normalized) < 6:
        raise ValueError("SO-101 rendered geometry requires six joint values")
    arm = [
        _canonical_radians(
            float(normalized[index]) * OVERLAY_JOINT_DIRECTIONS[index] + OVERLAY_JOINT_OFFSETS_DEG[index],
            JOINT_LIMITS_RAD[index],
        )
        for index in range(5)
    ]
    open_fraction = min(1.0, max(0.0, (float(normalized[5]) - GRIPPER_CLOSED_NORMALIZED) / (100.0 - GRIPPER_CLOSED_NORMALIZED)))
    arm.append(GRIPPER_LIMITS_RAD[0] + (GRIPPER_LIMITS_RAD[1] - GRIPPER_LIMITS_RAD[0]) * open_fraction)
    return np.asarray(arm, dtype=float)


def rendered_link_transforms(normalized: list[float] | tuple[float, ...]) -> dict[str, np.ndarray]:
    joints = overlay_joint_radians(normalized)
    transforms: dict[str, np.ndarray] = {}
    chain = np.eye(4)
    names = ("shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link")
    for index, (xyz, rpy) in enumerate(JOINT_ORIGINS):
        chain = chain @ _transform(xyz, rpy)
        rotation = np.eye(4)
        rotation[:3, :3] = _rz(float(joints[index]))
        chain = chain @ rotation
        transforms[names[index]] = chain.copy()
    chain = chain @ _transform(*GRIPPER_ORIGIN)
    jaw_rotation = np.eye(4)
    jaw_rotation[:3, :3] = _rz(float(joints[5]))
    transforms["moving_jaw_so101_v1_link"] = chain @ jaw_rotation
    return transforms


def rendered_geometry_points(normalized: list[float] | tuple[float, ...]) -> np.ndarray:
    transforms = rendered_link_transforms(normalized)
    points: list[np.ndarray] = []
    for link_name, (minimum, maximum) in LINK_BOUNDS.items():
        transform = transforms[link_name]
        for x in (minimum[0], maximum[0]):
            for y in (minimum[1], maximum[1]):
                for z in (minimum[2], maximum[2]):
                    points.append((transform @ np.array((x, y, z, 1.0)))[:3])
    return np.asarray(points, dtype=float)


def rendered_minimum_height(normalized: list[float] | tuple[float, ...]) -> float:
    """Lowest rendered moving-link vertex above the robot base plane (ROS +Z)."""
    return float(np.min(rendered_geometry_points(normalized)[:, 2]))


def forward_kinematics(joints_rad: np.ndarray) -> tuple[np.ndarray, list[np.ndarray], list[np.ndarray]]:
    transform = np.eye(4)
    origins: list[np.ndarray] = []
    axes: list[np.ndarray] = []
    for index, (xyz, rpy) in enumerate(JOINT_ORIGINS):
        transform = transform @ _transform(xyz, rpy)
        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, :3] @ np.array((0.0, 0.0, 1.0)))
        rotation = np.eye(4)
        rotation[:3, :3] = _rz(float(joints_rad[index]))
        transform = transform @ rotation
    transform = transform @ _transform(*TOOL_ORIGIN)
    return transform, origins, axes


def arm_pose(normalized: list[float] | tuple[float, ...]) -> ArmPose:
    transform, _, _ = forward_kinematics(normalized_to_joint_radians(normalized))
    return ArmPose(
        tuple(float(v) for v in transform[:3, 3]),
        tuple(tuple(float(v) for v in row) for row in transform[:3, :3]),
    )


def wrist_frame_twist(
    normalized: list[float] | tuple[float, ...],
    linear_velocity: list[float] | tuple[float, ...],
    angular_velocity: list[float] | tuple[float, ...],
) -> tuple[list[float], list[float]]:
    """Rotate a wrist/tool-local twist into the robot base frame.

    Local axes use camera-style controls: +X right, +Y up, +Z forward.
    The wrist camera mount is treated as aligned with the calibrated tool frame.
    """
    if len(linear_velocity) != 3 or len(angular_velocity) != 3:
        raise ValueError("wrist-frame velocity requires two three-value vectors")
    transform, _, _ = forward_kinematics(normalized_to_joint_radians(normalized))
    rotation = transform[:3, :3]
    linear = rotation @ np.asarray(linear_velocity, dtype=float)
    angular = rotation @ np.asarray(angular_velocity, dtype=float)
    return ([float(value) for value in linear], [float(value) for value in angular])


def resolved_rate_step(
    normalized: list[float],
    linear_velocity: list[float] | tuple[float, ...],
    angular_velocity: list[float] | tuple[float, ...],
    dt: float,
    *,
    damping: float = 0.09,
    target_rotation: list[list[float]] | tuple[tuple[float, ...], ...] | None = None,
    orientation_gain: float = 2.5,
) -> list[float]:
    """Advance the five arm joints with damped resolved-rate pose IK.

    When ``target_rotation`` is supplied, the rotational part becomes a
    closed-loop absolute tool-orientation hold. This counters the curved tool
    path and wrist drift that otherwise occur during Cartesian translation.
    """
    previous = [float(value) for value in normalized]
    joints = normalized_to_joint_radians(previous)
    end, origins, axes = forward_kinematics(joints)
    end_position = end[:3, 3]
    jacobian = np.zeros((6, 5), dtype=float)
    for index, (origin, axis) in enumerate(zip(origins, axes, strict=True)):
        jacobian[:3, index] = np.cross(axis, end_position - origin)
        jacobian[3:, index] = axis

    angular = np.asarray(angular_velocity, dtype=float)
    if target_rotation is not None:
        desired = np.asarray(target_rotation, dtype=float).reshape((3, 3))
        # Base-frame small-angle error from current orientation to desired.
        error_matrix = desired @ end[:3, :3].T
        error = 0.5 * np.array((
            error_matrix[2, 1] - error_matrix[1, 2],
            error_matrix[0, 2] - error_matrix[2, 0],
            error_matrix[1, 0] - error_matrix[0, 1],
        ))
        angular = angular + np.clip(error * float(orientation_gain), -1.2, 1.2)
    linear = np.asarray(linear_velocity, dtype=float)
    position_jacobian = jacobian[:3, :]
    rotation_jacobian = jacobian[3:, :]
    # The SO-101 has five arm joints, so a stacked 6D least-squares solve can
    # rotate a requested lateral translation until it resembles forward motion.
    # Solve translation as the primary task, then spend only its null space on
    # wrist orientation. WASD therefore remains an orthogonal Cartesian plane.
    position_inverse = position_jacobian.T @ np.linalg.inv(
        position_jacobian @ position_jacobian.T + np.eye(3) * (damping * damping)
    )
    delta_velocity = position_inverse @ linear
    null_space = np.eye(5) - position_inverse @ position_jacobian
    null_rotation = rotation_jacobian @ null_space
    remaining_angular = angular - rotation_jacobian @ delta_velocity
    rotation_inverse = null_rotation.T @ np.linalg.inv(
        null_rotation @ null_rotation.T + np.eye(3) * (damping * damping)
    )
    orientation_weight = 0.42 if target_rotation is not None else 0.35
    delta_velocity += orientation_weight * (null_space @ rotation_inverse @ remaining_angular)
    delta = delta_velocity
    delta *= max(0.0, min(float(dt), 0.075))
    max_delta = math.radians(3.0)
    largest = float(np.max(np.abs(delta)))
    if largest > max_delta:
        delta *= max_delta / largest
    next_joints = joints + delta
    for index, limits in enumerate(JOINT_LIMITS_RAD):
        next_joints[index] = min(limits[1], max(limits[0], next_joints[index]))
    result = [
        _preserve_turns(
            float(next_joints[index]),
            previous[index],
            JOINT_OFFSETS_DEG[index],
            JOINT_DIRECTIONS[index],
        )
        for index in range(5)
    ]
    result.append(previous[5])
    return result


def solve_pose_target(
    normalized: list[float],
    target_position: list[float] | tuple[float, ...],
    target_rotation: list[list[float]] | tuple[tuple[float, ...], ...],
    *,
    iterations: int = 6,
    damping: float = 0.045,
    max_iteration_degrees: float = 2.0,
    max_total_degrees: float = 4.0,
) -> list[float]:
    """Solve a persistent Cartesian target without changing IK branches.

    Translation is the primary task. Tool orientation uses only the remaining
    null space, while every iteration starts from the previous joint target.
    Small bounded iterations and a final whole-solve step limit make the
    solution continuous near singularities instead of turning Jacobian noise
    directly into motor motion.
    """
    if len(normalized) != 6 or len(target_position) != 3:
        raise ValueError("SO-101 pose-target IK requires six joints and a 3D target")
    previous = [float(value) for value in normalized]
    joints = normalized_to_joint_radians(previous)
    desired_position = np.asarray(target_position, dtype=float)
    desired_rotation = np.asarray(target_rotation, dtype=float).reshape((3, 3))
    iteration_limit = math.radians(max(0.1, float(max_iteration_degrees)))

    for _ in range(max(1, int(iterations))):
        end, origins, axes = forward_kinematics(joints)
        position_error = desired_position - end[:3, 3]
        position_length = float(np.linalg.norm(position_error))
        if position_length > 0.025:
            position_error *= 0.025 / position_length

        jacobian = np.zeros((6, 5), dtype=float)
        for index, (origin, axis) in enumerate(zip(origins, axes, strict=True)):
            jacobian[:3, index] = np.cross(axis, end[:3, 3] - origin)
            jacobian[3:, index] = axis
        position_jacobian = jacobian[:3, :]
        rotation_jacobian = jacobian[3:, :]
        position_inverse = position_jacobian.T @ np.linalg.inv(
            position_jacobian @ position_jacobian.T + np.eye(3) * (damping * damping)
        )
        delta = position_inverse @ position_error

        null_space = np.eye(5) - position_inverse @ position_jacobian
        error_matrix = desired_rotation @ end[:3, :3].T
        rotation_error = 0.5 * np.array((
            error_matrix[2, 1] - error_matrix[1, 2],
            error_matrix[0, 2] - error_matrix[2, 0],
            error_matrix[1, 0] - error_matrix[0, 1],
        ))
        null_rotation = rotation_jacobian @ null_space
        rotation_inverse = null_rotation.T @ np.linalg.inv(
            null_rotation @ null_rotation.T + np.eye(3) * (damping * damping)
        )
        delta += 0.32 * (null_space @ rotation_inverse @ rotation_error)

        largest = float(np.max(np.abs(delta)))
        if largest > iteration_limit:
            delta *= iteration_limit / largest
        joints += delta
        for index, limits in enumerate(JOINT_LIMITS_RAD):
            joints[index] = min(limits[1], max(limits[0], joints[index]))
        if position_length < 0.00025 and float(np.linalg.norm(rotation_error)) < math.radians(0.5):
            break

    result = [
        _preserve_turns(
            float(joints[index]),
            previous[index],
            JOINT_OFFSETS_DEG[index],
            JOINT_DIRECTIONS[index],
        )
        for index in range(5)
    ]
    result.append(previous[5])
    largest_total = max(abs(result[index] - previous[index]) for index in range(5))
    total_limit = max(0.1, float(max_total_degrees))
    if largest_total > total_limit:
        scale = total_limit / largest_total
        result = [
            previous[index] + (result[index] - previous[index]) * scale
            for index in range(5)
        ] + [previous[5]]
    return result


def joint_rate_step(
    normalized: list[float],
    velocities: list[float] | tuple[float, ...],
    dt: float,
    *,
    rotational_speed_degrees: float = 32.0,
    gripper_speed: float = 45.0,
) -> list[float]:
    """Advance each calibrated joint independently while enforcing URDF limits."""
    if len(normalized) != 6 or len(velocities) != 6:
        raise ValueError("SO-101 joint-rate control requires six values")
    previous = [float(value) for value in normalized]
    step_seconds = max(0.0, min(float(dt), 0.075))
    requested = [
        previous[index] + max(-1.0, min(1.0, float(velocities[index]))) * rotational_speed_degrees * step_seconds
        for index in range(5)
    ]
    clamped_joints = normalized_to_joint_radians(requested)
    result = [
        _preserve_turns(
            float(clamped_joints[index]),
            previous[index],
            JOINT_OFFSETS_DEG[index],
            JOINT_DIRECTIONS[index],
        )
        for index in range(5)
    ]
    result.append(
        max(
            0.0,
            min(100.0, previous[5] + max(-1.0, min(1.0, float(velocities[5]))) * gripper_speed * step_seconds),
        )
    )
    return result


def tool_rate_step(
    normalized: list[float],
    motions: list[float] | tuple[float, ...],
    dt: float,
    *,
    linear_speed_mps: float = 0.09,
    rotational_speed_degrees: float = 34.0,
    gripper_speed: float = 45.0,
    damping: float = 0.018,
) -> list[float]:
    """Move the tool with stable planar controls plus direct base and wrist axes.

    Motions are base yaw, radial reach, vertical lift, wrist pitch, wrist roll,
    and gripper. The two-link reach/lift solve deliberately excludes base and
    wrist joints so those controls stay independent and predictable.
    """
    if len(normalized) != 6 or len(motions) != 6:
        raise ValueError("SO-101 tool-rate control requires six values")
    previous = [float(value) for value in normalized]
    controls = np.clip(np.asarray(motions, dtype=float), -1.0, 1.0)
    step_seconds = max(0.0, min(float(dt), 0.075))
    joints = normalized_to_joint_radians(previous)
    next_joints = joints.copy()

    direct_step = math.radians(rotational_speed_degrees) * step_seconds
    next_joints[0] += controls[0] * direct_step * JOINT_DIRECTIONS[0]
    next_joints[3] += controls[3] * direct_step * JOINT_DIRECTIONS[3]
    next_joints[4] += controls[4] * direct_step * JOINT_DIRECTIONS[4]

    if step_seconds > 0.0 and (abs(controls[1]) > 1e-6 or abs(controls[2]) > 1e-6):
        end, origins, axes = forward_kinematics(joints)
        end_position = end[:3, 3]
        vertical = axes[0] / max(1e-9, float(np.linalg.norm(axes[0])))
        radial = end_position - origins[1]
        radial -= vertical * float(np.dot(radial, vertical))
        radial_norm = float(np.linalg.norm(radial))
        if radial_norm < 1e-6:
            radial = np.cross(axes[1], vertical)
            radial_norm = float(np.linalg.norm(radial))
        if radial_norm < 1e-6:
            radial = np.array((1.0, 0.0, 0.0), dtype=float)
            radial -= vertical * float(np.dot(radial, vertical))
            radial_norm = float(np.linalg.norm(radial))
        radial /= max(1e-9, radial_norm)

        columns = []
        for index in (1, 2):
            columns.append(np.cross(axes[index], end_position - origins[index]))
        jacobian = np.array(
            (
                (float(np.dot(radial, columns[0])), float(np.dot(radial, columns[1]))),
                (float(np.dot(vertical, columns[0])), float(np.dot(vertical, columns[1]))),
            ),
            dtype=float,
        )
        desired = controls[1:3] * linear_speed_mps * step_seconds
        normal = jacobian @ jacobian.T + np.eye(2) * (damping * damping)
        delta = jacobian.T @ np.linalg.solve(normal, desired)
        max_planar_delta = max(math.radians(0.35), min(math.radians(4.0), direct_step * 1.35))
        largest = float(np.max(np.abs(delta)))
        if largest > max_planar_delta:
            delta *= max_planar_delta / largest

        # At a folded or fully extended singularity, first-order IK can return
        # almost no radial motion. A tiny bounded nonlinear search picks a
        # deterministic bend direction and lets the arm move out of it.
        candidate_values = (-max_planar_delta, -0.5 * max_planar_delta, 0.0, 0.5 * max_planar_delta, max_planar_delta)
        candidates = [delta]
        candidates.extend(np.array((a, b), dtype=float) for a in candidate_values for b in candidate_values)
        best_delta = delta
        best_score = math.inf
        for candidate in candidates:
            trial = joints.copy()
            trial[1] += float(candidate[0])
            trial[2] += float(candidate[1])
            for index in (1, 2):
                trial[index] = min(JOINT_LIMITS_RAD[index][1], max(JOINT_LIMITS_RAD[index][0], trial[index]))
            trial_end, _, _ = forward_kinematics(trial)
            displacement = trial_end[:3, 3] - end_position
            projected = np.array((np.dot(displacement, radial), np.dot(displacement, vertical)))
            error = projected - desired
            score = float(np.dot(error, error)) + 1e-8 * float(np.dot(candidate, candidate))
            if score < best_score:
                best_score = score
                best_delta = candidate
        next_joints[1] += float(best_delta[0])
        next_joints[2] += float(best_delta[1])

    for index, limits in enumerate(JOINT_LIMITS_RAD):
        next_joints[index] = min(limits[1], max(limits[0], next_joints[index]))
    result = [
        _preserve_turns(
            float(next_joints[index]),
            previous[index],
            JOINT_OFFSETS_DEG[index],
            JOINT_DIRECTIONS[index],
        )
        for index in range(5)
    ]
    result.append(
        max(
            0.0,
            min(100.0, previous[5] + controls[5] * gripper_speed * step_seconds),
        )
    )
    return result
