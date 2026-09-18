"""Rebuild exact support vertices for the SO-101 rest-return floor check."""

import hashlib
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))
from so101_kinematics import LINK_BOUNDS


def build() -> None:
    assets = ROOT / "robot_modules/so101/assets"
    payload = {}
    for name in ("base_link", *LINK_BOUNDS):
        path = assets / f"{name}.glb"
        model = o3d.io.read_triangle_model(str(path))
        vertices = np.concatenate([np.asarray(item.mesh.vertices) for item in model.meshes])
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(vertices))
        hull, _ = cloud.compute_convex_hull()
        # No simplification: the hull preserves the minimum height in every
        # orientation, including curved tips that a bounding box overestimates.
        payload[name] = np.asarray(hull.vertices, dtype=np.float64)
        payload[f"{name}_sha256"] = np.asarray(hashlib.sha256(path.read_bytes()).hexdigest())
    np.savez_compressed(assets / "rest_collision_hulls.npz", **payload)


if __name__ == "__main__":
    build()
