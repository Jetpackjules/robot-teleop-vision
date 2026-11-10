"""Bounded stereo matching in holes adjacent to measured preview geometry."""

from dataclasses import dataclass

import cv2
import numpy as np


CAMERA_SERIAL = "262522075787"
FRAME_SHAPE = (480, 848)
DISPARITIES = 16
GAP_RADIUS = 13
TILE_W = 48
TILE_H = 112
TILE_STEP = 16
MIN_SUPPORT = 12


@dataclass(frozen=True)
class Recovery:
    rows: np.ndarray
    columns: np.ndarray
    millimetres: np.ndarray
    tiles: int = 0


_EMPTY_INDEX = np.empty(0, dtype=np.intp)
_EMPTY_DEPTH = np.empty(0, dtype=np.uint16)
_EMPTY_RECOVERY = Recovery(_EMPTY_INDEX, _EMPTY_INDEX, _EMPTY_DEPTH)


class ScopedIrRecovery:
    """One instance per camera worker. Match new points in the current pair only."""

    def __init__(self):
        block = 5
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=DISPARITIES,
            blockSize=block,
            P1=8 * block**2,
            P2=32 * block**2,
            disp12MaxDiff=1,
            uniquenessRatio=14,
            speckleWindowSize=12,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.footprint = np.ones((2 * GAP_RADIUS + 1,) * 2, np.uint8)

    def recover(self, depth, support, left, right, disparity_scale):
        if (depth.shape != FRAME_SHAPE or depth.dtype != np.uint16
                or support.shape != depth.shape or support.dtype != np.uint16
                or any(image.shape != depth.shape or image.dtype != np.uint8 for image in (left, right))
                or not np.isfinite(disparity_scale) or not 5000 <= disparity_scale <= 100000):
            return _EMPTY_RECOVERY

        seed_mask = cv2.inRange(support, 160, 1600)
        x, y, width, height = cv2.boundingRect(seed_mask)
        if width == 0 or height == 0:
            return _EMPTY_RECOVERY
        x0, y0 = max(0, x-GAP_RADIUS), max(0, y-GAP_RADIUS)
        x1 = min(depth.shape[1], x+width+GAP_RADIUS)
        y1 = min(depth.shape[0], y+height+GAP_RADIUS)
        seeds = seed_mask[y0:y1, x0:x1] // 255
        density = cv2.boxFilter(
            seeds, cv2.CV_16U, self.footprint.shape, normalize=False, borderType=cv2.BORDER_CONSTANT,
        )
        candidates = (depth[y0:y1, x0:x1] == 0) & (density >= MIN_SUPPORT)
        support_roi = support[y0:y1, x0:x1]
        # Prioritize well-supported gaps over the thin fringe around an
        # isolated return. Keep selection spatially deterministic frame to frame.
        weight = np.where(candidates, np.minimum(density, 96), 0).astype(np.uint8)
        rows, columns, depths = [], [], []
        tiles = 0
        for u, v, w, h in self._windows(weight):
            local = support_roi[v:v+h, u:u+w]
            anchors = local[seeds[v:v+h, u:u+w] != 0]
            if anchors.size < MIN_SUPPORT:
                continue
            shift = max(0, int(disparity_scale / np.median(anchors)) - DISPARITIES // 2)
            # Only compute the depth envelope around the two selected tiles.
            # Padding by the filter radius gives each tile the original neighbourhood.
            sx, sy = max(0, u-GAP_RADIUS), max(0, v-GAP_RADIUS)
            ex, ey = min(seeds.shape[1], u+w+GAP_RADIUS), min(seeds.shape[0], v+h+GAP_RADIUS)
            local_seeds = seeds[sy:ey, sx:ex]
            local_depth = support_roi[sy:ey, sx:ex]
            region = np.s_[v-sy:v-sy+h, u-sx:u-sx+w]
            far = cv2.dilate(np.where(local_seeds, local_depth, 0), self.footprint)[region]
            near = cv2.erode(np.where(local_seeds, local_depth, np.uint16(65535)), self.footprint)[region]
            found = self._match(left, right, x0+u, y0+v, w, h, shift, disparity_scale,
                                candidates[v:v+h, u:u+w], near, far)
            if found is None:
                continue
            tiles += 1
            row, col, values = found
            if row.size:
                candidates[v+row, u+col] = False
                rows.append(y0+v+row)
                columns.append(x0+u+col)
                depths.append(values)
        if not depths:
            return Recovery(_EMPTY_INDEX, _EMPTY_INDEX, _EMPTY_DEPTH, tiles)
        return Recovery(np.concatenate(rows), np.concatenate(columns), np.concatenate(depths), tiles)

    @staticmethod
    def _windows(weight):
        height, width = weight.shape
        w, h = min(TILE_W, width), min(TILE_H, height)
        if w < 8 or h < 8:
            return
        xs = np.unique(np.r_[np.arange(0, width-w+1, TILE_STEP), width-w])
        ys = np.unique(np.r_[np.arange(0, height-h+1, TILE_STEP), height-h])
        x, y = np.meshgrid(xs, ys)
        summed = cv2.integral(weight, sdepth=cv2.CV_32S)
        scores = summed[y+h, x+w] + summed[y, x] - summed[y+h, x] - summed[y, x+w]
        # Reserve one search for each half of the measured region. Otherwise a
        # single large dropout can spend the entire budget on one object.
        upper = 2 * (y + h//2) < height
        for region in (upper, ~upper):
            eligible = np.where(region, scores, 0)
            index = np.unravel_index(eligible.argmax(), eligible.shape)
            if eligible[index] >= 360:
                yield int(x[index]), int(y[index]), w, h

    def _match(self, left, right, x, y, width, height, shift, scale, candidates, near, far):
        # Offset the right crop by the coarse disparity. SGBM then searches
        # just the remaining disparities in either direction.
        margin = DISPARITIES + 4
        lx = max(shift, x-margin)
        rx = min(left.shape[1], x+width+margin)
        ty, by = max(0, y-6), min(left.shape[0], y+height+6)
        if rx-lx < DISPARITIES+8 or by-ty < 8:
            return None
        a = np.ascontiguousarray(left[ty:by, lx:rx])
        b = np.ascontiguousarray(right[ty:by, lx-shift:rx-shift])
        forward = self.matcher.compute(a, b)
        backward = self.matcher.compute(
            np.ascontiguousarray(b[:, ::-1]), np.ascontiguousarray(a[:, ::-1]),
        )[:, ::-1]
        row, col = np.nonzero(candidates)
        u, v = col+x-lx, row+y-ty
        inside = (u >= DISPARITIES) & (u < rx-lx)
        row, col, u, v = row[inside], col[inside], u[inside], v[inside]
        d = forward[v, u].astype(np.int32)
        matched = np.clip(u - ((d + 8)//16), 0, rx-lx-1)
        reverse = backward[v, matched].astype(np.int32)
        disparity = shift + d/16.0
        accepted = (d >= 0) & (reverse >= 0) & (np.abs(d-reverse) <= 20) & (disparity > 0)
        row, col, disparity = row[accepted], col[accepted], disparity[accepted]
        distance = np.rint(scale / disparity)
        valid = (distance >= near[row, col].astype(np.int32)-28) & (distance <= far[row, col]+28)
        return row[valid], col[valid], distance[valid].astype(np.uint16)
