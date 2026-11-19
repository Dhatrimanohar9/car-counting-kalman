"""
road_tracker.py
===============
Stage 2 of the pipeline: give every vehicle one ID that stays with it.

Two pieces live here:

ConstantVelocityKalman
    A plain 4-state Kalman filter (x, y, vx, vy) written out with numpy so
    every line of the maths is visible:
        predict : x <- F x ,  P <- F P F' + Q
        correct : K = P H' (H P H' + R)^-1,  x <- x + K(z - H x)

VehicleTracker
    Hungarian (optimal) assignment of detections to predicted track positions,
    a distance gate so an ID never jumps to a neighbouring lane, and the track
    lifecycle tentative -> confirmed -> coasting -> deleted.  Track IDs come
    from one monotone counter, so an ID is never reused inside a run.
"""

from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

import settings as S
from motion_detector import Detection


# --------------------------------------------------------------------------- #
# Kalman filter
# --------------------------------------------------------------------------- #

class ConstantVelocityKalman:
    """Constant-velocity Kalman filter for a 2-D point.

    State   : [x, y, vx, vy]        (pixels, pixels per frame)
    Measure : [x, y]
    """

    def __init__(self, x0: float, y0: float) -> None:
        dt = float(S.KF_DT)

        # State transition: p <- p + v*dt
        self.F = np.array([[1.0, 0.0, dt, 0.0],
                           [0.0, 1.0, 0.0, dt],
                           [0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0]])
        # We only measure the position.
        self.H = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]])
        # Process noise: how much the constant-velocity assumption may break.
        self.Q = np.diag([S.KF_Q_POS, S.KF_Q_POS, S.KF_Q_VEL, S.KF_Q_VEL])
        # Measurement noise: centroid jitter of the detector.
        self.R = np.eye(2) * S.KF_R_MEAS
        # Initial uncertainty: position known, velocity unknown.
        self.P = np.diag([S.KF_P_POS, S.KF_P_POS, S.KF_P_VEL, S.KF_P_VEL])

        self.x = np.array([[x0], [y0], [0.0], [0.0]])
        self.I = np.eye(4)

    def predict(self) -> np.ndarray:
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:2, 0]

    def correct(self, zx: float, zy: float) -> np.ndarray:
        z = np.array([[zx], [zy]])
        innovation = z - self.H @ self.x
        innovation_cov = self.H @ self.P @ self.H.T + self.R
        gain = self.P @ self.H.T @ np.linalg.inv(innovation_cov)
        self.x = self.x + gain @ innovation
        self.P = (self.I - gain @ self.H) @ self.P
        return self.x[:2, 0]

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self.x[2, 0]), float(self.x[3, 0])


# --------------------------------------------------------------------------- #
# One track
# --------------------------------------------------------------------------- #

class Track:
    """Everything the system knows about one vehicle."""

    def __init__(self, detection: Detection, frame_index: int, track_id: int) -> None:
        self.id = track_id
        self.filter = ConstantVelocityKalman(*detection.centroid)

        self.bbox: Tuple[int, int, int, int] = detection.bbox
        self.centroid: Tuple[float, float] = detection.centroid
        # Position in the *previous* frame.  The counter compares the two, so
        # this must be updated on every frame, including coasted ones.
        self.previous_centroid: Tuple[float, float] = detection.centroid
        self.area: float = detection.area

        self.birth_frame = frame_index
        self.last_seen_frame = frame_index
        self.hits = 1          # frames in which this track was matched
        self.misses = 0        # consecutive frames without a detection
        self.confirmed = False

        self.first_centroid: Tuple[float, float] = detection.centroid
        # Filled in by the counter: label of the line part that counted it.
        self.counted_direction: Optional[str] = None
        self.counted_frame: Optional[int] = None

        self.trail: deque = deque(maxlen=S.HISTORY_LEN)
        self.trail.append(detection.centroid)

    # -- state changes -------------------------------------------------- #

    @property
    def age(self) -> int:
        return self.last_seen_frame - self.birth_frame + 1

    def predict(self) -> None:
        self.filter.predict()

    def match(self, detection: Detection, frame_index: int) -> None:
        """Fuse the detection with the prediction."""
        self.previous_centroid = self.centroid
        cx, cy = self.filter.correct(*detection.centroid)
        self.centroid = (float(cx), float(cy))
        self.bbox = detection.bbox
        self.area = detection.area
        self.trail.append(self.centroid)

        self.hits += 1
        self.misses = 0
        self.last_seen_frame = frame_index
        if self.hits >= S.MIN_HITS:
            self.confirmed = True

    def coast(self) -> None:
        """No detection this frame: keep the predicted position."""
        self.previous_centroid = self.centroid
        predicted = self.filter.x[:2, 0]
        self.centroid = (float(predicted[0]), float(predicted[1]))
        self.trail.append(self.centroid)
        self.misses += 1


# --------------------------------------------------------------------------- #
# The tracker
# --------------------------------------------------------------------------- #

def _iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1, y1 = max(ax, bx), max(ay, by)
    x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    inter = (x2 - x1) * (y2 - y1)
    union = aw * ah + bw * bh - inter
    return inter / float(union) if union > 0 else 0.0


class VehicleTracker:
    """Multi-object tracker with Hungarian association."""

    def __init__(self) -> None:
        self._tracks: Dict[int, Track] = {}
        self._next_id = 1
        self.records: Dict[int, dict] = {}

    # -- accessors ------------------------------------------------------ #

    @property
    def tracks(self) -> List[Track]:
        return list(self._tracks.values())

    @property
    def confirmed_tracks(self) -> List[Track]:
        return [t for t in self._tracks.values() if t.confirmed]

    # -- one frame ------------------------------------------------------ #

    def update(self, detections: List[Detection], frame_index: int) -> List[Track]:
        """Advance the tracker by one frame; return the confirmed tracks."""
        for track in self._tracks.values():
            track.predict()

        matches, unmatched_detections = self._associate(detections)

        for track_id, det_index in matches:
            self._tracks[track_id].match(detections[det_index], frame_index)

        # Tracks without a detection either coast or die.
        for track_id in list(self._tracks.keys()):
            track = self._tracks[track_id]
            if track.misses == 0 and track.last_seen_frame == frame_index:
                continue
            track.coast()
            if track.misses > S.MAX_LOST:
                self._close_record(track)
                del self._tracks[track_id]

        for det_index in unmatched_detections:
            detection = detections[det_index]
            if self._is_duplicate(detection):
                continue
            self._create_track(detection, frame_index)

        return self.confirmed_tracks

    # -- internals ------------------------------------------------------ #

    def _associate(self, detections: List[Detection]):
        """Hungarian assignment with a distance gate."""
        track_ids = list(self._tracks.keys())
        if not track_ids or not detections:
            return [], list(range(len(detections)))

        # Predicted position after the predict() step above is the filter's
        # prior mean; it is what we compare against the measurements.
        predicted = {tid: self._tracks[tid].filter.x[:2, 0] for tid in track_ids}

        cost = np.zeros((len(track_ids), len(detections)))
        for row, tid in enumerate(track_ids):
            px, py = predicted[tid]
            for col, det in enumerate(detections):
                dx = px - det.centroid[0]
                dy = py - det.centroid[1]
                cost[row, col] = float(np.hypot(dx, dy))

        rows, cols = linear_sum_assignment(cost)

        matches = []
        used_detections = set()
        for row, col in zip(rows, cols):
            if cost[row, col] <= S.GATE_DIST:
                matches.append((track_ids[row], int(col)))
                used_detections.add(int(col))

        unmatched = [i for i in range(len(detections)) if i not in used_detections]
        return matches, unmatched

    def _is_duplicate(self, detection: Detection) -> bool:
        """True if this detection is really a vehicle we already track.

        Splitting a merged blob, or a shadow fragment separating from a car,
        can produce a second blob on the same vehicle; giving it a new ID
        would count that vehicle twice.
        """
        for track in self._tracks.values():
            if _iou(track.bbox, detection.bbox) >= S.DUP_IOU:
                return True
        return False

    def _create_track(self, detection: Detection, frame_index: int) -> None:
        track = Track(detection, frame_index, self._next_id)
        self._next_id += 1
        self._tracks[track.id] = track
        self.records[track.id] = {
            "id": track.id,
            "birth_frame": frame_index,
            "first_centroid": detection.centroid,
            "first_area": detection.area,
        }

    def _close_record(self, track: Track) -> None:
        record = self.records.get(track.id)
        if record is None:
            return
        record.update({
            "last_frame": track.last_seen_frame,
            "last_centroid": track.centroid,
            "hits": track.hits,
            "confirmed": track.confirmed,
            "counted_direction": track.counted_direction,
        })
