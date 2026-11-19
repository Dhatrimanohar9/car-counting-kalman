"""
counting.py
===========
Stage 3 of the pipeline: decide when a tracked vehicle crosses the virtual
line, and count it once, per direction.

Geometry
--------
Each part of the line is a straight segment A->B.  Its unit normal is
n = (-dy, dx) / |B-A|, and the signed distance of a point P is

    s = (P - A) . n

Inside the segment, s > 0 means "on the positive side" and s < 0 "on the
negative side".  A vehicle crosses when its signed side flips between two
consecutive frames.

Why the buffer matters
----------------------
The centroid of a background-subtracted blob jitters by 2-4 px even when the
vehicle moves smoothly, so a car driving exactly along the line would produce
a burst of tiny sign flips.  Therefore the side is only updated when |s| is
larger than LINE_BUFFER_PX: inside that dead band the previous side is kept.
The vehicle must commit to one side and then reach the other side, which is
the usual hysteresis trick; a fast car that jumps right over the band is still
counted, because it is the *state* that has to change, not an individual frame.

Each (track, line part) pair can be counted exactly once.  Since the line
parts cover different carriageways, every vehicle is counted at most once in
total.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import settings as S
from road_tracker import Track


@dataclass
class LineSegment:
    """One straight part of the virtual line, in pixels."""

    name: str
    a: Tuple[float, float]
    b: Tuple[float, float]
    positive_label: str
    negative_label: str

    def __post_init__(self) -> None:
        direction = np.array(self.b, dtype=float) - np.array(self.a, dtype=float)
        self.length = float(np.hypot(*direction))
        self.unit = direction / self.length
        # Normal pointing to the left of the segment direction.
        self.normal = np.array([-self.unit[1], self.unit[0]], dtype=float)

    def signed_distance(self, point: Tuple[float, float]) -> float:
        return float(np.dot(np.array(point, dtype=float) - np.array(self.a, dtype=float),
                            self.normal))

    def along(self, point: Tuple[float, float]) -> float:
        """Position of the point along the segment, 0 at A and 1 at B."""
        return float(np.dot(np.array(point, dtype=float) - np.array(self.a, dtype=float),
                            self.unit) / self.length)

    def label_for_side(self, side: int) -> str:
        return self.positive_label if side > 0 else self.negative_label


@dataclass
class Crossing:
    """One counted vehicle."""

    track_id: int
    segment: str
    direction: str
    frame_index: int
    time_s: float
    centroid: Tuple[float, float]
    bbox: Tuple[int, int, int, int]
    area: float
    speed_px_per_frame: float
    along: float          # where along the line part the crossing happened


class LineCounter:
    """Holds the two line parts and the two running counts."""

    def __init__(self, frame_size: Tuple[int, int], fps: float) -> None:
        height, width = frame_size
        self.fps = fps if fps > 0 else 30.0

        self.segments: List[LineSegment] = []
        for spec in S.COUNT_LINES:
            ax, ay = spec["a"]
            bx, by = spec["b"]
            self.segments.append(LineSegment(
                name=spec["name"],
                a=(ax * width, ay * height),
                b=(bx * width, by * height),
                positive_label=spec["positive_label"],
                negative_label=spec["negative_label"],
            ))

        # side of each line part for each track: (track_id, segment) -> -1/+1
        self._side: Dict[Tuple[int, str], int] = {}
        # line parts already counted for each track
        self._counted: set = set()
        # counts per direction label
        self.counts: Dict[str, int] = {"IN": 0, "OUT": 0}
        self.events: List[Crossing] = []

    # ------------------------------------------------------------------ #

    @property
    def total(self) -> int:
        return sum(self.counts.values())

    def count_of(self, label: str) -> int:
        return self.counts.get(label, 0)

    def update(self, tracks: Sequence[Track], frame_index: int) -> List[Crossing]:
        """Look for crossings of every confirmed track in this frame."""
        new_events: List[Crossing] = []

        for track in tracks:
            if not track.confirmed:
                continue

            for segment in self.segments:
                key = (track.id, segment.name)
                if key in self._counted:
                    continue

                current_side = self._definite_side(segment, track.centroid)
                if current_side == 0:
                    continue  # inside the dead band: keep the remembered side

                previous_side = self._side.get(key, 0)
                if previous_side == 0:
                    # First frame outside the band: only record the side.
                    self._side[key] = current_side
                    continue

                if current_side == previous_side:
                    continue

                # The vehicle changed side.  Only accept it if that happened
                # inside the drawn part of the line.
                where = segment.along(track.centroid)
                if not (-S.LINE_END_TOLERANCE <= where <= 1.0 + S.LINE_END_TOLERANCE):
                    self._side[key] = current_side
                    continue

                self._side[key] = current_side
                self._counted.add(key)

                label = segment.label_for_side(current_side)
                self.counts[label] = self.counts.get(label, 0) + 1

                vx, vy = track.filter.velocity
                event = Crossing(
                    track_id=track.id,
                    segment=segment.name,
                    direction=label,
                    frame_index=frame_index,
                    time_s=frame_index / self.fps,
                    centroid=track.centroid,
                    bbox=track.bbox,
                    area=track.area,
                    speed_px_per_frame=float(np.hypot(vx, vy)),
                    along=where,
                )
                self.events.append(event)
                new_events.append(event)

                track.counted_direction = label
                track.counted_frame = frame_index

        return new_events

    # ------------------------------------------------------------------ #

    @staticmethod
    def _definite_side(segment: LineSegment, point: Tuple[float, float]) -> int:
        """+1 / -1 when the point is clear of the buffer, 0 when inside it."""
        distance = segment.signed_distance(point)
        if distance > S.LINE_BUFFER_PX:
            return 1
        if distance < -S.LINE_BUFFER_PX:
            return -1
        return 0
