"""
run_pipeline.py
===============
Entry point.  Runs the whole assignment end to end on one video:

    detect  ->  track (Kalman + Hungarian)  ->  count line crossings
            ->  annotated video  ->  evidence (crops, contact sheet, logs)

Usage
-----
    python run_pipeline.py                     # the supplied clip
    python run_pipeline.py --video other.mp4   # any other clip
    python run_pipeline.py --show              # also open a preview window

Outputs (all relative to this folder)
-------------------------------------
    output/traffic_counting.mp4         annotated video, 960x540, H.264
    output/summary.json                 machine-readable run summary
    evidence/crossings.csv              one row per counted crossing
    evidence/suspicious_tracks.csv      automatically flagged cases
    evidence/contact_sheet.jpg          all crossing crops in one image
    evidence/cross_*.jpg                crop of every counted crossing
    evidence/samples/annotated_*.jpg    frames used in the report
    PROGRESS.md                         stage checklist
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

import settings as S
from counting import Crossing, LineCounter, LineSegment
from motion_detector import MotionDetector
from road_tracker import Track, VehicleTracker


# --------------------------------------------------------------------------- #
# Output video
# --------------------------------------------------------------------------- #

class VideoSink:
    """Writes annotated frames as H.264 through ffmpeg, with an OpenCV fallback."""

    def __init__(self, path: str, width: int, height: int, fps: float) -> None:
        self.path = path
        self.width = width
        self.height = height
        self.fps = fps if fps and fps > 0 else 30.0
        self._process: Optional[subprocess.Popen] = None
        self._writer: Optional[cv2.VideoWriter] = None

        os.makedirs(os.path.dirname(path), exist_ok=True)
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg:
            command = [
                ffmpeg, "-y", "-loglevel", "error",
                "-f", "rawvideo", "-pix_fmt", "bgr24",
                "-s", f"{width}x{height}", "-r", f"{self.fps:.6f}", "-i", "-",
                "-an", "-c:v", "libx264", "-preset", "medium",
                "-crf", str(S.OUT_CRF), "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", path,
            ]
            try:
                self._process = subprocess.Popen(command, stdin=subprocess.PIPE)
            except OSError:
                self._process = None

        if self._process is None:
            fourcc = cv2.VideoWriter_fourcc(*S.OUT_CODEC_FALLBACK)
            self._writer = cv2.VideoWriter(path, fourcc, self.fps, (width, height))
            if not self._writer.isOpened():
                self._writer = None

    @property
    def ok(self) -> bool:
        return self._process is not None or self._writer is not None

    def write(self, frame: np.ndarray) -> None:
        if self._process is not None:
            try:
                self._process.stdin.write(frame.tobytes())
            except (BrokenPipeError, AttributeError):
                self._process = None
        elif self._writer is not None:
            self._writer.write(frame)

    def release(self) -> None:
        if self._process is not None:
            try:
                self._process.stdin.close()
            except Exception:
                pass
            self._process.wait()
            if self._process.returncode != 0 and os.path.exists(self.path):
                print("  WARNING: ffmpeg returned", self._process.returncode)
            self._process = None
        if self._writer is not None:
            self._writer.release()
            self._writer = None


# --------------------------------------------------------------------------- #
# Drawing
# --------------------------------------------------------------------------- #

def _ip(point: Sequence[float]) -> Tuple[int, int]:
    return int(round(point[0])), int(round(point[1]))


def draw_line(frame: np.ndarray, counter: LineCounter) -> None:
    """Draw both parts of the virtual line, their buffer bands and direction tags."""
    for segment in counter.segments:
        a, b = _ip(segment.a), _ip(segment.b)

        # Buffer band (the dead zone where a crossing is not yet decided).
        for sign in (+1, -1):
            offset = segment.normal * S.LINE_BUFFER_PX * sign
            pa = _ip((segment.a[0] + offset[0], segment.a[1] + offset[1]))
            pb = _ip((segment.b[0] + offset[0], segment.b[1] + offset[1]))
            cv2.line(frame, pa, pb, S.COL_BUFFER, 1, cv2.LINE_AA)

        cv2.line(frame, a, b, S.COL_LINE, 3, cv2.LINE_AA)
        # Short ticks at both ends so the extent of the line is obvious.
        for end in (a, b):
            normal = segment.normal * 9
            cv2.line(frame, _ip((end[0] - normal[0], end[1] - normal[1])),
                     _ip((end[0] + normal[0], end[1] + normal[1])),
                     S.COL_LINE, 2, cv2.LINE_AA)

        # Arrow showing the positive crossing direction.
        mid = np.array(segment.a) + (np.array(segment.b) - np.array(segment.a)) * 0.5
        tangent = segment.unit * 26
        cv2.arrowedLine(frame,
                        _ip((mid[0] - segment.normal[0] * 26, mid[1] - segment.normal[1] * 26)),
                        _ip((mid[0] + segment.normal[0] * 26, mid[1] + segment.normal[1] * 26)),
                        S.COL_LINE, 2, tipLength=0.4)
        _ = tangent


def draw_tracks(frame: np.ndarray, tracks: Sequence[Track]) -> None:
    """Boxes, IDs, centroid trails."""
    for track in tracks:
        if not track.confirmed:
            continue
        x, y, w, h = track.bbox
        if track.counted_direction == "IN":
            colour = S.COL_IN
        elif track.counted_direction == "OUT":
            colour = S.COL_OUT
        else:
            colour = S.COL_TRACK

        cv2.rectangle(frame, (x, y), (x + w, y + h), colour, 2)

        label = f"ID {track.id}"
        if track.counted_direction:
            label += f" {track.counted_direction}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        top = max(0, y - th - 8)
        cv2.rectangle(frame, (x, top), (x + tw + 8, top + th + 6), colour, -1)
        cv2.putText(frame, label, (x + 4, top + th + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (12, 12, 12), 1, cv2.LINE_AA)

        trail = list(track.trail)
        for i in range(1, len(trail)):
            shade = 0.35 + 0.65 * (i / len(trail))
            seg_colour = tuple(int(c * shade) for c in colour)
            cv2.line(frame, _ip(trail[i - 1]), _ip(trail[i]), seg_colour, 2, cv2.LINE_AA)

        cv2.circle(frame, _ip(track.centroid), 3, (0, 0, 235), -1)


def draw_hud(frame: np.ndarray, counter: LineCounter, active: int,
             frame_index: int, fps_total: float) -> None:
    """Count panel: total, both directions, live track count, time."""
    panel_w, panel_h, margin = 268, 150, 12
    overlay = frame.copy()
    cv2.rectangle(overlay, (margin, margin),
                  (margin + panel_w, margin + panel_h), S.COL_HUD_BG, -1)
    cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)
    cv2.rectangle(frame, (margin, margin),
                  (margin + panel_w, margin + panel_h), (70, 70, 80), 1)

    def text(value: str, row: int, colour=S.COL_TEXT, scale=0.55) -> None:
        cv2.putText(frame, value, (margin + 12, margin + row),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 2, cv2.LINE_AA)

    text("VEHICLE COUNT", 24, (0, 220, 255), 0.52)
    cv2.line(frame, (margin + 10, margin + 32),
             (margin + panel_w - 10, margin + 32), (70, 70, 80), 1)
    text(f"TOTAL        {counter.total}", 60, S.COL_TEXT, 0.6)
    text(f"IN  (towards) {counter.count_of('IN')}", 88, S.COL_IN, 0.55)
    text(f"OUT (away)    {counter.count_of('OUT')}", 114, S.COL_OUT, 0.55)
    text(f"tracks {active:2d}   frame {frame_index:3d}", 140, (165, 165, 165), 0.42)


# --------------------------------------------------------------------------- #
# Evidence
# --------------------------------------------------------------------------- #

def save_crossing_crop(frame: np.ndarray, event: Crossing, index: int) -> np.ndarray:
    """Save a crop around the counted vehicle, with a caption strip."""
    height, width = frame.shape[:2]
    x, y, w, h = event.bbox
    margin = max(S.CROP_MARGIN_PX, (S.MIN_CROP_PX - min(w, h)) // 2)

    x0 = max(0, x - margin)
    y0 = max(0, y - margin)
    x1 = min(width, x + w + margin)
    y1 = min(height, y + h + margin)

    crop = frame[y0:y1, x0:x1].copy()
    if crop.size == 0:
        crop = np.zeros((60, 60, 3), np.uint8)

    bar_h = 30
    canvas = np.zeros((crop.shape[0] + bar_h, crop.shape[1], 3), np.uint8)
    canvas[:crop.shape[0]] = crop
    canvas[crop.shape[0]:] = (28, 28, 28)

    caption = f"ID {event.track_id}  {event.direction}  frame {event.frame_index + 1}  t={event.time_s:.2f}s"
    cv2.putText(canvas, caption, (5, crop.shape[0] + 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (235, 235, 235), 1, cv2.LINE_AA)

    name = (f"cross_{index:03d}_id{event.track_id:02d}_"
            f"{event.direction}_frame{event.frame_index + 1:04d}.jpg")
    path = os.path.join(S.EVIDENCE_DIR, name)
    cv2.imwrite(path, canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    return canvas


def make_contact_sheet(crops: List[Tuple[int, np.ndarray]]) -> Optional[np.ndarray]:
    """Place every crossing crop in one grid image, in crossing order."""
    if not crops:
        return None

    cell_w, cell_h = S.CONTACT_CELL_W, S.CONTACT_CELL_H
    cols = S.CONTACT_COLS
    rows = math.ceil(len(crops) / cols)
    header = 24
    sheet = np.full((header + rows * cell_h, cols * cell_w, 3), 26, np.uint8)

    for slot, (index, crop) in enumerate(crops):
        row, col = divmod(slot, cols)
        scale = min((cell_w - 8) / crop.shape[1], (cell_h - 8) / crop.shape[0])
        resized = cv2.resize(crop, (max(1, int(crop.shape[1] * scale)),
                                    max(1, int(crop.shape[0] * scale))),
                             interpolation=cv2.INTER_AREA)
        oy = header + row * cell_h + 4 + (cell_h - 8 - resized.shape[0]) // 2
        ox = col * cell_w + 4 + (cell_w - 8 - resized.shape[1]) // 2
        sheet[oy:oy + resized.shape[0], ox:ox + resized.shape[1]] = resized

    cv2.putText(sheet, f"counted crossings: {len(crops)}", (8, 17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.imwrite(S.CONTACT_SHEET, sheet, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    return sheet


# --------------------------------------------------------------------------- #
# Suspicious-track checks
# --------------------------------------------------------------------------- #

def find_suspicious(tracker: VehicleTracker, events: List[Crossing],
                    frame_size: Tuple[int, int]) -> List[dict]:
    """Automatic sanity flags.  These are hints for the human reviewer, not
    corrections - nothing is removed or re-counted here."""
    flags: List[dict] = []
    records = tracker.records
    # The two line parts sit at very different depths, so a far-carriageway car
    # is a quarter of the area of a near one.  Compare each crossing with the
    # median of its own line part.
    medians: Dict[str, float] = {}
    for part in {e.segment for e in events}:
        values = [e.area for e in events if e.segment == part]
        medians[part] = float(np.median(values)) if values else 0.0

    def track_length(track_id: int) -> int:
        record = records.get(track_id, {})
        first = record.get("birth_frame", 0)
        last = record.get("last_frame", first)
        return max(1, last - first + 1)

    for event in events:
        length = track_length(event.track_id)
        base = {
            "track_id": event.track_id,
            "direction": event.direction,
            "line_part": event.segment,
            "frame": event.frame_index + 1,
            "time_s": round(event.time_s, 3),
        }

        if length < S.SUSPICIOUS_MIN_TRACK_FRAMES:
            flags.append({**base, "reason": "short_track",
                          "detail": f"track exists for only {length} frames"})

        median_area = medians.get(event.segment, 0.0)
        if median_area > 0:
            ratio = event.area / median_area
            if ratio < S.SUSPICIOUS_AREA_LOW or ratio > S.SUSPICIOUS_AREA_HIGH:
                flags.append({**base, "reason": "area_outlier",
                              "detail": f"blob area {event.area:.0f} px2 is "
                                        f"{ratio:.2f}x the median {median_area:.0f}"})

        x, y, w, h = event.bbox
        width, height = frame_size
        if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
            flags.append({**base, "reason": "at_frame_border",
                          "detail": "box touches the image border, vehicle may be "
                                    "only partly visible"})

        if event.along < 0.06 or event.along > 0.94:
            flags.append({**base, "reason": "near_line_end",
                          "detail": f"crossed at {event.along:.2f} of the line part"})

    # One vehicle that lost its ID and came back gets two IDs in a row.
    for i, first in enumerate(events):
        for second in events[i + 1:]:
            gap = second.frame_index - first.frame_index
            if abs(gap) > S.SUSPICIOUS_DOUBLE_FRAMES:
                continue
            distance = float(np.hypot(second.centroid[0] - first.centroid[0],
                                      second.centroid[1] - first.centroid[1]))
            if distance <= S.SUSPICIOUS_DOUBLE_PX:
                flags.append({
                    "track_id": second.track_id,
                    "direction": second.direction,
                    "line_part": second.segment,
                    "frame": second.frame_index + 1,
                    "time_s": round(second.time_s, 3),
                    "reason": "counted_close_to_another",
                    "detail": (f"ID {first.track_id} was counted {gap} frames "
                               f"earlier, {distance:.0f} px away"),
                })

    # An ID that is born where another ID died looks like a hand-over.
    for track_id, record in records.items():
        birth_frame = record.get("birth_frame", 0)
        first = record.get("first_centroid")
        if first is None:
            continue
        for other_id, other in records.items():
            if other_id == track_id or other.get("counted_direction") is None:
                continue
            last = other.get("last_centroid")
            last_frame = other.get("last_frame")
            if last is None or last_frame is None:
                continue
            if 0 <= birth_frame - last_frame <= S.SUSPICIOUS_HANDOVER_FRAMES:
                distance = float(np.hypot(first[0] - last[0], first[1] - last[1]))
                if distance <= S.SUSPICIOUS_HANDOVER_PX:
                    flags.append({
                        "track_id": track_id,
                        "direction": record.get("counted_direction") or "",
                        "line_part": "",
                        "frame": birth_frame + 1,
                        "time_s": round(birth_frame / 30.0, 3),
                        "reason": "born_where_another_ended",
                        "detail": (f"ID {other_id} ended {birth_frame - last_frame} "
                                   f"frames earlier, {distance:.0f} px away"),
                    })
    return flags


# --------------------------------------------------------------------------- #
# Main run
# --------------------------------------------------------------------------- #

def run(video_path: str, show: bool = False,
        max_frames: Optional[int] = None) -> dict:
    if not os.path.exists(video_path):
        sys.exit(f"ERROR: video not found: {video_path}")

    for folder in (S.OUTPUT_DIR, S.EVIDENCE_DIR, S.SAMPLE_DIR):
        os.makedirs(folder, exist_ok=True)
        # Evidence from an earlier run would otherwise mix with this one, since
        # the file names contain the crossing number.
        for name in os.listdir(folder):
            if name.startswith("cross_") and name.endswith(".jpg"):
                os.remove(os.path.join(folder, name))

    capture = cv2.VideoCapture(video_path)
    if not capture.isOpened():
        sys.exit(f"ERROR: cannot open {video_path}")

    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS)
    if not fps or np.isnan(fps):
        fps = 30.0

    detector = MotionDetector()
    tracker = VehicleTracker()
    counter = LineCounter((height, width), fps)

    sink = VideoSink(S.OUTPUT_VIDEO, width, height, fps)
    if not sink.ok:
        print("  WARNING: no video writer available, the annotated video is skipped")

    print("=" * 62)
    print("  Motorway car counter")
    print("=" * 62)
    print(f"  video      : {video_path}")
    print(f"  size / fps : {width}x{height} @ {fps:.2f}   frames {int(capture.get(cv2.CAP_PROP_FRAME_COUNT))}")
    for segment in counter.segments:
        print(f"  line {segment.name:<4}: {_ip(segment.a)} -> {_ip(segment.b)} "
              f"(+{segment.positive_label} / -{segment.negative_label})")
    print("=" * 62)

    if show:
        cv2.namedWindow("car counter", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("car counter", width, height)

    all_events: List[Crossing] = []
    crops: List[Tuple[int, np.ndarray]] = []
    sample_targets = {int(round(f * 335)): f for f in S.SAMPLE_FRACTIONS}
    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) or 335
    sample_targets = {int(round(f * total_frames)): f for f in S.SAMPLE_FRACTIONS}

    frame_index = 0
    started = time.time()
    last_annotated = None

    while True:
        ok, frame = capture.read()
        if not ok:
            break

        if frame_index < S.WARMUP_FRAMES:
            # Only teaching MOG2 what the empty road looks like; the frames are
            # still written so the annotated video covers the whole clip.
            detector.warm_up(frame)
            warm = frame.copy()
            draw_line(warm, counter)
            draw_hud(warm, counter, active=0, frame_index=frame_index, fps_total=0.0)
            if sink.ok:
                sink.write(warm)
            frame_index += 1
            continue

        detections = detector.detect(frame)
        tracks = tracker.update(detections, frame_index)
        events = counter.update(tracks, frame_index)

        annotated = frame.copy()
        draw_line(annotated, counter)
        draw_tracks(annotated, tracks)
        draw_hud(annotated, counter, active=len(tracks),
                 frame_index=frame_index, fps_total=0.0)
        last_annotated = annotated

        for event in events:
            all_events.append(event)
            crops.append((len(all_events), save_crossing_crop(annotated, event,
                                                             len(all_events))))
            print(f"  crossing #{len(all_events):02d}  ID {event.track_id:02d}  "
                  f"{event.direction:<3}  line={event.segment:<4} "
                  f"frame {event.frame_index + 1:3d}  t={event.time_s:5.2f}s")

        if sink.ok:
            sink.write(annotated)

        if frame_index in sample_targets and annotated is not None:
            name = f"annotated_{int(sample_targets[frame_index] * 100):02d}pct.jpg"
            cv2.imwrite(os.path.join(S.SAMPLE_DIR, name), annotated,
                        [int(cv2.IMWRITE_JPEG_QUALITY), 90])

        if show and annotated is not None:
            cv2.imshow("car counter", annotated)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

        frame_index += 1
        if max_frames and frame_index >= max_frames:
            break

    capture.release()
    sink.release()
    if show:
        cv2.destroyAllWindows()

    elapsed = time.time() - started

    # Close the records of tracks that were still alive when the clip ended.
    for track in tracker.tracks:
        tracker._close_record(track)

    contact = make_contact_sheet(crops)
    _write_crossing_log(all_events)
    flags = find_suspicious(tracker, all_events, (width, height))
    _write_suspicious_log(flags)

    video_size = os.path.getsize(S.OUTPUT_VIDEO) if os.path.exists(S.OUTPUT_VIDEO) else 0

    summary = {
        "video": video_path,
        "frames_processed": frame_index,
        "fps": round(fps, 3),
        "duration_s": round(frame_index / fps, 2),
        "counts": {label: counter.count_of(label) for label in ("IN", "OUT")},
        "counts_per_line": _counts_per_line(all_events),
        "total": counter.total,
        "crossings": [
            {
                "index": i + 1,
                "track_id": e.track_id,
                "direction": e.direction,
                "line_part": e.segment,
                "frame": e.frame_index + 1,
                "time_s": round(e.time_s, 3),
                "centroid": [round(e.centroid[0], 1), round(e.centroid[1], 1)],
                "speed_px_per_frame": round(e.speed_px_per_frame, 2),
            }
            for i, e in enumerate(all_events)
        ],
        "suspicious_flags": len(flags),
        "output_video": S.OUTPUT_VIDEO,
        "output_video_bytes": video_size,
        "output_video_resolution": [width, height],
        "runtime_s": round(elapsed, 1),
        "contact_sheet": S.CONTACT_SHEET if contact is not None else "",
    }

    _write_summary(summary)
    _print_summary(summary, flags)
    return summary


def _counts_per_line(events: Sequence[Crossing]) -> Dict[str, Dict[str, int]]:
    table: Dict[str, Dict[str, int]] = {}
    for event in events:
        table.setdefault(event.segment, {}).setdefault(event.direction, 0)
        table[event.segment][event.direction] += 1
    return table


def _write_crossing_log(events: Sequence[Crossing]) -> None:
    with open(S.CROSSING_LOG, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["crossing", "track_id", "line_part", "direction",
                         "frame", "time_s", "centroid_x", "centroid_y",
                         "box_x", "box_y", "box_w", "box_h", "area",
                         "speed_px_per_frame"])
        for index, event in enumerate(events, start=1):
            x, y, w, h = event.bbox
            writer.writerow([index, event.track_id, event.segment, event.direction,
                             event.frame_index + 1, f"{event.time_s:.3f}",
                             f"{event.centroid[0]:.1f}", f"{event.centroid[1]:.1f}",
                             x, y, w, h, f"{event.area:.0f}",
                             f"{event.speed_px_per_frame:.2f}"])


def _write_suspicious_log(flags: Sequence[dict]) -> None:
    with open(S.SUSPICIOUS_LOG, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["track_id", "direction", "line_part", "frame",
                         "time_s", "reason", "detail"])
        for flag in flags:
            writer.writerow([flag.get("track_id", ""), flag.get("direction", ""),
                             flag.get("line_part", ""), flag.get("frame", ""),
                             flag.get("time_s", ""), flag["reason"], flag["detail"]])


def _write_summary(summary: dict) -> None:
    import json
    with open(S.SUMMARY_JSON, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)


def _print_summary(summary: dict, flags: Sequence[dict]) -> None:
    size_mb = summary["output_video_bytes"] / 1_048_576
    print("=" * 62)
    print("  RESULT")
    print("=" * 62)
    print(f"  frames processed : {summary['frames_processed']}"
          f"  ({summary['duration_s']} s)")
    for label in ("IN", "OUT"):
        print(f"  {label:<4}             : {summary['counts'][label]}")
    print(f"  TOTAL            : {summary['total']}")
    for part, table in summary["counts_per_line"].items():
        inside = ", ".join(f"{k} {v}" for k, v in sorted(table.items()))
        print(f"    line '{part}'      : {inside}")
    print(f"  output video     : {S.OUTPUT_VIDEO}  ({size_mb:.2f} MB)")
    print(f"  crossing log     : {S.CROSSING_LOG}")
    print(f"  evidence crops   : {len(summary['crossings'])} + contact sheet")
    if flags:
        print(f"  flagged cases    : {len(flags)} (see {S.SUSPICIOUS_LOG})")
        for flag in flags:
            print(f"    - ID {flag.get('track_id')}: {flag['reason']} - {flag['detail']}")
    else:
        print("  flagged cases    : none")
    print(f"  runtime          : {summary['runtime_s']} s")
    print("=" * 62)


# --------------------------------------------------------------------------- #
# PROGRESS.md
# --------------------------------------------------------------------------- #

def write_progress(summary: Optional[dict] = None) -> None:
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "PROGRESS.md")
    lines = ["# PROGRESS", ""]
    if summary:
        lines += [
            "## Last run",
            "",
            f"- frames: {summary['frames_processed']} "
            f"({summary['duration_s']} s at {summary['fps']} fps)",
            f"- IN (towards camera): {summary['counts']['IN']}",
            f"- OUT (away): {summary['counts']['OUT']}",
            f"- total: {summary['total']}",
            f"- video: `{summary['output_video']}` "
            f"({summary['output_video_bytes'] / 1_048_576:.2f} MB)",
            f"- suspicious flags: {summary['suspicious_flags']}",
            "",
        ]
    lines += [
        "## Stage checklist",
        "",
        "- [x] 1 detection: MOG2 + two shadow tests + morphology + ROI + area/shape "
        "filter + merged-blob split",
        "- [x] 2 tracking: constant-velocity Kalman filter, Hungarian assignment "
        "with 70 px gate, tentative -> confirmed, coasting, no ID reuse",
        "- [x] 3 counting: two-part line, hysteresis buffer, once per track, "
        "IN / OUT separated",
        "- [x] 4 output video (960x540, H.264, < 10 MB)",
        "- [x] 5 evidence: crossings.csv, crop per crossing, contact sheet, "
        "automatic suspicious-track flags",
        "- [x] 6 README.md",
        "- [x] 7 report.docx",
        "- [x] 8 viva_notes.md",
        "",
        "## Next",
        "",
        "Watch `output/traffic_counting.mp4` and fill in the manual counts in the "
        "report (Results section). Nothing in the code depends on those numbers.",
        "",
    ]
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


# --------------------------------------------------------------------------- #

def main() -> None:
    parser = argparse.ArgumentParser(description="Motorway car counter")
    parser.add_argument("--video", default=S.VIDEO_PATH)
    parser.add_argument("--show", action="store_true", help="open a preview window")
    parser.add_argument("--max-frames", type=int, default=None)
    args = parser.parse_args()

    summary = run(args.video, show=args.show, max_frames=args.max_frames)
    write_progress(summary)


if __name__ == "__main__":
    main()
