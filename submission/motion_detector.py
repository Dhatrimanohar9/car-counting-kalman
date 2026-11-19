"""
motion_detector.py
==================
Stage 1 of the pipeline: find moving vehicles in a single frame.

The detector is a classical background-subtraction detector.  For every frame

    1. blur the frame                     (sensor / compression noise)
    2. MOG2 background model -> foreground mask, shadow pixels included
    3. drop the shadow pixels             (MOG2 marker + colour/brightness test)
    4. keep only the two road surfaces    (region of interest polygons)
    5. opening                            (remove speckle)
    6. closing                            (re-join fragments of one vehicle)
    7. contours -> area / size / shape filter
    8. split a blob that is obviously two touching vehicles

Each threshold lives in settings.py with the reason for its value.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

import settings as S


@dataclass
class Detection:
    """One moving region found in one frame."""

    bbox:     Tuple[int, int, int, int]   # (x, y, w, h) in pixels
    centroid: Tuple[float, float]         # (cx, cy) in pixels
    area:     float                       # foreground area inside the contour
    split:    bool = False                # created by splitting a merged blob


class MotionDetector:
    """Stateful detector: the background model is learned while frames arrive."""

    def __init__(self) -> None:
        self._subtractor = cv2.createBackgroundSubtractorMOG2(
            history=S.BGS_HISTORY,
            varThreshold=S.BGS_VAR_THRESHOLD,
            detectShadows=S.BGS_DETECT_SHADOWS,
        )
        self._open_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, S.OPEN_KSIZE)
        self._close_k = cv2.getStructuringElement(cv2.MORPH_RECT, S.CLOSE_KSIZE)
        self._roi_mask: Optional[np.ndarray] = None
        self._frame_shape: Optional[Tuple[int, int]] = None

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def warm_up(self, frame: np.ndarray) -> None:
        """Feed a frame to the background model without producing detections.

        Used for the first few frames so the model is not built from an
        all-zero mask.  A fast learning rate is safe here: the camera is fixed
        and the road is empty in the first half second of the clip.
        """
        blurred = cv2.GaussianBlur(frame, S.BLUR_KSIZE, S.BLUR_SIGMA)
        self._subtractor.apply(blurred, learningRate=0.4)

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Return the moving vehicles found in *frame* (BGR image)."""
        self._ensure_roi(frame.shape)

        blurred = cv2.GaussianBlur(frame, S.BLUR_KSIZE, S.BLUR_SIGMA)
        raw = self._subtractor.apply(blurred)

        # MOG2 encodes: 255 = foreground, 127 = shadow, 0 = background.
        fg = np.where(raw >= S.FOREGROUND_MIN_VALUE, 255, 0).astype(np.uint8)

        # Shadows that MOG2 did not mark are removed with a brightness test
        # against the learned background.
        background = self._subtractor.getBackgroundImage()
        if background is not None:
            fg[self._colour_shadow_mask(frame, background)] = 0

        # Ignore everything that is not road.
        fg[self._roi_mask == 0] = 0

        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, self._open_k,
                              iterations=S.OPEN_ITERS)
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, self._close_k,
                              iterations=S.CLOSE_ITERS)

        return self._contours_to_detections(fg)

    # ------------------------------------------------------------------ #
    # Shadow removal
    # ------------------------------------------------------------------ #

    @staticmethod
    def _colour_shadow_mask(frame: np.ndarray, background: np.ndarray) -> np.ndarray:
        """Pixels that are 'the same colour, but darker' than the background.

        A cast shadow multiplies the light, so its pixels are darker versions
        of the road: brightness ratio well below 1 while hue/saturation stay
        close to the background.  A dark vehicle does not fit that pattern -
        it is far more saturated than grey asphalt - so it is kept.
        """
        grey_f = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        grey_b = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY).astype(np.float32)
        sat_f = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 1].astype(np.float32)
        sat_b = cv2.cvtColor(background, cv2.COLOR_BGR2HSV)[:, :, 1].astype(np.float32)

        ratio = grey_f / np.maximum(grey_b, 1.0)
        darker = (ratio > S.SHADOW_DARK) & (ratio < S.SHADOW_LIGHT)
        same_colour = np.abs(sat_f - sat_b) < S.SHADOW_SAT_TOL
        return (darker & same_colour).astype(np.uint8) * 255

    # ------------------------------------------------------------------ #
    # Region of interest
    # ------------------------------------------------------------------ #

    def _ensure_roi(self, shape: Sequence[int]) -> None:
        """Build the road mask once, for the actual frame size."""
        height, width = int(shape[0]), int(shape[1])
        if self._roi_mask is not None and self._frame_shape == (height, width):
            return

        mask = np.zeros((height, width), dtype=np.uint8)
        for polygon in S.ROI_POLYGONS:
            pts = np.array(
                [[int(round(x * width)), int(round(y * height))] for x, y in polygon],
                dtype=np.int32,
            )
            cv2.fillPoly(mask, [pts], 255)

        if S.ROI_TOP_IGNORE_PX > 0:
            mask[:S.ROI_TOP_IGNORE_PX, :] = 0

        self._roi_mask = mask
        self._frame_shape = (height, width)

    # ------------------------------------------------------------------ #
    # Mask -> detections
    # ------------------------------------------------------------------ #

    def _contours_to_detections(self, mask: np.ndarray) -> List[Detection]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found: List[Detection] = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < S.MIN_AREA or area > S.MAX_AREA:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < S.MIN_WIDTH or h < S.MIN_HEIGHT:
                continue
            aspect = w / float(h)
            if aspect < S.MIN_ASPECT or aspect > S.MAX_ASPECT:
                continue
            if area / float(w * h) < S.MIN_FILL_RATIO:
                continue

            parts = self._split_if_merged(mask, contour, x, y, w, h)
            if parts:
                found.extend(parts)
            else:
                found.append(self._detection_from_contour(contour, (x, y, w, h), area))

        return found

    @staticmethod
    def _detection_from_contour(contour, bbox, area: float,
                                split: bool = False) -> Detection:
        moments = cv2.moments(contour)
        if moments["m00"] > 0:
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
        else:  # degenerate contour, fall back to the box centre
            x, y, w, h = bbox
            cx, cy = x + w / 2.0, y + h / 2.0
        return Detection(bbox=bbox, centroid=(float(cx), float(cy)),
                         area=area, split=split)

    def _split_if_merged(self, mask: np.ndarray, contour, x: int, y: int,
                         w: int, h: int) -> List[Detection]:
        """Separate two vehicles that share one blob.

        The mask is projected onto the axis along which the blob looks too
        long (columns for side-by-side vehicles, rows for nose-to-tail ones).
        The minimum of that projection inside the middle of the blob is the
        gap between the two bodies; the blob is cut there if the gap is deep
        enough and both halves are big enough to be vehicles.
        """
        if w >= S.SPLIT_MIN_ASPECT * h and w >= S.SPLIT_MIN_WIDTH:
            cut = self._find_valley(np.sum(mask[y:y + h, x:x + w] > 0, axis=0))
            if cut is None:
                return []
            left_w, right_w = cut, w - cut
            if min(left_w, right_w) < S.SPLIT_MIN_PART * w:
                return []
            left = (x, y, left_w, h)
            right = (x + cut, y, right_w, h)
            return [self._box_detection(mask, left), self._box_detection(mask, right)]

        if h >= S.SPLIT_MIN_ASPECT * w and h >= S.SPLIT_MIN_WIDTH:
            cut = self._find_valley(np.sum(mask[y:y + h, x:x + w] > 0, axis=1))
            if cut is None:
                return []
            top_h, bottom_h = cut, h - cut
            if min(top_h, bottom_h) < S.SPLIT_MIN_PART * h:
                return []
            top = (x, y, w, top_h)
            bottom = (x, y + cut, w, bottom_h)
            return [self._box_detection(mask, top), self._box_detection(mask, bottom)]

        return []

    @staticmethod
    def _find_valley(profile: np.ndarray) -> Optional[int]:
        """Return the cut position if the profile has a convincing valley."""
        n = len(profile)
        lo, hi = int(0.22 * n), int(0.78 * n)
        if hi - lo < 4:
            return None
        segment = profile[lo:hi].astype(np.float32)
        idx = int(np.argmin(segment)) + lo
        mean = float(np.mean(profile)) or 1.0
        if profile[idx] <= S.SPLIT_VALLEY * mean:
            return idx
        return None

    @staticmethod
    def _box_detection(mask: np.ndarray, box: Tuple[int, int, int, int]) -> Detection:
        """Build a detection for a sub-box of a merged blob.

        The centroid is the centre of mass of the *mask pixels* inside that
        sub-box, not the box centre, so it follows the vehicle body even when
        the boxes are not equally filled.
        """
        x, y, w, h = box
        part = mask[y:y + h, x:x + w]
        ys, xs = np.nonzero(part)
        if len(xs) > 0:
            cx = x + float(xs.mean())
            cy = y + float(ys.mean())
            area = float(len(xs))
        else:
            cx, cy, area = x + w / 2.0, y + h / 2.0, float(w * h)
        return Detection(bbox=box, centroid=(cx, cy), area=area, split=True)
