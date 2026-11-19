"""
settings.py
===========
Every tunable number used by the car counting system, in one place, with the
reason it has that value.

All values were chosen by measuring the supplied clip
    input/traffic.mp4  -  960 x 540, 29.97 fps, 335 frames, 11.2 s
(camera looking down a motorway from an overpass; traffic on the near
carriageway comes towards the camera, traffic on the far carriageway behind
the double guardrail drives away to the right).

Nothing here is fitted to a desired answer.  Where a number came from a
measurement, the measurement is quoted in the comment above it.
"""

import os

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))

VIDEO_PATH   = os.path.join(_HERE, "input", "traffic.mp4")

OUTPUT_DIR   = os.path.join(_HERE, "output")
OUTPUT_VIDEO = os.path.join(OUTPUT_DIR, "traffic_counting.mp4")
SUMMARY_JSON = os.path.join(OUTPUT_DIR, "summary.json")

EVIDENCE_DIR  = os.path.join(_HERE, "evidence")
CROSSING_LOG  = os.path.join(EVIDENCE_DIR, "crossings.csv")
SUSPICIOUS_LOG = os.path.join(EVIDENCE_DIR, "suspicious_tracks.csv")
CONTACT_SHEET = os.path.join(EVIDENCE_DIR, "contact_sheet.jpg")
SAMPLE_DIR    = os.path.join(EVIDENCE_DIR, "samples")   # 3 frames used in the report

# ---------------------------------------------------------------------------
# Virtual counting line
# ---------------------------------------------------------------------------
# The line has two straight parts because the two carriageways lie at very
# different angles in the image:
#
#   near part  : horizontal, y = 0.559 * 540 = 302 px, x from 180 to 775 px.
#                At that height the near carriageway is 600 px wide and its
#                lane boundaries are clean, so a whole vehicle fits inside the
#                frame while it is being counted (blobs there are 90-130 px
#                wide, area 4000-9000 px^2 -> very reliable detection).
#   far part   : vertical, x = 0.833 * 960 = 800 px, y from 60 to 150 px.
#                The far carriageway is only a diagonal sliver at the edge of
#                the image; the road surface exists between y = 55 and y = 165
#                at x = 800 (measured from the road boundaries), so the
#                segment is kept inside that band.  Its traffic moves to the
#                right, i.e. away from the camera.
#
# Coordinates are stored as fractions of the frame size so the settings stay
# readable and the file still makes sense if the clip is resized.
COUNT_LINES = [
    # name, start point (x, y), end point (x, y), label for crossing in the
    # +normal sense, label for crossing in the -normal sense.
    # normal = (-dy, dx) of the segment direction:
    #   near: direction (1, 0) -> normal (0, 1)  -> +normal means y grows
    #         (vehicle moving down the image = towards the camera).
    #   far : direction (0, 1) -> normal (-1, 0) -> +normal means x shrinks,
    #         -normal means x grows (vehicle moving right = away from camera).
    dict(name="near", a=(0.188, 0.559), b=(0.807, 0.559),
         positive_label="IN",  negative_label="OUT"),
    dict(name="far",  a=(0.833, 0.111), b=(0.833, 0.278),
         positive_label="IN",  negative_label="OUT"),
]

# A vehicle has to be at least this far on one side of a line part and then
# this far on the other side before a crossing is accepted.  The dead band
# swallows the 2-4 px centroid jitter of a background-subtracted blob, so a
# car that pauses or wobbles on the line cannot be counted twice, while a car
# that jumps the whole band in one frame (fast, close vehicles move up to
# 18 px/frame here) is still counted because only the *state* has to change.
LINE_BUFFER_PX = 14

# A crossing is only accepted if the vehicle crosses inside the drawn part of
# the line, with this tolerance at the ends (fraction of the segment length).
LINE_END_TOLERANCE = 0.04

# ---------------------------------------------------------------------------
# Region of interest
# ---------------------------------------------------------------------------
# Only these two road surfaces are processed.  The polygons were traced from
# the measured road boundaries of the clip (asphalt runs per row/column), so
# the grass verges, the guardrails and the treeline - the main sources of
# false motion - are never turned into detections.
# Points are fractions of (width, height).
ROI_POLYGONS = [
    # near carriageway: left edge (411,0)->(30,540), right edge (562,0)->(950,540)
    [(0.585, 0.0), (0.428, 0.0), (0.031, 1.0), (0.990, 1.0)],
    # far carriageway: road surface between the frame top and the guardrail
    [(0.646, 0.0), (1.0, 0.0), (1.0, 0.426), (0.729, 0.222)],
]
# The top 45 rows hold the horizon, the distant trees and the sign gantry.
# Flickering foliage there produced 300+ small blobs per 100 frames in a test
# run, and no vehicle has to be counted above y = 60 anyway.
ROI_TOP_IGNORE_PX = 45

# ---------------------------------------------------------------------------
# Detection: background subtraction (Zivkovic MOG2)
# ---------------------------------------------------------------------------
# history = 200 frames (~6.7 s).  A vehicle needs ~2 s to leave the area, so
# the model still "remembers" an empty road behind it and does not absorb slow
# traffic; 500 (the default) adapted so slowly that the sun/shade boundary at
# the top of the frame stayed foreground for the whole clip.
BGS_HISTORY = 200
# varThreshold is a squared Mahalanobis distance. 32 was slightly better than
# the default 16 at ignoring the block noise of this H.264 stream while still
# picking up the small (60 x 45 px) far-carriageway cars.
BGS_VAR_THRESHOLD = 32.0
BGS_DETECT_SHADOWS = True

# MOG2 marks a pixel as shadow by writing 127; keeping only >= 200 drops those.
FOREGROUND_MIN_VALUE = 200

# Second, explicit shadow test on top of MOG2's own one.  Here the sun is low
# and to the right, and every vehicle drags a hard shadow that MOG2 sometimes
# reports as foreground (one car+shadow blob measured 151 x 201 px, 22000 px^2,
# i.e. twice the car).  A shadowed pixel keeps its colour but only a fraction
# of its brightness, so a pixel is rejected as shadow when
#   SHADOW_DARK < grey(frame)/grey(background) < SHADOW_LIGHT
#   and |saturation(frame) - saturation(background)| < SHADOW_SAT_TOL.
# The saturation term is what keeps dark *cars* (which are much more saturated
# than grey asphalt) from being erased.  Range 0.45-0.88 with 50 units of
# saturation tolerance left every car intact in the checks on frames 0/120/240.
SHADOW_DARK    = 0.45
SHADOW_LIGHT   = 0.88
SHADOW_SAT_TOL = 50

# Pre-blur: 5x5, sigma 1.2.  Removes sensor grain without smearing the edges
# of a 60 px distant car (a 9x9 blur did smear them in a comparison).
BLUR_KSIZE = (5, 5)
BLUR_SIGMA = 1.2

# Opening 3x3: kills isolated 1-3 px speckle (rain/compression noise).
OPEN_KSIZE  = (3, 3)
OPEN_ITERS  = 1
# Closing 9x9, twice: a car mask often breaks into pieces (dark windscreen,
# sunroof, number plate) and the gaps are up to ~8 px.  Lanes here are 90-100 px
# apart at the counting line, so a 9 px closing cannot fuse two vehicles.
CLOSE_KSIZE = (9, 9)
CLOSE_ITERS = 2

# ---------------------------------------------------------------------------
# Detection: blob filtering
# ---------------------------------------------------------------------------
# Areas measured on the clip: distant near-lane car ~900 px^2, car at the
# counting line ~4000-9000 px^2, close car at the bottom ~20000 px^2,
# far-carriageway car ~1900 px^2.  Noise after the filters above: 150-700 px^2.
# 700 therefore removes the noise and keeps every vehicle that is big enough to
# matter.  (This is a floor only - a track also has to survive MIN_HITS frames
# before it can ever be counted.)
MIN_AREA = 700.0
# A full-frame blob appears in the first frames while MOG2 builds its model
# (960*540 = 518000 px^2); 45000 still accepts a close-up lorry plus its shadow.
MAX_AREA = 45000.0
MIN_WIDTH  = 26
MIN_HEIGHT = 16
# width/height of a single vehicle at this camera angle is 0.6-1.4; the wide
# end allows for a lorry seen slightly from the side.
MIN_ASPECT = 0.25
MAX_ASPECT = 4.0
# How much of its own bounding box a blob must fill.  Measured vehicles: 0.46 -
# 0.79 (they are compact).  The thin strips that appear where a guardrail
# shadow is cut by the edge of the road mask fill 0.01 - 0.03 of a very large
# box, and were the only source of false counts in the first full run.
MIN_FILL_RATIO = 0.33

# Merged / touching vehicles.
# Two cars in neighbouring lanes touch when they are level with each other at
# the same height in the image; the joined blob is then clearly wider than
# tall (>= 1.55 * height) while a single car is not.  The split point is the
# darkest column of the mask inside the blob (the lane stripe or the gap
# between the bodies).  Fixed thresholds, no tuning against the answer.
SPLIT_MIN_ASPECT = 1.55   # w/h needed before a horizontal split is attempted
SPLIT_MIN_WIDTH  = 130    # px, a single car at the line is at most ~130 wide
SPLIT_VALLEY     = 0.55   # split only if the valley is < 55% of the mean column
SPLIT_MIN_PART   = 0.22   # each half must keep at least 22% of the width

# Frames 0-14 are used only to teach MOG2 what the empty road looks like (the
# first masks are otherwise the whole frame). ~0.5 s; the first vehicle that
# reaches a counting line does so at frame ~70, so no real vehicle is skipped.
WARMUP_FRAMES = 15

# ---------------------------------------------------------------------------
# Tracking: Kalman filter (constant velocity)
# ---------------------------------------------------------------------------
# State x = [x, y, vx, vy]^T, measurement z = [x, y]^T, dt = 1 frame.
KF_DT = 1.0
# Q: how much the motion may deviate from constant velocity per frame.
# q_pos 0.05 px^2  - the centroid cannot teleport.
# q_vel 0.8 (px/frame)^2 - cars do accelerate and change lanes, so the velocity
#   needs to be allowed to change; 0.8 keeps the filter responsive (lag ~1
#   frame) while still smoothing the 2-4 px measurement jitter.
KF_Q_POS = 0.05
KF_Q_VEL = 0.8
# R: variance of the measured centroid. Background-subtracted centroids of the
# same parked-in-traffic car moved by 2-4 px, so sigma ~= 2.5 px -> R = 6.
KF_R_MEAS = 6.0
# P0: position is known from the first detection (sigma ~ 5 px -> 25), velocity
# is completely unknown (sigma ~ 20 px/frame -> 400).
KF_P_POS = 25.0
KF_P_VEL = 400.0

# ---------------------------------------------------------------------------
# Tracking: association and track lifecycle
# ---------------------------------------------------------------------------
# Hungarian assignment on Euclidean distance between predicted and measured
# centroid.  Gate 70 px: a confirmed track's prediction is normally within a
# few px, 70 px is still smaller than the gap between two neighbouring lanes
# (90-100 px at the counting line), so an ID cannot jump to the car beside it.
GATE_DIST = 70.0
# A track that is not detected for more than MAX_LOST frames (0.2 s) is
# deleted.  Longer coasting caused ID swaps in testing: after ~0.5 s the
# prediction sits where the next car has already arrived.
MAX_LOST = 6
# A new track is tentative until it has been seen in MIN_HITS frames.  Only
# confirmed tracks are drawn and counted, which removes the one-frame ghost
# detections that shadow flicker produces.
MIN_HITS = 3
# A detection that overlaps an existing track's box this much is treated as a
# duplicate of that vehicle (happens when a merged blob is split in a later
# frame) and does not start a new ID.
DUP_IOU = 0.35
# Length of the centroid trail drawn behind each box.
HISTORY_LEN = 40

# ---------------------------------------------------------------------------
# Counting
# ---------------------------------------------------------------------------
# Only tracks that have been confirmed for this many frames may be counted,
# so a spurious 2-frame blob on the line never reaches the counter.
MIN_HITS_TO_COUNT = MIN_HITS

# ---------------------------------------------------------------------------
# Output video and evidence
# ---------------------------------------------------------------------------
# 960x540 = 540p, inside the "720p or lower" limit.  Frames are piped to
# ffmpeg (libx264, CRF 26) which keeps 11 s of 540p well under 10 MB.
OUT_CRF = 26
OUT_CODEC_FALLBACK = "mp4v"     # used only if the ffmpeg binary is missing

# Crops saved for every counted crossing: bounding box plus this margin, and
# never smaller than MIN_CROP px on either side so the car is always visible
# in context.
CROP_MARGIN_PX = 50
MIN_CROP_PX    = 150
CONTACT_COLS   = 4
CONTACT_CELL_W = 240
CONTACT_CELL_H = 175

# Sample frames saved for the report (fraction of the clip).
SAMPLE_FRACTIONS = (0.25, 0.55, 0.88)

# ---------------------------------------------------------------------------
# Suspicious-track rules (stage 5 of the assignment)
# ---------------------------------------------------------------------------
# Tracks shorter than this that still got counted are flagged: too little
# evidence to trust.
SUSPICIOUS_MIN_TRACK_FRAMES = 12
# Two counted IDs that follow each other within this many frames and this many
# pixels look like one vehicle that lost and regained its identity.
SUSPICIOUS_HANDOVER_FRAMES = 12
SUSPICIOUS_HANDOVER_PX     = 60.0
# Two crossings this close in time and space look like one vehicle counted
# twice.
SUSPICIOUS_DOUBLE_FRAMES = 8
SUSPICIOUS_DOUBLE_PX     = 70.0
# Blob area at the moment of crossing outside this factor of the median
# counted area -> the box was probably a fragment or a merge.
SUSPICIOUS_AREA_LOW  = 0.35
SUSPICIOUS_AREA_HIGH = 3.0

# ---------------------------------------------------------------------------
# Colours (BGR)
# ---------------------------------------------------------------------------
COL_LINE     = (0, 225, 255)   # yellow counting line
COL_BUFFER   = (110, 110, 110)
COL_IN       = (0, 210, 0)     # green  - counted, moving towards the camera
COL_OUT      = (0, 140, 255)   # orange - counted, moving away
COL_TRACK    = (255, 170, 40)  # blue  - confirmed, not counted yet
COL_TEXT     = (240, 240, 240)
COL_HUD_BG   = (18, 18, 24)
