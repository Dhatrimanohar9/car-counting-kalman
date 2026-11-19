# Car counting on a motorway camera

Course assignment: count the vehicles in a surveillance-style traffic video,
one ID per vehicle, using background subtraction, a Kalman filter per track, and
a virtual counting line, with the two directions counted separately and the
counts shown live on the video.

Everything is original code written for this clip. The video is a real camera
recording, not a rendered or synthetic sequence.

## The clip

`input/traffic.mp4`

| | |
|---|---|
| source | DeGirum PySDKExamples (GitHub), highway example clip |
| resolution | 960 x 540 |
| frame rate | 29.97 fps |
| frames | 335 |
| duration | 11.18 s |
| camera | fixed, looking down a motorway from an overpass |
| traffic | two lanes groups coming **towards** the camera on the near carriageway; a separate carriageway behind the double guardrail on the right, driving **away** to the right |

## How to run

```bash
pip install -r requirements.txt
python run_pipeline.py
```

That is the whole thing: the annotated video, the crossing log, the evidence
crops, the contact sheet and `PROGRESS.md` are all written by that one command.
Tested with Python 3.11, OpenCV 5.0, NumPy 2.4, SciPy 1.17 and ffmpeg 7.
`ffmpeg` is used to compress the output; if it is missing the script falls back
to OpenCV's `mp4v` writer, which produces a bigger file.

Options:

```bash
python run_pipeline.py --video other_clip.mp4   # different footage
python run_pipeline.py --show                   # also open a preview window
python run_pipeline.py --max-frames 100         # stop early (debugging)
```

## Result on this clip

| Direction | Count |
|---|---|
| IN  – towards the camera (near carriageway) | 11 |
| OUT – away from the camera (far carriageway) | 3 |
| **total** | **14** |

335 frames processed in about 11 s (≈29 frames/s on one CPU core). The
annotated video is `output/traffic_counting.mp4`, 960x540, H.264, **1.28 MB**,
11.18 s.

These numbers are what the program produced. They have **not** been checked
against a hand count yet — the blank column in the report is for that.

## Pipeline

```
frame
  |
  |-- 1  MotionDetector.detect(frame)      -> list of Detection(bbox, centroid, area)
  |        blur -> MOG2 background model -> two shadow tests -> road-mask (ROI)
  |        -> opening/closing -> area + size + fill filter -> split merged blobs
  |
  |-- 2  VehicleTracker.update(detections)  -> confirmed Track objects
  |        Kalman predict -> Hungarian assignment with 70 px gate
  |        -> match / coast / birth / delete, tentative -> confirmed
  |
  |-- 3  LineCounter.update(tracks)         -> crossing events
  |        signed side of each line part -> hysteresis buffer -> count once
  |
  |-- 4  draw line + boxes + IDs + HUD, write frame to ffmpeg
  |
  `-- 5  per crossing: crop -> evidence/, row -> crossings.csv
```

### 1. Detection (`motion_detector.py`)

* **Background subtraction** with OpenCV's MOG2 (mixture of Gaussians per
  pixel). It learns the empty road and reports pixels that do not fit.
* **Shadow removal, twice.** MOG2 marks shadow pixels with the value 127 when
  it detects them, and those are thrown away by keeping only values ≥ 200.
  Anything MOG2 missed is caught by a second test: a shadow is the same colour
  as the road but darker, so a pixel is dropped when
  `0.45 < grey(frame)/grey(background) < 0.88` **and** its saturation is within
  50 units of the background. The saturation condition is what keeps dark cars
  (which are much more colourful than grey asphalt) from being erased.
* **Noise cleanup**: a 5x5 Gaussian blur before the background model, a 3x3
  opening to delete single-pixel speckle, and a 9x9 closing (twice) to glue
  together the pieces of one car body (the dark windscreen and the number plate
  break the mask apart).
* **Region of interest**: two polygons traced from the measured road boundaries
  of this clip, so grass, guardrails and trees are never examined. The top 45
  rows (horizon and treeline, the noisiest part of the frame) are also ignored.
* **Filters**: minimum area, minimum width/height, plausible aspect ratio, and
  a minimum "fill ratio" (area divided by bounding-box area) that rejects thin
  strips of shadow.
* **Merged vehicles**: when a blob is clearly wider than tall (≥ 1.55 : 1) and
  at least 130 px wide, two vehicles in neighbouring lanes have touched. The
  mask is projected onto the x axis, the deepest valley in the middle of the
  blob is the gap between the two bodies, and the blob is cut there. The same
  test works vertically for nose-to-tail vehicles.

### 2. Tracking (`road_tracker.py`)

* **Kalman filter**, one per track, constant velocity model. State is
  `[x, y, vx, vy]`, measurement is `[x, y]`. The matrices are written out with
  NumPy; `predict` and `correct` are the textbook equations.
* **Association**: every track is predicted first, then an assignment cost
  matrix of Euclidean distances between predicted and measured centroids is
  solved with the Hungarian algorithm (`scipy.optimize.linear_sum_assignment`).
  Pairs farther than 70 px are rejected — that gate is smaller than the gap
  between two lanes at the counting line, so an ID cannot jump to the car next
  to it.
* **Lifecycle**: a new detection starts a *tentative* track; after 3 matched
  frames it becomes *confirmed* and is drawn and counted. A track that is not
  detected coasts on its Kalman prediction for up to 6 frames (0.2 s) and is
  then deleted. IDs come from one counter that never goes back, so no ID is
  reused.
* **Duplicate suppression**: a detection that overlaps an existing track's box
  by more than 35 % does not start a new ID. Without this, a merged blob that
  splits in a later frame would be counted twice.

### 3. Counting (`counting.py`)

The virtual line has two straight parts, because the two carriageways lie at
very different angles in the image:

* **near**: horizontal, y = 302 px, x from 180 to 775. Traffic crosses it
  moving down the image, i.e. towards the camera → direction **IN**.
* **far**: vertical, x = 800 px, y from 60 to 150. The far carriageway is only
  a diagonal sliver at the edge of the frame; its traffic crosses this part
  moving right, away from the camera → direction **OUT**.

For each part the code computes the signed distance of the centroid from the
line, `s = (P - A) · n`. A crossing is counted when the sign of `s` changes
between two consecutive frames. To stop centroid jitter (2-4 px) from creating
fake crossings, the side is only updated when `|s| > 14 px`: inside that dead
band the previous side is remembered, so a vehicle has to genuinely commit to
one side and then reach the other. Each (track, line part) pair can be counted
exactly once, so a vehicle that wobbles on the line is still counted once.

### 4. Output video

Every frame is annotated with the line parts, one box per confirmed track with
its ID and direction, the centroid trail, and a panel showing the running IN,
OUT and total counts. Frames are piped into `ffmpeg` (libx264, CRF 26) at
960x540, which keeps the file at 1.28 MB.

## Files

| file | what it is |
|---|---|
| `settings.py` | every parameter, with the reason for its value |
| `motion_detector.py` | stage 1 |
| `road_tracker.py` | stage 2 (Kalman filter + tracker) |
| `counting.py` | stage 3 (line crossing logic) |
| `run_pipeline.py` | entry point, drawing, video, evidence, logs |
| `make_report.py` | builds `report.docx` from the run summary |
| `output/traffic_counting.mp4` | annotated result video |
| `output/summary.json` | machine-readable run summary |
| `evidence/crossings.csv` | one row per counted crossing |
| `evidence/cross_*.jpg` | crop around every counted vehicle |
| `evidence/contact_sheet.jpg` | all crops in one image |
| `evidence/suspicious_tracks.csv` | automatic sanity flags |
| `evidence/samples/annotated_*.jpg` | frames used in the report |
| `report.docx` | written report |
| `viva_notes.md` | short explanations for the viva |

## Counting parameters at a glance

| parameter | value | why |
|---|---|---|
| `BGS_HISTORY` | 200 frames | long enough to remember an empty road behind a passing car, short enough to follow the light change |
| `BGS_VAR_THRESHOLD` | 32 | ignores compression noise but still sees the small (60x45 px) far-carriageway cars |
| `SHADOW_DARK/LIGHT` | 0.45 / 0.88 | brightness range of a cast shadow |
| `SHADOW_SAT_TOL` | 50 | keeps coloured (dark) cars |
| `BLUR_KSIZE` | 5x5, sigma 1.2 | removes grain, keeps edges of a 60 px car |
| `OPEN_KSIZE` | 3x3 | speckle removal |
| `CLOSE_KSIZE` | 9x9, x2 | rejoins fragments of one car; lanes are 90-100 px apart so it cannot fuse two cars |
| `MIN_AREA` | 700 px² | noise after filtering is 150-700 px², a car at the line is 4000-9000 px² |
| `MAX_AREA` | 45000 px² | rejects the whole-frame mask of the first frames |
| `MIN_FILL_RATIO` | 0.33 | cars fill 0.46-0.79 of their box, shadow strips fill 0.01-0.03 |
| `SPLIT_MIN_ASPECT` | 1.55 | one car is about 0.6-1.4 wide per height, two side by side are > 1.55 |
| `KF_Q_VEL` | 0.8 | allows acceleration and lane changes without lag |
| `KF_R_MEAS` | 6 | measured centroid jitter of 2-4 px |
| `GATE_DIST` | 70 px | smaller than the 90-100 px lane spacing |
| `MAX_LOST` | 6 frames | 0.2 s: longer coasting caused ID swaps |
| `MIN_HITS` | 3 | kills one-frame ghost detections |
| `LINE_BUFFER_PX` | 14 | swallows centroid jitter near the line |

Full reasoning for each value is in the comments of `settings.py`.

## Verification that does not need eyes

* **Crossing log** (`evidence/crossings.csv`): track ID, line part, direction,
  frame number, timestamp, centroid, box, area, speed.
* **Crop per crossing** in `evidence/`, plus `contact_sheet.jpg`, so every
  counted vehicle can be checked by eye.
* **Automatic flags** (`evidence/suspicious_tracks.csv`): tracks shorter than
  12 frames that were counted, blob areas far from the median of the same line
  part, boxes touching the frame border, crossings at the very end of a line
  part, two IDs counted within 8 frames and 70 px of each other, and IDs born
  within 12 frames and 60 px of where another ID ended.
* **Independent sweep** with a deliberately more sensitive detector
  (min area 250 instead of 700) found the same 11 passages across the near line
  and the same 3 passages across the far line as the final run.

## Before / after a real bug fix

The first full run counted 17 vehicles (13 IN, 4 OUT). Three of those were
counted on empty road: thin strips of guardrail shadow that the road mask cuts
off, giving contours like a 235x294 px box with only 929 px² of foreground.
They were counted because the centroid of such a strip is where the line is.
Adding the fill-ratio filter removed exactly those three, giving 14 (11 IN,
3 OUT). No number was adjusted by hand; the filter is a property of the blob
shape, not of the answer.

## Known limitations on real footage

* **Heavy occlusion is not modelled.** MOG2 sees the union of two overlapping
  vehicles; the split heuristic recovers side-by-side vehicles but not one car
  hiding another. A car completely hidden behind a lorry is not counted.
* **The far carriageway is barely visible.** Only about 150 x 90 px of it is on
  screen, so its traffic gives 3 counts in 11 s. Vehicles that cross the line
  outside the visible sliver cannot be counted at all.
* **Distant vehicles are small.** Near the horizon a car is 400-500 px², below
  the 700 px² floor, so a track is born only once the vehicle is closer. This
  does not change the count (they are still tracked through the line) but the
  IDs start a little late.
* **Stationary vehicles are absorbed.** A car stopped on the line for more than
  the background history is slowly learned into the background and stops being
  detected; on this clip traffic always moves, so it did not occur.
* **Shadows at low sun.** The colour shadow test relies on the shadow being a
  darker version of the road. Very soft shadows (overcast) or shadows falling
  on grass inside the ROI would not be fully removed.
* **The line position is fixed for this camera.** Coordinates are fractions of
  the frame, but the road mask polygons were traced for this viewpoint; another
  camera would need new polygons and a new line.
* **Speed is in pixels per frame**, not km/h, because the camera is not
  calibrated.
* Counting depends on the tracker keeping one ID per vehicle. A vehicle that is
  missed for more than 6 frames (0.2 s) gets a new ID and would be counted
  twice; the suspicious-track log is designed to reveal exactly that, and it
  reported nothing on this clip.

## Credit

Video: **DeGirum PySDKExamples** on GitHub (highway / traffic example clip),
used only as test footage for this assignment.

Libraries: OpenCV (background subtraction, image processing), NumPy (Kalman
filter), SciPy (Hungarian algorithm), python-docx (report), ffmpeg (video
compression).
