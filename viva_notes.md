# Viva notes

Short answers, in plain words, for the questions that usually come up.

---

## The big picture in five lines

1. Find what is moving in each frame (background subtraction).
2. Link the moving things from frame to frame so each vehicle keeps one ID
   (Kalman filter + Hungarian assignment).
3. Draw an imaginary line across the road.
4. Every time an ID goes from one side of the line to the other, add one to
   that direction's counter.
5. Write the counters on the video.

---

## Stage 1 – Detection

**What is background subtraction?**
The camera never moves, so the road pixels look almost the same in every frame.
We build a model of "what the road normally looks like" and then call every
pixel that does not match the model "foreground", i.e. something that moved.
The model is updated slowly, so a car that stays for a while eventually becomes
part of the background.

**Why MOG2 and not a single average image?**
MOG2 keeps a small mixture of Gaussians per pixel. That handles two things an
average cannot: a pixel that sometimes shows road and sometimes shows tree
shadow, and small lighting changes. It also can label shadows, which we use.

**Why is the blur there?**
Compressed video has block noise and the sensor has grain. A 5x5 Gaussian blur
removes most of it. Too much blur would smear the edge of a small distant car,
so the kernel stays small.

**How are shadows removed?**
Two ways:
1. MOG2 writes the value 127 for a pixel it thinks is shadow. Keeping only
   values ≥ 200 throws those away.
2. Anything it misses is caught by a colour test. A shadow is the same colour
   as the road, just darker: brightness drops to 45-88 % of the background but
   saturation stays about the same. So a pixel is dropped if
   `0.45 < grey/background_grey < 0.88` and the saturation difference is under
   50. A dark blue car is much more saturated than grey asphalt, so it survives
   the test.

**Why opening and then closing?**
Opening (erode then dilate) deletes small specks and thin hairs of noise.
Closing (dilate then erode) fills small holes: a car's windscreen and number
plate are dark and often break the mask into pieces. Closing is 9x9 because the
gaps inside a car are about 8 px. Lanes are 90-100 px apart, so 9 px of closing
cannot join two different cars.

**Why a region of interest?**
Grass, leaves, guardrails and the treeline produce a lot of false motion. The
road boundaries were measured once from the clip, and two polygons cover only
the asphalt. The top 45 rows are also dropped because the horizon and the
treetops there were the noisiest part of the frame.

**Why minimum area?**
Measured noise blobs are 150-700 px². A car at the counting line is
4000-9000 px², a far-carriageway car about 1900 px². 700 px² is a floor that
keeps vehicles and drops noise. A track also has to survive three frames before
it can be counted, so one lucky noise blob never counts.

**What is the fill ratio?**
Area divided by the area of the bounding box. A real car fills 46-79 % of its
box. A thin strip of shadow fills 1-3 % of a huge box. Adding this filter was
the fix for three false counts in the first run.

**How are touching vehicles handled?**
If one blob is at least 1.55 times wider than it is tall (and 130 px wide), it
is probably two cars side by side. We project the mask onto the x axis and look
for the deepest valley in the middle — that is the gap between the two bodies.
If the valley is below 55 % of the average column height, we cut there and get
two detections. The same thing is done vertically for cars one behind the other.
It is a heuristic: it works for side-by-side traffic, it cannot separate one car
completely hidden behind another.

---

## Stage 2 – Tracking

**Why a Kalman filter at all?**
The measured centroid jumps around by 2-4 px because the mask changes shape
(shadow, windscreen). The Kalman filter keeps a position and a velocity and
mixes the prediction with the measurement, so the track is smooth. It also
predicts where the vehicle will be next frame, which is what makes association
reliable: we match a detection to a *prediction*, not to the last noisy
position.

**What is the state?**
`[x, y, vx, vy]`. Position and speed in pixels per frame. Constant velocity
model: `x_new = x + vx`, `y_new = y + vy`.

**What do Q and R mean?**
* `R` = how much we distrust the measurement. Measured jitter is 2-4 px, so
  variance `R = 6`.
* `Q` = how much the motion is allowed to differ from constant velocity.
  `q_pos = 0.05` (the centroid cannot teleport), `q_vel = 0.8` (cars accelerate
  and change lanes).
* Large Q = trusts the measurement more (jumpy), large R = trusts the model
  more (laggy).

**What does the Kalman filter do when there is no detection?**
It just predicts. That is the "coasting" state: the box keeps moving at the
last known velocity for up to 6 frames. After that the track is deleted.

**Why 6 frames?**
At 30 fps, 0.2 s. In 0.2 s a car moves 20-60 px. If we coasted longer, the
prediction would sit where the *next* car has already arrived, and the ID could
be handed over to a different vehicle.

**How are detections matched to tracks?**
Cost matrix of distances between each predicted position and each detection,
solved with the Hungarian algorithm (optimal one-to-one assignment). Pairs
further than 70 px are rejected. The gate is smaller than the gap between two
lanes at the line (90-100 px), so an ID cannot jump sideways to the car in the
next lane.

**Why Hungarian and not nearest neighbour?**
With several cars, nearest neighbour can give two tracks the same detection.
The Hungarian algorithm always produces the cheapest overall one-to-one
assignment, which is what we want.

**Tentative vs confirmed?**
A new track is tentative and is not drawn or counted. After 3 matched frames it
becomes confirmed. That removes one-frame ghost detections caused by shadows
flickering.

**Can an ID be reused?**
No. IDs come from one counter that only increases. If a track dies, its number
is never given to another vehicle.

**Why the "duplicate" check?**
If a merged blob splits in a later frame, the detector suddenly reports two
blobs where the tracker already has one track. The new blob overlaps the
existing box by more than 35 %, so it is treated as part of the same vehicle
instead of starting a second ID. Without it, one car could be counted twice.

---

## Stage 3 – Counting

**How is a crossing detected?**
The line part is a segment from A to B with normal `n`. The signed distance of
the centroid is `s = (P - A) · n`. If `s` changes sign between two consecutive
frames, the vehicle crossed.

**Why a buffer of 14 px?**
The centroid jitters. A car driving exactly along the line would flip the sign
many times. So the side is only updated when `|s| > 14`: inside the dead band
the previous side is kept. The vehicle must commit to one side and then reach
the other. A fast car that jumps the whole band in one frame is still counted,
because it is the state that changes, not a single frame.

**How is it counted only once?**
Every (track ID, line part) pair is remembered after being counted, so it can
never be counted a second time, no matter how much it wobbles near the line.

**How are the directions separated?**
The sign of the crossing tells the direction. Each line part has a label for
the positive side and the negative side. On the near line, moving down the
image (towards the camera) is IN; on the far line, moving right (away) is OUT.

**Why does the line have two parts?**
Because the two carriageways are at very different angles in the image. The
near carriageway runs down the frame, so a horizontal line at y = 302 is
perpendicular to it. The far carriageway is a diagonal sliver in the corner, so
a vertical part at x = 800 sits on its road surface. One straight line cannot
be well placed on both.

**Why y = 302?**
At that height the near carriageway is 600 px wide, the lane markings are
clear, cars are 90-130 px wide and fully inside the frame when they are
counted. Higher up they are too small, lower down they are cut by the bottom
edge of the frame.

---

## Stage 4 – Output video

Frames are written with OpenCV drawing calls, then pushed through ffmpeg with
libx264 at CRF 26. 960x540 is 540p, which is inside the 720p limit, and the
whole 11 s clip comes to 1.28 MB.

---

## Results and honesty

* System output on this clip: **11 IN, 3 OUT, total 14**.
* 14 crossing crops and a contact sheet are in `evidence/` so each one can be
  checked by eye.
* `crossings.csv` has the ID, line part, direction, frame, time, position, box,
  area and speed of every count.
* `suspicious_tracks.csv` lists anything that looks fragile (very short tracks,
  odd blob sizes, boxes at the frame border, two IDs counted close together,
  IDs born where another ended). On the final run it was empty.
* The manual count is **not** filled in by me. The numbers above are what the
  program produced, and the manual column in the report is left blank on
  purpose.

**The bug that was found and fixed:** the first run counted 17 (13 IN, 4 OUT),
but three of those boxes sat on empty road — they were thin strips of guardrail
shadow cut by the edge of the road mask (929 px² of foreground inside a
235x294 px box). The fill-ratio filter removes them, and the count became 14.
Nothing was tuned to make a number match.

---

## Questions about parameters, one line each

| parameter | short answer |
|---|---|
| `BGS_HISTORY = 200` | how many frames the background model averages over (~6.7 s) |
| `BGS_VAR_THRESHOLD = 32` | how different a pixel must be to count as foreground |
| `SHADOW_DARK / LIGHT = 0.45 / 0.88` | brightness range that means "this is a shadow" |
| `SHADOW_SAT_TOL = 50` | how much the colour may change before it is a real object, not a shadow |
| `BLUR_KSIZE = 5x5, sigma 1.2` | light smoothing of sensor and compression noise |
| `OPEN_KSIZE = 3x3` | removes single-pixel speckle |
| `CLOSE_KSIZE = 9x9, 2x` | fills the gaps inside a car body |
| `MIN_AREA = 700` | smallest blob that can be a vehicle (noise is below this) |
| `MAX_AREA = 45000` | biggest blob accepted; rejects the whole-frame mask of frame 0 |
| `MIN_WIDTH / HEIGHT = 26 / 16` | a blob must be at least this big in both directions |
| `MIN/MAX_ASPECT = 0.25 / 4` | plausible shape of a vehicle seen from this angle |
| `MIN_FILL_RATIO = 0.33` | a real vehicle fills at least a third of its box |
| `SPLIT_MIN_ASPECT = 1.55` | wider than this means two vehicles touching |
| `ROI_POLYGONS` | the two road surfaces, traced from the clip |
| `ROI_TOP_IGNORE_PX = 45` | drops the noisy horizon and treeline |
| `KF_Q_POS = 0.05` | position cannot teleport |
| `KF_Q_VEL = 0.8` | allows acceleration and lane changes |
| `KF_R_MEAS = 6` | measured centroid jitter of 2-4 px |
| `KF_P_POS / P_VEL = 25 / 400` | initial uncertainty: position known, speed unknown |
| `GATE_DIST = 70` | maximum distance for a match, smaller than lane spacing |
| `MAX_LOST = 6` | frames a track may coast before it is deleted |
| `MIN_HITS = 3` | frames before a track is confirmed |
| `DUP_IOU = 0.35` | overlap that counts as "same vehicle, not a new one" |
| `LINE_BUFFER_PX = 14` | dead band that absorbs centroid jitter at the line |
| `OUT_CRF = 26` | H.264 quality/size trade-off for the output video |

---

## Likely follow-up questions

**Why not YOLO or a pre-trained detector?**
The assignment asks for motion detection, and background subtraction needs no
training data. A detector would also work, but it would answer a different
question and would need a model file. Every line here is classical computer
vision that can be explained from first principles.

**What happens with a moving camera?**
Background subtraction breaks, because the whole image is foreground. You would
need ego-motion compensation or a learned detector.

**How would you improve the count?**
Use a detector for occluded vehicles, keep a per-track appearance descriptor so
the ID survives longer occlusions, calibrate the image so speeds can be given in
km/h, and count over a longer recording so the numbers are statistically
meaningful.

**Why is the OUT count so small?**
Only about 150 x 90 px of the far carriageway is visible in the corner of the
frame. Most vehicles on that road never appear. That is a property of the
footage, not of the algorithm.
