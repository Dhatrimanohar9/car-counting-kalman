# PROGRESS

## Last run

- frames: 335 (11.18 s at 29.97 fps)
- IN (towards camera): 11
- OUT (away): 3
- total: 14
- video: `/Users/akuladhatrimanohar/Documents/VDA ASSIN-1/output/traffic_counting.mp4` (1.28 MB)
- suspicious flags: 0

## Stage checklist

- [x] 1 detection: MOG2 + two shadow tests + morphology + ROI + area/shape filter + merged-blob split
- [x] 2 tracking: constant-velocity Kalman filter, Hungarian assignment with 70 px gate, tentative -> confirmed, coasting, no ID reuse
- [x] 3 counting: two-part line, hysteresis buffer, once per track, IN / OUT separated
- [x] 4 output video (960x540, H.264, < 10 MB)
- [x] 5 evidence: crossings.csv, crop per crossing, contact sheet, automatic suspicious-track flags
- [x] 6 README.md
- [x] 7 report.docx
- [x] 8 viva_notes.md

## Next

Watch `output/traffic_counting.mp4` and fill in the manual counts in the report (Results section). Nothing in the code depends on those numbers.
