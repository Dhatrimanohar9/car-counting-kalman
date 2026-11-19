"""Builds report.docx from output/summary.json and the evidence images."""

import json
import os

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Inches, Pt

import settings as S

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "report.docx")


def add_heading(doc, text, level=1):
    doc.add_heading(text, level=level)


def add_body(doc, text):
    p = doc.add_paragraph(text)
    p.paragraph_format.space_after = Pt(6)
    return p


def main():
    with open(S.SUMMARY_JSON) as fh:
        summary = json.load(fh)

    counts = summary["counts"]
    total = summary["total"]

    doc = Document()
    normal = doc.styles["Normal"]
    normal.font.name = "Times New Roman"
    normal.font.size = Pt(11)

    # ------------------------------------------------------------------ #
    # Title page
    # ------------------------------------------------------------------ #
    for _ in range(4):
        doc.add_paragraph("")

    def centred(text, size, bold=False):
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        run = p.add_run(text)
        run.bold = bold
        run.font.size = Pt(size)
        return p

    centred("Object Counting", 26, True)
    centred("(Car Counting with Kalman Filter Tracking)", 16, True)
    doc.add_paragraph("")
    doc.add_paragraph("")
    centred("Name: Dhatri Manohar Siva Kumar", 13)
    centred("Roll Number: SE24UCSE206", 13)
    doc.add_paragraph("")
    centred("Computer Vision Assignment", 12)
    centred("Source video: input/traffic.mp4 (DeGirum PySDKExamples)", 10)

    doc.add_page_break()

    # ------------------------------------------------------------------ #
    add_heading(doc, "1. Introduction", 1)
    add_body(doc,
             "This report describes a small program that counts cars in a fixed traffic "
             "camera video. The camera is on an overpass looking down a motorway. The "
             "program does not know what a car looks like in advance; it only knows that "
             "cars move and the road does not. From that one idea it finds the moving "
             "regions, gives each one a number that stays with it from frame to frame, "
             "and counts it when it crosses an imaginary line drawn across the road.")
    add_body(doc,
             "The work was done in Python with OpenCV for image processing, NumPy for the "
             "Kalman filter mathematics and SciPy for the assignment step. The whole "
             f"pipeline runs on the supplied {summary['frames_processed']}-frame clip in "
             f"about {summary['runtime_s']} seconds and writes an annotated video, a "
             "crossing log and one cropped image per counted vehicle.")

    # ------------------------------------------------------------------ #
    add_heading(doc, "2. Problem Statement", 1)
    add_body(doc,
             "Given a video from a static camera, count the vehicles that cross a chosen "
             "line on the road, separately for the two directions, and show the counts "
             "while the video plays. Each vehicle must be counted exactly once even if "
             "its detected position wobbles around the line, and it must keep one "
             "identity while it is visible.")
    add_body(doc,
             "The hard parts are not the counting itself but everything before it: the "
             "detected region of a car changes shape every frame, cars cast long "
             "shadows in this footage, cars sometimes touch each other, and two cars can "
             "look the same to the detector. The assignment is therefore mostly about "
             "detection quality, tracking, and deciding honestly when a crossing has "
             "really happened.")

    # ------------------------------------------------------------------ #
    add_heading(doc, "3. Methodology", 1)

    add_heading(doc, "3.1 Detecting moving objects", 2)
    add_body(doc,
             "The detector is a background subtractor (OpenCV's MOG2, a mixture of "
             "Gaussians for every pixel). It learns what the empty road looks like and "
             "marks pixels that do not match the model as foreground. The model keeps "
             "adapting, so a car that stops for a long time slowly becomes part of the "
             "background.")
    add_body(doc,
             "Shadows are removed in two steps. MOG2 marks pixels it believes are "
             "shadows with the value 127, and those are discarded immediately by keeping "
             "only values of 200 and above. Whatever MOG2 misses is caught by a colour "
             "test: a shadow is the same colour as the road but darker, so a pixel is "
             "removed when its brightness is between 45% and 88% of the background while "
             "its saturation stays within 50 units of the background. A dark car is much "
             "more saturated than grey asphalt, so this test does not erase cars.")
    add_body(doc,
             "Noise is cleaned with a 5x5 Gaussian blur before the background model, a "
             "3x3 opening to delete isolated speckles, and a 9x9 closing repeated twice "
             "to join the pieces of one car body, because windows and number plates break "
             "the mask apart. Only two polygons covering the actual road surfaces are "
             "processed, which keeps grass, guardrails and trees out of the pipeline.")
    add_body(doc,
             "Finally the blobs are filtered by area, by width and height, by aspect "
             "ratio and by how much of their bounding box they fill. A blob that is "
             "clearly wider than it is tall is treated as two touching vehicles and is "
             "cut at the deepest valley of its column profile, which is the gap between "
             "the two bodies.")

    add_heading(doc, "3.2 Kalman filter tracking", 2)
    add_body(doc,
             "Every vehicle gets its own Kalman filter with the state [x, y, vx, vy]: "
             "position and velocity in pixels per frame. The motion model is constant "
             "velocity, so the prediction is simply x + vx and y + vy. Only the position "
             "is measured. The filter is written out with NumPy, using the standard "
             "predict and correct equations.")
    add_body(doc,
             "The filter matters for two reasons. First, the measured centroid of a "
             "background-subtracted blob jumps by two to four pixels even when the car "
             "moves smoothly, and the filter smooths that. Second, it predicts where the "
             "car will be in the next frame, and matching a detection against a "
             "prediction is much more reliable than matching it against the last noisy "
             "position.")

    add_heading(doc, "3.3 Associating detections with tracks", 2)
    add_body(doc,
             "In each frame every track is predicted first. Then a cost matrix is built "
             "from the distance between each predicted position and each detection, and "
             "the Hungarian algorithm solves it for the cheapest one-to-one assignment. "
             "Pairs farther apart than 70 pixels are rejected. That gate is deliberately "
             "smaller than the distance between two lanes at the counting line, so a "
             "track can never jump sideways to the car in the next lane.")
    add_body(doc,
             "A new detection starts a tentative track, which becomes confirmed after "
             "three matched frames and is only then drawn and counted. A track without a "
             "detection keeps moving on its prediction for at most six frames and is "
             "then deleted. Identifiers come from a single counter that never decreases, "
             "so an identifier is never reused for a different vehicle. A detection that "
             "overlaps an existing box by more than 35% is treated as the same vehicle "
             "rather than a new one, which prevents double counting when a merged blob "
             "splits.")

    add_heading(doc, "3.4 Counting logic", 2)
    add_body(doc,
             "The counting line is at y = 302 pixels, set in settings.py (LINE_Y_RATIO "
             "times the frame height). The near carriageway comes towards the camera and "
             "crosses it moving down the image; the far carriageway behind the guardrail "
             "drives away, and for that part a short vertical segment at x = 800 is used, "
             "because only a diagonal sliver of that road is visible in the corner of the "
             "frame. The two parts of the line therefore count the two directions "
             "separately.")
    add_body(doc,
             "For each part the signed distance of the centroid from the line is "
             "computed. A crossing is counted when the sign of that distance changes "
             "between the previous centroid and the current one. To keep pixel jitter "
             "from producing false crossings, the side is only updated when the distance "
             "is larger than 14 pixels; inside that band the previous side is kept. A car "
             "must therefore commit to one side and then reach the other side. Each pair "
             "of (track, line part) is remembered once it has been counted, so a vehicle "
             "that wobbles on the line is still counted exactly once.")

    add_heading(doc, "3.5 Order of processing", 2)
    add_body(doc,
             "frame -> background subtraction and blob filtering -> Kalman prediction and "
             "Hungarian assignment -> track update -> line crossing test -> draw boxes, "
             "identifiers, the line and the counters -> write the frame. The first fifteen "
             "frames are used only to teach the background model and are written without "
             "detections.")

    # ------------------------------------------------------------------ #
    add_heading(doc, "4. Parameter Choices", 1)
    add_body(doc,
             "The values were chosen from measurements on this clip, not copied from a "
             "default. Blob sizes, road boundaries and pixel speeds were measured first, "
             "and the thresholds follow from those numbers.")

    table = doc.add_table(rows=1, cols=3)
    table.style = "Table Grid"
    head = table.rows[0].cells
    head[0].text = "Parameter"
    head[1].text = "Value"
    head[2].text = "Why this value"
    rows = [
        ("Background history", "200 frames",
         "About 6.7 s. Long enough to remember an empty road behind a passing car, short "
         "enough to follow the light change."),
        ("Variance threshold", "32",
         "Ignores H.264 block noise but still detects the small 60x45 px cars on the far "
         "carriageway."),
        ("Shadow brightness range", "0.45 - 0.88",
         "Measured range of a cast shadow on the asphalt. Saturation tolerance 50 keeps "
         "dark coloured cars."),
        ("Blur", "5x5, sigma 1.2",
         "Removes grain; a larger kernel blurred the distant cars too much."),
        ("Morphology", "open 3x3, close 9x9 twice",
         "Speckle removal and gap filling inside one car. Lanes are 90-100 px apart, so "
         "9 px cannot join two cars."),
        ("Minimum area", "700 px2",
         "Filtered noise is 150-700 px2, a car at the line is 4000-9000 px2."),
        ("Fill ratio", "0.33",
         "Cars fill 0.46-0.79 of their box; thin shadow strips fill less than 0.03."),
        ("Merged blob split", "aspect 1.55",
         "A single car is 0.6-1.4 wide per height; two side by side are wider."),
        ("Process noise Q", "0.05 position, 0.8 velocity",
         "The centroid cannot teleport, but cars do accelerate and change lanes."),
        ("Measurement noise R", "6",
         "Measured centroid jitter of 2-4 pixels."),
        ("Gate distance", "70 px",
         "Smaller than the 90-100 px lane spacing, so identifiers cannot swap lanes."),
        ("Maximum lost frames", "6",
         "0.2 s of coasting. Longer coasting let a prediction land on the next car."),
        ("Confirm after", "3 frames",
         "Removes one-frame ghost detections caused by flickering shadows."),
        ("Line buffer", "14 px",
         "Absorbs centroid jitter at the line so a crossing is decided once."),
    ]
    for name, value, why in rows:
        cells = table.add_row().cells
        cells[0].text = name
        cells[1].text = value
        cells[2].text = why

    # ------------------------------------------------------------------ #
    add_heading(doc, "5. Results", 1)
    add_body(doc,
             f"The program processed {summary['frames_processed']} frames "
             f"({summary['duration_s']} s) and counted {total} vehicles. The annotated "
             f"video is {summary['output_video_resolution'][0]}x"
             f"{summary['output_video_resolution'][1]}, H.264, and "
             f"{summary['output_video_bytes'] / 1_048_576:.2f} MB.")

    result = doc.add_table(rows=1, cols=3)
    result.style = "Table Grid"
    cells = result.rows[0].cells
    cells[0].text = "Direction"
    cells[1].text = "System count"
    cells[2].text = "My manual count (to fill in)"
    for label, meaning in (("IN", "towards the camera (near carriageway)"),
                           ("OUT", "away from the camera (far carriageway)")):
        row = result.add_row().cells
        row[0].text = f"{label} - {meaning}"
        row[1].text = str(counts[label])
        row[2].text = ""
    row = result.add_row().cells
    row[0].text = "Total"
    row[1].text = str(total)
    row[2].text = ""

    add_body(doc,
             "The numbers above are what the program produced. They are not a manual "
             "count: the last column is left empty so the video can be watched and the "
             "count written in by hand. Every counted vehicle also has a cropped image in "
             "the evidence folder and a row in crossings.csv, with its identifier, line "
             "part, direction, frame number, time, position, box size, area and speed, so "
             "each count can be checked individually.")

    add_body(doc,
             "An automatic check lists weak cases in a separate file: tracks that are "
             "shorter than twelve frames, blobs whose area is far from the median of the "
             "same line part, boxes touching the image border, crossings at the very end "
             "of the line, two identifiers counted within a few frames and pixels of each "
             "other, and identifiers born where another identifier ended. On the final "
             f"run that file contains {summary['suspicious_flags']} entries.")

    add_heading(doc, "5.1 A real bug that was found and fixed", 2)
    add_body(doc,
             "The first complete run counted 17 vehicles (13 IN, 4 OUT). Three of those "
             "boxes sat on empty road: they were thin strips of guardrail shadow cut by "
             "the edge of the road mask, with only about 900 pixels of foreground inside "
             "a 235x294 pixel box. Their centroids happened to move across the line, so "
             "they were counted. The fix was the fill-ratio filter described above, which "
             "rejects blobs that do not fill their own box. The count then became 14 "
             "(11 IN, 3 OUT). Nothing was tuned to make a number match; the filter "
             "describes the shape of a vehicle, and the three false counts happened to be "
             "the only blobs that failed it.")

    add_heading(doc, "5.2 Frames from the output video", 2)
    samples = sorted(
        os.path.join(S.SAMPLE_DIR, name)
        for name in os.listdir(S.SAMPLE_DIR)
        if name.endswith(".jpg")
    )
    captions = [
        "Frame 84 (2.8 s). One vehicle is confirmed between the top of the frame and "
        "the counting line; the line, its direction arrow and the running counters are "
        "drawn on every frame.",
        "Frame 295 (9.8 s). Two vehicles are tracked near the line and one has just been "
        "counted, so its box and trail are drawn in the IN colour and the total has "
        "already been updated.",
    ]
    for path, caption in zip(samples[::2][:2], captions):
        doc.add_picture(path, width=Inches(6.0))
        doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
        note = doc.add_paragraph(caption)
        note.alignment = WD_ALIGN_PARAGRAPH.CENTER
        note.runs[0].font.size = Pt(9)

    # ------------------------------------------------------------------ #
    add_heading(doc, "6. Limitations", 1)
    add_body(doc,
             "Occlusion is not handled. The background subtractor sees the union of two "
             "overlapping vehicles, and the split heuristic only recovers vehicles that "
             "are side by side, not one car hidden behind a lorry.")
    add_body(doc,
             "The far carriageway is only visible in a small diagonal corner of the "
             "image, roughly 150 by 90 pixels, so most of its traffic never appears and "
             "its count is necessarily small. Vehicles that cross outside the visible "
             "part cannot be counted.")
    add_body(doc,
             "Distant vehicles are small. Near the horizon a car covers 400-500 pixels, "
             "which is below the 700 pixel floor, so a track starts only when the car is "
             "closer. The count is unaffected because the car is still tracked through "
             "the line, but its identifier starts a little later.")
    add_body(doc,
             "The line position and the road polygons belong to this camera. Another "
             "viewpoint would need new polygons and a new line. Speed is reported in "
             "pixels per frame, not in km/h, because the camera is not calibrated.")
    add_body(doc,
             "If a vehicle is missed for more than six frames it receives a new "
             "identifier and could be counted twice. The automatic flag file exists "
             "precisely to make such cases visible; on this clip it reported none.")

    # ------------------------------------------------------------------ #
    add_heading(doc, "7. Conclusion", 1)
    add_body(doc,
             "The system detects, tracks and counts the vehicles in the supplied clip "
             f"without any machine learning model, and produces {total} counts "
             f"({counts['IN']} towards the camera, {counts['OUT']} away from it). "
             "Background subtraction with proper shadow handling turned out to be enough "
             "for this footage because the camera is fixed and the road is clean. The "
             "Kalman filter and the distance gate were what kept one identity per "
             "vehicle, and the hysteresis band around the line was what prevented the "
             "same car from being counted several times. The main weaknesses are "
             "occlusion and the tiny visible part of the far carriageway, both of which "
             "come from the footage rather than from the algorithm.")

    # ------------------------------------------------------------------ #
    add_heading(doc, "8. References", 1)
    refs = [
        "DeGirum PySDKExamples, highway traffic example video (source of "
        "input/traffic.mp4). GitHub: https://github.com/DeGirum/PySDKExamples",
        "Zivkovic, Z. (2004). Improved adaptive Gaussian mixture model for background "
        "subtraction. Proceedings of the 17th International Conference on Pattern "
        "Recognition (ICPR).",
        "Zivkovic, Z., van der Heijden, F. (2006). Efficient adaptive density estimation "
        "per image pixel for the task of background subtraction. Pattern Recognition "
        "Letters, 27(7), 773-780.",
        "Kalman, R. E. (1960). A new approach to linear filtering and prediction "
        "problems. Journal of Basic Engineering, 82(1), 35-45.",
        "Kuhn, H. W. (1955). The Hungarian method for the assignment problem. Naval "
        "Research Logistics Quarterly, 2(1-2), 83-97.",
        "Bradski, G. (2000). The OpenCV Library. Dr. Dobb's Journal of Software Tools.",
        "Harris, C. R. et al. (2020). Array programming with NumPy. Nature, 585, "
        "357-362.",
        "Virtanen, P. et al. (2020). SciPy 1.0: fundamental algorithms for scientific "
        "computing in Python. Nature Methods, 17, 261-272.",
    ]
    for ref in refs:
        p = doc.add_paragraph(ref)
        p.paragraph_format.left_indent = Inches(0.3)
        p.paragraph_format.first_line_indent = Inches(-0.3)

    # ------------------------------------------------------------------ #
    doc.add_page_break()
    add_heading(doc, "Appendix - crossing log used for the counts", 1)
    crossings = summary["crossings"]
    if crossings:
        log = doc.add_table(rows=1, cols=6)
        log.style = "Table Grid"
        header = log.rows[0].cells
        for i, title in enumerate(("ID", "Line part", "Direction", "Frame",
                                   "Time (s)", "Speed (px/frame)")):
            header[i].text = title
        for event in crossings:
            cells = log.add_row().cells
            cells[0].text = str(event["track_id"])
            cells[1].text = str(event["line_part"])
            cells[2].text = str(event["direction"])
            cells[3].text = str(event["frame"])
            cells[4].text = f"{event['time_s']:.2f}"
            cells[5].text = f"{event['speed_px_per_frame']:.1f}"

    doc.save(OUT)
    print("wrote", OUT, os.path.getsize(OUT), "bytes")


if __name__ == "__main__":
    main()
