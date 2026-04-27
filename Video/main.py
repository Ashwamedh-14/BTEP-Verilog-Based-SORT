from pathlib import Path
import sys
import warnings

import cv2
import numpy as np

VIDEO_PATH = "crop_output.mp4"
OUTPUT_VIDEO = "output_detected.mp4"
HW_MEASUREMENTS_CSV = "detections_hw.csv"
SORT_DETECTIONS_CSV = "detections_sort_input.csv"
CUSTOM_BBOX_CSV = "detections_bbox.csv"
REFERENCE_SORT_DET = "detections_for_sort_det.txt"
REFERENCE_SORT_TRACKS = "tracks_sort_reference.csv"

# Enable this to run Python SORT (from ../sort/sort.py) directly after detection.
RUN_REFERENCE_SORT = True

cap = cv2.VideoCapture(VIDEO_PATH)

# Get video properties
fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Video writer
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
out = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (width, height))

fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50)

hw_rows = []
sort_rows = []
bbox_rows = []

cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Detections", 800, 600)

frame_index = 1

while True:
    ret, frame = cap.read()
    if not ret:
        break

    fgmask = fgbg.apply(frame)
    fgmask = cv2.medianBlur(fgmask, 5)

    contours, _ = cv2.findContours(
        fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    max_area = 0
    best_box = None

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area > max_area:
            x, y, w, h = cv2.boundingRect(cnt)
            max_area = area
            best_box = (x, y, w, h)

    if best_box and max_area > 2000:
        x, y, w, h = best_box

        cx = int(round(x + (w / 2.0)))
        cy = int(round(y + (h / 2.0)))
        aspect_ratio = (w / h) if h else 0.0

        # Draw bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Optional: label
        cv2.putText(
            frame,
            "Detected Object",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        # Hardware feed format: [cx, cy, h, r]
        hw_rows.append(f"{frame_index},{cx},{cy},{h},{aspect_ratio:.6f}")

        # Custom SORT bbox feed: frame, x1, y1, x2, y2
        bbox_rows.append(f"{frame_index},{x},{y},{x + w},{y + h}")

        # SORT-style input for single-object comparison:
        # frame, id, x1, y1, w, h, score
        sort_rows.append(f"{frame_index},1,{x},{y},{w},{h},1.0")

    # Show live
    cv2.imshow("Detections", frame)

    # Save frame
    out.write(frame)

    # Press ESC to exit
    if cv2.waitKey(1) == 27:
        break

    frame_index += 1

cap.release()
out.release()
cv2.destroyAllWindows()

Path(HW_MEASUREMENTS_CSV).write_text(("\n".join(hw_rows) + "\n") if hw_rows else "", encoding="utf-8")
Path(SORT_DETECTIONS_CSV).write_text(("\n".join(sort_rows) + "\n") if sort_rows else "", encoding="utf-8")
Path(CUSTOM_BBOX_CSV).write_text((("\n".join(bbox_rows) + "\n") if bbox_rows else ""), encoding="utf-8")
Path(REFERENCE_SORT_DET).write_text((("\n".join(sort_rows) + "\n") if sort_rows else ""), encoding="utf-8")


def run_single_object_reference_sort(det_file: Path, output_file: Path, total_frames: int) -> tuple[bool, str]:
    """Run reference SORT on one stream and save MOT-style tracks."""
    sort_dir = Path(__file__).resolve().parents[1] / "sort"
    if not sort_dir.exists():
        return False, f"Reference SORT folder not found: {sort_dir}"

    try:
        sys.path.insert(0, str(sort_dir))
        # Suppress known invalid escape-sequence warning from filterpy on Python 3.13.
        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore",
                message=r"invalid escape sequence.*",
                category=SyntaxWarning,
                module=r"filterpy\.common\.helpers",
            )
            from sort import Sort  # type: ignore
    except Exception as exc:
        return False, f"Could not import sort.Sort ({exc})"

    dets_by_frame: dict[int, list[list[float]]] = {}
    for raw in det_file.read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        frame, _det_id, x, y, w, h, score = raw.split(",")
        frame_i = int(frame)
        x1 = float(x)
        y1 = float(y)
        x2 = x1 + float(w)
        y2 = y1 + float(h)
        dets_by_frame.setdefault(frame_i, []).append([x1, y1, x2, y2, float(score)])

    tracker = Sort(max_age=5, min_hits=1, iou_threshold=0.1)
    out_rows = []

    for frame in range(1, total_frames + 1):
        frame_dets = np.array(dets_by_frame.get(frame, []), dtype=float)
        if frame_dets.size == 0:
            frame_dets = np.empty((0, 5), dtype=float)

        tracks = tracker.update(frame_dets)
        if len(tracks) == 0:
            continue

        # Single-object mode: keep only the largest active track each frame.
        if len(tracks) > 1:
            areas = (tracks[:, 2] - tracks[:, 0]) * (tracks[:, 3] - tracks[:, 1])
            track = tracks[int(np.argmax(areas))]
        else:
            track = tracks[0]

        x1, y1, x2, y2, track_id = track
        width = x2 - x1
        height = y2 - y1
        out_rows.append(
            f"{frame},{int(track_id)},{x1:.2f},{y1:.2f},{width:.2f},{height:.2f},1,-1,-1,-1"
        )

    output_file.write_text(("\n".join(out_rows) + "\n") if out_rows else "", encoding="utf-8")
    return True, f"Saved reference SORT tracks: {output_file}"

print("Saved video:", OUTPUT_VIDEO)
print("Saved hardware feed:", HW_MEASUREMENTS_CSV)
print("Saved custom bbox feed:", CUSTOM_BBOX_CSV)
print("Saved SORT input:", SORT_DETECTIONS_CSV)
print("Saved reference SORT detections:", REFERENCE_SORT_DET)

if RUN_REFERENCE_SORT:
    ok, message = run_single_object_reference_sort(
        Path(REFERENCE_SORT_DET),
        Path(REFERENCE_SORT_TRACKS),
        frame_index - 1,
    )
    print(message)
    if not ok:
        print("Tip: install dependencies from ../sort/requirements.txt to run reference SORT.")
