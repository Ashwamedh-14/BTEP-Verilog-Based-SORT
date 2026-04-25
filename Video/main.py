from pathlib import Path

import cv2

VIDEO_PATH = "crop_output.mp4"
OUTPUT_VIDEO = "output_detected.mp4"
HW_MEASUREMENTS_CSV = "detections_hw.csv"
SORT_DETECTIONS_CSV = "detections_sort_input.csv"

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

print("Saved video:", OUTPUT_VIDEO)
print("Saved hardware feed:", HW_MEASUREMENTS_CSV)
print("Saved SORT input:", SORT_DETECTIONS_CSV)
