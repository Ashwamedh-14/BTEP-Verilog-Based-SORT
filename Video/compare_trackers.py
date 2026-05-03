from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, median
from typing import Iterable


@dataclass
class Box:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def w(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def h(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def cx(self) -> float:
        return self.x1 + self.w / 2.0

    @property
    def cy(self) -> float:
        return self.y1 + self.h / 2.0


def parse_numeric_row(raw: str) -> list[float] | None:
    line = raw.strip()
    if not line:
        return None

    parts = [p.strip() for p in line.split(",")]
    try:
        return [float(p) for p in parts]
    except ValueError:
        # Skip headers or non-numeric lines from simulator logs.
        return None


def iou(a: Box, b: Box) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)

    iw = max(0.0, x2 - x1)
    ih = max(0.0, y2 - y1)
    inter = iw * ih

    ua = a.w * a.h
    ub = b.w * b.h
    union = ua + ub - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def load_reference_tracks(path: Path) -> dict[int, Box]:
    """
    Reference format (Python SORT output):
      frame,track_id,x,y,w,h,1,-1,-1,-1
    """
    by_frame: dict[int, list[Box]] = {}

    for raw in path.read_text(encoding="utf-8").splitlines():
        vals = parse_numeric_row(raw)
        if vals is None or len(vals) < 6:
            continue

        frame = int(round(vals[0]))
        x, y, w, h = vals[2], vals[3], vals[4], vals[5]
        box = Box(x1=x, y1=y, x2=x + w, y2=y + h)
        by_frame.setdefault(frame, []).append(box)

    # Keep one object per frame (largest area), matching single-object comparison.
    result: dict[int, Box] = {}
    for frame, boxes in by_frame.items():
        result[frame] = max(boxes, key=lambda b: b.w * b.h)

    return result


def to_box_from_custom(vals: list[float], fmt: str) -> tuple[int, Box] | None:
    if fmt == "mot":
        # frame,id,x,y,w,h,...
        if len(vals) < 6:
            return None
        frame = int(round(vals[0]))
        x, y, w, h = vals[2], vals[3], vals[4], vals[5]
        return frame, Box(x1=x, y1=y, x2=x + w, y2=y + h)

    if fmt == "xyxy":
        # frame,x1,y1,x2,y2
        if len(vals) < 5:
            return None
        frame = int(round(vals[0]))
        x1, y1, x2, y2 = vals[1], vals[2], vals[3], vals[4]
        return frame, Box(x1=x1, y1=y1, x2=x2, y2=y2)

    if fmt == "cxcywh":
        # frame,cx,cy,w,h
        if len(vals) < 5:
            return None
        frame = int(round(vals[0]))
        cx, cy, w, h = vals[1], vals[2], vals[3], vals[4]
        return frame, Box(x1=cx - w / 2.0, y1=cy - h / 2.0, x2=cx + w / 2.0, y2=cy + h / 2.0)

    if fmt == "cxcyhr":
        # frame,cx,cy,h,r where r = w/h
        if len(vals) < 5:
            return None
        frame = int(round(vals[0]))
        cx, cy, h, r = vals[1], vals[2], vals[3], vals[4]
        w = h * r
        return frame, Box(x1=cx - w / 2.0, y1=cy - h / 2.0, x2=cx + w / 2.0, y2=cy + h / 2.0)

    if fmt == "cxcysr":
        # frame,cx,cy,s,r where s = area and r = w/h
        if len(vals) < 5:
            return None
        frame = int(round(vals[0]))
        cx, cy, s, r = vals[1], vals[2], vals[3], vals[4]
        if s <= 0 or r <= 0:
            return frame, Box(cx, cy, cx, cy)
        w = math.sqrt(s * r)
        h = s / w if w > 0 else 0.0
        return frame, Box(x1=cx - w / 2.0, y1=cy - h / 2.0, x2=cx + w / 2.0, y2=cy + h / 2.0)

    raise ValueError(f"Unsupported custom format: {fmt}")


def load_custom_tracks(
    path: Path,
    fmt: str,
    frame_offset: int = 0,
    coord_scale: float = 1.0,
) -> dict[int, Box]:
    by_frame: dict[int, list[Box]] = {}

    for raw in path.read_text(encoding="utf-8").splitlines():
        vals = parse_numeric_row(raw)
        if vals is None:
            continue

        parsed = to_box_from_custom(vals, fmt)
        if parsed is None:
            continue

        frame, box = parsed
        frame += frame_offset
        if coord_scale != 1.0:
            box = Box(
                x1=box.x1 * coord_scale,
                y1=box.y1 * coord_scale,
                x2=box.x2 * coord_scale,
                y2=box.y2 * coord_scale,
            )
        by_frame.setdefault(frame, []).append(box)

    # Keep largest if multiple entries exist in a frame.
    result: dict[int, Box] = {}
    for frame, boxes in by_frame.items():
        result[frame] = max(boxes, key=lambda b: b.w * b.h)

    return result


def compute_metrics(reference: dict[int, Box], custom: dict[int, Box]) -> list[dict[str, float | int]]:
    rows: list[dict[str, float | int]] = []

    shared_frames = sorted(set(reference).intersection(custom))
    for frame in shared_frames:
        ref = reference[frame]
        cur = custom[frame]

        dx = cur.cx - ref.cx
        dy = cur.cy - ref.cy
        center_err = math.hypot(dx, dy)
        dw = cur.w - ref.w
        dh = cur.h - ref.h
        frame_iou = iou(ref, cur)

        rows.append(
            {
                "frame": frame,
                "ref_x1": ref.x1,
                "ref_y1": ref.y1,
                "ref_x2": ref.x2,
                "ref_y2": ref.y2,
                "custom_x1": cur.x1,
                "custom_y1": cur.y1,
                "custom_x2": cur.x2,
                "custom_y2": cur.y2,
                "iou": frame_iou,
                "center_err": center_err,
                "dx": dx,
                "dy": dy,
                "dw": dw,
                "dh": dh,
            }
        )

    return rows


def summarize(metrics: Iterable[dict[str, float | int]]) -> dict[str, float]:
    rows = list(metrics)
    if not rows:
        return {
            "frames_compared": 0.0,
            "mean_iou": 0.0,
            "median_iou": 0.0,
            "min_iou": 0.0,
            "mean_center_err": 0.0,
            "median_center_err": 0.0,
            "max_center_err": 0.0,
            "mean_abs_dw": 0.0,
            "mean_abs_dh": 0.0,
        }

    ious = [float(r["iou"]) for r in rows]
    center_errs = [float(r["center_err"]) for r in rows]
    abs_dw = [abs(float(r["dw"])) for r in rows]
    abs_dh = [abs(float(r["dh"])) for r in rows]

    return {
        "frames_compared": float(len(rows)),
        "mean_iou": mean(ious),
        "median_iou": median(ious),
        "min_iou": min(ious),
        "mean_center_err": mean(center_errs),
        "median_center_err": median(center_errs),
        "max_center_err": max(center_errs),
        "mean_abs_dw": mean(abs_dw),
        "mean_abs_dh": mean(abs_dh),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare reference Python SORT tracks with custom SORT (.vvp flow) output."
    )
    parser.add_argument(
        "--reference",
        type=Path,
        default=Path("tracks_sort_reference.csv"),
        help="Reference tracks from Python SORT (default: tracks_sort_reference.csv).",
    )
    parser.add_argument(
        "--custom",
        type=Path,
        default=Path("detections_sort.csv"),
        help="Custom SORT output exported from your .vvp flow.",
    )
    parser.add_argument(
        "--custom-format",
        choices=["mot", "xyxy", "cxcywh", "cxcyhr", "cxcysr"],
        default="cxcyhr",
        help=(
            "Format of custom file. "
            "For detections_sort.csv and detections_hw.csv use cxcyhr. For detections_sort_input.csv use mot."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("comparison_details.csv"),
        help="Frame-wise comparison CSV output.",
    )
    parser.add_argument(
        "--custom-frame-offset",
        type=int,
        default=0,
        help="Integer offset applied to custom frame index before matching.",
    )
    parser.add_argument(
        "--custom-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to custom coordinates before comparison.",
    )

    args = parser.parse_args()

    if not args.reference.exists():
        raise FileNotFoundError(f"Reference file not found: {args.reference}")
    if not args.custom.exists():
        raise FileNotFoundError(f"Custom file not found: {args.custom}")

    reference = load_reference_tracks(args.reference)
    custom = load_custom_tracks(
        args.custom,
        args.custom_format,
        frame_offset=args.custom_frame_offset,
        coord_scale=args.custom_scale,
    )

    metrics = compute_metrics(reference, custom)
    summary = summarize(metrics)

    with args.output.open("w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "frame",
            "ref_x1",
            "ref_y1",
            "ref_x2",
            "ref_y2",
            "custom_x1",
            "custom_y1",
            "custom_x2",
            "custom_y2",
            "iou",
            "center_err",
            "dx",
            "dy",
            "dw",
            "dh",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(metrics)

    print("Comparison complete.")
    print(f"Reference: {args.reference}")
    print(f"Custom: {args.custom} (format={args.custom_format})")
    print(f"Custom frame offset: {args.custom_frame_offset}")
    print(f"Custom coordinate scale: {args.custom_scale}")
    print(f"Output details: {args.output}")
    print(f"Reference frames: {len(reference)}")
    print(f"Custom frames: {len(custom)}")
    print(f"Frames compared: {int(summary['frames_compared'])}")
    print(f"Mean IoU: {summary['mean_iou']:.4f}")
    print(f"Median IoU: {summary['median_iou']:.4f}")
    print(f"Min IoU: {summary['min_iou']:.4f}")
    print(f"Mean center error (px): {summary['mean_center_err']:.2f}")
    print(f"Median center error (px): {summary['median_center_err']:.2f}")
    print(f"Max center error (px): {summary['max_center_err']:.2f}")
    print(f"Mean |dw| (px): {summary['mean_abs_dw']:.2f}")
    print(f"Mean |dh| (px): {summary['mean_abs_dh']:.2f}")


if __name__ == "__main__":
    main()
