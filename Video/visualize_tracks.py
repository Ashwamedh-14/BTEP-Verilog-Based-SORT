from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from compare_trackers import Box, iou, load_custom_tracks, load_reference_tracks, to_box_from_custom


def draw_box(frame, box: Box, color: tuple[int, int, int], label: str) -> None:
    x1, y1, x2, y2 = int(round(box.x1)), int(round(box.y1)), int(round(box.x2)), int(round(box.y2))
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    text_origin_y = max(20, y1 - 10)
    cv2.putText(
        frame,
        label,
        (x1, text_origin_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        color,
        2,
    )


def load_raw_detections(
    path: Path,
    fmt: str = "mot",
    frame_offset: int = 0,
    coord_scale: float = 1.0,
) -> dict[int, Box]:
    """
    Load raw detections (one per frame, no tracking/filtering).
    Format: frame,id,x,y,w,h,... (MOT format) or similar.
    Returns dict[frame_number] = Box (single detection per frame).
    """
    by_frame: dict[int, Box] = {}

    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = [p.strip() for p in line.split(",")]
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue

        result = to_box_from_custom(vals, fmt)
        if result is None:
            continue

        frame, box = result
        frame = frame + frame_offset
        
        # Apply coordinate scaling if needed
        if coord_scale != 1.0:
            box = Box(
                x1=box.x1 * coord_scale,
                y1=box.y1 * coord_scale,
                x2=box.x2 * coord_scale,
                y2=box.y2 * coord_scale,
            )

        # Keep only the largest detection per frame for consistency
        if frame not in by_frame or (box.w * box.h) > (by_frame[frame].w * by_frame[frame].h):
            by_frame[frame] = box

    return by_frame


def generate_diagnostics_plot(
    reference_tracks: dict[int, Box],
    custom_tracks: dict[int, Box],
    raw_detections: dict[int, Box] | None = None,
    output_path: Path | None = None,
    show_plot: bool = False,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise ImportError(
            "matplotlib is required for plotting. Install it with: pip install matplotlib"
        ) from exc

    if not reference_tracks and not custom_tracks:
        print("No track data available for plotting.")
        return

    all_frames_sorted = sorted(set(reference_tracks).union(custom_tracks).union(raw_detections or {}))
    start_frame = all_frames_sorted[0]
    end_frame = all_frames_sorted[-1]
    frame_axis = list(range(start_frame, end_frame + 1))

    has_reference = [1 if frame in reference_tracks else 0 for frame in frame_axis]
    has_custom = [1 if frame in custom_tracks else 0 for frame in frame_axis]
    has_raw = [1 if frame in (raw_detections or {}) else 0 for frame in frame_axis]

    ref_area = [
        reference_tracks[frame].w * reference_tracks[frame].h if frame in reference_tracks else None
        for frame in frame_axis
    ]
    custom_area = [
        custom_tracks[frame].w * custom_tracks[frame].h if frame in custom_tracks else None
        for frame in frame_axis
    ]
    raw_area = [
        raw_detections[frame].w * raw_detections[frame].h if raw_detections and frame in raw_detections else None
        for frame in frame_axis
    ] if raw_detections else None

    shared_frames = sorted(set(reference_tracks).intersection(custom_tracks))
    iou_values: list[float] = []
    center_error_values: list[float] = []

    for frame in shared_frames:
        ref = reference_tracks[frame]
        cur = custom_tracks[frame]
        iou_values.append(iou(ref, cur))
        dx = cur.cx - ref.cx
        dy = cur.cy - ref.cy
        center_error_values.append((dx * dx + dy * dy) ** 0.5)

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    ax_iou, ax_center, ax_area, ax_presence = axes.flat

    if shared_frames:
        ax_iou.plot(shared_frames, iou_values, color="#1f77b4", linewidth=1.8)
        ax_iou.set_ylim(0.0, 1.0)
        ax_iou.grid(alpha=0.25)
    else:
        ax_iou.text(0.5, 0.5, "No shared frames", ha="center", va="center", transform=ax_iou.transAxes)
    ax_iou.set_title("IoU vs Frame")
    ax_iou.set_xlabel("Frame")
    ax_iou.set_ylabel("IoU")

    if shared_frames:
        ax_center.plot(shared_frames, center_error_values, color="#d62728", linewidth=1.8)
        ax_center.grid(alpha=0.25)
    else:
        ax_center.text(0.5, 0.5, "No shared frames", ha="center", va="center", transform=ax_center.transAxes)
    ax_center.set_title("Center Error vs Frame")
    ax_center.set_xlabel("Frame")
    ax_center.set_ylabel("Pixels")

    ax_area.plot(frame_axis, ref_area, label="Ref (SORT)", color="#2ca02c", linewidth=1.5)
    ax_area.plot(frame_axis, custom_area, label="Custom (HW)", color="#ff7f0e", linewidth=1.5)
    if raw_area:
        ax_area.plot(frame_axis, raw_area, label="Raw (MOG2)", color="#1f77b4", linewidth=1.5, linestyle="--")
    ax_area.set_title("Bounding Box Area")
    ax_area.set_xlabel("Frame")
    ax_area.set_ylabel("Area (px^2)")
    ax_area.grid(alpha=0.25)
    ax_area.legend()

    ax_presence.step(frame_axis, has_reference, where="mid", label="Ref (SORT)", color="#2ca02c")
    ax_presence.step(frame_axis, has_custom, where="mid", label="Custom (HW)", color="#ff7f0e")
    if raw_detections:
        ax_presence.step(frame_axis, has_raw, where="mid", label="Raw (MOG2)", color="#1f77b4", linestyle="--")
    ax_presence.set_ylim(-0.1, 1.1)
    ax_presence.set_yticks([0, 1])
    ax_presence.set_title("Track Presence by Frame")
    ax_presence.set_xlabel("Frame")
    ax_presence.set_ylabel("Present")
    ax_presence.grid(alpha=0.25)
    ax_presence.legend()

    missing_reference = sum(1 for frame in frame_axis if frame not in reference_tracks)
    missing_custom = sum(1 for frame in frame_axis if frame not in custom_tracks)
    missing_raw = sum(1 for frame in frame_axis if frame not in (raw_detections or {}))
    
    raw_info = f" | Missing raw: {missing_raw}" if raw_detections else ""
    fig.suptitle(
        (
            f"Tracking Diagnostics | Shared frames: {len(shared_frames)} | "
            f"Missing ref: {missing_reference} | Missing custom: {missing_custom}{raw_info}"
        ),
        fontsize=12,
    )

    if output_path is not None:
        fig.savefig(output_path, dpi=180)
        print(f"Saved diagnostics plot: {output_path}")

    if show_plot:
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize tracker boxes on a video (reference SORT and/or custom .vvp flow output)."
    )
    parser.add_argument("--video", type=Path, default=Path("test_output.mp4"), help="Input video path.")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("tracks_overlay.mp4"),
        help="Output video with overlays.",
    )

    parser.add_argument(
        "--reference",
        type=Path,
        default=Path("tracks_sort_reference.csv"),
        help="Reference Python SORT tracks file.",
    )
    parser.add_argument(
        "--no-reference",
        action="store_true",
        help="Disable drawing reference tracks.",
    )

    parser.add_argument(
        "--custom",
        type=Path,
        default=Path("detections_sort.csv"),
        help="Custom tracker output file (CSV or extracted .vvp log lines).",
    )
    parser.add_argument(
        "--custom-format",
        choices=["mot", "xyxy", "cxcywh", "cxcyhr", "cxcysr"],
        default="cxcyhr",
        help="Format of custom tracker output.",
    )
    parser.add_argument(
        "--custom-frame-offset",
        type=int,
        default=0,
        help="Offset to apply to custom frame numbers.",
    )
    parser.add_argument(
        "--custom-scale",
        type=float,
        default=1.0,
        help="Scale factor to apply to custom coordinates.",
    )
    parser.add_argument(
        "--no-custom",
        action="store_true",
        help="Disable drawing custom tracks.",
    )

    parser.add_argument(
        "--raw",
        type=Path,
        default=None,
        help="Raw detection boxes file (e.g., detections_sort_input.csv for MOG2 output).",
    )
    parser.add_argument(
        "--raw-format",
        choices=["mot", "xyxy", "cxcywh", "cxcyhr", "cxcysr"],
        default="mot",
        help="Format of raw detections file.",
    )
    parser.add_argument(
        "--raw-frame-offset",
        type=int,
        default=0,
        help="Offset to apply to raw detection frame numbers.",
    )
    parser.add_argument(
        "--raw-scale",
        type=float,
        default=1.0,
        help="Scale factor to apply to raw detection coordinates.",
    )
    parser.add_argument(
        "--no-raw",
        action="store_true",
        help="Disable drawing raw detections.",
    )

    parser.add_argument(
        "--plots",
        action="store_true",
        help="Generate matplotlib diagnostics plots after rendering the video.",
    )
    parser.add_argument(
        "--plot-output",
        type=Path,
        default=None,
        help="Output image path for matplotlib diagnostics plot.",
    )
    parser.add_argument(
        "--show-plots",
        action="store_true",
        help="Display matplotlib figure window (in addition to optional save).",
    )

    args = parser.parse_args()

    if not args.video.exists():
        raise FileNotFoundError(f"Video file not found: {args.video}")

    draw_reference = not args.no_reference
    draw_custom = not args.no_custom
    draw_raw = not args.no_raw and args.raw is not None

    if not draw_reference and not draw_custom and not draw_raw:
        raise ValueError("All sources are disabled. Enable at least one of reference/custom/raw.")

    reference_tracks: dict[int, Box] = {}
    custom_tracks: dict[int, Box] = {}
    raw_detections: dict[int, Box] = {}

    if draw_reference:
        if not args.reference.exists():
            raise FileNotFoundError(f"Reference tracks file not found: {args.reference}")
        reference_tracks = load_reference_tracks(args.reference)

    if draw_custom:
        if not args.custom.exists():
            raise FileNotFoundError(f"Custom tracks file not found: {args.custom}")
        custom_tracks = load_custom_tracks(
            args.custom,
            args.custom_format,
            frame_offset=args.custom_frame_offset,
            coord_scale=args.custom_scale,
        )

    if draw_raw:
        if not args.raw.exists():
            raise FileNotFoundError(f"Raw detections file not found: {args.raw}")
        raw_detections = load_raw_detections(
            args.raw,
            args.raw_format,
            frame_offset=args.raw_frame_offset,
            coord_scale=args.raw_scale,
        )

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {args.video}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(str(args.output), fourcc, fps, (width, height))

    frame_idx = 1
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        ref_box = reference_tracks.get(frame_idx)
        custom_box = custom_tracks.get(frame_idx)
        raw_box = raw_detections.get(frame_idx)

        if raw_box is not None:
            draw_box(frame, raw_box, (255, 127, 0), "Raw Det")

        if ref_box is not None:
            draw_box(frame, ref_box, (0, 255, 0), "Ref SORT")

        if custom_box is not None:
            draw_box(frame, custom_box, (0, 165, 255), "Custom HW")

        if ref_box is not None and custom_box is not None:
            frame_iou = iou(ref_box, custom_box)
            cv2.putText(
                frame,
                f"IoU (Ref-HW): {frame_iou:.3f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )

        if raw_box is not None and ref_box is not None:
            raw_ref_iou = iou(raw_box, ref_box)
            cv2.putText(
                frame,
                f"IoU (Raw-Ref): {raw_ref_iou:.3f}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )

        cv2.putText(
            frame,
            f"Frame: {frame_idx}",
            (20, height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        out.write(frame)
        frame_idx += 1

    cap.release()
    out.release()

    print("Visualization complete.")
    print(f"Input video: {args.video}")
    print(f"Output video: {args.output}")
    if draw_reference:
        print(f"Reference frames available: {len(reference_tracks)}")
    if draw_custom:
        print(f"Custom frames available: {len(custom_tracks)}")
    if draw_raw:
        print(f"Raw detections available: {len(raw_detections)}")

    if args.plots or args.show_plots:
        plot_output = args.plot_output
        if plot_output is None:
            plot_output = args.output.with_name(f"{args.output.stem}_diagnostics.png")
        generate_diagnostics_plot(
            reference_tracks=reference_tracks,
            custom_tracks=custom_tracks,
            raw_detections=raw_detections if draw_raw else None,
            output_path=plot_output,
            show_plot=args.show_plots,
        )


if __name__ == "__main__":
    main()
