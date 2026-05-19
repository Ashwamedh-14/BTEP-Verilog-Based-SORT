# Enhanced Tracker Visualization with Raw Detections

The `visualize_tracks.py` script now supports visualizing three layers of tracking data:

1. **Raw Detections (Blue)** — Bounding boxes from MOG2 background subtraction (no temporal filtering)
2. **Reference SORT (Green)** — Python reference SORT tracker output (Kalman-filtered)
3. **Hardware Tracker (Orange)** — Your Verilog/fixed-point hardware implementation (Kalman-filtered)

## Color Scheme

- **Green** (`Ref SORT`): Reference Python implementation
- **Orange** (`Custom HW`): Hardware-oriented tracker output
- **Blue** (`Raw Det`): Raw MOG2 detections (unfiltered)

## Basic Usage Examples

### 1. Visualize all three layers (Raw + Reference + Hardware)
```bash
python visualize_tracks.py \
  --video test_output.mp4 \
  --reference tracks_sort_reference.csv \
  --custom detections_hw.csv \
  --raw detections_sort_input.csv \
  --custom-format cxcyhr \
  --raw-format mot \
  --output tracks_overlay_all.mp4 \
  --plots \
  --plot-output diagnostics_all.png
```

### 2. Compare Hardware vs Reference (ignoring raw)
```bash
python visualize_tracks.py \
  --video test_output.mp4 \
  --reference tracks_sort_reference.csv \
  --custom detections_hw.csv \
  --custom-format cxcyhr \
  --no-raw \
  --output tracks_hw_vs_ref.mp4 \
  --plots
```

### 3. Show raw detections vs reference (to understand the parameter sweep baseline)
```bash
python visualize_tracks.py \
  --video test_output.mp4 \
  --reference tracks_sort_reference.csv \
  --raw detections_sort_input.csv \
  --raw-format mot \
  --no-custom \
  --output tracks_raw_vs_ref.mp4 \
  --plots
```

## Output Files

- **Video overlay**: Shows bounding boxes with labels and IoU scores on each frame
- **Diagnostics plot** (`*_diagnostics.png`): 4-panel matplotlib figure showing:
  - **Top-left**: IoU between reference and hardware over time
  - **Top-right**: Center error (pixels) between trackers
  - **Bottom-left**: Bounding box area for all three sources
  - **Bottom-right**: Track presence indicator

## Key Metrics Displayed

### On Video (Text Overlay)
- `IoU (Ref-HW)`: Intersection-over-Union between reference SORT and hardware output
- `IoU (Raw-Ref)`: Intersection-over-Union between raw detections and reference SORT
- Frame number

### In Diagnostics Plot
- **IoU plot**: Should show ~0.94 for Ref-HW (your good result), much lower for raw vs reference
- **Center error plot**: Low for filtered outputs, high when raw detections diverge
- **Area plot**: Raw (dashed blue line) shows instability; filtered outputs (solid lines) are smooth
- **Presence plot**: Track continuity across all three sources

## File Format Reference

### `detections_sort_input.csv` (Raw MOG2 detections)
MOT format: `frame,id,x,y,w,h,...`
- One detection per frame (largest contour)
- No temporal filtering

### `tracks_sort_reference.csv` (Reference SORT output)
MOT format: `frame,track_id,x,y,w,h,1,-1,-1,-1`
- Kalman-filtered, motion-predicted tracks
- Multiple tracks possible (keeping largest per frame)

### `detections_hw.csv` (Hardware tracker output)
Format: `frame,cx,cy,h,r` (center-height-ratio, i.e., `cxcyhr`)
- Your fixed-point hardware implementation
- Kalman-filtered output

## Interpretation for Results Chapter

**This visualization directly addresses your earlier question:**

- **0.14xx IoU**: Raw detections vs. Reference SORT
  - Why low? Raw boxes change frame-to-frame, SORT predicts smoothly
  - The parameter sweep tuned Q/R to minimize this

- **0.9429 IoU**: Hardware vs. Reference SORT
  - Why high? Both are Kalman-filtered with similar parameters
  - Shows hardware tracker replicates reference behavior

The visualization makes this distinction clear by showing all three on the same frame.

## Command-Line Arguments

```
--video VIDEO              Input video path (default: test_output.mp4)
--output OUTPUT            Output video with overlays (default: tracks_overlay.mp4)
--reference REF            Reference tracks CSV (default: tracks_sort_reference.csv)
--no-reference             Skip drawing reference tracks
--custom CUSTOM            Custom tracker output (default: detections_sort.csv)
--custom-format FMT        Format: mot|xyxy|cxcywh|cxcyhr|cxcysr (default: cxcyhr)
--custom-frame-offset N    Frame offset for custom file
--custom-scale SCALE       Coordinate scale factor for custom
--no-custom                Skip drawing custom tracks
--raw RAW                  Raw detections CSV file
--raw-format FMT           Format of raw file (default: mot)
--raw-frame-offset N       Frame offset for raw file
--raw-scale SCALE          Coordinate scale factor for raw
--no-raw                   Skip drawing raw detections
--plots                    Generate diagnostics matplotlib plots
--plot-output PATH         Save diagnostics plot to this path
--show-plots               Display matplotlib plots interactively
```

## Example for Results Chapter

Generate a comprehensive visualization showing why hardware matches reference so closely:

```bash
# Generate the three-layer comparison
python visualize_tracks.py \
  --video test_output.mp4 \
  --reference tracks_sort_reference.csv \
  --custom detections_hw.csv \
  --raw detections_sort_input.csv \
  --custom-format cxcyhr \
  --raw-format mot \
  --output results_visualization.mp4 \
  --plots \
  --plot-output figure_tracking_comparison.png
```

This produces:
1. Video with all three trackers overlaid (for supplementary material)
2. Diagnostic plot with 4 panels (for main Results chapter)

The diagnostics plot clearly shows:
- Raw detections are noisy (blue dashed line jumps around)
- Reference SORT smooths these (green line is stable)
- Hardware matches reference (orange ≈ green, 0.9429 IoU)
