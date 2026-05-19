# visualize_tracks.py Enhancement Summary

## What Was Added

Enhanced `visualize_tracks.py` to support visualizing raw detection boxes alongside reference and hardware tracker outputs. This directly addresses your question about why hardware matches reference (0.9429 IoU) when the sweep showed only 0.14xx.

### New Features

#### 1. Raw Detection Loading
- **New function**: `load_raw_detections()` 
- Loads single-detection-per-frame data (e.g., MOG2 background subtraction output)
- Supports multiple formats: MOT, xyxy, cxcywh, cxcyhr, cxcysr
- Keeps largest detection per frame for consistency

#### 2. Video Overlay Enhancements
- **Blue boxes**: Raw detections (unfiltered)
- **Green boxes**: Reference SORT (Kalman-filtered)
- **Orange boxes**: Hardware tracker (Kalman-filtered)
- Displays two IoU values per frame:
  - `IoU (Ref-HW)`: Reference vs Hardware (~0.9429)
  - `IoU (Raw-Ref)`: Raw vs Reference (~0.14-0.15)

#### 3. Diagnostics Plots
Updated matplotlib plots to include:
- **Area panel**: Shows raw detection instability vs smooth filtered outputs
- **Presence panel**: Tracks which source has detections in each frame
- **Legend**: Clearly distinguishes "Raw (MOG2)", "Custom (HW)", "Ref (SORT)"

#### 4. Command-Line Arguments
```
--raw RAW                    Raw detections CSV file
--raw-format FMT             Format: mot (default) | xyxy | cxcywh | cxcyhr | cxcysr
--raw-frame-offset N         Frame number offset for raw file
--raw-scale SCALE            Coordinate scaling factor
--no-raw                     Disable raw detection visualization
```

## Visual Output Example

### Video Frame Shows:
```
┌─────────────────────────────────────┐
│                                     │
│  [Blue box]  [Green box] [Orange]   │  ← Three overlaid bounding boxes
│                                     │
│  IoU (Ref-HW): 0.9429              │  ← High: both are filtered
│  IoU (Raw-Ref): 0.1247             │  ← Low: raw is unfiltered
│                                     │
│  Frame: 50                          │
└─────────────────────────────────────┘
```

### Diagnostics Plot (4 panels):
```
┌────────────────────────┬────────────────────────┐
│ IoU vs Frame           │ Center Error vs Frame   │
│ (Ref-HW ≈ 0.94)       │ (varies with raw)       │
├────────────────────────┼────────────────────────┤
│ Bounding Box Area      │ Track Presence         │
│ Raw(blue) jumps around │ Shows all 3 sources    │
│ Filtered lines smooth  │ active per frame       │
└────────────────────────┴────────────────────────┘
```

## Use Case: For Your Results Chapter

**Before explaining the 0.9429 vs 0.14xx discrepancy**, generate:

```bash
python visualize_tracks.py \
  --video test_output.mp4 \
  --reference tracks_sort_reference.csv \
  --custom detections_hw.csv \
  --raw detections_sort_input.csv \
  --custom-format cxcyhr \
  --raw-format mot \
  --plots \
  --plot-output figure_tracking_layers.png
```

Then in your Results section, you can write:

> "Figure X visualizes three tracking layers side-by-side: raw MOG2 detections (blue), reference SORT output (green), and hardware tracker output (orange). The raw detections show frame-to-frame instability (IoU ≈ 0.12-0.15 vs reference), while the hardware output closely matches the Kalman-filtered reference (IoU ≈ 0.94). This demonstrates that the high hardware-reference alignment is due to both using temporal filtering, not because raw detections inherently match better."

## Technical Details

### Raw Detection Format Support
The script reuses `to_box_from_custom()` from `compare_trackers.py`, so it supports:
- **MOT**: frame,id,x,y,w,h,... → converts to Box
- **xyxy**: frame,x1,y1,x2,y2
- **cxcywh**: frame,cx,cy,w,h
- **cxcyhr**: frame,cx,cy,h,r (height-ratio)
- **cxcysr**: frame,cx,cy,s,r (area-ratio)

### Frame-Wise IoU Display
When all three sources exist in a frame:
1. `IoU(ref, hardware)` displays in position (20, 40)
2. `IoU(raw, ref)` displays in position (20, 70)
3. Allows visual confirmation that filtering helps

## Files Modified

- **visualize_tracks.py**: Added 100+ lines for raw detection support
- **VISUALIZE_USAGE.md**: Comprehensive usage guide with examples

## Next Steps for Your Report

1. Generate the three-layer visualization
2. Include the diagnostics plot in Results section
3. Add caption explaining the three overlays
4. Reference this visualization when explaining the 0.14xx vs 0.9429 IoU values
