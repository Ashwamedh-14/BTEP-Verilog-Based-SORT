# BTEP: Real-Time Image Tracking in Verilog HDL

This repository contains a B.Tech end-term project on implementing a hardware-oriented, real-time object tracking pipeline inspired by SORT (Simple Online and Realtime Tracking).

The project combines:
- Python-based video preprocessing and baseline tracking
- Verilog HDL modules for fixed-point Kalman filtering and IoU-based association
- Comparison and visualization tools for hardware vs. software outputs
- LaTeX sources for proposal, specifications, and final report

## Project Goal

Design and evaluate a hardware-friendly object tracker that approximates a software SORT baseline while remaining suitable for HDL simulation and FPGA-oriented workflows.

## Repository Layout

- `Video/`
  - Python pipeline for detection extraction, tracker comparison, and visual overlays
  - Main scripts: `main.py`, `compare_trackers.py`, `visualize_tracks.py`
  - Stores generated tracking artifacts and overlay outputs
- `sort_verilog/`
  - Verilog implementation of tracker building blocks
  - Core modules: `SORT_Controller.v`, `KalmanBlock.v`, `IoU_Block.v`, `memory.v`, `arithmetic.v`
  - Includes parameter sweep script: `sweep_qr.sh`
- `Specs/`
  - Technical specification document (`main.tex`) and build helper (`compile.sh`)
- `Related Works/`
  - Supporting references/materials

## End-to-End Workflow

1. Run video preprocessing and detection export (Python):
  - Generates detection outputs and optional reference SORT tracks.
2. Run Verilog simulation:
  - Produces hardware tracker outputs for evaluation.
3. Compare hardware output against software reference:
   - Computes IoU and center/size error metrics.
4. Visualize overlays and diagnostics:
   - Creates annotated video and plots for analysis/reporting.

## Prerequisites

### Python side
- Python 3.12+
- OpenCV
- Ultralytics
- Matplotlib
- Jupyter (optional)
- Baseline SORT dependencies (if enabled in your local setup)

### Verilog side
- Icarus Verilog (`iverilog`, `vvp`)

### Documentation side
- LaTeX toolchain with `latexmk`
- Inkscape (used by `Specs/compile.sh` for SVG to PDF+LaTeX export)

## Setup

From repository root:

```bash
python3 -m venv env
source env/bin/activate
pip install --upgrade pip
pip install -e ./Video
pip install filterpy lap scipy numpy matplotlib
```

## Usage

### 1) Generate detections and reference tracking

```bash
cd Video
python main.py
```

This stage generates detection and tracking artifacts used by downstream analysis scripts.

### 2) Run Verilog tracker simulation

```bash
cd sort_verilog
iverilog -g2012 -o sort_top_sim SORT_Controller.v KalmanBlock.v IoU_Block.v sort_tb.v
vvp sort_top_sim
```

The hardware simulation output is consumed by the Python comparison scripts.

### 3) Compare hardware vs reference

```bash
cd Video
python compare_trackers.py \
  --reference <reference_tracks> \
  --custom <hardware_tracks> \
  --custom-format cxcyhr \
  --output <comparison_output>
```

### 4) Create overlay visualization + diagnostics

```bash
cd Video
python visualize_tracks.py \
  --video <input_video> \
  --reference <reference_tracks> \
  --custom <hardware_tracks> \
  --raw <raw_detections> \
  --custom-format cxcyhr \
  --raw-format mot \
  --output <overlay_video> \
  --plots \
  --plot-output <diagnostics_plot>
```

### 5) Sweep Q/R parameters (optional)

```bash
bash sort_verilog/sweep_qr.sh
```

This script runs multiple simulation configurations and writes a summary table in the video workspace.

## Building Documents

### Specifications Build

```bash
cd Specs
bash compile.sh
```

### Final report (LaTeX)

```bash
cd Specs
latexmk -pdf main.tex
```

## Notes

- Keep generated outputs and build artifacts outside versioned documentation references where possible.

## License

This repository is distributed under the license in `LICENSE`.
