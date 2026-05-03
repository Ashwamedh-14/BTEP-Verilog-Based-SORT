#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
sort_dir="$repo_root/sort_verilog"
video_dir="$repo_root/Video"
reference_file="$video_dir/tracks_sort_reference.csv"
python_bin="$repo_root/.venv/bin/python"

q_values=(1 2 4)
r_h_values=(10 20)
r_r_values=(10 20 40)

if [[ ! -x "$python_bin" ]]; then
    python_bin=$(command -v python3)
fi

if [[ ! -f "$reference_file" ]]; then
    echo "Reference file not found: $reference_file" >&2
    exit 1
fi

cd "$sort_dir"
iverilog -g2012 -o sort_top.vvp SORT_Controller.v KalmanBlock.v IoU_Block.v sort_tb.v

results_file="$video_dir/qr_sweep_results.csv"
printf 'Q_SCALE,R_H_SCALE,R_R_SCALE,mean_iou,median_iou,mean_center_err,max_center_err,mean_abs_dw,mean_abs_dh\n' > "$results_file"

for q_scale in "${q_values[@]}"; do
    for r_h_scale in "${r_h_values[@]}"; do
        for r_r_scale in "${r_r_values[@]}"; do
            custom_file="$video_dir/detections_sort_Q${q_scale}_RH${r_h_scale}_RR${r_r_scale}.csv"
            comparison_file="$video_dir/comparison_Q${q_scale}_RH${r_h_scale}_RR${r_r_scale}.csv"

            vvp sort_top.vvp +Q_SCALE="$q_scale" +R_POS_SCALE=1 +R_H_SCALE="$r_h_scale" +R_R_SCALE="$r_r_scale" >/tmp/sort_sweep.log
            cp "$video_dir/detections_sort.csv" "$custom_file"

            comparison_output=$(
                "$python_bin" "$video_dir/compare_trackers.py" \
                    --reference "$reference_file" \
                    --custom "$custom_file" \
                    --custom-format cxcyhr \
                    --output "$comparison_file"
            )

            mean_iou=$(printf '%s\n' "$comparison_output" | awk -F': ' '/Mean IoU:/ {print $2; exit}')
            median_iou=$(printf '%s\n' "$comparison_output" | awk -F': ' '/Median IoU:/ {print $2; exit}')
            mean_center_err=$(printf '%s\n' "$comparison_output" | awk -F': ' '/Mean center error \(px\):/ {print $2; exit}')
            max_center_err=$(printf '%s\n' "$comparison_output" | awk -F': ' '/Max center error \(px\):/ {print $2; exit}')
            mean_abs_dw=$(printf '%s\n' "$comparison_output" | awk -F': ' '/Mean \|dw\| \(px\):/ {print $2; exit}')
            mean_abs_dh=$(printf '%s\n' "$comparison_output" | awk -F': ' '/Mean \|dh\| \(px\):/ {print $2; exit}')

            printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
                "$q_scale" "$r_h_scale" "$r_r_scale" \
                "$mean_iou" "$median_iou" "$mean_center_err" "$max_center_err" "$mean_abs_dw" "$mean_abs_dh" \
                >> "$results_file"

            printf 'Q=%s RH=%s RR=%s meanIoU=%s meanCenterErr=%s\n' \
                "$q_scale" "$r_h_scale" "$r_r_scale" "$mean_iou" "$mean_center_err"
        done
    done
done

echo
echo 'Top runs:'
sort -t, -k4,4nr -k6,6n "$results_file" | head -n 10