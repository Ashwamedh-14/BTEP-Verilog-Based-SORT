`timescale 1ns/1ps

module sort_tb;

reg clk;
reg rst;
reg frame_valid;

reg [15:0] det_x1;
reg [15:0] det_y1;
reg [15:0] det_x2;
reg [15:0] det_y2;

reg [111:0] Q;
reg [63:0] R;

reg [111:0] state_reg;
reg [159:0] cov_reg;

wire busy;
wire done;
wire matched;
wire save_signal;
wire reset_signal;
wire [111:0] state_out;
wire [159:0] cov_out;
wire [15:0] age;
wire [15:0] hits;
wire [15:0] time_since_update;

integer in_fd;
integer out_fd;
integer scan_count;
integer frame_i;
integer det_id;
integer x_i;
integer y_i;
integer w_i;
integer h_i;
real score_dummy;
integer cx_i;
integer cy_i;
integer r_i;
integer tracker_h;
integer tracker_r;
integer line_ok;
integer q_scale;
integer r_pos_scale;
integer r_h_scale;
integer r_r_scale;
reg [8*256-1:0] line_buf;

reg initialized;

function [15:0] fxp16_from_int;
    input integer value;
    begin
        fxp16_from_int = value[15:0];
    end
endfunction

function [15:0] fxp8_8_from_ratio;
    input integer numerator;
    input integer denominator;
    integer scaled;
    begin
        if (denominator == 0) begin
            fxp8_8_from_ratio = 16'd0;
        end else begin
            scaled = (numerator << 8) / denominator;
            fxp8_8_from_ratio = scaled[15:0];
        end
    end
endfunction

function [15:0] fxp12_4_from_int;
    input integer value;
    integer scaled;
    begin
        scaled = value << 4;
        fxp12_4_from_int = scaled[15:0];
    end
endfunction

SORT_Controller #(
    .IOU_THRESHOLD(3),
    .MAX_AGE(5)
) dut (
    .clk(clk),
    .rst(rst),
    .frame_valid(frame_valid),
    .det_x1(det_x1),
    .det_y1(det_y1),
    .det_x2(det_x2),
    .det_y2(det_y2),
    .Q(Q),
    .R(R),
    .state_in(state_reg),
    .cov_in(cov_reg),
    .busy(busy),
    .done(done),
    .matched(matched),
    .save_signal(save_signal),
    .reset_signal(reset_signal),
    .state_out(state_out),
    .cov_out(cov_out),
    .age(age),
    .hits(hits),
    .time_since_update(time_since_update)
);

always #5 clk = ~clk;

initial begin
    clk = 1'b0;
    rst = 1'b1;
    frame_valid = 1'b0;

    det_x1 = 16'd0;
    det_y1 = 16'd0;
    det_x2 = 16'd0;
    det_y2 = 16'd0;

    q_scale = 4;
    r_pos_scale = 1;
    r_h_scale = 20;
    r_r_scale = 40;

    if (!$value$plusargs("Q_SCALE=%d", q_scale)) q_scale = 4;
    if (!$value$plusargs("R_POS_SCALE=%d", r_pos_scale)) r_pos_scale = 1;
    if (!$value$plusargs("R_H_SCALE=%d", r_h_scale)) r_h_scale = 20;
    if (!$value$plusargs("R_R_SCALE=%d", r_r_scale)) r_r_scale = 40;

    // Fixed-point packing used by the behavioral model.
    // Q and R are packed as 16-bit fixed-point values to match the
    // covariance arithmetic in the current Verilog modules.
    Q = {
        fxp12_4_from_int(q_scale), fxp12_4_from_int(q_scale), fxp12_4_from_int(q_scale),
        fxp12_4_from_int(q_scale), fxp12_4_from_int(q_scale), fxp12_4_from_int(q_scale),
        fxp12_4_from_int(q_scale)
    };
    // R packing in update path: [cx, cy, h, r]
    R = {
        fxp12_4_from_int(r_pos_scale),
        fxp12_4_from_int(r_pos_scale),
        fxp12_4_from_int(r_h_scale),
        fxp12_4_from_int(r_r_scale)
    };

    $display("Using Q_SCALE=%0d R_POS_SCALE=%0d R_H_SCALE=%0d R_R_SCALE=%0d", q_scale, r_pos_scale, r_h_scale, r_r_scale);

    state_reg = 112'd0;
    // High initial covariance, with values represented in FXP(12,4).
    cov_reg = {
        fxp12_4_from_int(10), fxp12_4_from_int(10), fxp12_4_from_int(10), fxp12_4_from_int(10),
        fxp12_4_from_int(64), fxp12_4_from_int(64), fxp12_4_from_int(64),
        16'd0, 16'd0, 16'd0
    };

    initialized = 1'b0;

    in_fd = $fopen("../Video/detections_sort_input.csv", "r");
    if (in_fd == 0) begin
        $display("ERROR: Could not open ../Video/detections_sort_input.csv");
        $finish;
    end

    out_fd = $fopen("../Video/detections_sort.csv", "w");
    if (out_fd == 0) begin
        $display("ERROR: Could not open ../Video/detections_sort.csv for writing");
        $finish;
    end

    repeat (4) @(posedge clk);
    rst = 1'b0;

    while (!$feof(in_fd)) begin
        // Input format from main.py:
        // frame,id,x,y,w,h,score
        line_ok = $fgets(line_buf, in_fd);
        if (line_ok != 0) begin
            scan_count = $sscanf(line_buf, "%d,%d,%d,%d,%d,%d,%f", frame_i, det_id, x_i, y_i, w_i, h_i, score_dummy);
        end else begin
            scan_count = 0;
        end

        if (scan_count == 7) begin
            if (h_i <= 0 || w_i <= 0) begin
                scan_count = 7;
            end else if (!initialized) begin
                cx_i = x_i + (w_i / 2);
                cy_i = y_i + (h_i / 2);
                r_i = fxp8_8_from_ratio(w_i, h_i);
                if (r_i <= 0) begin
                    r_i = 16'd256;
                end

                state_reg[111:96] = fxp16_from_int(cx_i);
                state_reg[95:80] = fxp16_from_int(cy_i);
                state_reg[79:64] = fxp16_from_int(h_i);
                state_reg[63:48] = r_i;
                state_reg[47:32] = 16'd0;
                state_reg[31:16] = 16'd0;
                state_reg[15:0] = 16'd0;

                $fwrite(out_fd, "%0d,%0d,%0d,%0d,%0d\n", frame_i, cx_i, cy_i, h_i, (r_i >> 8));
                initialized = 1'b1;
            end else begin
                det_x1 = x_i;
                det_y1 = y_i;
                det_x2 = x_i + w_i;
                det_y2 = y_i + h_i;

                @(posedge clk);
                frame_valid = 1'b1;
                @(posedge clk);
                frame_valid = 1'b0;

                wait (done == 1'b1);
                @(posedge clk);

                state_reg = state_out;
                cov_reg = cov_out;

                tracker_h = state_out[79:64];
                tracker_r = state_out[63:48] >> 8;
                if (tracker_h <= 0) begin
                    tracker_h = 1;
                end
                if (tracker_r <= 0) begin
                    tracker_r = 1;
                end

                $fwrite(
                    out_fd,
                    "%0d,%0d,%0d,%0d,%0d\n",
                    frame_i,
                    state_out[111:96],
                    state_out[95:80],
                    tracker_h,
                    tracker_r
                );
            end
        end
    end

    $fclose(in_fd);
    $fclose(out_fd);

    $display("Done. Wrote custom tracker output to ../Video/detections_sort.csv");
    #20;
    $finish;
end

endmodule
