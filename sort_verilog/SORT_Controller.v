// ============================================================
// SORT CONTROLLER - Main Tracking State Machine
// ============================================================
// Single object tracker implementing the SORT algorithm with
// Kalman filter prediction, measurement update, and IoU matching.
//
// FIXED-POINT NOTATION:
//   State Vector x (7 elements - 112 bits total):
//     - cx, cy, h: FXP(16,0) - Unsigned position/dimension values
//     - r: FXP(8,8) - Aspect ratio (width/height)
//     - vx, vy, vh: FXP(8,8) - Signed velocity values
//
//   Covariance Matrix P (10 elements - 160 bits total):
//     - Diagonal: sigma2_cx, sigma2_cy, sigma2_h, sigma2_r, sigma2_vx, sigma2_vy, sigma2_vh - FXP(12,4)
//     - Cross-cov: sigma_vx, sigma_vy, sigma_vh - FXP(8,8)
//
//   Measurement z (4 elements - 64 bits):
//     - cx, cy, h: FXP(16,0)
//     - r: FXP(8,8)
//
//   Noise Covariances:
//     - Q: Process noise - FXP(8,8) per element
//     - R: Measurement noise - FXP(8,8) per element
//
// STATE MACHINE FLOW:
//   S_IDLE → S_PREDICT → S_IOU → S_UPDATE/S_NOUPDATE → S_META → S_DONE → S_IDLE
//
// IMPORTANT NOTES:
//   - All arithmetic operations maintain fixed-point precision with bit shifts
//   - Division operations must account for fractional bit position conversion
//   - Results are validated for overflow/underflow in update stages
// ============================================================
module SORT_Controller #(
    parameter IOU_THRESHOLD = 3,
    parameter MAX_AGE = 30
) (
    input wire clk,
    input wire rst,
    input wire frame_valid,

    input wire [15:0] det_x1,
    input wire [15:0] det_y1,
    input wire [15:0] det_x2,
    input wire [15:0] det_y2,

    input wire [111:0] Q,
    input wire [63:0] R,

    input wire [111:0] state_in,
    input wire [159:0] cov_in,

    output reg busy,
    output reg done,
    output reg matched,
    output reg save_signal,
    output reg reset_signal,

    output reg [111:0] state_out,
    output reg [159:0] cov_out,

    output reg [15:0] age,
    output reg [15:0] hits,
    output reg [15:0] time_since_update
);

localparam S_IDLE     = 3'd0;
localparam S_PREDICT  = 3'd1;
localparam S_IOU      = 3'd2;
localparam S_UPDATE   = 3'd3;
localparam S_NOUPDATE = 3'd4;
localparam S_META     = 3'd5;
localparam S_DONE     = 3'd6;

reg [2:0] state;

// State and covariance storage (FXP notation as specified above)

// =====================
// Bit Assignments for State Vector
// =====================
// x = [cx(111:96), cy(95:80), h(79:64), r(63:48), vx(47:32), vy(31:16), vh(15:0)]
// All in fixed-point format as specified above

reg [111:0] x_base;
reg [159:0] p_base;
reg [111:0] x_pred_reg;
reg [159:0] p_pred_reg;
reg [111:0] x_next_reg;
reg [159:0] p_next_reg;

reg [15:0] det_x1_reg, det_y1_reg, det_x2_reg, det_y2_reg;

reg reset_pending;

wire [63:0] z_state;
wire [63:0] z_init_state;
wire [15:0] x_pred_cx, x_pred_cy, x_pred_h;
wire [15:0] p_pred_cx, p_pred_cy, p_pred_h, p_pred_r;
wire [15:0] p_pred_vx, p_pred_vy, p_pred_vh;
wire [15:0] p_pred_cov_vx, p_pred_cov_vy, p_pred_cov_vh;

wire [111:0] x_pred_wire;
wire [159:0] p_pred_wire;
wire [63:0] x_pred_meas;
wire [63:0] p_pred_meas;

wire [16:0] pred_x1, pred_y1, pred_x2, pred_y2;
wire iou_match;
wire [63:0] innov_y;
wire [63:0] innov_s;
wire [111:0] x_upd_wire;
wire [159:0] p_upd_wire;

wire [15:0] next_age_update;
wire [15:0] next_tsu_no_update;

assign x_pred_wire = {
    x_pred_cx, x_pred_cy, x_pred_h,
    x_base[63:48], x_base[47:32], x_base[31:16], x_base[15:0]
};

assign p_pred_wire = {
    p_pred_cx, p_pred_cy, p_pred_h, p_pred_r,
    p_pred_vx, p_pred_vy, p_pred_vh,
    p_pred_cov_vx, p_pred_cov_vy, p_pred_cov_vh
};

// x_pred_meas must match ST_to_BBox input layout: [r(63:48), h(47:32), cy(31:16), cx(15:0)]
assign x_pred_meas = {x_base[63:48], x_pred_h, x_pred_cy, x_pred_cx};
assign p_pred_meas = p_pred_wire[159:96];

assign next_age_update = age + 16'd1;
assign next_tsu_no_update = time_since_update + 16'd1;

// =====================
// Module Instantiations
// =====================

// Convert detection bounding box to state vector format

BBox_to_ST u_det_to_state (
    .x1(det_x1_reg),
    .y1(det_y1_reg),
    .x2(det_x2_reg),
    .y2(det_y2_reg),
    .X(z_state)
);

BBox_to_ST u_det_init_state (
    .x1(det_x1),
    .y1(det_y1),
    .x2(det_x2),
    .y2(det_y2),
    .X(z_init_state)
);

localparam [159:0] DEFAULT_COV = {
    16'd160, 16'd160, 16'd160, 16'd160,
    16'd1024, 16'd1024, 16'd1024,
    16'd0, 16'd0, 16'd0
};

Predict u_predict (
    .cx(x_base[111:96]),
    .cy(x_base[95:80]),
    .h(x_base[79:64]),
    .vx(x_base[47:32]),
    .vy(x_base[31:16]),
    .vh(x_base[15:0]),

    .px(p_base[159:144]),
    .py(p_base[143:128]),
    .ph(p_base[127:112]),
    .pr(p_base[111:96]),
    .pvx(p_base[95:80]),
    .pvy(p_base[79:64]),
    .pvh(p_base[63:48]),

    .p_cov_vx(p_base[47:32]),
    .p_cov_vy(p_base[31:16]),
    .p_cov_vh(p_base[15:0]),

    .Q(Q),

    .x_out(x_pred_cx),
    .y_out(x_pred_cy),
    .h_out(x_pred_h),

    .px_out(p_pred_cx),
    .py_out(p_pred_cy),
    .ph_out(p_pred_h),
    .pr_out(p_pred_r),

    .p_vx_out(p_pred_vx),
    .p_vy_out(p_pred_vy),
    .p_vh_out(p_pred_vh),

    .p_cov_vx_out(p_pred_cov_vx),
    .p_cov_vy_out(p_pred_cov_vy),
    .p_cov_vh_out(p_pred_cov_vh)
);

ST_to_BBox u_state_to_bbox (
    .X(x_pred_meas),
    .x1(pred_x1),
    .y1(pred_y1),
    .x2(pred_x2),
    .y2(pred_y2)
);

IOU #(
    .threshold(IOU_THRESHOLD)
) u_iou (
    .Ax1(pred_x1[15:0]),
    .Ay1(pred_y1[15:0]),
    .Ax2(pred_x2[15:0]),
    .Ay2(pred_y2[15:0]),
    .Bx1(det_x1_reg),
    .By1(det_y1_reg),
    .Bx2(det_x2_reg),
    .By2(det_y2_reg),
    .valid(iou_match)
);

Innovate u_innovate (
    .z(z_state),
    .x(x_pred_meas),
    .P(p_pred_meas),
    .R(R),
    .y(innov_y),
    .S(innov_s)
);

Update u_update (
    .x_in(x_pred_wire),
    .P_in(p_pred_wire),
    .S(innov_s),
    .y(innov_y),
    .x_out(x_upd_wire),
    .P_out(p_upd_wire)
);

// =====================
// STATE MACHINE
// =====================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= S_IDLE;
        busy <= 1'b0;
        done <= 1'b0;
        matched <= 1'b0;
        save_signal <= 1'b0;
        reset_signal <= 1'b0;

        state_out <= 112'd0;
        cov_out <= 160'd0;
        x_base <= 112'd0;
        p_base <= 160'd0;
        x_pred_reg <= 112'd0;
        p_pred_reg <= 160'd0;
        x_next_reg <= 112'd0;
        p_next_reg <= 160'd0;

        det_x1_reg <= 16'd0;
        det_y1_reg <= 16'd0;
        det_x2_reg <= 16'd0;
        det_y2_reg <= 16'd0;

        age <= 16'd0;
        hits <= 16'd0;
        time_since_update <= 16'd0;
        reset_pending <= 1'b0;
    end else begin
        done <= 1'b0;
        save_signal <= 1'b0;
        reset_signal <= 1'b0;

        case (state)
            S_IDLE: begin
                busy <= 1'b0;
                matched <= 1'b0;

                if (reset_pending) begin
                    x_base <= 112'd0;
                    p_base <= 160'd0;
                    age <= 16'd0;
                    hits <= 16'd0;
                    time_since_update <= 16'd0;
                    reset_pending <= 1'b0;
                end else if (frame_valid) begin
                    busy <= 1'b1;

                    x_base <= state_in;
                    p_base <= cov_in;

                    det_x1_reg <= det_x1;
                    det_y1_reg <= det_y1;
                    det_x2_reg <= det_x2;
                    det_y2_reg <= det_y2;

                    if ((state_in == 112'd0) || (cov_in == 160'd0)) begin
                        x_base <= {z_init_state[15:0], z_init_state[31:16], z_init_state[47:32], z_init_state[63:48], 48'd0};
                        p_base <= DEFAULT_COV;
                        state_out <= {z_init_state[15:0], z_init_state[31:16], z_init_state[47:32], z_init_state[63:48], 48'd0};
                        cov_out <= DEFAULT_COV;
                        age <= 16'd0;
                        hits <= 16'd1;
                        time_since_update <= 16'd0;
                        save_signal <= 1'b1;
                        state <= S_DONE;
                    end else begin
                        state <= S_PREDICT;
                    end
                end
            end

            S_PREDICT: begin
                x_pred_reg <= x_pred_wire;
                p_pred_reg <= p_pred_wire;
                // Prediction complete; proceed to IOU check
                state <= S_IOU;
            end

            S_IOU: begin
                if (iou_match) begin
                    matched <= 1'b1;
                    state <= S_UPDATE;
                end else begin
                    matched <= 1'b0;
                    state <= S_NOUPDATE;
                end
            end

            S_UPDATE: begin
                x_next_reg <= x_upd_wire;
                p_next_reg <= p_upd_wire;
                state <= S_META;
            end

            S_NOUPDATE: begin
                x_next_reg <= x_pred_reg;
                p_next_reg <= p_pred_reg;
                state <= S_META;
            end

            S_META: begin
                // By default, publish the next state
                state_out <= x_next_reg;
                cov_out <= p_next_reg;

                age <= next_age_update;

                if (matched) begin
                    // Successful measurement update
                    hits <= hits + 16'd1;
                    time_since_update <= 16'd0;
                    save_signal <= 1'b1;
                end else begin
                    // No measurement matched this frame
                    time_since_update <= next_tsu_no_update;

                    // If the track has aged out, clear its state so the
                    // controller is immediately available for reassignment.
                    if (next_tsu_no_update > MAX_AGE) begin
                        reset_signal <= 1'b1;
                        reset_pending <= 1'b1;

                        // Clear published state/covariance so downstream
                        // consumers see that the track was dropped.
                        state_out <= 112'd0;
                        cov_out <= 160'd0;

                        // Clear meta counters immediately for clarity
                        age <= 16'd0;
                        hits <= 16'd0;
                        time_since_update <= 16'd0;

                        // Do not assert save_signal when dropping the track
                        save_signal <= 1'b0;
                    end else begin
                        save_signal <= 1'b1;
                    end
                end

                state <= S_DONE;
            end

            S_DONE: begin
                done <= 1'b1;
                busy <= 1'b0;
                state <= S_IDLE;
            end

            default: begin
                state <= S_IDLE;
            end
        endcase
    end
end

endmodule
