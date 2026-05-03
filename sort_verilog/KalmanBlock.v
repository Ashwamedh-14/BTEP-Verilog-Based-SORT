// ============================================================
// MULT-DIV WITH FIXED-POINT SUPPORT AND BIT SHIFTS
// ============================================================
// Performs Y = (A * B) / C with proper fixed-point bit shifting
//
// Fixed-Point Notation Guide:
//   FXP(16,0): Position values (cx, cy, h, x, y) - no fractional bits
//   FXP(8,8):  Fractional/velocity values (r, vx, vy, vh) - 8 fractional bits
//   FXP(12,4): Variance/covariance (P, S matrices) - 4 fractional bits
//
// Bit Shift Rules for Different Operations:
//   Variance/Variance (FXP12.4 / FXP12.4): No shift needed (frac bits cancel)
//   Variance^2/Variance (FXP*FXP / FXP12.4): Right shift by 4 bits
//   Gain calculation (FXP12.4 / FXP12.4 → FXP8.8): Left shift by 8 bits before div
//   Velocity/Velocity: Handle 8 fractional bits appropriately
//
// Parameters: shift_left = bits to shift A left before division (for FXP correction)
//             shift_right = bits to shift result right after division (for FXP correction)
// ============================================================
module Mult_Div_16bit(Y, A, B, C);
input [15:0] A, B, C;
output reg [15:0] Y;

reg [31:0] temp;

always @(*) begin
    temp = A * B;

    if (C == 0)
        Y = 16'd0;
    else
        Y = temp / C;
end

endmodule

// ============================================================
// MULT-DIV WITH EXPLICIT FIXED-POINT SHIFT CONTROL
// ============================================================
// Performs Y = ((A * B) >> shift_right) / C with left shift before division
// Useful for maintaining precision when converting between different FXP formats
// ============================================================
module Mult_Div_Shift_16bit(Y, A, B, C, shift_left, shift_right);
input [15:0] A, B, C;
input [3:0] shift_left, shift_right;
output reg [15:0] Y;

reg [31:0] temp;
reg [31:0] numerator;

always @(*) begin
    temp = A * B;
    numerator = temp << shift_left;

    if (C == 0)
        Y = 16'd0;
    else
        Y = (numerator / C) >> shift_right;
end

endmodule


// ============================================================
// PREDICT - Kalman Filter Prediction Step
// ============================================================
// Predicts state and covariance for next time step
// 
// Input Fixed-Point Formats:
//   cx, cy, h:  FXP(16,0) - Unsigned position/height
//   vx, vy, vh: FXP(8,8)  - Signed velocities
//   px, py, ph: FXP(12,4) - Unsigned variances (diagonal of P)
//   pr:         FXP(12,4) - Unsigned variance
//   pvx, pvy, pvh: FXP(12,4) - Unsigned velocity variances
//   p_cov_vx, p_cov_vy, p_cov_vh: FXP(8,8) - Signed cross-covariances
//   Q: Process noise covariance, each element FXP(8,8)
//
// State Prediction (Eq from Spec):
//   cx' = cx + vx  (FXP16.0 + FXP8.8 → right shift by 8)
//   cy' = cy + vy  (FXP16.0 + FXP8.8 → right shift by 8)
//   h'  = h + vh   (FXP16.0 + FXP8.8 → right shift by 8)
//
// Covariance Prediction (from Eq. P_k|k-1):
//   px' = px + 2*p_cov_vx + q_p  (shift by 1 for 2x multiplication)
//   py' = py + 2*p_cov_vy + q_p
//   ph' = ph + 2*p_cov_vh + q_p
//   pr' = pr + q_p
//   pvx' = pvx + q_v
//   pvy' = pvy + q_v
//   pvh' = pvh + q_v
// ============================================================
module Predict(
    cx, cy, h, vx, vy, vh,
    px, py, ph, pr, pvx, pvy, pvh,
    p_cov_vx, p_cov_vy, p_cov_vh,
    Q,

    x_out, y_out, h_out,
    px_out, py_out, ph_out, pr_out,
    p_vx_out, p_vy_out, p_vh_out,
    p_cov_vx_out, p_cov_vy_out, p_cov_vh_out
);

input [15:0] cx, cy, h, vx, vy, vh;
input [15:0] px, py, ph, pr, pvx, pvy, pvh;
input [15:0] p_cov_vx, p_cov_vy, p_cov_vh;
input [111:0] Q;

output reg [15:0] x_out, y_out, h_out;
output reg [15:0] px_out, py_out, ph_out, pr_out;
output reg [15:0] p_vx_out, p_vy_out, p_vh_out;
output reg [15:0] p_cov_vx_out, p_cov_vy_out, p_cov_vh_out;

always @(*) begin
    // State prediction: cx' = cx + vx
    // Note: vx is FXP(8,8), so right shift by 8 to get into FXP(16,0) range
    x_out = cx + (vx >>> 8);
    y_out = cy + (vy >>> 8);
    h_out = h + (vh >>> 8);

    // Covariance prediction (Eq. P_k|k-1 from spec)
    // px' = px + 2*p_cov_vx + Q[15:0]
    // p_cov_vx is FXP(8,8), left shift by 1 for 2x, then right shift by 8 for FXP conversion
    px_out = px + ((p_cov_vx << 1) >>> 8) + Q[15:0];
    py_out = py + ((p_cov_vy << 1) >>> 8) + Q[31:16];
    ph_out = ph + ((p_cov_vh << 1) >>> 8) + Q[47:32];

    // These elements don't couple with cross-covariances
    pr_out = pr + Q[63:48];
    p_vx_out = pvx + Q[79:64];
    p_vy_out = pvy + Q[95:80];
    p_vh_out = pvh + Q[111:96];

    // Cross-covariance update: sig_vx' = sig_vx + pvx
    // Both are FXP(8,8), no shift needed
    p_cov_vx_out = p_cov_vx + pvx;
    p_cov_vy_out = p_cov_vy + pvy;
    p_cov_vh_out = p_cov_vh + pvh;
end

endmodule


// ============================================================
// INNOVATE - Compute Innovation (Residual) and Innovation Covariance
// ============================================================
// Computes residual y = z - H*x and innovation covariance S = H*P*H^T + R
//
// Input Fixed-Point Formats:
//   z: [r(FXP8.8), h(FXP16.0), cy(FXP16.0), cx(FXP16.0)] - Measurement
//   x: [r(FXP8.8), h(FXP16.0), cy(FXP16.0), cx(FXP16.0)] - Predicted state
//   P: [pr(FXP12.4), ph(FXP12.4), py(FXP12.4), px(FXP12.4)] - Predicted variance (diagonal)
//   R: [rr(FXP8.8), rh(FXP8.8), ry(FXP8.8), rx(FXP8.8)] - Measurement noise covariance
//
// Output Fixed-Point Formats:
//   y: Innovation residual - same format as input difference
//   S: Innovation covariance - FXP(12,4) for position/size, FXP(8,8) for aspect ratio
//
// Innovation (Residual) from Eq. (Innovation_yk):
//   y = z - x  (Element-wise subtraction, formats preserved)
//
// Innovation Covariance from Eq. (Innovation_Cov_Actual):
//   S_i = P_i + R_i  (Diagonal addition, FXP formats mixed)
// ============================================================
module Innovate(
    z, x, P, R,
    y, S
);

input [63:0] z, x;
input [63:0] P, R;

output reg [63:0] y, S;

always @(*) begin
    // Residual: y = z - x (element-wise, preserves input formats)
    // Each 16-bit chunk is: measurement - prediction
    y[15:0]   = z[15:0]   - x[15:0];   // cx: FXP(16,0) - FXP(16,0) → FXP(16,0)
    y[31:16]  = z[31:16]  - x[31:16];  // cy: FXP(16,0) - FXP(16,0) → FXP(16,0)
    y[47:32]  = z[47:32]  - x[47:32];  // h: FXP(16,0) - FXP(16,0) → FXP(16,0)
    y[63:48]  = z[63:48]  - x[63:48];  // r: FXP(8,8) - FXP(8,8) → FXP(8,8)

    // Innovation covariance: S = P + R (diagonal elements)
    // Note: P contains variance (FXP12.4), R contains measurement noise (FXP8.8)
    // For proper addition, ensure R is converted to match P's fractional bits
    // R values in measurement noise typically smaller, stored as FXP(8,8)
    // When adding FXP(12,4) + FXP(8,8), result depends on alignment
    // For now treating as direct sum - user must ensure compatibility
    S[15:0]   = P[15:0]   + R[15:0];   // S_x: variance + noise for x
    S[31:16]  = P[31:16]  + R[31:16];  // S_y: variance + noise for y
    S[47:32]  = P[47:32]  + R[47:32];  // S_h: variance + noise for h
    S[63:48]  = P[63:48]  + R[63:48];  // S_r: variance + noise for aspect ratio
end

endmodule


// ============================================================
// UPDATE - Kalman Filter Update Step
// ============================================================
// Updates state and covariance using measurement and innovation
//
// Input Fixed-Point Formats:
//   x_in: [vh(FXP8.8), vy(FXP8.8), vx(FXP8.8), r(FXP8.8), h(FXP16.0), cy(FXP16.0), cx(FXP16.0)]
//   P_in: 10 elements total
//         - sigma^2_cx, sigma^2_cy, sigma^2_h, sigma^2_r: FXP(12,4)
//         - sigma^2_vx, sigma^2_vy, sigma^2_vh: FXP(12,4)
//         - sigma_vx, sigma_vy, sigma_vh: FXP(8,8)
//   S: Innovation covariance - FXP(12,4)
//   y: Innovation residual - FXP(16,0)
//
// Key Equations from Specification:
//   State Update (Eq. X_update):
//     x_k = x_k|k-1 + (sigma^2 / rho) * y
//     Computation: K*y = (sigma^2_fxp * y_fxp) / rho_fxp
//     Since sigma^2 is FXP(12,4) and y is FXP(16,0):
//     - temp = sigma^2_fxp * y_fxp → 4 fractional bits
//     - result = temp / rho_fxp → correct FXP(16,0) output (4 frac bits cancel)
//
//   Covariance Update (Eq. P-update):
//     P_k = sigma^2 - (sigma^4 / rho)
//     Computation: sigma^4 / rho = (sigma^2_fxp * sigma^2_fxp) / rho_fxp
//     Since both sigma^2 are FXP(12,4):
//     - temp = sigma^2_fxp * sigma^2_fxp → 8 fractional bits
//     - result = (temp / rho_fxp) >> 4 → FXP(12,4) output
//     Right shift by 4 needed to go from 8 frac bits to 4 frac bits
//
//   Cross-Covariance Update:
//     P_sig = sigma - (sigma^2 * sigma_cross / rho)
//     sigma^2 is FXP(12,4), sigma_cross is FXP(8,8):
//     - temp = sigma^2_fxp * sigma_cross_fxp → 12 fractional bits
//     - result = (temp / rho_fxp) >> 8 → FXP(8,8) output
// ============================================================
module Update(
    x_in, P_in, S, y,
    x_out, P_out
);

input [111:0] x_in;
input [159:0] P_in;
input [63:0] S, y;

output reg [111:0] x_out;
output reg [159:0] P_out;

// =====================
// Semantic mapping
// =====================

// Variances - FXP(12,4)
wire [15:0] sigma2_cx = P_in[159:144];
wire [15:0] sigma2_cy = P_in[143:128];
wire [15:0] sigma2_h  = P_in[127:112];
wire [15:0] sigma2_r  = P_in[111:96];

wire [15:0] sigma2_vx = P_in[95:80];
wire [15:0] sigma2_vy = P_in[79:64];
wire [15:0] sigma2_vh = P_in[63:48];

// Cross covariance - FXP(8,8)
wire [15:0] sigma_vx = P_in[47:32];
wire [15:0] sigma_vy = P_in[31:16];
wire [15:0] sigma_vh = P_in[15:0];

// =====================
// Gain * residual: K*y = (sigma^2 * y) / S
// =====================
wire [31:0] ky_cx_temp, ky_cy_temp, ky_h_temp, ky_r_temp;
wire [31:0] ky_vx_temp, ky_vy_temp, ky_vh_temp;
wire [15:0] k_cx, k_cy, k_h, k_r;
wire [15:0] k_vx, k_vy, k_vh;

// Multiply: sigma^2_fxp (FXP12.4) * y_fxp (FXP16.0) → temp (4 frac bits)
// Divide by S (FXP12.4): cancels the 4 frac bits, leaves 0 frac bits for FXP(16,0)
// Correct mapping of residual components (y):
// y[15:0] = cx residual, y[31:16] = cy, y[47:32] = h, y[63:48] = r
assign ky_cx_temp = sigma2_cx * $signed(y[15:0]);
assign ky_cy_temp = sigma2_cy * $signed(y[31:16]);
assign ky_h_temp  = sigma2_h  * $signed(y[47:32]);
assign ky_r_temp  = sigma2_r  * $signed(y[63:48]);

// Use corresponding residuals for cross-covariance updates (approximation)
assign ky_vx_temp = sigma_vx * $signed(y[15:0]);
assign ky_vy_temp = sigma_vy * $signed(y[31:16]);
assign ky_vh_temp = sigma_vh * $signed(y[47:32]);

// Divide and handle fixed-point: right shift by 4 to normalize result to FXP(16,0)
assign k_cx = S[63:48] != 0 ? (ky_cx_temp >>> 4) / S[63:48] : 16'd0;
assign k_cy = S[47:32] != 0 ? (ky_cy_temp >>> 4) / S[47:32] : 16'd0;
assign k_h  = S[31:16] != 0 ? (ky_h_temp >>> 4)  / S[31:16] : 16'd0;
assign k_r  = S[15:0]  != 0 ? (ky_r_temp >>> 4)  / S[15:0]  : 16'd0;

assign k_vx = S[63:48] != 0 ? (ky_vx_temp >>> 4) / S[63:48] : 16'd0;
assign k_vy = S[47:32] != 0 ? (ky_vy_temp >>> 4) / S[47:32] : 16'd0;
assign k_vh = S[31:16] != 0 ? (ky_vh_temp >>> 4) / S[31:16] : 16'd0;

// =====================
// Covariance update: P_new = P - (P^2 / S)
// =====================
wire [31:0] p_cx_sq_temp, p_cy_sq_temp, p_h_sq_temp, p_r_sq_temp;
wire [31:0] p_vx_sq_temp, p_vy_sq_temp, p_vh_sq_temp;
wire [31:0] p_cx_div_temp, p_cy_div_temp, p_h_div_temp, p_r_div_temp;
wire [31:0] p_vx_div_temp, p_vy_div_temp, p_vh_div_temp;
wire [15:0] p_cx_new, p_cy_new, p_h_new, p_r_new;
wire [15:0] p_vx_new, p_vy_new, p_vh_new;
wire [15:0] p_cov_vx, p_cov_vy, p_cov_vh;

// Variance update: (sigma^2 * sigma^2) / S
// Both sigma^2 are FXP(12,4), temp has 8 frac bits
// Right shift by 4 to normalize division result to FXP(12,4)
assign p_cx_sq_temp = sigma2_cx * sigma2_cx;
assign p_cy_sq_temp = sigma2_cy * sigma2_cy;
assign p_h_sq_temp  = sigma2_h  * sigma2_h;
assign p_r_sq_temp  = sigma2_r  * sigma2_r;

assign p_vx_sq_temp = sigma2_vx * sigma2_vx;
assign p_vy_sq_temp = sigma2_vy * sigma2_vy;
assign p_vh_sq_temp = sigma2_vh * sigma2_vh;

// Right shift by 4 (to go from 8 frac bits to 4 frac bits for FXP12.4)
assign p_cx_div_temp = p_cx_sq_temp >> 4;
assign p_cy_div_temp = p_cy_sq_temp >> 4;
assign p_h_div_temp  = p_h_sq_temp >> 4;
assign p_r_div_temp  = p_r_sq_temp >> 4;

assign p_vx_div_temp = p_vx_sq_temp >> 4;
assign p_vy_div_temp = p_vy_sq_temp >> 4;
assign p_vh_div_temp = p_vh_sq_temp >> 4;

assign p_cx_new = S[63:48] != 0 ? p_cx_div_temp / S[63:48] : sigma2_cx;
assign p_cy_new = S[47:32] != 0 ? p_cy_div_temp / S[47:32] : sigma2_cy;
assign p_h_new  = S[31:16] != 0 ? p_h_div_temp / S[31:16] : sigma2_h;
assign p_r_new  = S[15:0]  != 0 ? p_r_div_temp / S[15:0] : sigma2_r;

assign p_vx_new = S[63:48] != 0 ? p_vx_div_temp / S[63:48] : sigma2_vx;
assign p_vy_new = S[47:32] != 0 ? p_vy_div_temp / S[47:32] : sigma2_vy;
assign p_vh_new = S[31:16] != 0 ? p_vh_div_temp / S[31:16] : sigma2_vh;

// Cross-covariance update: sigma - (sigma^2 * sigma_cross / S)
// sigma^2 is FXP(12,4), sigma_cross is FXP(8,8) → temp has 12 fractional bits
// Right shift by 8 to normalize to FXP(8,8)
wire [31:0] p_scov_vx_temp, p_scov_vy_temp, p_scov_vh_temp;
wire [31:0] p_scov_vx_div_temp, p_scov_vy_div_temp, p_scov_vh_div_temp;

assign p_scov_vx_temp = sigma2_cx * sigma_vx;
assign p_scov_vy_temp = sigma2_cy * sigma_vy;
assign p_scov_vh_temp = sigma2_h  * sigma_vh;

// Right shift by 8 (to go from 12 frac bits to 4 frac bits before division)
assign p_scov_vx_div_temp = p_scov_vx_temp >> 8;
assign p_scov_vy_div_temp = p_scov_vy_temp >> 8;
assign p_scov_vh_div_temp = p_scov_vh_temp >> 8;

assign p_cov_vx = S[63:48] != 0 ? p_scov_vx_div_temp / S[63:48] : sigma_vx;
assign p_cov_vy = S[47:32] != 0 ? p_scov_vy_div_temp / S[47:32] : sigma_vy;
assign p_cov_vh = S[31:16] != 0 ? p_scov_vh_div_temp / S[31:16] : sigma_vh;

// =====================
// OUTPUT
// =====================
always @(*) begin
    // State update: x_k = x_k|k-1 + K*y
    x_out[111:96] = x_in[111:96] + k_cx;
    x_out[95:80]  = x_in[95:80]  + k_cy;
    x_out[79:64]  = x_in[79:64]  + k_h;
    x_out[63:48]  = x_in[63:48]  + k_r;
    x_out[47:32]  = x_in[47:32]  + k_vx;
    x_out[31:16]  = x_in[31:16]  + k_vy;
    x_out[15:0]   = x_in[15:0]   + k_vh;

    // Covariance update: P_k = P - (P^2/S)
    P_out[159:144] = sigma2_cx - p_cx_new;
    P_out[143:128] = sigma2_cy - p_cy_new;
    P_out[127:112] = sigma2_h  - p_h_new;
    P_out[111:96]  = sigma2_r  - p_r_new;

    P_out[95:80] = sigma2_vx - p_vx_new;
    P_out[79:64] = sigma2_vy - p_vy_new;
    P_out[63:48] = sigma2_vh - p_vh_new;

    // Cross covariance update: sigma - (sigma^2 * sigma_cross / S)
    P_out[47:32] = sigma_vx - p_cov_vx;
    P_out[31:16] = sigma_vy - p_cov_vy;
    P_out[15:0]  = sigma_vh - p_cov_vh;
end

endmodule
