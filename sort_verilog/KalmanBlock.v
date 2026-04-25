// ============================================================
// MULT-DIV WITH SAFETY
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
// PREDICT
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
    // State prediction
    x_out = cx + vx;
    y_out = cy + vy;
    h_out = h + vh;

    // Covariance prediction (from your Eq. 9)
    px_out = px + (p_cov_vx << 1) + Q[15:0];
    py_out = py + (p_cov_vy << 1) + Q[31:16];
    ph_out = ph + (p_cov_vh << 1) + Q[47:32];

    pr_out = pr + Q[63:48];
    p_vx_out = pvx + Q[79:64];
    p_vy_out = pvy + Q[95:80];
    p_vh_out = pvh + Q[111:96];

    p_cov_vx_out = p_cov_vx + pvx;
    p_cov_vy_out = p_cov_vy + pvy;
    p_cov_vh_out = p_cov_vh + pvh;
end

endmodule


// ============================================================
// INNOVATION
// ============================================================
module Innovate(
    z, x, P, R,
    y, S
);

input [63:0] z, x;
input [63:0] P, R;

output reg [63:0] y, S;

always @(*) begin
    // Residual
    y[15:0]   = z[15:0]   - x[15:0];
    y[31:16]  = z[31:16]  - x[31:16];
    y[47:32]  = z[47:32]  - x[47:32];
    y[63:48]  = z[63:48]  - x[63:48];

    // Innovation covariance (diagonal)
    S[15:0]   = P[15:0]   + R[15:0];
    S[31:16]  = P[31:16]  + R[31:16];
    S[47:32]  = P[47:32]  + R[47:32];
    S[63:48]  = P[63:48]  + R[63:48];
end

endmodule


// ============================================================
// UPDATE (CLEAN + SEMANTIC)
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

// Variances
wire [15:0] sigma2_cx = P_in[159:144];
wire [15:0] sigma2_cy = P_in[143:128];
wire [15:0] sigma2_h  = P_in[127:112];
wire [15:0] sigma2_r  = P_in[111:96];

wire [15:0] sigma2_vx = P_in[95:80];
wire [15:0] sigma2_vy = P_in[79:64];
wire [15:0] sigma2_vh = P_in[63:48];

// Cross covariance
wire [15:0] sigma_vx = P_in[47:32];
wire [15:0] sigma_vy = P_in[31:16];
wire [15:0] sigma_vh = P_in[15:0];

// =====================
// Gain * residual
// =====================
wire [15:0] k_cx, k_cy, k_h, k_r;
wire [15:0] k_vx, k_vy, k_vh;

// K*y
Mult_Div_16bit u1(k_cx, sigma2_cx, y[63:48], S[63:48]);
Mult_Div_16bit u2(k_cy, sigma2_cy, y[47:32], S[47:32]);
Mult_Div_16bit u3(k_h , sigma2_h , y[31:16], S[31:16]);
Mult_Div_16bit u4(k_r , sigma2_r , y[15:0] , S[15:0]);

Mult_Div_16bit u5(k_vx, sigma_vx, y[63:48], S[63:48]);
Mult_Div_16bit u6(k_vy, sigma_vy, y[47:32], S[47:32]);
Mult_Div_16bit u7(k_vh, sigma_vh, y[31:16], S[31:16]);

// =====================
// Covariance update
// =====================
wire [15:0] p_cx_new, p_cy_new, p_h_new, p_r_new;
wire [15:0] p_vx_new, p_vy_new, p_vh_new, p_cov_vx, p_cov_vy, p_cov_vh;

// P - P^2 / S
Mult_Div_16bit p1(p_cx_new, sigma2_cx, sigma2_cx, S[63:48]);
Mult_Div_16bit p2(p_cy_new, sigma2_cy, sigma2_cy, S[47:32]);
Mult_Div_16bit p3(p_h_new , sigma2_h , sigma2_h , S[31:16]);
Mult_Div_16bit p4(p_r_new , sigma2_r , sigma2_r , S[15:0]);

Mult_Div_16bit p5(p_vx_new, sigma2_vx, sigma2_vx, S[63:48]);
Mult_Div_16bit p6(p_vy_new, sigma2_vy, sigma2_vy, S[47:32]);
Mult_Div_16bit p7(p_vh_new, sigma2_vh, sigma2_vh, S[31:16]);

Mult_Div_16bit p8(p_cov_vx, sigma2_cx, sigma_vx, S[63:48]);
Mult_Div_16bit p9(p_cov_vy, sigma2_cy, sigma_vy, S[47:32]);
Mult_Div_16bit p10(p_cov_vh, sigma2_h, sigma_vh, S[31:16]);

// =====================
// OUTPUT
// =====================
always @(*) begin

    // State update
    x_out[111:96] = x_in[111:96] + k_cx;
    x_out[95:80]  = x_in[95:80]  + k_cy;
    x_out[79:64]  = x_in[79:64]  + k_h;
    x_out[63:48]  = x_in[63:48]  + k_r;
    x_out[47:32]  = x_in[47:32]  + k_vx;
    x_out[31:16]  = x_in[31:16]  + k_vy;
    x_out[15:0]   = x_in[15:0]   + k_vh;

    // Covariance update
    P_out[159:144] = sigma2_cx - p_cx_new;
    P_out[143:128] = sigma2_cy - p_cy_new;
    P_out[127:112] = sigma2_h  - p_h_new;
    P_out[111:96]  = sigma2_r  - p_r_new;

    P_out[95:80] = sigma2_vx - p_vx_new;
    P_out[79:64] = sigma2_vy - p_vy_new;
    P_out[63:48] = sigma2_vh - p_vh_new;

    // Cross covariance (approx update)
    P_out[47:32] = sigma_vx - p_cov_vx;
    P_out[31:16] = sigma_vy - p_cov_vy;
    P_out[15:0]  = sigma_vh - p_cov_vh;

end

endmodule
