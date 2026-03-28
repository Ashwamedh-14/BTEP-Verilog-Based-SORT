module Predict(
    cx, cy, h, vx, vy, vh, // State Vector inputs
    px, py, ph, pr, pvx, pvy, pvh, p_cov_vx, p_cov_vy, p_cov_vh, // State Covariance Matrix Input
    Q, // Matrix Q
    x_out, y_out, h_out, // State Vector predicted output
    px_out, py_out, ph_out, pr_out, p_vx_out, p_vy_out, p_vh_out, p_cov_vx_out, p_cov_vy_out, p_cov_vh_out
);

input [15:0] cx, cy, h, vx, vy, vh, px, py, ph, pr, pvx, pvy, pvh, p_cov_vx, p_cov_vy, p_cov_vh;
input [111:0] Q; // 7 X 16 bits = 112 bits, 7 elements each of 16 bits

output reg [15:0] x_out, y_out, h_out, px_out, py_out, ph_out, pr_out, p_vx_out, p_vy_out, p_vh_out;
output reg [15:0] p_cov_vx_out, p_cov_vy_out, p_cov_vh_out;

always @(*) begin
    x_out = cx + vx;
    y_out = cy + vy;
    h_out = h + vh;

    px_out = px + (p_cov_vx << 1) + Q[15:0];
    py_out =py + (p_cov_vy << 1) + Q[31:16];
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


module Innovate(
    z, x, P, R, y, S
);
input [63:0] z, x, R, P;
output reg [63:0] y, S;

always @(*) begin
    y[15:0] = z[15:0] - x[15:0];
    y[31:16] = z[31:16] - x[31:16];
    y[47:32] = z[47:32] - x[47:32];
    y[63:48] = z[63:48] - x[63:48];

    S[15:0] = P[15:0] + R[15:0];
    S[31:16] = P[31:16] + R[31:16];
    S[47:32] = P[47:32] + R[47:32];
    S[63:48] = P[63:48] + R[63:48];
end
endmodule

module Predict(x, y, )

module KalmanBlock(); // Entire Kalman Block

endmodule
