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

module Update(
    x_in, P_in, S, y,
    x_out, P_out
);
input [111:0] x_in;
input [159:0] P_in;
input [63:0] S, y;

output reg [111:0] x_out;
output reg [159:0] P_out;

wire [15:0] ky1, ky2, ky3, ky4, ky5, ky6, ky7;
wire [15:0] kp1, kp2, kp3, kp4, kp5, kp6, kp7, kp8, kp9, kp10;

Mult_Div_16bit inst1(ky1, P[159:144], y[63:48], S[63:48]);
Mult_Div_16bit inst2(ky2, P[143:128], y[47:32], S[47:32]);
Mult_Div_16bit inst3(ky3, P[127:112], y[31:16], S[31:16]);
Mult_Div_16bit inst4(ky4, P[111:96], y[15:0], S[15:0]);
Mult_Div_16bit inst5(ky5, P[47:32], y[63:48], S[63:48]);
Mult_Div_16bit inst6(ky6, P[31:16], y[47:32], S[47:32]);
Mult_Div_16bit inst7(ky7, P[15:0], y[31:16], S[31:16]);

always @(*) begin
    
    // X block
    x_out[111:96] = x_in[111:96] + ky1;
    x_out[95:80] = x_in[95:80] + ky2;
    x_out[79:64] = x_in[79:64] + ky3;
    x_out[63:48] = x_in[63:48] + ky4;
    x_out[47:32] = x_in[47:32] + ky5;
    x_out[31:16] = x_in[31:16] + ky6;
    x_out[15:0] = x_in[15:0] + ky7;

    // P Block
end

endmodule



module KalmanBlock(
    

); // Entire Kalman Block

endmodule
