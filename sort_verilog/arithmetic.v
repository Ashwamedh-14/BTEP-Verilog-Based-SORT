// ============================================================
// 16-bit Adder/Subtractor with Fixed-Point Support
// ============================================================
// Supports mixed fixed-point formats:
//  - FXP(16,0): Unsigned position values (cx, cy, h, x, y, h)
//  - FXP(8,8):  Fractional values (r, vx, vy, vh, velocities)
//  - FXP(12,4): Covariance/variance values (P matrix, S matrix)
//
// All operands must have compatible fractional bit positions
// when adding/subtracting different signal types
// ============================================================
module Adder_Subtractor_16bit(Sum, Cout, Overflow, A, B, Sub);
input [15:0] A, B;
input Sub;

output wire Cout, Overflow;
output wire [15:0] Sum;

wire [16:0] S_temp;
wire [15:0] B_mask;

assign B_mask = B ^ {16{Sub}};
assign S_temp = A + B_mask + Sub;
assign {Cout, Sum} = S_temp;
assign Overflow = (A[15] == B_mask[15]) && (Sum[15] != A[15]);
endmodule



