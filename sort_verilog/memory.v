// ============================================================
// MEMORY UTILITIES MODULE - Helper Functions and Utilities
// ============================================================
// This file contains utility modules for the SORT tracker.
// Main modules (Predict, Innovate, Update) are in KalmanBlock.v
// Coordinate conversion modules are in IoU_Block.v
//
// FIXED-POINT NOTATION REFERENCE:
//   FXP(16,0): Integer pixel positions (cx, cy, h, x, y)
//   FXP(8,8):  Fractional velocities (r, vx, vy, vh) - 8 bits fractional
//   FXP(12,4): Variance/covariance values (P, S matrices) - 4 bits fractional
// ============================================================

// ============================================================
// MULT-DIV WITH SAFETY AND FIXED-POINT SUPPORT
// ============================================================
// Performs Y = (A * B) / C with overflow protection
// Returns 0 if divisor C is 0
//
// IMPORTANT: This module does NOT handle fixed-point bit shifting.
// Caller must apply appropriate right shifts based on input formats:
//   - FXP(12,4) * FXP(12,4) / FXP(12,4): >> 4 to normalize result
//   - FXP(16,0) * FXP(8,8) / FXP(12,4): >> 4 to normalize result  
//   - FXP(12,4) * FXP(8,8) / FXP(12,4): >> 8 to normalize result
//
// For fixed-point aware operations with automatic shifting,
// use Mult_Div_Shift_16bit module instead.
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
