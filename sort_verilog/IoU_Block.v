
// ============================================================
// Module to convert State Vector to Bounding Box
// ============================================================
// Converts Kalman filter state vector to bounding box coordinates
//
// Input X format: [r(FXP8.8), h(FXP16.0), cy(FXP16.0), cx(FXP16.0)]
// cx, cy: Center coordinates - FXP(16,0)
// h: Height - FXP(16,0)
// r: Aspect ratio (w/h) - FXP(8,8)
//
// Output: Bounding box corners (17-bit to prevent overflow)
// x1, y1: Lower-left corner
// x2, y2: Upper-right corner
//
// Fixed-Point Calculations:
//   w = h * r = FXP(16,0) * FXP(8,8) → FXP(24,8)
//   Result should be FXP(16,0), so right shift by 8
//   x1 = cx - (w >> 1)
//   x2 = cx + (w >> 1)
//   y1 = cy - (h >> 1)
//   y2 = cy + (h >> 1)
// ============================================================
module ST_to_BBox(X, x1, y1, x2, y2);
input [63:0] X;

output reg [16:0] x1, y1, x2, y2;
reg [15:0] w;
wire [15:0] cx, cy, h, r;

assign cx = X[15:0];
assign cy = X[31:16];
assign h  = X[47:32];
assign r  = X[63:48];

always @(*) begin
    // w = h * r, with FXP shift: (FXP16.0 * FXP8.8) >> 8 → FXP16.0
    w = (h * r) >> 8;
    
    // Bounding box corners
    x1 = cx - (w >> 1); // cx - w/2 (lower-left x)
    x2 = cx + (w >> 1); // cx + w/2 (upper-right x)
    y1 = cy - (h >> 1); // cy - h/2 (lower-left y)
    y2 = cy + (h >> 1); // cy + h/2 (upper-right y)
end
endmodule

// ============================================================
// Module to convert Bounding Box to State Vector
// ============================================================
// Converts bounding box coordinates back to Kalman filter state vector
//
// Input: Bounding box corners in FXP(16,0)
// x1, y1: Lower-left corner
// x2, y2: Upper-right corner
//
// Output X format: [r(FXP8.8), h(FXP16.0), cy(FXP16.0), cx(FXP16.0)]
//
// Fixed-Point Calculations:
//   w = x2 - x1 (FXP16.0)
//   h = y2 - y1 (FXP16.0)
//   cx = x1 + (w >> 1) (FXP16.0)
//   cy = y1 + (h >> 1) (FXP16.0)
//   r = w / h with FXP shift: (w << 8) / h → FXP8.8
// ============================================================
module BBox_to_ST(x1, y1, x2, y2, X);
input [15:0] x1, y1, x2, y2;
output reg [63:0] X;

reg [15:0] w, h; // width, height
reg [31:0] w_extended;

always @(*) begin
    w = x2 - x1;
    h = y2 - y1;

    X[15:0]  = x1 + (w >> 1); // cx = x1 + w/2 (center x, FXP16.0)
    X[31:16] = y1 + (h >> 1); // cy = y1 + h/2 (center y, FXP16.0)
    X[47:32] = h;             // height h (FXP16.0)
    
    // r = w/h in FXP8.8 format
    // To maintain precision: (w << 8) / h → FXP8.8 with division
    // Use extended precision to avoid overflow during shift
    if (h != 0) begin
        w_extended = w << 8;
        X[63:48] = (w_extended / h) & 16'hFFFF;  // Mask to 16 bits for safety
    end else begin
        X[63:48] = 16'd0; // Avoid division by zero
    end
end

endmodule




// ============================================================
// IOU (Intersection Over Union) Module
// ============================================================
// Computes IoU between predicted and detected bounding boxes
// All coordinates are in FXP(16,0) format (integer pixel values)
//
// Inputs:
//   Ax1, Ay1, Ax2, Ay2: Bounding box A (predicted) - FXP(16,0)
//   Bx1, By1, Bx2, By2: Bounding box B (detected) - FXP(16,0)
//   threshold: IoU threshold (scaled by 10), typical range 0-10
//
// Output:
//   valid: 1 if IoU > threshold/10, else 0
//
// Calculation:
//   IoU = area_intersection / area_union
//   Decision: valid = (10 * area_i > threshold * area_u) ? 1 : 0
//   This avoids division: 10*a/u > threshold ⟺ 10*a > threshold*u
//
// Fixed-Point Note:
//   All area calculations use 32-bit arithmetic to prevent overflow
//   when multiplying 16-bit dimensions (max 640x640 image)
// ============================================================
// x2, y2 represent top right corner and x1, y1 represent the lower left
// corner
// Input the threshold as an integer between 0 and 10
module IOU#(
    parameter threshold = 3
)(
    Ax1, Ay1, Ax2, Ay2,
    Bx1, By1, Bx2, By2,
    valid
);

input [15:0] Ax1, Ay1, Ax2, Ay2, Bx1, By1, Bx2, By2;
output reg valid;

reg [15:0] xx1, yy1, xx2, yy2; // Intersection box corners
reg [15:0] h, w; // height and width of the bounding box
reg [31:0] area_u, area_i, area_a, area_b;  // Union, Intersection, Area A, Area B

always @(*) begin
    // Finding the lower-left and upper-right corners of the intersection box
    xx1 = (Ax1 > Bx1) ? Ax1 : Bx1;
    yy1 = (Ay1 > By1) ? Ay1 : By1;
    xx2 = (Ax2 < Bx2) ? Ax2 : Bx2;
    yy2 = (Ay2 < By2) ? Ay2 : By2;

    // Finding the height, width, and area of the intersecting box
    // Both width and height are in FXP(16,0), so no shift needed
    w = (xx2 > xx1) ? xx2 - xx1 : 16'd0;
    h = (yy2 > yy1) ? yy2 - yy1 : 16'd0;
    area_i = w * h;  // 32-bit result

    // Finding the areas of A and B
    // Each area is product of two FXP(16,0) values = 32-bit result
    area_a = (Ax2 - Ax1) * (Ay2 - Ay1);
    area_b = (Bx2 - Bx1) * (By2 - By1);
    
    // Union area: A + B - Intersection
    area_u = area_a + area_b - area_i;

    // IoU comparison: 10 * intersection / union > threshold
    // Avoids division by comparing: 10 * area_i > threshold * area_u
    valid = (10 * area_i > threshold * area_u) ? 1'b1 : 1'b0;
end

endmodule
