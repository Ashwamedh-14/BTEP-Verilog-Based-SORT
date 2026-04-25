
// Module to convert State Vector to Bounding box
// X = State_Vector[0:4], i.e, X = [cx, cy, h, r]
module ST_to_BBox(X, x1, y1, x2, y2);
input [63:0] X;

output reg [16:0] x1, y1, x2, y2;
reg [15:0] w;

always @(*) begin
    w = X[47:32] * X[63:48]; // h * r = w
    x1 = X[15:0] - (w >> 1); // cx - w/2
    x2 = X[15:0] + (w >> 1); // cx + w/2
    y1 = X[31:16] - (X[47:32] >> 1); // cy - h/2
    y2 = X[31:16] + (X[47:32] >> 1); // cy + h/2
end
endmodule

// Module to conver Bounding Box to State Vector
// X = State_Vector[0:4], i.e, X = [cx, cy, h, r]
module BBox_to_ST(x1, y1, x2, y2, X);
input [15:0] x1, y1, x2, y2;
output reg [63:0] X;

reg [15:0] w, h; // width, height

always @(*) begin
    w = x2 - x1;
    h = y2 - y1;

    X[63:48] = w / h;
    X[47:32] = h;
    X[15:0] = x1 + (w >> 1); // cx = x1 + w/2;
    X[31:16] = y1 + (h >> 1); // cy = y1 + h/2;
end

endmodule




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

reg [15:0] xx1, yy1, xx2, yy2; // To store the corresponding lower left and upper right corner
reg [15:0] h, w; // height and width of the bounding box
reg [31:0] area_u, area_i, area_a, area_b;  // Union Area, Intersection Area, Area of Box A, Area of Box B

always @(*) begin
    // Finding the lower left and upper right point of the intersecion
    xx1 = (Ax1 > Bx1) ? Ax1 : Bx1;
    yy1 = (Ay1 > By1) ? Ay1 : By1;
    xx2 = (Ax2 > Bx2) ? Bx2 : Ax2;
    yy2 = (Ay2 > By2) ? By2 : Ay2;

    // Finding the height, width, and area of the intersecting box
    w = (xx2 > xx1) ? xx2 - xx1 : 0;
    h = (yy2 > yy1) ? yy2 - yy1 : 0;
    area_i = w * h;

    // Finding the areas of A, B, and Union
    area_a = (Ax2 - Ax1) * (Ay2 - Ay1);
    area_b = (Bx2 - Bx1) * (By2 - By1);
    area_u = area_a + area_b - area_i; // A + B - (intersection of (A,B))

    // Sending a signal
    valid = (10 * area_i > threshold * area_u) ? 1'b1 : 1'b0;
end

endmodule
