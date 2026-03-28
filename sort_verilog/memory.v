

// 8 bit register
module Reg_8bit(Out, In, clk, rst, write_en);
input [7:0] In;
input clk, rst, write_en;
output reg [7:0] Out;

always @(posedge clk) begin
    if (rst == 1'b1)
        Out <= 8'b00000000;
    else if (write_en == 1'b0)
        Out <= Out;
    else
        Out <= In;
end

endmodule


// 16 bit register using 8 bit register
module Reg_16bit(Out, In, clk, rst, write_en);
input [15:0] In;
input rst, clk, write_en;

output wire [15:0] Out;

Reg_8bit inst_low(Out[7:0], In[7:0], clk, rst, write_en);
Reg_8bit inst_high(Out[15:8], In[15:8], clk, rst, write_en);

endmodule

// 32 bit register using 16 bit registers
module Reg_32bit(Out, In, clk, rst, write_en);
input [31:0] In;
input clk, rst, write_en;

output wire [31:0] Out;

Reg_16bit inst_low(Out[15:0], In[15:0], clk, rst, write_en);
Reg_16bit inst_high(Out[31:16], In[31:16], clk, rst, write_en);

endmodule


// 64 bit register using 32 bit registers

module Reg_64bit(Out, In, clk, rst, write_en);
input [63:0] In;
input clk, rst, write_en;

output wire [63:0] Out;

Reg_32bit inst_low(Out[31:0], In[31:0], clk, rst, write_en);
Reg_32bit inst_high(Out[63:32], In[63:32], clk, rst, write_en);

endmodule

module X_Block(        // Block to store the State register
    clk, rst, en, cx, cy, s, r, vx, vy, vs,
    cx_out, cy_out, s_out, r_out, vx_out, vy_out, vs_out
);

input clk, rst, en;
input [15:0] cx, cy, r, vx, vy, vs;
input [19:0] s;

output reg [15:0] cx_out, cy_out, r_out, vx_out, vy_out, vs_out;
output reg [19:0] s_out;

always @(posedge clk) begin
    if (rst == 1'b1) begin
        cx_out <= 16'b0000_0000_0000_0000;
        cy_out <= 16'b0000_0000_0000_0000;
        s_out <= 20'b0000_0000_0000_0000_0000;
        r_out <= 16'b0000_0000_0000_0000; 
        vx_out <= 16'b0000_0000_0000_0000;
        vy_out <= 16'b0000_0000_0000_0000;
        vs_out <= 16'b0000_0000_0000_0000;
    end
    else if (en == 1'b1) begin
        cx_out <= cx;
        cy_out <= cy;
        s_out <= s;
        r_out <= r;
        vx_out <= vx;
        vy_out <= vy;
        vs_out <= vs;
    end
    else;
end
endmodule

module P_block(     // block to hold the covariance matrix
    clk, rst, en, x, y, h, r, vx, vy, vh, cov_vx, cov_vy, cov_vh,
    x_out, y_out, h_out, r_out, vx_out, vy_out, vh_out, cov_vx_out, cov_vy_out, cov_vh_out
);

input clk, rst, en;
input [15:0] x, y, h, r, vx, vy, vh, cov_vx, cov_vy, cov_vh;

output reg [15:0] x_out, y_out, h_out, r_out, vx_out, vy_out, vh_out, cov_vx_out, cov_vy_out, cov_vh_out;

always @(posedge clk) begin
    if (rst == 1'b1) begin
        x_out <= 16'b0000_0000_0000_0000;
        y_out <= 16'b0000_0000_0000_0000;
        h_out <= 16'b0000_0000_0000_0000;
        r_out <= 16'b0000_0000_0000_0000;
        vx_out <= 16'b0000_0000_0000_0000;
        vy_out <= 16'b0000_0000_0000_0000;
        vh_out <= 16'b0000_0000_0000_0000;
        cov_vx_out <= 16'b0000_0000_0000_0000;
        cov_vy_out <= 16'b0000_0000_0000_0000;
        cov_vh_out <= 16'b0000_0000_0000_0000;
    end
    else if (en == 1'b1) begin
        x_out <= x;
        y_out <= y;
        h_out <= h;
        r_out <= r;
        vx_out <= vx;
        vy_out <= vy;
        vh_out <= vh;
        cov_vx_out <= cov_vx;
        cov_vy_out <= cov_vy;
        cov_vh_out <= vh_out;
    end
    else ;
end
endmodule

module Q_block(   // block to hold noise variance 
    clk, rst, en, x, y, h, r, vx, vy, vh,
    x_out, y_out, h_out, r_out, vx_out, vy_out, vh_out
);

input clk, rst, en;
input [16:0] x, y, h, r, vx, vy, vh;
output reg [16:0] x_out, y_out, h_out, r_out, vx_out, vy_out, vh_out;

always @(posedge clk) begin
    if (rst == 1'b1) begin
        x_out <= 16'b0000_0000_0000_0000;
        y_out <= 16'b0000_0000_0000_0000;
        h_out <= 16'b0000_0000_0000_0000;
        r_out <= 16'b0000_0000_0000_0000;
        vx_out <= 16'b0000_0000_0000_0000;
        vy_out <= 16'b0000_0000_0000_0000;
        vh_out <= 16'b0000_0000_0000_0000;
    end
    else if (en == 1'b1) begin
        x_out <= x;
        y_out <= y;
        h_out <= h;
        r_out <= r;
        vx_out <= vx;
        vy_out <= vy;
        vh_out <= vh;
    end
    else ;
end

endmodule
