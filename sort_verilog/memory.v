

// 8 bit register
module Reg_8bit(Out, In, clk, rst);
input [7:0] In;
input clk, rst;
output reg [7:0] Out;

always @(posedge clk) begin
    if (rst == 1'b1)
        Out <= 8'b00000000;
    else
        Out <= In;
end

endmodule


// 16 bit register using 8 bit register
module Reg_16bit(Out, In, clk, rst);
input [15:0] In;
input rst, clk;

output wire [15:0] Out;

Reg_8bit inst_low(Out[7:0], In[7:0], clk, rst);
Reg_8bit inst_high(Out[15:8], In[15:8], clk, rst);

endmodule

// 32 bit register using 16 bit registers
module Reg_32bit(Out, In, clk, rst);
input [31:0] In;
input clk, rst;

output wire [31:0] Out;

Reg_16bit inst_low(Out[15:0], In[15:0], clk, rst);
Reg_16bit inst_high(Out[31:16], In[31:16], clk, rst);

endmodule


// 64 bit register using 32 bit registers

module Reg_64bit(Out, In, clk, rst);
input [63:0] In;
input clk, rst;

output wire [63:0] Out;

Reg_32bit inst_low(Out[31:0], In[31:0], clk, rst);
Reg_32bit inst_high(Out[63:32], In[63:32], clk, rst);

endmodule
