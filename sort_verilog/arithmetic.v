// 16 bit adder
module Adder_Subtractor_16bit(S, Cout, Overflow, A, B, Sub);
input [15:0] A, B;
input Sub;

output wire Cout, Overflow;
output wire [15:0] Sum;

wire [16:0] S_temp;
wire [15:0] B_mask;

assign B_mask = B ^ {16{Sub}};
assign S_temp = A + B_mask + Sub;
assign {Cout, S} = S_temp;
assign Overflow = (A[15] == B_mask[15]) && (S[15] != A[15]);
endmodule

module Division_16bit(Y, A, B);
input [15:0] A, B;
output reg [15:0] Y;

endmodule


module Mult_Div_16bit(Y, A, B, C);
input [15:0] A, B, C;
output reg [15:0] Y;

reg [31:0] temp;

always @(*) begin
    temp = A * B;
    Y = temp / C;
end

endmodule
