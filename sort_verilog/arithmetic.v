

// half adder
module Half_Adder(S, Cout, A, B);
input A, B;
output wire S, Cout;

assign S = A ^ B;
assign Cout = A & B;

endmodule


// full adder
module Full_Adder(S, Cout, A, B, Cin);
input A, B, Cin;
output wire S, Cout;
wire [1:0] temp;

assign S = Cin ^ A ^ B;
assign temp[0] = A & B;
assign temp[1] = Cin & (A ^ B);
assign Cout = temp[0] | temp[1];

endmodule

// 16 bit adder
module Adder_16bit_Signed(S, Cout, A, B, Cin);
input [15:0] A, B;
input Cin;

output wire [15:0] S;
output wire Cout;



endmodule
