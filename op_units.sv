`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/27/2025 09:59:06 PM
// Design Name: 
// Module Name: op_units
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module int_add ( //probably wrong
input  logic [15:0] a,
input  logic [15:0] b,
input  logic        clk,
input  logic        rst_n,
output logic [31:0] result
);

assign result = a + b;

endmodule


module bias_add ( //probably wrong
input  logic [15:0] a,
input  logic [15:0] b,
input  logic        clk,
input  logic        rst_n,
output logic [31:0] result
);

assign result = a + b;

endmodule


module int_mul ( //probably wrong
input  logic [7:0] a,
input  logic [7:0] b,
input  logic        clk,
input  logic        rst_n,
output logic [15:0] result
);

assign result  = a * b;

endmodule
