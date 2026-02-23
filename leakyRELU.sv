`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/27/2025 09:55:26 PM
// Design Name: 
// Module Name: leakyRELU
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


module leaky_relu(
    input  logic clk,
    input  logic rst_n,
    input  logic [15:0] x,
    output logic [15:0] y
);

    logic [15:0] neg_scaled;
    logic [15:0] alpha = 16'h2E66;

    fp_mul mult (
        .clk(clk),
        .rst_n(rst_n),
        .a(x),
        .b(alpha),
        .result(neg_scaled)
    );
    

    assign y = x[15] ? neg_scaled : x; 

endmodule