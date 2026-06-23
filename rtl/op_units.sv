`timescale 1ns / 1ps

module int_mul (
    input  logic signed [7:0]  a,
    input  logic signed [7:0]  b,
    input  logic               clk,
    input  logic               rst_n,
    output logic signed [31:0] result
);
    logic signed [31:0] a_ext, b_ext;
    assign a_ext = {{24{a[7]}}, a};
    assign b_ext = {{24{b[7]}}, b};
    assign result = a_ext * b_ext;
endmodule


module int_add (
    input  logic signed [31:0] a,
    input  logic signed [31:0] b,
    input  logic               clk,
    input  logic               rst_n,
    output logic signed [31:0] result
);
    assign result = a + b;
endmodule


module bias_add (
    input  logic signed [31:0] a,
    input  logic signed [31:0] b,
    input  logic               clk,
    input  logic               rst_n,
    output logic signed [31:0] result
);
    assign result = a + b;
endmodule
