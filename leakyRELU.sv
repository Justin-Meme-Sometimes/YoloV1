`timescale 1ns / 1ps

// alpha = 1/8 = 0.125 (close to standard 0.1), applied via arithmetic right shift.
// Operates on 32-bit accumulated value before int8 clipping.
module leaky_relu (
    input  logic signed [31:0] x,
    output logic signed [31:0] y
);
    assign y = x[31] ? (x >>> 3) : x;
endmodule
