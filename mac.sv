`timescale 1ns / 1ps

// 64-wide parallel multiply-accumulate.
// Inputs are int8; intermediates and output are int32.
// out_valid = in_valid — the combinational tree is immediate once inputs are stable.
module mac_array (
    input  logic clk,
    input  logic rst_n,
    input  logic in_valid,
    input  logic signed [7:0]  weights [63:0],
    input  logic signed [7:0]  ifmap   [63:0],
    output logic signed [31:0] result,
    output logic out_valid
);
    logic signed [31:0] mult_result [0:63];
    logic signed [31:0] s1 [0:31];
    logic signed [31:0] s2 [0:15];
    logic signed [31:0] s3 [0:7];
    logic signed [31:0] s4 [0:3];
    logic signed [31:0] s5 [0:1];
    logic signed [31:0] s_res;

    genvar i, j;

    // No pipeline delay needed: tree is fully combinational, and the caller
    // already has a 1-cycle register stage on the input data (buffer lookup).
    assign out_valid = in_valid;
    assign result    = s_res;

    generate
        for (i = 0; i < 64; i++) begin : gen_mul
            int_mul b_mul (
                .rst_n(rst_n), .clk(clk),
                .a(weights[i]), .b(ifmap[i]),
                .result(mult_result[i])
            );
        end
    endgenerate

    generate
        for (j = 0; j < 32; j++) begin : gen_add32
            int_add b_32 (.rst_n(rst_n), .clk(clk),
                .a(mult_result[2*j]), .b(mult_result[2*j+1]), .result(s1[j]));
        end
        for (j = 0; j < 16; j++) begin : gen_add16
            int_add b_16 (.rst_n(rst_n), .clk(clk),
                .a(s1[2*j]), .b(s1[2*j+1]), .result(s2[j]));
        end
        for (j = 0; j < 8; j++) begin : gen_add8
            int_add b_8 (.rst_n(rst_n), .clk(clk),
                .a(s2[2*j]), .b(s2[2*j+1]), .result(s3[j]));
        end
        for (j = 0; j < 4; j++) begin : gen_add4
            int_add b_4 (.rst_n(rst_n), .clk(clk),
                .a(s3[2*j]), .b(s3[2*j+1]), .result(s4[j]));
        end
        for (j = 0; j < 2; j++) begin : gen_add2
            int_add b_2 (.rst_n(rst_n), .clk(clk),
                .a(s4[2*j]), .b(s4[2*j+1]), .result(s5[j]));
        end
        int_add b_final (.rst_n(rst_n), .clk(clk),
            .a(s5[0]), .b(s5[1]), .result(s_res));
    endgenerate

endmodule


module accumulator (
    input  logic clk,
    input  logic rst_n,
    input  logic valid,
    input  logic clr,
    input  logic signed [31:0] accum_in,
    output logic signed [31:0] accum_out
);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accum_out <= '0;
        end else begin
            if (clr)
                accum_out <= '0;
            else if (valid)
                accum_out <= accum_out + accum_in;
        end
    end
endmodule


// Wraps mac_array + accumulator.
// result = raw 32-bit accumulated dot product (bias and activation applied in conv_2d).
// out_valid mirrors in_valid; accum_out becomes valid after the last chunk is processed.
module conv_iteration (
    input  logic clk,
    input  logic rst_n,
    input  logic in_valid,
    input  logic accum_clr,
    input  logic signed [7:0]  weights [63:0],
    input  logic signed [7:0]  ifmap   [63:0],
    output logic signed [31:0] result,
    output logic out_valid
);
    logic signed [31:0] mac_result;
    logic mac_out_valid;

    mac_array conv_mac (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid),
        .weights(weights), .ifmap(ifmap),
        .result(mac_result), .out_valid(mac_out_valid)
    );

    accumulator acc (
        .clk(clk), .rst_n(rst_n),
        .clr(accum_clr),
        .valid(mac_out_valid),
        .accum_in(mac_result),
        .accum_out(result)
    );

    assign out_valid = mac_out_valid;
endmodule


module shift_reg
    #(parameter DATA_WIDTH = 1)
    (input  logic clk,
     input  logic resetn,
     input  logic din,
     output logic [DATA_WIDTH-1:0] dout);

    logic [DATA_WIDTH-1:0] shiftVal;
    always_ff @(posedge clk or negedge resetn) begin
        if (!resetn)
            shiftVal <= '0;
        else
            shiftVal <= {shiftVal[DATA_WIDTH-2:0], din};
    end
    assign dout = shiftVal;
endmodule
