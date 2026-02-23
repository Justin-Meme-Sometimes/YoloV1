// `timescale 1ns / 1ps
// //////////////////////////////////////////////////////////////////////////////////
// // Company: 
// // Engineer: 
// // 
// // Create Date: 12/27/2025 09:58:23 PM
// // Design Name: 
// // Module Name: mac
// // Project Name: 
// // Target Devices: 
// // Tool Versions: 
// // Description: 
// // 
// // Dependencies: 
// // 
// // Revision:
// // Revision 0.01 - File Created
// // Additional Comments:
// // 
// //////////////////////////////////////////////////////////////////////////////////


module mac_array (
    input  logic clk,
    input  logic rst_n,
    input  logic in_valid,
    input  logic [15:0] weights [63:0],
    input  logic [15:0] ifmap   [63:0],
    output logic [15:0] result,
    output logic out_valid);

    // NUM_MACS parallel multiply-accumulates
    // Each MAC: acc = acc + (weight × ifmap)
    // Final result: sum of all accumulators
    logic [15:0] mult_result [0:63];
    logic [15:0] s1 [0:31];
    logic [15:0] s2 [0:15];
    logic [15:0] s3 [0:7];
    logic [15:0] s4 [0:3];
    logic [15:0] s5 [0:1];
    logic [15:0] s_res;

    logic [29:0] out_val_n;
    genvar i, j;

    shift_reg #(6) s_r (.clk(clk), .resetn(rst_n), .din(in_valid), .dout(out_val_n));

    assign out_valid = out_val_n[29];
    assign result = s_res;

    generate 
        for(i = 0; i < 64; i++) begin //5 cycles each
            int_mul b_mul (.rst_n(rst_n), .clk(clk), .a(weights[i]), .b(ifmap[i]), .result(mult_result[i]));
        end
    endgenerate

    generate
        for(j = 0; j < 32; j++)  begin //5 cycles each
            int_add b_32 (.rst_n(rst_n), .clk(clk), .a(mult_result[2*j]), .b(mult_result[2*j+1]), .result(s1[j]));
        end
        for(j = 0; j < 16; j++)  begin //5 cycles each
            int_add b_16 (.rst_n(rst_n), .clk(clk), .a(s1[2*j]), .b(s1[2*j+1]), .result(s2[j]));
        end
        for(j = 0; j < 8; j++)  begin //5 cycles each
            int_add b_8 (.rst_n(rst_n), .clk(clk), .a(s2[2*j]), .b(s2[2*j+1]), .result(s3[j]));
        end
        for(j = 0; j < 4; j++)  begin //5 cycles each 
            int_add b_4 (.rst_n(rst_n), .clk(clk), .a(s3[2*j]), .b(s3[2*j+1]), .result(s4[j]));
        end 
        for(j = 0; j < 2; j++)  begin //5 cycles each 
            int_add b_2 (.rst_n(rst_n), .clk(clk), .a(s4[2*j]), .b(s4[2*j+1]), .result(s5[j]));
        end 
        //5 cycles each
        int_add b_final (.rst_n(rst_n), .clk(clk), .a(s5[0]), .b(s5[1]), .result(s_res));
    endgenerate
endmodule

module accumulator (input logic clk,
                    input logic rst_n,
                    input logic valid,
                    input logic clr,
                    input logic [31:0] accum_in,
                    output logic [31:0] accum_out);
    
    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            accum_out <= '0;
        end else begin
            if (clr) begin
                accum_out <= 0;
            end else if(valid) begin
                accum_out <=  accum_out + accum_in;
            end
        end
    end

endmodule


module conv_iteration (
    input  logic clk,
    input  logic rst_n,
    input  logic in_valid,
    input  logic accum_clr,
    input  logic [15:0] weights [63:0],
    input  logic [15:0] ifmap   [63:0],
    input logic [15:0] bias,
    output logic [15:0] result,
    output logic out_valid);

    logic [15:0] mac_result;
    logic [15:0] biased_sum;
    logic [32:0] accum_out;
    logic mac_out_valid, accum_valid;
    logic out_valid_n;

    mac_array conv_mac (.clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .weights(weights),
    .ifmap(ifmap),
    .result(mac_result),
    .out_valid(mac_out_valid));

    accumulator acc 
    (.clk(clk),
     .rst_n(rst_n),
     .clr(accum_clr),
     .valid(mac_out_valid),
     .accum_in(mac_result),
     .accum_out(accum_out));

    // bias_add bias_adder (
    //     .clk(clk),
    //     .rst_n(rst_n),
    //     .a(mac_result),
    //     .b(bias),     // Select bias for current output channel
    //     .result(result));

    // Delay valid signal to account for bias_add latency
    shift_reg #(1) s_r (.clk(clk), .resetn(rst_n), .din(mac_out_valid), .dout(out_valid_n));

    assign out_valid = out_valid_n;

    // leakyRELU L_relu (
    //     .clk(clk),
    //     .rst_n(rst_n),
    //     .x(biased_sum),
    //     .y(relu_result));

    
endmodule



module shift_reg 
    #(parameter DATA_WIDTH = 30) 
    (input logic clk,
    input logic resetn,
    input logic din,
    output logic [DATA_WIDTH-1:0] dout);

  logic [DATA_WIDTH-1:0] shiftVal;

  always_ff @(posedge clk or negedge resetn) begin
    if(!resetn) begin
      shiftVal <= '0;
    end else begin
      shiftVal <= {shiftVal[DATA_WIDTH-2:0], din};
    end
  end

  assign dout = shiftVal;
endmodule
