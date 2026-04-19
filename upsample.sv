
module upsample
(parameter IBUF=32,
 parameter OBUF=32)
(   
    input logic clk,
    input logic rst_n,
    input logic start,
    input signed [7:0][IBUF-1:0]i_buffer,
    input logic [31:0] H_in,
    input logic [31:0] W_in,
    input logic [31:0] C,
    ouptut signed [7:0][OBUF-1:0]o_buffer,
    output logic done
);

    localparam IBUF_DEPTH = 1024;
    localparam OBUF_DEPTH = 4096;








endmodule;






