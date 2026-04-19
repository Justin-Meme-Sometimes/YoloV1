module route_concat #(
    parameter ABUF_DEPTH = 4096,   // 26*26*128 upsampled
    parameter BBUF_DEPTH = 4096,   // 26*26*256 saved route
    parameter OBUF_DEPTH = 16384   // 26*26*384 output
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic signed [7:0] a_buffer [ABUF_DEPTH-1:0],  // upsample output
    input  logic signed [7:0] b_buffer [BBUF_DEPTH-1:0],  // saved feature map
    input  logic [31:0] H,
    input  logic [31:0] W,
    input  logic [31:0] C_a,   // channels from buffer a
    input  logic [31:0] C_b,   // channels from buffer b
    output logic signed [7:0] o_buffer [OBUF_DEPTH-1:0],
    output logic done
);








endmodule