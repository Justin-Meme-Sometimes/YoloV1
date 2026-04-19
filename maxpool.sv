`timescale 1ns / 1ps

// 2x2 max pool, stride 2. Operates on a flat HWC int8 buffer.
// Output size: H_out = H_in/2, W_out = W_in/2 (integer division).
// Processes one output pixel per cycle (no internal FSM — driven externally).
module maxpool2d #(
    parameter IBUF_DEPTH = 1024,
    parameter OBUF_DEPTH = 256
) (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic signed [7:0]  i_buffer [IBUF_DEPTH-1:0],
    input  logic [31:0] H_in,
    input  logic [31:0] W_in,
    input  logic [31:0] C,
    output logic signed [7:0]  o_buffer [OBUF_DEPTH-1:0],
    output logic done
);
    logic [31:0] H_out, W_out;
    assign H_out = H_in >> 1;
    assign W_out = W_in >> 1;

    // Output pixel counter
    logic [31:0] y_out, x_out, c_out;
    logic running;

    logic [31:0] addr00, addr01, addr10, addr11;
    logic signed [7:0] v00, v01, v10, v11, max_val;

    localparam IADDR_BITS = $clog2(IBUF_DEPTH);
    localparam OADDR_BITS = $clog2(OBUF_DEPTH);

    // Address of the 2x2 window top-left = (y_out*2, x_out*2)
    assign addr00 = ((y_out*2+0) * W_in + (x_out*2+0)) * C + c_out;
    assign addr01 = ((y_out*2+0) * W_in + (x_out*2+1)) * C + c_out;
    assign addr10 = ((y_out*2+1) * W_in + (x_out*2+0)) * C + c_out;
    assign addr11 = ((y_out*2+1) * W_in + (x_out*2+1)) * C + c_out;

    assign v00 = i_buffer[addr00[IADDR_BITS-1:0]];
    assign v01 = i_buffer[addr01[IADDR_BITS-1:0]];
    assign v10 = i_buffer[addr10[IADDR_BITS-1:0]];
    assign v11 = i_buffer[addr11[IADDR_BITS-1:0]];

    // 4-input signed max
    logic signed [7:0] m0, m1;
    assign m0      = ($signed(v00) > $signed(v01)) ? v00 : v01;
    assign m1      = ($signed(v10) > $signed(v11)) ? v10 : v11;
    assign max_val = ($signed(m0)  > $signed(m1))  ? m0  : m1;

    logic [31:0] out_addr;
    assign out_addr = (y_out * W_out + x_out) * C + c_out;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out   <= '0; x_out   <= '0; c_out <= '0;
            running <= 1'b0;
            done    <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start && !running) begin
                y_out <= '0; x_out <= '0; c_out <= '0;
                running <= 1'b1;
            end else if (running) begin
                o_buffer[out_addr[OADDR_BITS-1:0]] <= max_val;

                if (c_out < C - 1) begin
                    c_out <= c_out + 1;
                end else begin
                    c_out <= '0;
                    if (x_out < W_out - 1) begin
                        x_out <= x_out + 1;
                    end else begin
                        x_out <= '0;
                        if (y_out < H_out - 1) begin
                            y_out <= y_out + 1;
                        end else begin
                            running <= 1'b0;
                            done    <= 1'b1;
                        end
                    end
                end
            end
        end
    end

endmodule
