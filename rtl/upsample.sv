`timescale 1ns / 1ps

// 2x nearest-neighbor upsample. Reads input feature map from ibram (128-bit words,
// 1-cycle latency) and writes output to obram (16-byte packed words).
// Input  HWC layout: i[y * W_in * C + x * C + c]
// Output HWC layout: o[y * W_out * C + x * C + c], W_out = 2*W_in, H_out = 2*H_in
module upsample #(
    parameter IBUF_DEPTH = 1024,
    parameter OBUF_DEPTH = 4096
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,

    // ibram port B — read input feature map (1-cycle registered output)
    output logic [31:0]  ibram_addr,
    input  logic [127:0] ibram_rdata,

    // obram port B — write output feature map (16-byte packed)
    output logic [31:0]  obram_addr,
    output logic [127:0] obram_wdata,
    output logic         obram_we,

    input  logic [31:0]  H_in,
    input  logic [31:0]  W_in,
    input  logic [31:0]  C,
    output logic         done
);

    logic [31:0] H_out, W_out;
    assign H_out = H_in << 1;
    assign W_out = W_in << 1;

    logic [31:0] y_out, x_out, c_out;
    logic        running;

    // Source byte address: each 2x2 output block maps back to one input pixel
    logic [31:0] src_byte, dst_byte;
    assign src_byte = ((y_out >> 1) * W_in + (x_out >> 1)) * C + c_out;
    assign dst_byte = (y_out * W_out + x_out) * C + c_out;

    // Issue ibram word address combinationally — BRAM registers on next edge
    assign ibram_addr = src_byte >> 4;

    // Pipeline registers (1-cycle BRAM read latency)
    logic [31:0] src_byte_r, dst_byte_r;
    logic        valid_r;

    // Extract byte from ibram word using registered byte lane
    logic [7:0] val;
    assign val = ibram_rdata[src_byte_r[3:0] * 8 +: 8];

    // Write packing: accumulate 16 bytes then write one 128-bit word
    logic [127:0] wr_buf;
    logic [31:0]  wr_word_addr;

    assign obram_wdata = wr_buf;
    assign obram_addr  = wr_word_addr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out        <= 0;
            x_out        <= 0;
            c_out        <= 0;
            running      <= 0;
            done         <= 0;
            valid_r      <= 0;
            src_byte_r   <= 0;
            dst_byte_r   <= 0;
            obram_we     <= 0;
            wr_word_addr <= 0;
            wr_buf       <= 0;
        end else begin
            done     <= 0;
            obram_we <= 0;

            // Advance pipeline
            src_byte_r <= src_byte;
            dst_byte_r <= dst_byte;
            valid_r    <= running;

            // Stage 2: BRAM data valid — pack byte into write buffer
            if (valid_r) begin
                wr_buf[dst_byte_r[3:0] * 8 +: 8] <= val;
                wr_word_addr <= dst_byte_r >> 4;
                // Write on full word or on last pixel (flush partial word)
                if (dst_byte_r[3:0] == 4'hF || !running)
                    obram_we <= 1;
            end

            // Stage 2 done signal: fires one cycle after last pixel drains
            if (!running && valid_r)
                done <= 1;

            // Stage 1: start and advance pixel counters
            if (start && !running) begin
                y_out   <= 0;
                x_out   <= 0;
                c_out   <= 0;
                running <= 1;
            end else if (running) begin
                if (c_out < C - 1) begin
                    c_out <= c_out + 1;
                end else begin
                    c_out <= 0;
                    if (x_out < W_out - 1) begin
                        x_out <= x_out + 1;
                    end else begin
                        x_out <= 0;
                        if (y_out < H_out - 1) begin
                            y_out <= y_out + 1;
                        end else begin
                            running <= 0;
                        end
                    end
                end
            end
        end
    end

endmodule
