`timescale 1ns / 1ps

// Concatenate two feature maps along the channel axis.
// a_buffer (C_a channels) is read from ibram; b_buffer (C_b channels) from wbram.
// Both BRAMs receive addresses every cycle; the correct source is selected one cycle
// later based on the registered c_out.  Output is written to obram, 16-byte packed.
// Output layout: o[y*W*(C_a+C_b) + x*(C_a+C_b) + c], c = 0..C_a+C_b-1
module route_concat #(
    parameter ABUF_DEPTH = 4096,
    parameter BBUF_DEPTH = 4096,
    parameter OBUF_DEPTH = 16384
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,

    // a_buffer = ibram (upsample output), 1-cycle registered read
    output logic [31:0]  ibram_addr,
    input  logic [127:0] ibram_rdata,

    // b_buffer = wbram (route saved feature map), 1-cycle registered read
    output logic [31:0]  wbram_addr,
    input  logic [127:0] wbram_rdata,

    // output = obram (16-byte packed writes)
    output logic [31:0]  obram_addr,
    output logic [127:0] obram_wdata,
    output logic         obram_we,

    input  logic [31:0]  H,
    input  logic [31:0]  W,
    input  logic [31:0]  C_a,
    input  logic [31:0]  C_b,
    output logic         done
);

    logic [31:0] y_out, x_out, c_out;
    logic        running;

    // a_buffer byte address (valid for c_out < C_a)
    logic [31:0] a_byte;
    assign a_byte = (y_out * W + x_out) * C_a + c_out;

    // b_buffer byte address (valid for c_out >= C_a; clamp index to avoid underflow)
    logic [31:0] c_b_idx, b_byte;
    assign c_b_idx = (c_out >= C_a) ? (c_out - C_a) : 32'd0;
    assign b_byte  = (y_out * W + x_out) * C_b + c_b_idx;

    // Output sequential byte address
    logic [31:0] dst_byte;
    assign dst_byte = (y_out * W + x_out) * (C_a + C_b) + c_out;

    // Issue both BRAM addresses every cycle — BRAM responds 1 cycle later
    assign ibram_addr = a_byte >> 4;
    assign wbram_addr = b_byte >> 4;

    // Pipeline registers (1-cycle BRAM read latency)
    logic [31:0] a_byte_r, b_byte_r, dst_byte_r, c_out_r;
    logic        valid_r;

    // Select the correct byte from the registered address's lane
    logic [7:0] val;
    assign val = (c_out_r < C_a) ? ibram_rdata[a_byte_r[3:0] * 8 +: 8]
                                  : wbram_rdata[b_byte_r[3:0] * 8 +: 8];

    logic [127:0] wr_buf;
    logic [31:0]  wr_word_addr;
    assign obram_wdata = wr_buf;
    assign obram_addr  = wr_word_addr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out        <= 0; x_out <= 0; c_out <= 0;
            running      <= 0;
            done         <= 0;
            valid_r      <= 0;
            a_byte_r     <= 0; b_byte_r <= 0; dst_byte_r <= 0; c_out_r <= 0;
            obram_we     <= 0;
            wr_word_addr <= 0;
            wr_buf       <= 0;
        end else begin
            done     <= 0;
            obram_we <= 0;

            // Advance pipeline
            a_byte_r   <= a_byte;
            b_byte_r   <= b_byte;
            dst_byte_r <= dst_byte;
            c_out_r    <= c_out;
            valid_r    <= running;

            // Stage 2: BRAM data valid — select, pack, write
            if (valid_r) begin
                wr_buf[dst_byte_r[3:0] * 8 +: 8] <= val;
                wr_word_addr <= dst_byte_r >> 4;
                // Write on full word or last byte (flush partial word)
                if (dst_byte_r[3:0] == 4'hF || !running) obram_we <= 1;
            end

            if (!running && valid_r) done <= 1;

            // Stage 1: start and advance pixel counters
            if (start && !running) begin
                y_out <= 0; x_out <= 0; c_out <= 0;
                running <= 1;
            end else if (running) begin
                if (c_out < C_a + C_b - 1) begin
                    c_out <= c_out + 1;
                end else begin
                    c_out <= 0;
                    if (x_out < W - 1) begin
                        x_out <= x_out + 1;
                    end else begin
                        x_out <= 0;
                        if (y_out < H - 1) begin
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
