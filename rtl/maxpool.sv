`timescale 1ns / 1ps

// 2x2 max pool, configurable stride. Reads input feature map from ibram
// (128-bit words, 1-cycle latency) and writes output to obram (16-byte packed words).
// Input/output layout: HWC, int8 signed.
// Processes one output pixel per 5 cycles (RD0→RD1→RD2→RD3→WRITE).
// Pixel counters are held stable across all 5 cycles so window addresses are valid.
module maxpool2d #(
    parameter IBUF_DEPTH = 1024,
    parameter OBUF_DEPTH = 256
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
    input  logic [31:0]  stride,
    output logic         done
);

    logic [31:0] H_out, W_out;
    assign H_out = H_in / stride;
    assign W_out = W_in / stride;

    logic [31:0] y_out, x_out, c_out;

    // 2x2 window byte addresses — stable while pixel counters are held across RD0..WRITE
    logic [31:0] addr00, addr01, addr10, addr11;
    assign addr00 = ((y_out*stride)   * W_in + x_out*stride)     * C + c_out;
    assign addr01 = ((y_out*stride)   * W_in + (x_out*stride+1)) * C + c_out;
    assign addr10 = ((y_out*stride+1) * W_in + x_out*stride)     * C + c_out;
    assign addr11 = ((y_out*stride+1) * W_in + (x_out*stride+1)) * C + c_out;

    typedef enum logic [2:0] { IDLE, RD0, RD1, RD2, RD3, WRITE } state_t;
    state_t state;

    logic signed [7:0] v00, v01, v10;

    // v11 read combinationally: addr11 was issued in RD3, ibram_rdata is valid in WRITE.
    // addr11[3:0] remains correct because pixel counters do not change until WRITE exit.
    logic signed [7:0] v11_comb;
    assign v11_comb = $signed(ibram_rdata[addr11[3:0] * 8 +: 8]);

    logic signed [7:0] m0, m1, max_val;
    assign m0      = ($signed(v00) > $signed(v01)) ? v00 : v01;
    assign m1      = ($signed(v10) > v11_comb)     ? v10 : v11_comb;
    assign max_val = ($signed(m0)  > $signed(m1))  ? m0  : m1;

    logic [31:0] dst_byte;
    assign dst_byte = (y_out * W_out + x_out) * C + c_out;

    logic is_last_pixel;
    assign is_last_pixel = (y_out == H_out - 1) && (x_out == W_out - 1) && (c_out == C - 1);

    logic [127:0] wr_buf;
    logic [31:0]  wr_word_addr;
    assign obram_wdata = wr_buf;
    assign obram_addr  = wr_word_addr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            y_out        <= 0; x_out <= 0; c_out <= 0;
            done         <= 0;
            v00          <= 0; v01 <= 0; v10 <= 0;
            obram_we     <= 0;
            wr_word_addr <= 0;
            wr_buf       <= 0;
            ibram_addr   <= 0;
        end else begin
            done     <= 0;
            obram_we <= 0;

            case (state)
                IDLE: begin
                    if (start) begin
                        y_out <= 0; x_out <= 0; c_out <= 0;
                        state <= RD0;
                    end
                end

                // Issue addr00; BRAM responds next cycle
                RD0: begin
                    ibram_addr <= addr00 >> 4;
                    state <= RD1;
                end

                // Capture v00; issue addr01
                RD1: begin
                    v00 <= $signed(ibram_rdata[addr00[3:0] * 8 +: 8]);
                    ibram_addr <= addr01 >> 4;
                    state <= RD2;
                end

                // Capture v01; issue addr10
                RD2: begin
                    v01 <= $signed(ibram_rdata[addr01[3:0] * 8 +: 8]);
                    ibram_addr <= addr10 >> 4;
                    state <= RD3;
                end

                // Capture v10; issue addr11
                RD3: begin
                    v10 <= $signed(ibram_rdata[addr10[3:0] * 8 +: 8]);
                    ibram_addr <= addr11 >> 4;
                    state <= WRITE;
                end

                // ibram_rdata has addr11 response; max_val uses v00/v01/v10 + v11_comb
                WRITE: begin
                    wr_buf[dst_byte[3:0] * 8 +: 8] <= max_val;
                    wr_word_addr <= dst_byte >> 4;
                    // Write on full word or last pixel (flush partial word)
                    if (dst_byte[3:0] == 4'hF || is_last_pixel) obram_we <= 1;

                    if (is_last_pixel) begin
                        done  <= 1;
                        state <= IDLE;
                    end else if (c_out < C - 1) begin
                        c_out <= c_out + 1;
                        state <= RD0;
                    end else begin
                        c_out <= 0;
                        if (x_out < W_out - 1) begin
                            x_out <= x_out + 1;
                        end else begin
                            x_out <= 0;
                            y_out <= y_out + 1;
                        end
                        state <= RD0;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
