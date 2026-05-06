`timescale 1ns / 1ps

// AXI4 burst read master — transfers byte_count bytes from DDR src_addr
// into a word-addressed dual-port BRAM starting at word 0.
// DATA_WIDTH matches the Zynq UltraScale+ HP port width (128 bits = 16 B/beat).
// Bursts are capped at 256 beats (4 KB). Assumes src_addr is 16-byte aligned
// and pynq.allocate() guarantees page-alignment so no 4 KB boundary wrapping occurs.
module axi_master_rd #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 128,
    parameter ID_WIDTH   = 1
)(
    input  logic clk,
    input  logic rst_n,

    // Command — hold stable from start pulse until done
    input  logic [ADDR_WIDTH-1:0] src_addr,
    input  logic [31:0]           byte_count,
    input  logic                  start,
    output logic                  done,

    // BRAM write port (word-addressed, one word = DATA_WIDTH bits)
    output logic [31:0]           bram_waddr,
    output logic [DATA_WIDTH-1:0] bram_wdata,
    output logic                  bram_we,

    // AXI4 read address channel
    output logic [ID_WIDTH-1:0]   M_AXI_ARID,
    output logic [ADDR_WIDTH-1:0] M_AXI_ARADDR,
    output logic [7:0]            M_AXI_ARLEN,
    output logic [2:0]            M_AXI_ARSIZE,
    output logic [1:0]            M_AXI_ARBURST,
    output logic                  M_AXI_ARLOCK,
    output logic [3:0]            M_AXI_ARCACHE,
    output logic [2:0]            M_AXI_ARPROT,
    output logic                  M_AXI_ARVALID,
    input  logic                  M_AXI_ARREADY,

    // AXI4 read data channel
    input  logic [ID_WIDTH-1:0]   M_AXI_RID,
    input  logic [DATA_WIDTH-1:0] M_AXI_RDATA,
    input  logic [1:0]            M_AXI_RRESP,
    input  logic                  M_AXI_RLAST,
    input  logic                  M_AXI_RVALID,
    output logic                  M_AXI_RREADY
);

    localparam BYTES_PER_BEAT = DATA_WIDTH / 8;   // 16
    localparam MAX_BURST      = 256;              // AXI4 max beats per burst

    typedef enum logic [1:0] { IDLE, ISSUE_AR, RECV, FINISH } state_t;
    state_t state;

    logic [ADDR_WIDTH-1:0] cur_addr;
    logic [31:0]           beats_left;
    logic [31:0]           bram_ptr;
    logic [7:0]            burst_len;  // ARLEN = beats - 1

    // Static AXI signals
    assign M_AXI_ARID    = '0;
    assign M_AXI_ARSIZE  = 3'($clog2(BYTES_PER_BEAT));  // 4 → 16 B/beat
    assign M_AXI_ARBURST = 2'b01;   // INCR
    assign M_AXI_ARLOCK  = 1'b0;
    assign M_AXI_ARCACHE = 4'b0011; // normal non-cacheable bufferable
    assign M_AXI_ARPROT  = 3'b000;

    // Driven from registers
    assign M_AXI_ARADDR  = cur_addr;
    assign M_AXI_ARLEN   = burst_len;
    assign M_AXI_RREADY  = (state == RECV);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            done          <= 1'b0;
            M_AXI_ARVALID <= 1'b0;
            bram_we       <= 1'b0;
            cur_addr      <= '0;
            beats_left    <= '0;
            bram_ptr      <= '0;
            burst_len     <= '0;
        end else begin
            done    <= 1'b0;
            bram_we <= 1'b0;

            case (state)
                IDLE: begin
                    if (start) begin
                        cur_addr   <= src_addr;
                        beats_left <= (byte_count + BYTES_PER_BEAT - 1) / BYTES_PER_BEAT;
                        bram_ptr   <= '0;
                        state      <= ISSUE_AR;
                    end
                end

                ISSUE_AR: begin
                    burst_len     <= (beats_left >= MAX_BURST) ? (MAX_BURST - 1)
                                                               : (beats_left - 1);
                    M_AXI_ARVALID <= 1'b1;
                    state         <= RECV;
                end

                RECV: begin
                    if (M_AXI_ARVALID && M_AXI_ARREADY)
                        M_AXI_ARVALID <= 1'b0;

                    if (M_AXI_RVALID) begin
                        bram_waddr <= bram_ptr;
                        bram_wdata <= M_AXI_RDATA;
                        bram_we    <= 1'b1;
                        bram_ptr   <= bram_ptr + 1;
                        beats_left <= beats_left - 1;

                        if (M_AXI_RLAST) begin
                            if (beats_left == 1)
                                state <= FINISH;
                            else begin
                                cur_addr <= cur_addr + (burst_len + 1) * BYTES_PER_BEAT;
                                state    <= ISSUE_AR;
                            end
                        end
                    end
                end

                FINISH: begin
                    done  <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
