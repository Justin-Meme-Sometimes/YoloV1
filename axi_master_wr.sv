`timescale 1ns / 1ps

// AXI4 burst write master — transfers byte_count bytes from a word-addressed
// dual-port BRAM starting at word 0 to DDR at dst_addr.
// Mirrors axi_master_rd: same DATA_WIDTH, same 256-beat burst cap.
module axi_master_wr #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 128,
    parameter ID_WIDTH   = 1
)(
    input  logic clk,
    input  logic rst_n,

    // Command — hold stable from start pulse until done
    input  logic [ADDR_WIDTH-1:0] dst_addr,
    input  logic [31:0]           byte_count,
    input  logic                  start,
    output logic                  done,

    // BRAM read port (word-addressed)
    output logic [31:0]           bram_raddr,
    input  logic [DATA_WIDTH-1:0] bram_rdata,

    // AXI4 write address channel
    output logic [ID_WIDTH-1:0]     M_AXI_AWID,
    output logic [ADDR_WIDTH-1:0]   M_AXI_AWADDR,
    output logic [7:0]              M_AXI_AWLEN,
    output logic [2:0]              M_AXI_AWSIZE,
    output logic [1:0]              M_AXI_AWBURST,
    output logic                    M_AXI_AWLOCK,
    output logic [3:0]              M_AXI_AWCACHE,
    output logic [2:0]              M_AXI_AWPROT,
    output logic                    M_AXI_AWVALID,
    input  logic                    M_AXI_AWREADY,

    // AXI4 write data channel
    output logic [DATA_WIDTH-1:0]   M_AXI_WDATA,
    output logic [DATA_WIDTH/8-1:0] M_AXI_WSTRB,
    output logic                    M_AXI_WLAST,
    output logic                    M_AXI_WVALID,
    input  logic                    M_AXI_WREADY,

    // AXI4 write response channel
    input  logic [ID_WIDTH-1:0]   M_AXI_BID,
    input  logic [1:0]            M_AXI_BRESP,
    input  logic                  M_AXI_BVALID,
    output logic                  M_AXI_BREADY
);

    localparam BYTES_PER_BEAT = DATA_WIDTH / 8;
    localparam MAX_BURST      = 256;

    typedef enum logic [2:0] { IDLE, ISSUE_AW, SEND, WAIT_B, FINISH } state_t;
    state_t state;

    logic [ADDR_WIDTH-1:0] cur_addr;
    logic [31:0]           beats_left;
    logic [31:0]           bram_ptr;
    logic [7:0]            burst_len;
    logic [7:0]            beat_cnt;

    assign M_AXI_AWID    = '0;
    assign M_AXI_AWSIZE  = 3'($clog2(BYTES_PER_BEAT));
    assign M_AXI_AWBURST = 2'b01;
    assign M_AXI_AWLOCK  = 1'b0;
    assign M_AXI_AWCACHE = 4'b0011;
    assign M_AXI_AWPROT  = 3'b000;
    assign M_AXI_AWADDR  = cur_addr;
    assign M_AXI_AWLEN   = burst_len;
    assign M_AXI_WSTRB   = '1;
    assign M_AXI_BREADY  = (state == WAIT_B);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            done          <= 1'b0;
            M_AXI_AWVALID <= 1'b0;
            M_AXI_WVALID  <= 1'b0;
            M_AXI_WLAST   <= 1'b0;
            cur_addr      <= '0;
            beats_left    <= '0;
            bram_ptr      <= '0;
            burst_len     <= '0;
            beat_cnt      <= '0;
            bram_raddr    <= '0;
        end else begin
            done <= 1'b0;

            case (state)
                IDLE: begin
                    if (start) begin
                        cur_addr   <= dst_addr;
                        beats_left <= (byte_count + BYTES_PER_BEAT - 1) / BYTES_PER_BEAT;
                        bram_ptr   <= '0;
                        state      <= ISSUE_AW;
                    end
                end

                ISSUE_AW: begin
                    burst_len     <= (beats_left >= MAX_BURST) ? (MAX_BURST - 1)
                                                               : (beats_left - 1);
                    beat_cnt      <= '0;
                    bram_raddr    <= bram_ptr;        // prefetch first word
                    M_AXI_AWVALID <= 1'b1;
                    state         <= SEND;
                end

                SEND: begin
                    if (M_AXI_AWVALID && M_AXI_AWREADY)
                        M_AXI_AWVALID <= 1'b0;

                    M_AXI_WVALID <= 1'b1;
                    M_AXI_WDATA  <= bram_rdata;
                    M_AXI_WLAST  <= (beat_cnt == burst_len);

                    if (M_AXI_WVALID && M_AXI_WREADY) begin
                        bram_ptr   <= bram_ptr + 1;
                        bram_raddr <= bram_ptr + 1;
                        beat_cnt   <= beat_cnt + 1;
                        beats_left <= beats_left - 1;

                        if (M_AXI_WLAST) begin
                            M_AXI_WVALID <= 1'b0;
                            M_AXI_WLAST  <= 1'b0;
                            state        <= WAIT_B;
                        end
                    end
                end

                WAIT_B: begin
                    if (M_AXI_BVALID) begin
                        if (beats_left == 0)
                            state <= FINISH;
                        else begin
                            cur_addr <= cur_addr + (burst_len + 1) * BYTES_PER_BEAT;
                            state    <= ISSUE_AW;
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
