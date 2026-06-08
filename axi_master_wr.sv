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
    localparam MAX_BURST  = 256;

    typedef enum logic [4:0] { IDLE, ISSUE_AW, SEND, WAIT_B, FINISH } state_t;
    state_t current_state, next_state;

    logic [ADDR_WIDTH-1:0] curr_addr;
    logic [31:0] beats_left;
    logic [31:0] bram_ptr;
    logic [8:0]  burst_cnt;
    logic [8:0]  burst_size;
   

    assign M_AXI_AWSIZE = $clog2(BYTES_PER_BEAT);
    assign M_AXI_AWBURST = 2'b01;
    assign M_AXI_AWLOCK = 1'd0;
    assign M_AXI_AWCACHE = 4'b0011;
    assign M_AXI_AWID = 1'd0;


    assign M_AXI_AWLEN = (beats_left < 256) ? (beats_left - 1) : (256 - 1);
    assign M_AXI_AWVALID = (current_state ==  ISSUE_AW);
    assign M_AXI_AWADDR = curr_addr;
    assign bram_raddr = bram_ptr;
    assign M_AXI_WSTRB = '1;
    assign M_AXI_AWPROT = '0;

    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            current_state <= IDLE;
        end else begin 
            current_state <= next_state;
        end
    end

    always_ff @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            curr_addr <= 0;
            bram_ptr  <= 0;
            beats_left <= 0;
            burst_cnt  <= 0;
            burst_size <= 0;
        end else begin
            if(current_state == IDLE && start) begin
                beats_left <= byte_count >> 4;
                curr_addr <= dst_addr;
                bram_ptr <= 0;
            end 
            if(current_state == SEND && next_state == WAIT_B) begin
                curr_addr <= curr_addr + burst_size * 16;
            end
            if(current_state == SEND && M_AXI_WREADY) begin
                bram_ptr <= bram_ptr + 1;
                beats_left <= beats_left - 1;
                  burst_cnt <= burst_cnt - 1; 
            end
            if(current_state == ISSUE_AW && next_state == SEND) begin
                burst_size <= (beats_left < 256) ? beats_left : 256;
                burst_cnt <= (beats_left < 256) ? beats_left : 256;
            end
        end
    end

    always_comb begin
        done = 0;
        next_state = current_state;
        M_AXI_WVALID = 0;
        M_AXI_WLAST = 0;
        M_AXI_BREADY = 0;
        M_AXI_WDATA = 0;
        case(current_state)
            IDLE: begin
                if(start) begin
                    next_state = ISSUE_AW;
                end else begin
                    next_state = IDLE;
                end
            end
            ISSUE_AW: begin
                if(M_AXI_AWREADY) begin
                    next_state = SEND;
                end
            end
            SEND: begin
                M_AXI_WVALID = 1;
                if(M_AXI_WREADY) begin
                    M_AXI_WDATA = bram_rdata;
                    next_state = SEND;
                    if(burst_cnt == 1) begin
                        M_AXI_WLAST = 1;
                        next_state = WAIT_B;
                    end
                end
            end
            WAIT_B: begin
                M_AXI_BREADY = 1;
                if(M_AXI_BVALID) begin
                    if(beats_left == 0) begin
                        next_state = FINISH;
                    end else begin
                        next_state = ISSUE_AW; //goes back here after we see another burst
                    end
                end
            end
            FINISH: begin
                done = 1;
                next_state = IDLE;
            end
        endcase
    end


endmodule
