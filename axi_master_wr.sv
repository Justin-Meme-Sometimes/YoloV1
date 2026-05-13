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

  

endmodule
