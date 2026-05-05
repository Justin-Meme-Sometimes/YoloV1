`timescale 1ns / 1ps

// AXI4-Lite slave register file for the YOLO accelerator.
// 16 x 32-bit registers, base+0x00 through base+0x3C.
//
// Register map:
//   0x00  CTRL         [0]=start (write 1 to pulse), [1]=done (read-only)
//   0x04  STATUS       [5:0]=current FSM state (read-only)
//   0x08  WEIGHT_LO    weight buffer DDR address [31:0]
//   0x0C  WEIGHT_HI    weight buffer DDR address [63:32]
//   0x10  IMAGE_LO     image buffer DDR address [31:0]
//   0x14  IMAGE_HI     image buffer DDR address [63:32]
//   0x18  BIAS_LO      bias buffer DDR address [31:0]
//   0x1C  BIAS_HI      bias buffer DDR address [63:32]
//   0x20  BUF_A_LO     ping-pong buffer A DDR address [31:0]
//   0x24  BUF_A_HI     ping-pong buffer A DDR address [63:32]
//   0x28  BUF_B_LO     ping-pong buffer B DDR address [31:0]
//   0x2C  BUF_B_HI     ping-pong buffer B DDR address [63:32]
//   0x30  DET1_LO      detection head 1 output DDR address [31:0]
//   0x34  DET1_HI      detection head 1 output DDR address [63:32]
//   0x38  DET2_LO      detection head 2 output DDR address [31:0]
//   0x3C  DET2_HI      detection head 2 output DDR address [63:32]

module axi_lite_slave #(
    parameter integer ADDR_WIDTH = 6,   // log2(16 regs * 4 bytes) = 6
    parameter integer DATA_WIDTH = 32
)(
    // AXI4-Lite slave
    input  logic                    S_AXI_ACLK,
    input  logic                    S_AXI_ARESETN,

    // Write address
    input  logic [ADDR_WIDTH-1:0]   S_AXI_AWADDR,
    input  logic [2:0]              S_AXI_AWPROT,
    input  logic                    S_AXI_AWVALID,
    output logic                    S_AXI_AWREADY,

    // Write data
    input  logic [DATA_WIDTH-1:0]   S_AXI_WDATA,
    input  logic [DATA_WIDTH/8-1:0] S_AXI_WSTRB,
    input  logic                    S_AXI_WVALID,
    output logic                    S_AXI_WREADY,

    // Write response
    output logic [1:0]              S_AXI_BRESP,
    output logic                    S_AXI_BVALID,
    input  logic                    S_AXI_BREADY,

    // Read address
    input  logic [ADDR_WIDTH-1:0]   S_AXI_ARADDR,
    input  logic [2:0]              S_AXI_ARPROT,
    input  logic                    S_AXI_ARVALID,
    output logic                    S_AXI_ARREADY,

    // Read data
    output logic [DATA_WIDTH-1:0]   S_AXI_RDATA,
    output logic [1:0]              S_AXI_RRESP,
    output logic                    S_AXI_RVALID,
    input  logic                    S_AXI_RREADY,

    // Register outputs to compute core
    output logic        start,
    output logic [63:0] weight_addr,
    output logic [63:0] image_addr,
    output logic [63:0] bias_addr,
    output logic [63:0] buf_a_addr,
    output logic [63:0] buf_b_addr,
    output logic [63:0] det1_addr,
    output logic [63:0] det2_addr,

    // Status inputs from compute core
    input  logic        done,
    input  logic [5:0]  fsm_state
);

    // ----------------------------------------------------------------
    // Internal register file  (16 regs)
    // ----------------------------------------------------------------
    logic [31:0] reg_file [0:15];

    // Convenience aliases
    assign start       = reg_file[0][0];
    assign weight_addr = {reg_file[3],  reg_file[2]};
    assign image_addr  = {reg_file[5],  reg_file[4]};
    assign bias_addr   = {reg_file[7],  reg_file[6]};
    assign buf_a_addr  = {reg_file[9],  reg_file[8]};
    assign buf_b_addr  = {reg_file[11], reg_file[10]};
    assign det1_addr   = {reg_file[13], reg_file[12]};
    assign det2_addr   = {reg_file[15], reg_file[14]};

    // ----------------------------------------------------------------
    // Write path
    // ----------------------------------------------------------------
    logic aw_active;   // write address latched
    logic [3:0] aw_idx;

    assign S_AXI_AWREADY = !aw_active;
    assign S_AXI_WREADY  = aw_active;
    assign S_AXI_BRESP   = 2'b00;

    always_ff @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            aw_active    <= 1'b0;
            aw_idx       <= '0;
            S_AXI_BVALID <= 1'b0;
            for (int i = 0; i < 16; i++) reg_file[i] <= '0;
        end else begin
            // Clear start pulse after one cycle
            reg_file[0][0] <= 1'b0;

            // Latch write address
            if (S_AXI_AWVALID && S_AXI_AWREADY) begin
                aw_active <= 1'b1;
                aw_idx    <= S_AXI_AWADDR[ADDR_WIDTH-1:2];  // word index
            end

            // Write data
            if (S_AXI_WVALID && S_AXI_WREADY) begin
                aw_active <= 1'b0;
                // Byte-enable aware write (skip read-only regs 0[1] and 1)
                if (aw_idx != 4'd1) begin
                    for (int b = 0; b < 4; b++) begin
                        if (S_AXI_WSTRB[b])
                            reg_file[aw_idx][b*8 +: 8] <= S_AXI_WDATA[b*8 +: 8];
                    end
                end
                S_AXI_BVALID <= 1'b1;
            end

            if (S_AXI_BVALID && S_AXI_BREADY)
                S_AXI_BVALID <= 1'b0;
        end
    end

    // ----------------------------------------------------------------
    // Read path
    // ----------------------------------------------------------------
    logic [3:0] ar_idx;
    logic [31:0] rdata_mux;

    always_comb begin
        case (ar_idx)
            4'd0:    rdata_mux = {30'b0, done, reg_file[0][0]};  // CTRL: [1]=done [0]=start
            4'd1:    rdata_mux = {26'b0, fsm_state};              // STATUS
            default: rdata_mux = reg_file[ar_idx];
        endcase
    end

    always_ff @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_ARREADY <= 1'b1;
            S_AXI_RVALID  <= 1'b0;
            S_AXI_RRESP   <= 2'b00;
            S_AXI_RDATA   <= '0;
            ar_idx        <= '0;
        end else begin
            if (S_AXI_ARVALID && S_AXI_ARREADY) begin
                ar_idx        <= S_AXI_ARADDR[ADDR_WIDTH-1:2];
                S_AXI_ARREADY <= 1'b0;
                S_AXI_RVALID  <= 1'b1;
                S_AXI_RDATA   <= rdata_mux;
            end
            if (S_AXI_RVALID && S_AXI_RREADY) begin
                S_AXI_RVALID  <= 1'b0;
                S_AXI_ARREADY <= 1'b1;
            end
        end
    end

endmodule
