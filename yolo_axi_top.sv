`timescale 1ns / 1ps

// Top-level AXI wrapper for the YOLO accelerator on Ultra96v2 (Zynq UltraScale+ ZU3EG).
//
// Memory layout (all DDR, allocated by Python via pynq.allocate):
//   weight_addr  — 8,649,648 B  all conv weights concatenated
//   bias_addr    — 16,384 B     all biases (int32, padded)
//   image_addr   — 519,168 B    input image (416x416x3 int8)
//   buf_a_addr   — 2,752,512 B  ping-pong side A
//   buf_b_addr   — 2,752,512 B  ping-pong side B
//   up_buf_addr  — 348,928 B    upsample output (26x26x128x4 worst case)
//   route_addr   — 172,032 B    saved Conv5 output  (26x26x256)
//   route8_addr  — 43,264 B     saved Conv8 output  (13x13x256)
//   det1_addr    — 43,095 B     detection head 1 output (13x13x255)
//   det2_addr    — 172,380 B    detection head 2 output (26x26x255)
//
// On-chip BRAM tiles (3 × dp_bram, 256 KB each):
//   ibram  — input feature map tile
//   wbram  — weight tile (or route_buf for concat)
//   obram  — output feature map tile
//
// AXI interfaces:
//   S_AXI_LITE   — AXI-Lite slave, 64-byte register map (see axi_lite_slave.sv)
//   M_AXI_HP0    — AXI4 master, 128-bit, reads DDR → ibram/wbram
//   M_AXI_HP1    — AXI4 master, 128-bit, writes obram → DDR

module yolo_axi_top #(
    parameter ADDR_WIDTH  = 64,
    parameter DATA_WIDTH  = 128,   // AXI HP width (16 B/beat)
    parameter ID_WIDTH    = 1,
    parameter BRAM_WORDS  = 16384  // 16384 × 16 B = 256 KB per BRAM tile
)(
    input  logic clk,
    input  logic rst_n,

    // ----------------------------------------------------------------
    // AXI4-Lite slave (control + buffer addresses from PS)
    // ----------------------------------------------------------------
    input  logic [5:0]              S_AXI_AWADDR,
    input  logic [2:0]              S_AXI_AWPROT,
    input  logic                    S_AXI_AWVALID,
    output logic                    S_AXI_AWREADY,
    input  logic [31:0]             S_AXI_WDATA,
    input  logic [3:0]              S_AXI_WSTRB,
    input  logic                    S_AXI_WVALID,
    output logic                    S_AXI_WREADY,
    output logic [1:0]              S_AXI_BRESP,
    output logic                    S_AXI_BVALID,
    input  logic                    S_AXI_BREADY,
    input  logic [5:0]              S_AXI_ARADDR,
    input  logic [2:0]              S_AXI_ARPROT,
    input  logic                    S_AXI_ARVALID,
    output logic                    S_AXI_ARREADY,
    output logic [31:0]             S_AXI_RDATA,
    output logic [1:0]              S_AXI_RRESP,
    output logic                    S_AXI_RVALID,
    input  logic                    S_AXI_RREADY,

    // ----------------------------------------------------------------
    // AXI4 HP0 master — reads DDR into BRAM (loads)
    // ----------------------------------------------------------------
    output logic [ID_WIDTH-1:0]     M_AXI_HP0_ARID,
    output logic [ADDR_WIDTH-1:0]   M_AXI_HP0_ARADDR,
    output logic [7:0]              M_AXI_HP0_ARLEN,
    output logic [2:0]              M_AXI_HP0_ARSIZE,
    output logic [1:0]              M_AXI_HP0_ARBURST,
    output logic                    M_AXI_HP0_ARLOCK,
    output logic [3:0]              M_AXI_HP0_ARCACHE,
    output logic [2:0]              M_AXI_HP0_ARPROT,
    output logic                    M_AXI_HP0_ARVALID,
    input  logic                    M_AXI_HP0_ARREADY,
    input  logic [ID_WIDTH-1:0]     M_AXI_HP0_RID,
    input  logic [DATA_WIDTH-1:0]   M_AXI_HP0_RDATA,
    input  logic [1:0]              M_AXI_HP0_RRESP,
    input  logic                    M_AXI_HP0_RLAST,
    input  logic                    M_AXI_HP0_RVALID,
    output logic                    M_AXI_HP0_RREADY,

    // ----------------------------------------------------------------
    // AXI4 HP1 master — writes BRAM back to DDR (stores)
    // ----------------------------------------------------------------
    output logic [ID_WIDTH-1:0]     M_AXI_HP1_AWID,
    output logic [ADDR_WIDTH-1:0]   M_AXI_HP1_AWADDR,
    output logic [7:0]              M_AXI_HP1_AWLEN,
    output logic [2:0]              M_AXI_HP1_AWSIZE,
    output logic [1:0]              M_AXI_HP1_AWBURST,
    output logic                    M_AXI_HP1_AWLOCK,
    output logic [3:0]              M_AXI_HP1_AWCACHE,
    output logic [2:0]              M_AXI_HP1_AWPROT,
    output logic                    M_AXI_HP1_AWVALID,
    input  logic                    M_AXI_HP1_AWREADY,
    output logic [DATA_WIDTH-1:0]   M_AXI_HP1_WDATA,
    output logic [DATA_WIDTH/8-1:0] M_AXI_HP1_WSTRB,
    output logic                    M_AXI_HP1_WLAST,
    output logic                    M_AXI_HP1_WVALID,
    input  logic                    M_AXI_HP1_WREADY,
    input  logic [ID_WIDTH-1:0]     M_AXI_HP1_BID,
    input  logic [1:0]              M_AXI_HP1_BRESP,
    input  logic                    M_AXI_HP1_BVALID,
    output logic                    M_AXI_HP1_BREADY
);

    // ----------------------------------------------------------------
    // AXI-Lite register outputs
    // ----------------------------------------------------------------
    logic        ctrl_start, ctrl_done;
    logic [63:0] weight_addr, image_addr, bias_addr;
    logic [63:0] buf_a_addr,  buf_b_addr;
    logic [63:0] up_buf_addr, route_addr, route8_addr;
    logic [63:0] det1_addr,   det2_addr;

    // ----------------------------------------------------------------
    // FSM sequencer signals
    // ----------------------------------------------------------------
    logic        fsm_start, fsm_done;
    logic        mem_stall;
    logic [5:0]  fsm_state;
    logic        ping;

    logic        conv_start, pool_start, up_start,  cat_start;
    logic        conv_done,  pool_done,  up_done,   cat_done;

    logic [31:0] cfg_H_in,  cfg_W_in,  cfg_C_in;
    logic [31:0] cfg_H_out, cfg_W_out, cfg_C_out;
    logic [31:0] cfg_stride, cfg_K;
    logic [31:0] cfg_weight_base, cfg_bias_base;

    logic save_route, save_route8, save_det1, save_det2, route_restore;

    // ----------------------------------------------------------------
    // AXI-Lite slave
    // ----------------------------------------------------------------
    axi_lite_slave u_ctrl (
        .S_AXI_ACLK    (clk),
        .S_AXI_ARESETN (rst_n),
        .S_AXI_AWADDR  (S_AXI_AWADDR),  .S_AXI_AWPROT (S_AXI_AWPROT),
        .S_AXI_AWVALID (S_AXI_AWVALID), .S_AXI_AWREADY(S_AXI_AWREADY),
        .S_AXI_WDATA   (S_AXI_WDATA),   .S_AXI_WSTRB  (S_AXI_WSTRB),
        .S_AXI_WVALID  (S_AXI_WVALID),  .S_AXI_WREADY (S_AXI_WREADY),
        .S_AXI_BRESP   (S_AXI_BRESP),   .S_AXI_BVALID (S_AXI_BVALID),
        .S_AXI_BREADY  (S_AXI_BREADY),
        .S_AXI_ARADDR  (S_AXI_ARADDR),  .S_AXI_ARPROT (S_AXI_ARPROT),
        .S_AXI_ARVALID (S_AXI_ARVALID), .S_AXI_ARREADY(S_AXI_ARREADY),
        .S_AXI_RDATA   (S_AXI_RDATA),   .S_AXI_RRESP  (S_AXI_RRESP),
        .S_AXI_RVALID  (S_AXI_RVALID),  .S_AXI_RREADY (S_AXI_RREADY),
        .start         (ctrl_start),
        .done          (ctrl_done),
        .fsm_state     (fsm_state),
        .weight_addr   (weight_addr),
        .image_addr    (image_addr),
        .bias_addr     (bias_addr),
        .buf_a_addr    (buf_a_addr),
        .buf_b_addr    (buf_b_addr),
        .det1_addr     (det1_addr),
        .det2_addr     (det2_addr)
    );

    // ----------------------------------------------------------------
    // FSM sequencer (pure control, no memory)
    // ----------------------------------------------------------------
    yolo_tiny_top u_fsm (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (fsm_start),
        .mem_stall      (mem_stall),
        .conv_done      (conv_done),
        .pool_done      (pool_done),
        .up_done        (up_done),
        .cat_done       (cat_done),
        .conv_start     (conv_start),
        .pool_start     (pool_start),
        .up_start       (up_start),
        .cat_start      (cat_start),
        .cfg_H_in       (cfg_H_in),  .cfg_W_in  (cfg_W_in),  .cfg_C_in  (cfg_C_in),
        .cfg_H_out      (cfg_H_out), .cfg_W_out (cfg_W_out), .cfg_C_out (cfg_C_out),
        .cfg_stride     (cfg_stride),
        .cfg_weight_base(cfg_weight_base),
        .cfg_bias_base  (cfg_bias_base),
        .cfg_K          (cfg_K),
        .ping           (ping),
        .save_route     (save_route),
        .save_route8    (save_route8),
        .save_det1      (save_det1),
        .save_det2      (save_det2),
        .route_restore  (route_restore),
        .fsm_state      (fsm_state),
        .done           (fsm_done)
    );

    assign ctrl_done = fsm_done;
    assign fsm_start = ctrl_start;

    // ----------------------------------------------------------------
    // On-chip BRAM tiles (256 KB each, 128-bit wide)
    // ibram: input feature map   — AXI loads into port A, compute reads port B
    // wbram: weights / route_buf — AXI loads into port A, compute reads port B
    // obram: output feature map  — compute writes port B, AXI reads port A
    // ----------------------------------------------------------------
    localparam BRAM_AW = $clog2(BRAM_WORDS);

    // ibram
    logic [31:0]            ibram_a_addr; logic [DATA_WIDTH-1:0] ibram_a_wdata;
    logic                   ibram_a_we;   logic [DATA_WIDTH-1:0] ibram_a_rdata;
    logic [31:0]            ibram_b_addr; logic [DATA_WIDTH-1:0] ibram_b_wdata;
    logic                   ibram_b_we;   logic [DATA_WIDTH-1:0] ibram_b_rdata;

    dp_bram #(.DATA_WIDTH(DATA_WIDTH), .DEPTH(BRAM_WORDS)) u_ibram (
        .clk_a(clk), .addr_a(ibram_a_addr), .wdata_a(ibram_a_wdata),
        .we_a(ibram_a_we), .rdata_a(ibram_a_rdata),
        .clk_b(clk), .addr_b(ibram_b_addr), .wdata_b(ibram_b_wdata),
        .we_b(ibram_b_we), .rdata_b(ibram_b_rdata)
    );

    // wbram
    logic [31:0]            wbram_a_addr; logic [DATA_WIDTH-1:0] wbram_a_wdata;
    logic                   wbram_a_we;   logic [DATA_WIDTH-1:0] wbram_a_rdata;
    logic [31:0]            wbram_b_addr; logic [DATA_WIDTH-1:0] wbram_b_wdata;
    logic                   wbram_b_we;   logic [DATA_WIDTH-1:0] wbram_b_rdata;

    dp_bram #(.DATA_WIDTH(DATA_WIDTH), .DEPTH(BRAM_WORDS)) u_wbram (
        .clk_a(clk), .addr_a(wbram_a_addr), .wdata_a(wbram_a_wdata),
        .we_a(wbram_a_we), .rdata_a(wbram_a_rdata),
        .clk_b(clk), .addr_b(wbram_b_addr), .wdata_b(wbram_b_wdata),
        .we_b(wbram_b_we), .rdata_b(wbram_b_rdata)
    );

    // obram
    logic [31:0]            obram_a_addr; logic [DATA_WIDTH-1:0] obram_a_wdata;
    logic                   obram_a_we;   logic [DATA_WIDTH-1:0] obram_a_rdata;
    logic [31:0]            obram_b_addr; logic [DATA_WIDTH-1:0] obram_b_wdata;
    logic                   obram_b_we;   logic [DATA_WIDTH-1:0] obram_b_rdata;

    dp_bram #(.DATA_WIDTH(DATA_WIDTH), .DEPTH(BRAM_WORDS)) u_obram (
        .clk_a(clk), .addr_a(obram_a_addr), .wdata_a(obram_a_wdata),
        .we_a(obram_a_we), .rdata_a(obram_a_rdata),
        .clk_b(clk), .addr_b(obram_b_addr), .wdata_b(obram_b_wdata),
        .we_b(obram_b_we), .rdata_b(obram_b_rdata)
    );

    // ----------------------------------------------------------------
    // AXI HP0 read master — DDR → BRAM (ibram or wbram, muxed by mem FSM)
    // ----------------------------------------------------------------
    logic [ADDR_WIDTH-1:0] rd_src_addr;
    logic [31:0]           rd_byte_count;
    logic                  rd_start, rd_done;
    logic [31:0]           rd_bram_waddr; logic [DATA_WIDTH-1:0] rd_bram_wdata;
    logic                  rd_bram_we;
    logic                  rd_to_wbram;   // 0=ibram, 1=wbram

    axi_master_rd #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ID_WIDTH(ID_WIDTH)) u_rd (
        .clk(clk), .rst_n(rst_n),
        .src_addr(rd_src_addr), .byte_count(rd_byte_count),
        .start(rd_start), .done(rd_done),
        .bram_waddr(rd_bram_waddr), .bram_wdata(rd_bram_wdata), .bram_we(rd_bram_we),
        .M_AXI_ARID   (M_AXI_HP0_ARID),   .M_AXI_ARADDR (M_AXI_HP0_ARADDR),
        .M_AXI_ARLEN  (M_AXI_HP0_ARLEN),  .M_AXI_ARSIZE (M_AXI_HP0_ARSIZE),
        .M_AXI_ARBURST(M_AXI_HP0_ARBURST),.M_AXI_ARLOCK (M_AXI_HP0_ARLOCK),
        .M_AXI_ARCACHE(M_AXI_HP0_ARCACHE),.M_AXI_ARPROT (M_AXI_HP0_ARPROT),
        .M_AXI_ARVALID(M_AXI_HP0_ARVALID),.M_AXI_ARREADY(M_AXI_HP0_ARREADY),
        .M_AXI_RID    (M_AXI_HP0_RID),    .M_AXI_RDATA  (M_AXI_HP0_RDATA),
        .M_AXI_RRESP  (M_AXI_HP0_RRESP),  .M_AXI_RLAST  (M_AXI_HP0_RLAST),
        .M_AXI_RVALID (M_AXI_HP0_RVALID), .M_AXI_RREADY (M_AXI_HP0_RREADY)
    );

    // Route read master output to ibram or wbram port A
    always_comb begin
        ibram_a_wdata = rd_bram_wdata; ibram_a_addr = rd_bram_waddr; ibram_a_we = 1'b0;
        wbram_a_wdata = rd_bram_wdata; wbram_a_addr = rd_bram_waddr; wbram_a_we = 1'b0;
        if (!rd_to_wbram) ibram_a_we = rd_bram_we;
        else              wbram_a_we = rd_bram_we;
    end

    // ----------------------------------------------------------------
    // AXI HP1 write master — BRAM (obram) → DDR
    // ----------------------------------------------------------------
    logic [ADDR_WIDTH-1:0] wr_dst_addr;
    logic [31:0]           wr_byte_count;
    logic                  wr_start, wr_done;

    axi_master_wr #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ID_WIDTH(ID_WIDTH)) u_wr (
        .clk(clk), .rst_n(rst_n),
        .dst_addr(wr_dst_addr), .byte_count(wr_byte_count),
        .start(wr_start), .done(wr_done),
        .bram_raddr(obram_a_addr), .bram_rdata(obram_a_rdata),
        .M_AXI_AWID   (M_AXI_HP1_AWID),   .M_AXI_AWADDR (M_AXI_HP1_AWADDR),
        .M_AXI_AWLEN  (M_AXI_HP1_AWLEN),  .M_AXI_AWSIZE (M_AXI_HP1_AWSIZE),
        .M_AXI_AWBURST(M_AXI_HP1_AWBURST),.M_AXI_AWLOCK (M_AXI_HP1_AWLOCK),
        .M_AXI_AWCACHE(M_AXI_HP1_AWCACHE),.M_AXI_AWPROT (M_AXI_HP1_AWPROT),
        .M_AXI_AWVALID(M_AXI_HP1_AWVALID),.M_AXI_AWREADY(M_AXI_HP1_AWREADY),
        .M_AXI_WDATA  (M_AXI_HP1_WDATA),  .M_AXI_WSTRB  (M_AXI_HP1_WSTRB),
        .M_AXI_WLAST  (M_AXI_HP1_WLAST),  .M_AXI_WVALID (M_AXI_HP1_WVALID),
        .M_AXI_WREADY (M_AXI_HP1_WREADY),
        .M_AXI_BID    (M_AXI_HP1_BID),    .M_AXI_BRESP  (M_AXI_HP1_BRESP),
        .M_AXI_BVALID (M_AXI_HP1_BVALID), .M_AXI_BREADY (M_AXI_HP1_BREADY)
    );

    // ----------------------------------------------------------------
    // Compute submodules
    // TODO: replace flat array ports with ibram_b / wbram_b / obram_b BRAM ports
    //       once submodule interfaces are updated for 1-cycle BRAM latency.
    //       For now, instantiations are shown as structural placeholders.
    // ----------------------------------------------------------------

    // conv_2d    u_conv  (.clk(clk), .rst_n(rst_n), .start_process(conv_start), .done(conv_done), ...);
    // maxpool2d  u_pool  (.clk(clk), .rst_n(rst_n), .start(pool_start),         .done(pool_done), ...);
    // upsample   u_up    (.clk(clk), .rst_n(rst_n), .start(up_start),           .done(up_done),   ...);
    // route_concat u_cat (.clk(clk), .rst_n(rst_n), .start(cat_start),          .done(cat_done),  ...);

    // Tie done signals low until submodules are wired
    assign conv_done = 1'b0;
    assign pool_done = 1'b0;
    assign up_done   = 1'b0;
    assign cat_done  = 1'b0;

    // ----------------------------------------------------------------
    // Tracked DDR addresses for special buffers
    // (set on save_route / save_route8 pulses)
    // ----------------------------------------------------------------
    logic [ADDR_WIDTH-1:0] live_route_addr;   // DDR region holding Conv5 output
    logic [ADDR_WIDTH-1:0] live_route8_addr;  // DDR region holding Conv8 output

    // Output DDR address: the side that was just written (after ping flipped)
    logic [ADDR_WIDTH-1:0] output_ddr_addr;
    assign output_ddr_addr = ping ? buf_b_addr : buf_a_addr;

    // Input DDR address: driven by mem FSM (normal = current ping side, or route8 after ROUTE_RESTORE)
    logic [ADDR_WIDTH-1:0] input_ddr_addr;

    // ----------------------------------------------------------------
    // Memory management FSM
    // ----------------------------------------------------------------
    typedef enum logic [2:0] {
        M_IDLE,
        M_LOAD_IBRAM,    // loading input feature map → ibram
        M_LOAD_WBRAM,    // loading weights (or route_buf) → wbram
        M_COMPUTING,     // mem_stall=0, submodule running
        M_STORE_OBRAM,   // storing obram → output DDR region
        M_SPECIAL        // SAVE_ROUTE / ROUTE_RESTORE / YOLO1 / YOLO2 handling
    } mem_state_t;

    mem_state_t  mem_state;
    logic        need_wbram;      // current layer needs a weight/route BRAM load
    logic        use_up_buf_out;  // upsample output goes to up_buf_addr (not ping-pong)
    logic        use_route8_in;   // input comes from live_route8_addr (after ROUTE_RESTORE)
    logic [ADDR_WIDTH-1:0] store_target;  // DDR address to store obram into

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_state       <= M_IDLE;
            mem_stall       <= 1'b0;
            rd_start        <= 1'b0;
            rd_to_wbram     <= 1'b0;
            wr_start        <= 1'b0;
            use_route8_in   <= 1'b0;
            live_route_addr  <= '0;
            live_route8_addr <= '0;
            store_target    <= '0;
        end else begin
            rd_start <= 1'b0;
            wr_start <= 1'b0;

            // -- Track special DDR addresses --
            if (save_route)  live_route_addr  <= output_ddr_addr;
            if (save_route8) live_route8_addr <= output_ddr_addr;
            if (route_restore) use_route8_in  <= 1'b1;  // cleared after Conv10 loads

            case (mem_state)
                M_IDLE: begin
                    // Any _START state for a compute layer triggers a load
                    if (conv_start || pool_start || up_start || cat_start) begin
                        mem_stall <= 1'b1;

                        // Determine which DDR region to load ibram from
                        if (use_route8_in) begin
                            rd_src_addr    <= live_route8_addr;
                            use_route8_in  <= 1'b0;
                        end else if (cat_start) begin
                            rd_src_addr    <= up_buf_addr;  // concat a_buffer = upsample output
                        end else if (fsm_state == 6'd1) begin
                            rd_src_addr    <= image_addr;   // Conv1 reads image directly
                        end else begin
                            rd_src_addr    <= input_ddr_addr;
                        end

                        rd_byte_count  <= cfg_H_in * cfg_W_in * cfg_C_in;
                        rd_to_wbram    <= 1'b0;
                        rd_start       <= 1'b1;

                        // Record what to store after compute and where
                        if (up_start)
                            store_target <= up_buf_addr;   // upsample → up_buf
                        else
                            store_target <= output_ddr_addr;

                        need_wbram <= conv_start || cat_start;
                        mem_state  <= M_LOAD_IBRAM;
                    end

                    // YOLO1 / YOLO2: stall while wrapper stores output to det regions
                    if (save_det1) begin
                        mem_stall    <= 1'b1;
                        wr_dst_addr  <= det1_addr;
                        wr_byte_count <= 32'd43095;
                        wr_start     <= 1'b1;
                        mem_state    <= M_SPECIAL;
                    end
                    if (save_det2) begin
                        mem_stall    <= 1'b1;
                        wr_dst_addr  <= det2_addr;
                        wr_byte_count <= 32'd172380;
                        wr_start     <= 1'b1;
                        mem_state    <= M_SPECIAL;
                    end
                end

                M_LOAD_IBRAM: begin
                    if (rd_done) begin
                        if (need_wbram) begin
                            // Start loading wbram
                            rd_to_wbram <= 1'b1;
                            if (cat_start) begin
                                rd_src_addr   <= live_route_addr;   // concat b = route_buf
                                rd_byte_count <= 32'd172032;        // 26*26*256
                            end else begin
                                rd_src_addr   <= weight_addr + cfg_weight_base;
                                rd_byte_count <= cfg_K * cfg_K * cfg_C_in * cfg_C_out;
                            end
                            rd_start  <= 1'b1;
                            mem_state <= M_LOAD_WBRAM;
                        end else begin
                            // No weights needed — release stall and let compute fire
                            mem_stall <= 1'b0;
                            mem_state <= M_COMPUTING;
                        end
                    end
                end

                M_LOAD_WBRAM: begin
                    if (rd_done) begin
                        mem_stall <= 1'b0;
                        mem_state <= M_COMPUTING;
                    end
                end

                M_COMPUTING: begin
                    // Wait for whichever submodule is running to complete
                    if (conv_done || pool_done || up_done || cat_done) begin
                        mem_stall    <= 1'b1;
                        wr_dst_addr  <= store_target;
                        wr_byte_count <= cfg_H_out * cfg_W_out * cfg_C_out;
                        wr_start     <= 1'b1;
                        mem_state    <= M_STORE_OBRAM;
                    end
                end

                M_STORE_OBRAM: begin
                    if (wr_done) begin
                        // Update input address for next layer: the output we just stored
                        // becomes next layer's input (after ping flip handled in yolo_tiny_top)
                        input_ddr_addr <= store_target;
                        mem_stall      <= 1'b0;
                        mem_state      <= M_IDLE;
                    end
                end

                M_SPECIAL: begin
                    if (wr_done) begin
                        mem_stall <= 1'b0;
                        mem_state <= M_IDLE;
                    end
                end
            endcase
        end
    end

    // obram port A driven by write master (read side only, we_a unused)
    assign obram_a_wdata = '0;
    assign obram_a_we    = 1'b0;

    // obram port B — compute submodules write here (TODO: wire to submodule outputs)
    assign obram_b_addr  = '0;
    assign obram_b_wdata = '0;
    assign obram_b_we    = 1'b0;

    // ibram/wbram port B — compute submodules read here (TODO: wire to submodule inputs)
    assign ibram_b_addr  = '0;
    assign ibram_b_wdata = '0;
    assign ibram_b_we    = 1'b0;
    assign wbram_b_addr  = '0;
    assign wbram_b_wdata = '0;
    assign wbram_b_we    = 1'b0;

endmodule