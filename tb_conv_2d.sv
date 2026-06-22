`timescale 1ns / 1ps

// Unit testbench for conv_2d with BRAM interface.
//
// NUM_MACS=9 = K*K*C_in eliminates invalid MAC slots so expected values are clean.
//
// Test A — inputs=1,   weights=1,   bias=0  → sum=9,      output=9
// Test B — inputs=1,   weights=1,   bias=-1 → sum=9-1=8,  output=8
// Test C — inputs=127, weights=127, bias=0  → 9*127*127=145161 clips → 127
// Test D — inputs=1,   weights=-1,  bias=0  → sum=-9, leaky relu (-9>>>3) → -2
module tb_conv_2d;

    localparam BRAM_WORDS = 4;

    logic clk = 0;
    always #5 clk = ~clk;

    logic rst_n, start_process, done;
    logic signed [31:0] bias [31:0];

    logic [31:0]  ibram_a_addr, ibram_b_addr;
    logic [127:0] ibram_a_wdata, ibram_a_rdata, ibram_b_rdata;
    logic         ibram_a_we;

    logic [31:0]  wbram_a_addr, wbram_b_addr;
    logic [127:0] wbram_a_wdata, wbram_a_rdata, wbram_b_rdata;
    logic         wbram_a_we;

    logic [31:0]  obram_a_addr, obram_b_addr;
    logic [127:0] obram_a_rdata, obram_b_wdata;
    logic         obram_b_we;

    conv_2d #(
        .K(3), .NUM_MACS(9),
        .IBUF_DEPTH(BRAM_WORDS), .WBUF_DEPTH(BRAM_WORDS), .OBUF_DEPTH(BRAM_WORDS)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .bias(bias),
        .C_out(32'd1), .H_out(32'd1), .W_out(32'd1),
        .C_in(32'd1),  .stride(32'd1), .W_in(32'd3),
        .weight_base(32'd0),
        .start_process(start_process),
        .ibram_addr(ibram_b_addr),  .ibram_rdata(ibram_b_rdata),
        .wbram_addr(wbram_b_addr),  .wbram_rdata(wbram_b_rdata),
        .obram_addr(obram_b_addr),  .obram_wdata(obram_b_wdata), .obram_we(obram_b_we),
        .done(done)
    );

    dp_bram #(.DATA_WIDTH(128), .DEPTH(BRAM_WORDS)) u_ibram (
        .clk_a(clk), .addr_a(ibram_a_addr), .wdata_a(ibram_a_wdata),
        .we_a(ibram_a_we),  .rdata_a(ibram_a_rdata),
        .clk_b(clk), .addr_b(ibram_b_addr), .wdata_b('0),
        .we_b(1'b0),        .rdata_b(ibram_b_rdata)
    );
    dp_bram #(.DATA_WIDTH(128), .DEPTH(BRAM_WORDS)) u_wbram (
        .clk_a(clk), .addr_a(wbram_a_addr), .wdata_a(wbram_a_wdata),
        .we_a(wbram_a_we),  .rdata_a(wbram_a_rdata),
        .clk_b(clk), .addr_b(wbram_b_addr), .wdata_b('0),
        .we_b(1'b0),        .rdata_b(wbram_b_rdata)
    );
    dp_bram #(.DATA_WIDTH(128), .DEPTH(BRAM_WORDS)) u_obram (
        .clk_a(clk), .addr_a(obram_a_addr), .wdata_a('0),
        .we_a(1'b0),        .rdata_a(obram_a_rdata),
        .clk_b(clk), .addr_b(obram_b_addr), .wdata_b(obram_b_wdata),
        .we_b(obram_b_we),  .rdata_b()
    );

    int cycle_cnt = 0;
    always @(posedge clk) cycle_cnt++;

    int pass_cnt = 0, fail_cnt = 0;

    task automatic do_reset();
        rst_n = 0; start_process = 0;
        $display("  [t=%0d] reset asserted", cycle_cnt);
        repeat(4) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);
        $display("  [t=%0d] reset released, DUT idle", cycle_cnt);
    endtask

    task automatic write_ibram(input logic [127:0] data, input string desc);
        @(posedge clk); #1;
        ibram_a_addr = 0; ibram_a_wdata = data; ibram_a_we = 1;
        @(posedge clk); #1;
        ibram_a_we = 0;
        $display("  [t=%0d] ibram word[0] loaded: %s", cycle_cnt, desc);
    endtask

    task automatic write_wbram(input logic [127:0] data, input string desc);
        @(posedge clk); #1;
        wbram_a_addr = 0; wbram_a_wdata = data; wbram_a_we = 1;
        @(posedge clk); #1;
        wbram_a_we = 0;
        $display("  [t=%0d] wbram word[0] loaded: %s", cycle_cnt, desc);
    endtask

    task automatic run_and_check(input string name, input logic signed [7:0] expected);
        logic signed [7:0] actual;
        int t_start, watchdog;

        $display("  [t=%0d] start_process asserted", cycle_cnt);
        @(posedge clk); #1; start_process = 1;
        t_start = cycle_cnt;
        @(posedge clk); #1; start_process = 0;
        $display("  [t=%0d]   preload phase: serializing %0d BRAM reads over %0d cycles",
                 cycle_cnt, 9, 10);

        for (watchdog = 0; watchdog < 5000 && !done; watchdog++)
            @(posedge clk);
        if (!done) begin $display("TIMEOUT: %s", name); $finish; end

        $display("  [t=%0d] done asserted (%0d cycles elapsed)",
                 cycle_cnt, cycle_cnt - t_start);
        $display("  [t=%0d]   MAC fired: sum = 9 x (input x weight)", cycle_cnt);
        $display("  [t=%0d]   bias added, leaky relu applied, int8 clipped", cycle_cnt);
        $display("  [t=%0d]   obram write: wr_buf[7:0] packed, obram_we pulsed", cycle_cnt);

        // Read obram byte 0 via port A (1-cycle registered latency)
        @(posedge clk); #1;
        obram_a_addr = 0;
        @(posedge clk); #1;
        @(posedge clk); #1;
        actual = $signed(obram_a_rdata[7:0]);

        $display("  [t=%0d] obram[word=0, byte=0] = %0d  (expected %0d)",
                 cycle_cnt, actual, expected);

        if (actual === expected) begin
            $display("  --> PASS [%s]", name);
            pass_cnt++;
        end else begin
            $display("  --> FAIL [%s]  expected %0d, got %0d", name, expected, actual);
            fail_cnt++;
        end
    endtask

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_conv_2d);

        ibram_a_we = 0; ibram_a_addr = 0; ibram_a_wdata = 0;
        wbram_a_we = 0; wbram_a_addr = 0; wbram_a_wdata = 0;
        obram_a_addr = 0;

        $display("========================================");
        $display(" conv_2d testbench");
        $display(" DUT: K=3, C_in=1, C_out=1, H_out=1, W_out=1");
        $display(" NUM_MACS=9 (= K*K*C_in, no invalid slots)");
        $display(" total_macs=9, num_chunks=1, preload=10 cycles");
        $display("========================================");

        // ------------------------------------------------------------------
        // Test A: inputs=1, weights=1, bias=0 → 9×1×1=9
        // ------------------------------------------------------------------
        $display("\n[Test A] inputs=1, weights=1, bias=0");
        $display("  expected: sum=9, bias+sum=9, relu=9, clip=9 → output=9");
        do_reset();
        for (int i = 0; i < 32; i++) bias[i] = 32'sd0;
        $display("  [t=%0d] bias[0..31] = 0", cycle_cnt);
        write_ibram({16{8'd1}}, "all bytes=1 (9 input pixels, each =1)");
        write_wbram({16{8'd1}}, "all bytes=1 (9 weights, each =1)");
        run_and_check("A", 8'sd9);

        // ------------------------------------------------------------------
        // Test B: same BRAM data, bias=-1 → 9-1=8
        // ------------------------------------------------------------------
        $display("\n[Test B] inputs=1, weights=1, bias=-1 (BRAM unchanged)");
        $display("  expected: sum=9, bias+sum=8, relu=8, clip=8 → output=8");
        do_reset();
        for (int i = 0; i < 32; i++) bias[i] = -32'sd1;
        $display("  [t=%0d] bias[0..31] = -1", cycle_cnt);
        $display("  [t=%0d] ibram/wbram unchanged from test A", cycle_cnt);
        run_and_check("B", 8'sd8);

        // ------------------------------------------------------------------
        // Test C: inputs=127, weights=127 → 9×127×127=145161 → clips to 127
        // ------------------------------------------------------------------
        $display("\n[Test C] inputs=127, weights=127, bias=0");
        $display("  expected: sum=145161, bias+sum=145161 > 127 → clip → output=127");
        do_reset();
        for (int i = 0; i < 32; i++) bias[i] = 32'sd0;
        $display("  [t=%0d] bias[0..31] = 0", cycle_cnt);
        write_ibram({16{8'd127}}, "all bytes=127");
        write_wbram({16{8'd127}}, "all bytes=127");
        run_and_check("C", 8'sd127);

        // ------------------------------------------------------------------
        // Test D: weights=-1 → sum=-9 → leaky relu: -9>>>3=-2
        // ------------------------------------------------------------------
        $display("\n[Test D] inputs=1, weights=-1, bias=0");
        $display("  expected: sum=-9, relu: x<0 → x>>>3 = -9>>>3 = -2, clip=-2 → output=-2");
        do_reset();
        for (int i = 0; i < 32; i++) bias[i] = 32'sd0;
        $display("  [t=%0d] bias[0..31] = 0", cycle_cnt);
        write_ibram({16{8'd1}},   "all bytes=1");
        write_wbram({16{8'hFF}},  "all bytes=0xFF=-1 signed");
        run_and_check("D", -8'sd2);

        // ------------------------------------------------------------------
        $display("\n========================================");
        $display(" Results: %0d passed, %0d failed", pass_cnt, fail_cnt);
        $display("========================================");
        $finish;
    end

endmodule
