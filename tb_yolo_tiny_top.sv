`timescale 1ns / 1ps

// Smoke testbench for yolo_tiny_top.
// Fills weight/image buffers with 1s, bias with 0, fires start, waits for done.
// Does NOT verify numerical output correctness (needs real quantized weights for that).
// WARNING: Full 416x416 simulation takes many millions of cycles.
module tb_yolo_tiny_top;

    localparam WEIGHT_DEPTH = 8649648;
    localparam BIAS_DEPTH   = 4096;
    localparam IMG_DEPTH    = 519168;   // 416*416*3
    localparam BUF_DEPTH    = 2752512;  // 416*416*16
    localparam ROUTE_DEPTH  = 172032;   // 26*26*256
    localparam UP_DEPTH     = 87360;    // 26*26*128
    localparam CAT_DEPTH    = 258048;   // 26*26*384
    localparam DET1_DEPTH   = 43095;    // 13*13*255
    localparam DET2_DEPTH   = 172380;   // 26*26*255

    // Max cycles before timeout
    localparam longint TIMEOUT = 2_000_000_000;

    logic clk, rst_n, start, done;

    logic signed [7:0]  weight_buf [WEIGHT_DEPTH-1:0];
    logic signed [31:0] bias_buf   [BIAS_DEPTH-1:0];
    logic signed [7:0]  image_buf  [IMG_DEPTH-1:0];
    logic signed [7:0]  det1_buf   [DET1_DEPTH-1:0];
    logic signed [7:0]  det2_buf   [DET2_DEPTH-1:0];

    yolo_tiny_top #(
        .WEIGHT_DEPTH(WEIGHT_DEPTH),
        .BIAS_DEPTH(BIAS_DEPTH),
        .IMG_DEPTH(IMG_DEPTH),
        .BUF_DEPTH(BUF_DEPTH),
        .ROUTE_DEPTH(ROUTE_DEPTH),
        .UP_DEPTH(UP_DEPTH),
        .CAT_DEPTH(CAT_DEPTH),
        .DET1_DEPTH(DET1_DEPTH),
        .DET2_DEPTH(DET2_DEPTH)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .weight_buf(weight_buf),
        .bias_buf(bias_buf),
        .image_buf(image_buf),
        .det1_buf(det1_buf),
        .det2_buf(det2_buf),
        .done(done)
    );

    // 100 MHz clock
    initial clk = 0;
    always #5 clk = ~clk;

    // Cycle counter + watchdog
    longint cycle_count;
    always @(posedge clk) begin
        cycle_count++;
        if (cycle_count >= TIMEOUT) begin
            $display("TIMEOUT: done never asserted after %0d cycles", TIMEOUT);
            $finish;
        end
    end

    initial begin
        rst_n       = 0;
        start       = 0;
        cycle_count = 0;

        // All weights = 1, bias = 0, image = 1
        for (int i = 0; i < WEIGHT_DEPTH; i++) weight_buf[i] = 8'sd1;
        for (int i = 0; i < BIAS_DEPTH;   i++) bias_buf[i]   = 32'sd0;
        for (int i = 0; i < IMG_DEPTH;    i++) image_buf[i]  = 8'sd1;

        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);

        $display("[%0t] Starting YOLOv3-Tiny inference", $time);
        start = 1;
        @(posedge clk);
        start = 0;

        @(posedge done);
        @(posedge clk);

        $display("[%0t] PASS: done asserted after %0d cycles", $time, cycle_count);

        $display("det1_buf[0:7]: %0d %0d %0d %0d %0d %0d %0d %0d",
            $signed(det1_buf[0]), $signed(det1_buf[1]),
            $signed(det1_buf[2]), $signed(det1_buf[3]),
            $signed(det1_buf[4]), $signed(det1_buf[5]),
            $signed(det1_buf[6]), $signed(det1_buf[7]));
        $display("det2_buf[0:7]: %0d %0d %0d %0d %0d %0d %0d %0d",
            $signed(det2_buf[0]), $signed(det2_buf[1]),
            $signed(det2_buf[2]), $signed(det2_buf[3]),
            $signed(det2_buf[4]), $signed(det2_buf[5]),
            $signed(det2_buf[6]), $signed(det2_buf[7]));

        $finish;
    end

endmodule
