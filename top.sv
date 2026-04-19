`timescale 1ns / 1ps

// Simple verification testbench.
// Input: 5x5, C_in=1, all activations = 1
// Weights: K=3, C_in=1, C_out=1, all weights = 1
// Bias = 0
// Expected output: 3x3, all values = 9 (= 3*3*1*1*1)
module top();

    logic clk, rst_n;

    // Buffers: sized for 5x5x1 input (25 entries), 9 weights, 9 output pixels
    localparam IBUF = 32;
    localparam WBUF = 32;
    localparam OBUF = 32;

    logic signed [7:0]  weight_buffer [WBUF-1:0];
    logic signed [7:0]  i_buffer      [IBUF-1:0];
    logic signed [31:0] bias          [31:0];

    logic [31:0] C_out, H_out, W_out, C_in, stride, W_in, weight_base;
    logic start_process;

    logic signed [7:0] output_buffer [OBUF-1:0];
    logic done;

    conv_2d #(
        .K(3), .NUM_MACS(64),
        .IBUF_DEPTH(IBUF), .WBUF_DEPTH(WBUF), .OBUF_DEPTH(OBUF)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .weight_buffer(weight_buffer),
        .i_buffer(i_buffer),
        .bias(bias),
        .C_out(C_out), .H_out(H_out), .W_out(W_out),
        .C_in(C_in), .stride(stride), .W_in(W_in),
        .weight_base(weight_base),
        .start_process(start_process),
        .output_buffer(output_buffer),
        .done(done)
    );

    // 100 MHz clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Configuration: 5x5 input, K=3, C_in=1, C_out=1, stride=1
    // Output size = (5-3)/1+1 = 3x3
    initial begin
        rst_n         = 0;
        start_process = 0;
        C_in          = 1;
        C_out         = 1;
        stride        = 1;
        W_in          = 5;
        H_out         = 3;
        W_out         = 3;
        weight_base   = 0;

        // All activations = 1
        for (int i = 0; i < IBUF; i++) i_buffer[i] = 8'sd0;
        for (int y = 0; y < 5; y++)
            for (int x = 0; x < 5; x++)
                i_buffer[y * 5 * 1 + x * 1 + 0] = 8'sd1;

        // All weights = 1
        for (int i = 0; i < WBUF; i++) weight_buffer[i] = 8'sd0;
        for (int co = 0; co < 1; co++)
            for (int ky = 0; ky < 3; ky++)
                for (int kx = 0; kx < 3; kx++)
                    weight_buffer[co*9 + ky*3 + kx] = 8'sd1;

        // Zero bias
        for (int i = 0; i < 32; i++) bias[i] = 32'sd0;

        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);

        $display("Starting conv2d: 5x5 input, K=3, C_in=1, C_out=1, stride=1");
        $display("Expected output: all 9s in a 3x3 grid");

        start_process = 1;
        @(posedge clk);
        start_process = 0;

        // Wait for done
        @(posedge done);
        @(posedge clk);

        $display("\nOutput buffer (3x3, C_out=1):");
        for (int y = 0; y < 3; y++) begin
            for (int x = 0; x < 3; x++) begin
                $write("  %0d", $signed(output_buffer[y*3*1 + x*1 + 0]));
            end
            $display("");
        end

        // Check correctness
        begin
            int errors = 0;
            for (int y = 0; y < 3; y++)
                for (int x = 0; x < 3; x++)
                    if ($signed(output_buffer[y*3 + x]) !== 8'sd9) begin
                        $display("FAIL at (%0d,%0d): got %0d, expected 9", y, x,
                                 $signed(output_buffer[y*3 + x]));
                        errors++;
                    end
            if (errors == 0)
                $display("\nPASS: all outputs correct");
            else
                $display("\nFAIL: %0d errors", errors);
        end

        $finish;
    end

endmodule
