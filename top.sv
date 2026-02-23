`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/27/2025 10:00:55 PM
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////




module top();

    // Clock and reset
    logic clk;
    logic rst_n;
    
    // Input buffers
    logic [15:0] weight_buffer [223:0];
    logic [15:0] i_buffer [223:0];
    logic [15:0] bias [31:0];
    
    // Configuration
    logic [31:0] C_out;
    logic [31:0] H_out;
    logic [31:0] W_out;
    logic [31:0] C_in;
    logic [31:0] stride;
    logic [31:0] W_in;
    logic [31:0] weight_base;

    // Control
    logic start_process;


    // ========================================
    // DUT Instantiation (K=3, 64 MACs)
    // ========================================

    conv_2d #(.K(3), .NUM_MACS(64)) dut (
        .clk(clk),
        .weight_buffer(weight_buffer),
        .i_buffer(i_buffer),
        .C_out(C_out),
        .H_out(H_out),
        .W_out(W_out),
        .C_in(C_in),
        .bias(bias),
        .stride(stride),
        .W_in(W_in),
        .weight_base(weight_base),
        .start_process(start_process),
        .rst_n(rst_n)
    );
    
    
    // ========================================
    // Clock Generation
    // ========================================
    
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100 MHz
    end
    
    
    // ========================================
    // Initialize Buffers
    // ========================================
    
    initial begin
        // Fill input buffer with test pattern
        for (int y = 0; y < 224; y++) begin
            for (int x = 0; x < 16; x++) begin
                // Simple pattern: 1.0 + position offset
                i_buffer[y][x] = y[15:0] + x[15:0];
            end
        end
        
        // Fill weight buffer with test pattern
        for (int i = 0; i < 224; i++) begin
            for (int j = 0; j < 16; j++) begin
                // All weights = 0.5 for simplicity
                weight_buffer[i][j] = 1'd1;
            end
        end

        for (int i = 0; i < 224; i++) begin
            for (int j = 0; j < 16; j++) begin
                // All weights = 0.5 for simplicity
                bias[i][j] = 16'h030F + i[15:0] + j[15:0];
            end
        end
    end
    
    
    // ========================================
    // Monitor outputs
    // ========================================
   
    
    
    // ========================================
    // Test Stimulus
    // ========================================
    
    initial begin
        $display("========================================");
        $display("Conv2D Testbench");
        $display("========================================\n");
        
        // Initialize
        rst_n = 0;
        start_process = 0;

        
        // Configuration
        //C_in = 3;           // RGB
        C_out = 16;         // First YOLO layer output channels
        stride = 1;         // No downsampling
        W_in = 224;         // Input width
        H_out = (W_in - 3) / 1 + 1;  // 222
        W_out = (W_in - 3) / 1 + 1;  // 222
        weight_base = 0;    // Weights start at 0

        $display("Configuration:");
        $display("  C_in = %0d", C_in);
        $display("  C_out = %0d", C_out);
        $display("  stride = %0d", stride);
        $display("  W_in = %0d", W_in);
        $display("  Kernel = 3x3");
        $display("  NUM_MACS = 64\n");
        
        // Reset
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);
        
        $display("Starting convolution...\n");
        
        // Start processing
        start_process = 1;
        @(posedge clk);
        start_process = 0;
        
        // Let it run for some cycles
        repeat(10000) @(posedge clk);
        
        // Check if done signal appears
      
        $display("\nFinal position: (y=%0d, x=%0d, c=%0d)", 
                 dut.y_out, dut.x_out, dut.c_out);
        
        repeat(10) @(posedge clk);
        
        $display("\n========================================");
        $display("Test complete");
        $display("========================================");
        
        $finish;
    end
    
    

    
  
endmodule
