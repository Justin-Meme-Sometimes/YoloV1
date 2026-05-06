`timescale 1ns / 1ps

// True dual-port BRAM tile.
// Port A: used by AXI loader (write from DDR) and AXI writer (read to DDR).
// Port B: used by the compute submodule (read input / write output).
//
// DATA_WIDTH = 128 matches the AXI HP bus width so one AXI beat = one BRAM word.
// DEPTH is in words (128-bit words).  For byte-level addressing from the compute
// side, the submodule shifts its byte address right by 4 (log2(16)).
//
// Vivado infers this as RAMB36E2 blocks automatically.
module dp_bram #(
    parameter DATA_WIDTH = 128,
    parameter DEPTH      = 16384   // words; 16384 * 16 B = 256 KB
)(
    // Port A — AXI loader side
    input  logic                    clk_a,
    input  logic [31:0]             addr_a,
    input  logic [DATA_WIDTH-1:0]   wdata_a,
    input  logic                    we_a,
    output logic [DATA_WIDTH-1:0]   rdata_a,

    // Port B — compute side (1-cycle read latency)
    input  logic                    clk_b,
    input  logic [31:0]             addr_b,
    input  logic [DATA_WIDTH-1:0]   wdata_b,
    input  logic                    we_b,
    output logic [DATA_WIDTH-1:0]   rdata_b
);

    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    always_ff @(posedge clk_a) begin
        if (we_a)
            mem[addr_a[31:0] % DEPTH] <= wdata_a;
        rdata_a <= mem[addr_a[31:0] % DEPTH];
    end

    always_ff @(posedge clk_b) begin
        if (we_b)
            mem[addr_b[31:0] % DEPTH] <= wdata_b;
        rdata_b <= mem[addr_b[31:0] % DEPTH];
    end

endmodule
