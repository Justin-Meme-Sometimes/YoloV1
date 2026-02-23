`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/27/2025 09:59:06 PM
// Design Name: 
// Module Name: op_units
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


module fp_add ( //probably wrong
input  logic [15:0] a,
input  logic [15:0] b,
input  logic        clk,
input  logic        rst_n,
output logic [15:0] result
);
// Extract fields (combinational temps; you can keep or remove later)
logic        sign_a, sign_b, sign_r;
logic [4:0]  exp_a, exp_b, exp_r;
logic [10:0] mant_a, mant_b;
logic [12:0] aligned_a, aligned_b;
logic [13:0] sum;
logic [4:0]  exp_diff;
logic [12:0] mant_sum_norm;
logic [4:0]  exp_adj;
logic        guard, round, sticky;
logic [9:0]  rounded_frac;
logic        round_bit;

// =========================
// Pipeline register DECLS
// =========================

// ---- Stage 1 ----
logic        s1_sign_a, s1_sign_b;
logic [4:0]  s1_exp_a, s1_exp_b, s1_exp_r, s1_exp_diff;
logic [10:0] s1_mant_a, s1_mant_b;
logic [12:0] s1_aligned_a, s1_aligned_b;

// ---- Stage 2 ----
logic        s2_sign_a, s2_sign_b, s2_sign_r;
logic [4:0]  s2_exp_diff, s2_exp_r;
logic [12:0] s2_aligned_a, s2_aligned_b;
logic [13:0] s2_sum;

// ---- Stage 3 ----
logic        s3_sign_a, s3_sign_b, s3_sign_r;
logic [4:0]  s3_exp_diff, s3_exp_r, s3_exp_r_edited;
logic [12:0] s3_aligned_a, s3_aligned_b, s3_mant_sum_norm;
logic [13:0] s3_sum;
logic        s3_guard, s3_round, s3_sticky;

// ---- Stage 4 ----
logic        s4_sign_a, s4_sign_b, s4_sign_r;
logic [4:0]  s4_exp_diff, s4_exp_r, s4_exp_r_edited;
logic [12:0] s4_aligned_a, s4_aligned_b, s4_mant_sum_norm;
logic [13:0] s4_sum;
logic        s4_guard, s4_round, s4_sticky;
logic        s4_round_bit;
logic [12:0]  s4_rounded_frac;
logic [11:0]  s4_kept;
logic [9:0]  s4_rounded_frac_edited;
logic [15:0] s4_result;

// =========================
// Pipeline registers (yours)
// =========================
logic is_zero_a, is_zero_b;
assign is_zero_a = (a[14:0] == 15'b0);  // Exp and mantissa both zero
assign is_zero_b = (b[14:0] == 15'b0);

always_ff @(posedge clk or negedge rst_n) begin // first stage registers
    if(!rst_n) begin
        s1_sign_a <= 0;
        s1_sign_b <= 0;
        s1_exp_a  <= 0;
        s1_exp_b  <= 0;
        s1_mant_a <= 0;  // Add hidden bit
        s1_mant_b <= 0;
    end else begin
        s1_sign_a <= a[15];
        s1_sign_b <= b[15];
        s1_exp_a  <= a[14:10];
        s1_exp_b  <= b[14:10];
        s1_mant_a <= (a[14:10] == 5'd0) ? {1'b0, a[9:0]} : {1'b1, a[9:0]};
        s1_mant_b <= (b[14:10] == 5'd0) ? {1'b0, b[9:0]} : {1'b1, b[9:0]};
    end
end

always_ff @(posedge clk or negedge rst_n) begin // second stage registers
    if(!rst_n) begin
        s2_sign_a    <= 0;
        s2_sign_b    <= 0;
        s2_exp_diff  <= 0;
        s2_aligned_a <= 0;
        s2_aligned_b <= 0;
        s2_exp_r     <= 0;
    end else begin
        s2_sign_a    <= s1_sign_a;
        s2_sign_b    <= s1_sign_b;
        s2_exp_diff  <= s1_exp_diff;
        s2_aligned_a <= s1_aligned_a;
        s2_aligned_b <= s1_aligned_b;
        s2_exp_r     <= s1_exp_r;
        // s2_sum / s2_sign_r are driven in comb, but you’re also
        // latching them in the next stage; ok to keep as-is.
    end
end

always_ff @(posedge clk or negedge rst_n) begin // third stage registers
    if(!rst_n) begin
        s3_sign_a       <= 0;
        s3_sign_b       <= 0;
        s3_exp_diff     <= 0;
        s3_aligned_a    <= 0;
        s3_aligned_b    <= 0;
        s3_exp_r        <= 0;
        s3_sum          <= 0;
        s3_sign_r       <= 0;
    end else begin
        s3_sign_a       <= s2_sign_a;
        s3_sign_b       <= s2_sign_b;
        s3_exp_diff     <= s2_exp_diff;
        s3_aligned_a    <= s2_aligned_a;
        s3_aligned_b    <= s2_aligned_b;
        s3_exp_r        <= s2_exp_r;
        s3_sum          <= s2_sum;
        s3_sign_r       <= s2_sign_r;
        // s3_* normalize outputs assigned in comb, then latched to s4
    end
end

always_ff @(posedge clk or negedge rst_n) begin // fourth stage registers
    if(!rst_n) begin
        s4_sign_a        <= 0;
        s4_sign_b        <= 0;
        s4_exp_diff      <= 0;
        s4_aligned_a     <= 0;
        s4_aligned_b     <= 0;
        s4_exp_r         <= 0;
        s4_sum           <= 0;
        s4_sign_r        <= 0;
        s4_mant_sum_norm <= 0;
        s4_guard         <= 0;
        s4_round         <= 0;
        s4_sticky        <= 0;
    end else begin
        s4_sign_a        <= s3_sign_a;
        s4_sign_b        <= s3_sign_b;
        s4_exp_diff      <= s3_exp_diff;
        s4_aligned_a     <= s3_aligned_a;
        s4_aligned_b     <= s3_aligned_b;
        s4_exp_r         <= s3_exp_r_edited;
        s4_sum           <= s3_sum;
        s4_sign_r        <= s3_sign_r;
        s4_mant_sum_norm <= s3_mant_sum_norm;
        s4_guard         <= s3_guard;
        s4_round         <= s3_round;
        s4_sticky        <= s3_sticky;
        // s4_round_bit, s4_rounded_frac, s4_result set in comb below
    end
end

// Drive module output from stage 4
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        result <= 0;
    end else begin
        result <= s4_result;
    end
end

// =========================
// Combinational datapath
// =========================
always_comb begin
    if (s1_exp_a > s1_exp_b) begin
        s1_exp_diff  = s1_exp_a - s1_exp_b;
        s1_aligned_a = {s1_mant_a, 2'b00};
        
        // Capture shifted-out bits for GRS
        if (s1_exp_diff == 0) begin
            s1_aligned_b = {s1_mant_b, 2'b00};
            // No bits shifted out, GRS handled in normalization
        end else if (s1_exp_diff < 13) begin
            s1_aligned_b = ({s1_mant_b, 2'b00} >> s1_exp_diff);
            // Could track sticky bit here if you want perfect rounding
        end else begin
            s1_aligned_b = 13'b0;  // Shifted out completely
        end
        s1_exp_r = s1_exp_a;
    end else begin
        // Mirror logic for other case
        s1_exp_diff  = s1_exp_b - s1_exp_a;
        s1_aligned_b = {s1_mant_b, 2'b00};
        if (s1_exp_diff < 13) begin
            s1_aligned_a = ({s1_mant_a, 2'b00} >> s1_exp_diff);
        end else begin
            s1_aligned_a = 13'b0;
        end
        s1_exp_r = s1_exp_b;
    end


    // === Handle signs (assumes same sign only, no subtraction) ===
    if (s2_sign_a == s2_sign_b) begin
        // Addition
        s2_sum = s2_aligned_a + s2_aligned_b;
        s2_sign_r = s2_sign_a;
    end else begin
        // Subtraction - need to handle magnitude comparison
        if (s2_aligned_a >= s2_aligned_b) begin
            s2_sum = s2_aligned_a - s2_aligned_b;
            s2_sign_r = s2_sign_a;
        end else begin
            s2_sum = s2_aligned_b - s2_aligned_a;
            s2_sign_r = s2_sign_b;
        end
    end

    // === Normalize ===
    if (s3_sum[13]) begin
        // Overflow: shift right by 1
        s3_mant_sum_norm = s3_sum[13:1];
        s3_guard         = s3_sum[0];
        s3_round         = 1'b0;
        s3_sticky        = 1'b0;
        s3_exp_r_edited  = s3_exp_r + 1;
    end else begin
        // No overflow - check if normalized
        if (s3_sum[12]) begin
            // Already normalized (format 1.xxx)
            s3_mant_sum_norm = s3_sum[12:0];
            s3_exp_r_edited  = s3_exp_r;
        end else begin
            // Shift left by 1 (format 01.xxx)
            s3_mant_sum_norm = s3_sum[11:0] << 1;
            s3_exp_r_edited  = s3_exp_r - 1;
        end
        s3_guard  = 1'b0;
        s3_round  = 1'b0;
        s3_sticky = 1'b0;
    end

    // === Rounding (round to nearest, tie to even) ===
    s4_round_bit = s4_guard & (s4_mant_sum_norm[0] | s4_round | s4_sticky);
    s4_kept = s4_mant_sum_norm[11:2];

    s4_rounded_frac = s4_mant_sum_norm[11:2] + s4_round_bit;

    if(s4_rounded_frac[10])begin
        //if overflow shift mantissa by 1 and increment exponent 
       s4_rounded_frac_edited = s4_rounded_frac[10:1];
       s4_exp_r_edited = s4_exp_r + 1;
    end else begin
       s4_rounded_frac_edited = s4_rounded_frac[9:0];
       s4_exp_r_edited = s4_exp_r;
    end

    // === Assemble result ===
    if (s4_exp_r_edited == 5'd0 || s4_mant_sum_norm[12:2] == 11'b0) begin
        s4_result = 16'h0000;  // Positive zero
    end else begin
        s4_result = {s4_sign_r, s4_exp_r_edited, s4_rounded_frac_edited};
    end
end
endmodule


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/14/2025 02:23:07 PM
// Design Name: 
// Module Name: fp_mul
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


module fp_mul ( //probably wrong
input  logic [15:0] a,
input  logic [15:0] b,
input  logic        clk,
input  logic        rst_n,
output logic [15:0] result
);
// Extract fields (combinational temps; you can keep or remove later)
// logic        sign_a, sign_b, sign_r;
// logic [4:0]  exp_a, exp_b, exp_r;
// logic [10:0] mant_a, mant_b;
// logic [12:0] aligned_a, aligned_b;
// logic [13:0] sum;
// logic [4:0]  exp_diff;
// logic [12:0] mant_sum_norm;
// logic [4:0]  exp_adj;
// logic        guard, round, sticky;
// logic [9:0]  rounded_frac;
// logic        round_bit;

// =========================
// Pipeline register DECLS
// =========================

// ---- Stage 1 ----
logic        s1_sign_a, s1_sign_b;
logic [4:0]  s1_exp_a, s1_exp_b, s1_exp_r, s1_exp_diff, s1_exp_a_E, s1_exp_b_E;
logic [10:0] s1_mant_a, s1_mant_b;
logic [11:0] s1_aligned_a, s1_aligned_b;

// ---- Stage 2 ----
logic        s2_sign_a, s2_sign_b, s2_sign_r;
logic [4:0]  s2_exp_diff, s2_exp_a, s2_exp_b, s2_exp_change;
logic [11:0] s2_aligned_a, s2_aligned_b;
logic [24:0] s2_product;

// ---- Stage 3 ----
logic        s3_sign_a, s3_sign_b, s3_sign_r;
logic [4:0]  s3_exp_diff, s3_exp_r, s3_exp_r_edited;
logic [11:0] s3_aligned_a, s3_aligned_b;
logic [9:0]  s3_mant_sum_norm;
logic [24:0] s3_product;
logic        s3_guard, s3_round, s3_sticky;

// ---- Stage 4 ----
logic        s4_sign_a, s4_sign_b, s4_sign_r;
logic [4:0]  s4_exp_diff, s4_exp_r, s4_exp_r_edited;
logic [11:0] s4_aligned_a, s4_aligned_b;
logic [23:0] s4_product;
logic        s4_guard, s4_round, s4_sticky;
logic        s4_round_bit;
logic [12:0]  s4_rounded_frac;
logic [11:0]  s4_kept;
logic [9:0]  s4_rounded_frac_edited, s4_mant_sum_norm;
logic [15:0] s4_result;

// =========================
// Pipeline registers (yours)
// =========================

always_ff @(posedge clk or negedge rst_n) begin // first stage registers
    if(!rst_n) begin
        s1_sign_a <= 0;
        s1_sign_b <= 0;
        s1_exp_a  <= 0;
        s1_exp_b  <= 0;
        s1_mant_a <= 0;  // Add hidden bit
        s1_mant_b <= 0;
    end else begin
        s1_sign_a <= a[15];
        s1_sign_b <= b[15];
        s1_exp_a  <= a[14:10];
        s1_exp_b  <= b[14:10];
        s1_mant_a <= {1'b1, a[9:0]};  // Add hidden bit
        s1_mant_b <= {1'b1, b[9:0]};
    end
end

always_ff @(posedge clk or negedge rst_n) begin // second stage registers
    if(!rst_n) begin
        s2_sign_a    <= 0;
        s2_sign_b    <= 0;
        s2_exp_diff  <= 0;
        s2_aligned_a <= 0;
        s2_aligned_b <= 0;
        s2_exp_a     <= 0;
        s2_exp_b     <= 0;
    end else begin
        s2_sign_a    <= s1_sign_a;
        s2_sign_b    <= s1_sign_b;
        s2_exp_diff  <= s1_exp_diff;
        s2_aligned_a <= s1_aligned_a;
        s2_aligned_b <= s1_aligned_b;
        s2_exp_a     <= s1_exp_a_E;
        s2_exp_b     <= s1_exp_b_E;
        // s2_sum / s2_sign_r are driven in comb, but you’re also
        // latching them in the next stage; ok to keep as-is.
    end
end

always_ff @(posedge clk or negedge rst_n) begin // third stage registers
    if(!rst_n) begin
        s3_sign_a       <= 0;
        s3_sign_b       <= 0;
        s3_exp_diff     <= 0;
        s3_aligned_a    <= 0;
        s3_aligned_b    <= 0;
        s3_exp_r        <= 0;
        s3_product      <= 0;
        s3_sign_r       <= 0;
    end else begin
        s3_sign_a       <= s2_sign_a;
        s3_sign_b       <= s2_sign_b;
        s3_exp_diff     <= s2_exp_diff;
        s3_aligned_a    <= s2_aligned_a;
        s3_aligned_b    <= s2_aligned_b;
        s3_exp_r        <= s2_exp_change;
        s3_product      <= s2_product;
        s3_sign_r       <= s2_sign_r;
        // s3_* normalize outputs assigned in comb, then latched to s4
    end
end

always_ff @(posedge clk or negedge rst_n) begin // fourth stage registers
    if(!rst_n) begin
        s4_sign_a        <= 0;
        s4_sign_b        <= 0;
        s4_exp_diff      <= 0;
        s4_aligned_a     <= 0;
        s4_aligned_b     <= 0;
        s4_exp_r         <= 0;
        s4_product       <= 0;
        s4_sign_r        <= 0;
        s4_mant_sum_norm <= 0;
        s4_guard         <= 0;
        s4_round         <= 0;
        s4_sticky        <= 0;
    end else begin
        s4_sign_a        <= s3_sign_a;
        s4_sign_b        <= s3_sign_b;
        s4_exp_diff      <= s3_exp_diff;
        s4_aligned_a     <= s3_aligned_a;
        s4_aligned_b     <= s3_aligned_b;
        s4_exp_r         <= s3_exp_r_edited;
        s4_product       <= s3_product;
        s4_sign_r        <= s3_sign_r;
        s4_mant_sum_norm <= s3_mant_sum_norm;
        s4_guard         <= s3_guard;
        s4_round         <= s3_round;
        s4_sticky        <= s3_sticky;
        // s4_round_bit, s4_rounded_frac, s4_result set in comb below
    end
end

// Drive module output from stage 4
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        result <= 0;
    end else begin
        result <= s4_result;
    end
end


logic is_zero_a, is_zero_b;
logic [5:0] debug_exp_sum;  // 6 bits to avoid overflow
assign is_zero_a = (a[14:0] == 15'b0);  // Exp and mantissa both zero
assign is_zero_b = (b[14:0] == 15'b0);

// =========================
// Combinational datapath
// =========================
always_comb begin
    // === Align exponents ===
    s1_aligned_a = {s1_mant_a, 1'b0};  
    s1_aligned_b = {s1_mant_b, 1'b0};  
    s1_exp_b_E   =  s1_exp_b;
    s1_exp_a_E   =  s1_exp_a;

    // === Handle signs (assumes same sign only, no subtraction) ===
    s2_sign_r = s2_sign_a ^ s2_sign_b;


   
    //always_comb begin
    debug_exp_sum = ({1'b0, s2_exp_a} + {1'b0, s2_exp_b});
    //end
    s2_exp_change = (s2_exp_a + s2_exp_b) - 5'd15; // can move the +15 to a different stage
    // === Add aligned mantissas ===
    s2_product = s2_aligned_a * s2_aligned_b;

    if (s3_product[23]) begin
        s3_mant_sum_norm = s3_product[23:14]; 
        s3_guard         = s3_product[13];
        s3_round         = s3_product[12];
        s3_sticky        = |(s3_product[11:0]);
        s3_exp_r_edited  = s3_exp_r + 1'b1;
    end else begin
        s3_mant_sum_norm = s3_product[22:13];
        s3_guard         = s3_product[12];
        s3_round         = s3_product[11];
        s3_sticky        = |(s3_product[10:0]);
        s3_exp_r_edited  = s3_exp_r;
    end

    s4_round_bit = s4_guard & (s4_mant_sum_norm[0] | s4_round | s4_sticky);
    s4_rounded_frac = {1'b0, s4_mant_sum_norm} + s4_round_bit; 

    if (s4_rounded_frac[10]) begin
        s4_rounded_frac_edited = s4_rounded_frac[10:1];
        s4_exp_r_edited = s4_exp_r + 1;
    end else begin
        s4_rounded_frac_edited = s4_rounded_frac[9:0];   
        s4_exp_r_edited = s4_exp_r;
    end

    //s4_result = {s4_sign_r, s4_exp_r_edited, s4_rounded_frac_edited};

   
    if(is_zero_a || is_zero_b)begin
        s4_result = 0;
    end else begin
        s4_result = {s4_sign_r, s4_exp_r_edited, s4_rounded_frac_edited};
    end
end
endmodule
