`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/27/2025 10:00:32 PM
// Design Name: 
// Module Name: yolo_v1
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


module conv_2d #(
    parameter K = 3,            // kernel size (1 or 3)
    parameter NUM_MACS = 64     // number of parallel MACs
) (
    input logic clk,
    input logic [15:0] weight_buffer [223:0],
    input logic [15:0] i_buffer [223:0],
    input logic [15:0] bias [31:0],
    input logic [31:0] C_out,
    input logic [31:0] H_out,
    input logic [31:0] W_out,
    input logic [31:0] C_in,
    input logic [31:0] stride,
    input logic [31:0] W_in,
    input logic [31:0] weight_base,
    input logic start_process,
    input logic rst_n);

    // Total MACs per output pixel = K*K*C_in
    // Number of chunks = ceil(K*K*C_in / NUM_MACS)
    logic [31:0] total_macs;
    logic [31:0] num_chunks;
    assign total_macs = K * K * C_in;
    assign num_chunks = (total_macs + NUM_MACS - 1) / NUM_MACS;

    // Output iteration counters
    logic clr_c_out, clr_x_out, clr_y_out;
    logic inc_c_out, inc_x_out, inc_y_out;
    logic is_c_out_max, is_x_out_max, is_y_out_max;

    // Chunk counter
    logic [31:0] chunk;
    logic clr_chunk, inc_chunk;
    logic is_last_chunk;

    logic done, accum_clr;
    logic pixel_done;
    logic stall;

    assign stall = 0;

    logic [15:0] output_buffer [223:0];
    logic [15:0] y_out, x_out, c_out;

    assign is_c_out_max = (c_out == C_out - 1);
    assign is_x_out_max = (x_out == W_out - 1);
    assign is_y_out_max = (y_out == H_out - 1);
    assign is_last_chunk = (chunk == num_chunks - 1);

    // Output iteration counters
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out <= '0;
            x_out <= '0;
            c_out <= '0;
        end else begin
            if (clr_x_out)          x_out <= 0;
            else if (inc_x_out)     x_out <= x_out + 1;

            if (clr_y_out)          y_out <= 0;
            else if (inc_y_out)     y_out <= y_out + 1;

            if (clr_c_out)          c_out <= 0;
            else if (inc_c_out)     c_out <= c_out + 1;
        end
    end

    // Chunk counter
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            chunk <= '0;
        end else begin
            if (clr_chunk)          chunk <= 0;
            else if (inc_chunk)     chunk <= chunk + 1;
        end
    end

    // Address computation functions
    function automatic logic [31:0] calc_input_addr(
        input logic [31:0] y_out,
        input logic [31:0] x_out,
        input logic [31:0] ky,
        input logic [31:0] kx,
        input logic [31:0] c_in,
        input logic [31:0] stride,
        input logic [31:0] input_width,
        input logic [31:0] num_channels
    );
        logic [31:0] input_y, input_x;
        input_y = (y_out * stride) + ky;
        input_x = (x_out * stride) + kx;
        calc_input_addr = (input_y * input_width * num_channels) +
                          (input_x * num_channels) +
                          c_in;
    endfunction

    function automatic logic [31:0] calc_weight_addr(
        input logic [31:0] c_out,
        input logic [31:0] ky,
        input logic [31:0] kx,
        input logic [31:0] c_in,
        input logic [31:0] kern_size,
        input logic [31:0] num_in_channels,
        input logic [31:0] base_addr
    );
        logic [31:0] weight_index;
        weight_index = (c_out * kern_size * kern_size * num_in_channels) +
                       (ky * kern_size * num_in_channels) +
                       (kx * num_in_channels) +
                       c_in;
        calc_weight_addr = base_addr + (weight_index * 2);
    endfunction

    // Chunked address generation — 64 addresses per chunk
    logic [31:0] input_addrs [63:0];
    logic [15:0] input_vals  [63:0];
    logic [31:0] weight_addrs [63:0];
    logic [15:0] weight_vals  [63:0];
    logic out_valid;

    integer idx;
    always_comb begin
        // Default: zero all addresses (for padding unused MACs)
        for (int m = 0; m < NUM_MACS; m++) begin
            input_addrs[m] = '0;
            weight_addrs[m] = '0;
        end

        for (int m = 0; m < NUM_MACS; m++) begin
            idx = (chunk * NUM_MACS) + m;
            if (idx < total_macs) begin
                // Convert flat index back to (ky, kx, c_in)
                automatic logic [31:0] c_idx, kx_idx, ky_idx, rem;
                c_idx  = idx % C_in;
                rem    = idx / C_in;
                kx_idx = rem % K;
                ky_idx = rem / K;

                input_addrs[m] = calc_input_addr(
                    y_out, x_out, ky_idx, kx_idx, c_idx,
                    stride, W_in, C_in
                );
                weight_addrs[m] = calc_weight_addr(
                    c_out, ky_idx, kx_idx, c_idx,
                    K, C_in, weight_base
                );
            end
        end
    end

    // Register the buffer lookups
    integer i;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NUM_MACS; i++) begin
                weight_vals[i] <= '0;
                input_vals[i]  <= '0;
            end
        end else begin
            for (i = 0; i < NUM_MACS; i++) begin
                weight_vals[i] <= weight_buffer[weight_addrs[i][7:0]];
                input_vals[i]  <= i_buffer[input_addrs[i][7:0]];
            end
        end
    end

    // Datapath: MAC array + accumulator
    logic [15:0] conv_res;
    logic activation_valid;
    logic conv_in_valid;

    assign conv_in_valid = pixel_done;

    conv_iteration conv_iter (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(conv_in_valid),
        .weights(weight_vals),
        .accum_clr(accum_clr),
        .bias(bias[c_out[4:0]]),
        .ifmap(input_vals),
        .result(conv_res),
        .out_valid(activation_valid)
    );

    // Output address and write
    logic [31:0] output_addr;
    logic [31:0] w_out;

    assign w_out = (W_in - K) / stride + 1;
    assign output_addr = (y_out * w_out * C_out) + (x_out * C_out) + c_out;

    always_ff @(posedge clk) begin
        if (activation_valid) begin
            output_buffer[output_addr] <= conv_res;
        end
    end

    // FSM instantiations
    conv_chunk_fsm chunk_fsm (
        .clk(clk),
        .rst_n(rst_n),
        .start_process(start_process),
        .stall(stall),
        .is_last_chunk(is_last_chunk),
        .is_c_out_max(is_c_out_max),
        .is_x_out_max(is_x_out_max),
        .is_y_out_max(is_y_out_max),
        .clr_chunk(clr_chunk),
        .inc_chunk(inc_chunk),
        .clr_c_out(clr_c_out),
        .clr_x_out(clr_x_out),
        .clr_y_out(clr_y_out),
        .inc_c_out(inc_c_out),
        .inc_x_out(inc_x_out),
        .inc_y_out(inc_y_out),
        .accum_clr(accum_clr),
        .pixel_done(pixel_done),
        .done(done)
    );

endmodule




// Unified FSM with inner chunk loop
// For each (y, x, c_out): iterates through chunks, then advances output position
module conv_chunk_fsm(
    input  logic clk,
    input  logic rst_n,
    input  logic start_process,
    input  logic stall,
    input  logic is_last_chunk,
    input  logic is_c_out_max,
    input  logic is_x_out_max,
    input  logic is_y_out_max,
    output logic clr_chunk,
    output logic inc_chunk,
    output logic clr_c_out,
    output logic clr_x_out,
    output logic clr_y_out,
    output logic inc_c_out,
    output logic inc_x_out,
    output logic inc_y_out,
    output logic accum_clr,
    output logic pixel_done,
    output logic done
);

    // IDLE:       waiting for start
    // CHUNK_LOAD: feed current chunk to MAC array (pixel_done=1 triggers MAC)
    // CHUNK_WAIT: wait for MAC pipeline result (if needed)
    // ADVANCE:    all chunks done for this (y,x,c_out), advance output position
    // DONE:       all output positions complete
    enum { IDLE, CHUNK_LOAD, ADVANCE, ALL_DONE } current_state, next_state;

    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) current_state <= IDLE;
        else        current_state <= next_state;
    end

    always_comb begin
        // Defaults
        clr_chunk = 1'b0; inc_chunk = 1'b0;
        clr_c_out = 1'b0; clr_x_out = 1'b0; clr_y_out = 1'b0;
        inc_c_out = 1'b0; inc_x_out = 1'b0; inc_y_out = 1'b0;
        accum_clr = 1'b0;
        pixel_done = 1'b0;
        done = 1'b0;
        next_state = current_state;

        case (current_state)
            IDLE: begin
                if (start_process) begin
                    // Clear all counters for fresh start
                    clr_c_out = 1'b1;
                    clr_x_out = 1'b1;
                    clr_y_out = 1'b1;
                    clr_chunk = 1'b1;
                    accum_clr = 1'b1;
                    next_state = CHUNK_LOAD;
                end
            end

            CHUNK_LOAD: begin
                if (!stall) begin
                    pixel_done = 1'b1;  // triggers MAC for current chunk

                    if (is_last_chunk) begin
                        // All chunks done for this output position
                        next_state = ADVANCE;
                    end else begin
                        // More chunks to process
                        inc_chunk = 1'b1;
                        next_state = CHUNK_LOAD;
                    end
                end
            end

            ADVANCE: begin
                // Result is ready, advance to next (c_out, x_out, y_out)
                clr_chunk = 1'b1;
                accum_clr = 1'b1;

                if (!is_c_out_max) begin
                    inc_c_out = 1'b1;
                    next_state = CHUNK_LOAD;
                end else if (!is_x_out_max) begin
                    clr_c_out = 1'b1;
                    inc_x_out = 1'b1;
                    next_state = CHUNK_LOAD;
                end else if (!is_y_out_max) begin
                    clr_c_out = 1'b1;
                    clr_x_out = 1'b1;
                    inc_y_out = 1'b1;
                    next_state = CHUNK_LOAD;
                end else begin
                    // All done
                    next_state = ALL_DONE;
                end
            end

            ALL_DONE: begin
                done = 1'b1;
                next_state = IDLE;
            end
        endcase
    end

endmodule



// module conv_iteration_in(
//   input  logic clk,
//   input  logic rst_n,
//   input  logic ready_iter,     // start request
//   input  logic accept_step,    // advance gate (e.g., valid&&ready from datapath)
//   input  logic is_ky_max,
//   input  logic is_kx_max,
//   input  logic is_cin_max,
//   output logic clr_ky,
//   output logic clr_kx,
//   output logic clr_cin,
//   output logic inc_kx,
//   output logic inc_ky,
//   output logic inc_cin,
//   output logic pixel_done,
//   output logic done
// );

//   enum { IDLE, READ_PATCH_CIN, READ_PATCH_KX, READ_PATCH_KY, VALID } current_state, next_state;


//   always_ff @(posedge clk, negedge rst_n) begin
//     if (!rst_n) current_state <= IDLE;
//     else        current_state <= next_state;
//   end

//   always_comb begin
//     // defaults every cycle
    
//     inc_kx = 1'b0; inc_ky = 1'b0; inc_cin = 1'b0;
//     clr_kx = 1'b0; clr_ky = 1'b0; clr_cin = 1'b0;
//     done  = 1'b0;

//     case (current_state)
//       IDLE: begin
//         if (ready_iter) begin
//           // ensure a clean start of scan
//           clr_cin = 1'b1;
//           clr_kx = 1'b1;
//           next_state = READ_PATCH_CIN;
//         end
//         pixel_done = 0;
//       end

//       READ_PATCH_CIN: begin
//         if (accept_step) begin
//           if (!is_cin_max) begin
//             inc_cin = 1'b1;                 
//             next_state = READ_PATCH_CIN;
//           end else begin
//             clr_cin = 1'b1; inc_kx = 1'b1;   
//             next_state = READ_PATCH_KX;
//           end
//         end
//         pixel_done = 0;
//       end

//       READ_PATCH_KX: begin
//         if (accept_step) begin
//           if (is_kx_max) begin
//             clr_kx = 1'b1; inc_ky = 1'b1;  
//             next_state = READ_PATCH_KY;
//           end else begin
//             inc_cin = 1'b1;
//             next_state = READ_PATCH_CIN;
//           end
//         end
//         pixel_done = 0;
//       end

//       READ_PATCH_KY: begin
//         if (accept_step) begin
//           if (is_ky_max) begin
//             clr_ky = 1'b1;                
//             next_state = VALID;
//           end else begin
//             inc_cin = 1;
//             next_state = READ_PATCH_CIN;
//             pixel_done = 0;
//           end
//         end
//       end

//       VALID: begin
//         done = 1'b1;                       
//         next_state = IDLE;
//         pixel_done = 1;
//       end
//     endcase
//   end
  
// endmodule


