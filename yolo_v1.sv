`timescale 1ns / 1ps

// Single conv2d layer. Int8 weights and activations, int32 accumulation.
// Bias is int32 (batch-norm folded in). Leaky relu + int8 clip applied at output.
//
// Buffer layout (all HWC / NHWC order):
//   i_buffer[y * W_in * C_in + x * C_in + c]
//   weight_buffer[weight_base + c_out * K * K * C_in + ky * K * C_in + kx * C_in + c_in]
//   output_buffer[y * W_out * C_out + x * C_out + c]
module conv_2d #(
    parameter K          = 3,
    parameter NUM_MACS   = 64,
    parameter IBUF_DEPTH = 256,
    parameter WBUF_DEPTH = 256,
    parameter OBUF_DEPTH = 256
) (
    input  logic clk,
    input  logic rst_n,
    input  logic signed [7:0]  weight_buffer [WBUF_DEPTH-1:0],
    input  logic signed [7:0]  i_buffer      [IBUF_DEPTH-1:0],
    input  logic signed [31:0] bias          [31:0],
    input  logic [31:0] C_out,
    input  logic [31:0] H_out,
    input  logic [31:0] W_out,
    input  logic [31:0] C_in,
    input  logic [31:0] stride,
    input  logic [31:0] W_in,
    input  logic [31:0] weight_base,
    input  logic start_process,
    output logic signed [7:0]  output_buffer [OBUF_DEPTH-1:0],
    output logic done
);
    // ----------------------------------------------------------------
    // Chunk / output-position bookkeeping
    // ----------------------------------------------------------------
    logic [31:0] total_macs, num_chunks;
    assign total_macs = K * K * C_in;
    assign num_chunks = (total_macs + NUM_MACS - 1) / NUM_MACS;

    logic [31:0] chunk;
    logic [15:0] y_out, x_out, c_out;

    logic is_last_chunk;
    logic is_c_out_max, is_x_out_max, is_y_out_max;

    assign is_last_chunk  = (chunk  == num_chunks - 1);
    assign is_c_out_max   = (c_out  == C_out - 1);
    assign is_x_out_max   = (x_out  == W_out - 1);
    assign is_y_out_max   = (y_out  == H_out - 1);

    logic clr_chunk,   inc_chunk;
    logic clr_c_out,   inc_c_out;
    logic clr_x_out,   inc_x_out;
    logic clr_y_out,   inc_y_out;
    logic accum_clr,   pixel_done, output_write;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) chunk <= '0;
        else begin
            if (clr_chunk)      chunk <= 0;
            else if (inc_chunk) chunk <= chunk + 1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out <= '0; x_out <= '0; c_out <= '0;
        end else begin
            if (clr_y_out)      y_out <= 0;
            else if (inc_y_out) y_out <= y_out + 1;

            if (clr_x_out)      x_out <= 0;
            else if (inc_x_out) x_out <= x_out + 1;

            if (clr_c_out)      c_out <= 0;
            else if (inc_c_out) c_out <= c_out + 1;
        end
    end

    // ----------------------------------------------------------------
    // Address generation (chunked, 64 addresses per cycle)
    // ----------------------------------------------------------------
    function automatic logic [31:0] calc_input_addr(
        input logic [31:0] yo, xo, ky, kx, ci, str, win, ncin
    );
        logic [31:0] iy, ix;
        iy = yo * str + ky;
        ix = xo * str + kx;
        calc_input_addr = iy * win * ncin + ix * ncin + ci;
    endfunction

    function automatic logic [31:0] calc_weight_addr(
        input logic [31:0] co, ky, kx, ci, kern, ncin, base
    );
        logic [31:0] idx;
        idx = co * kern * kern * ncin + ky * kern * ncin + kx * ncin + ci;
        calc_weight_addr = base + idx;
    endfunction

    logic [31:0] flat_idx   [0:63];
    logic [31:0] m_c_idx    [0:63];
    logic [31:0] m_kx_idx   [0:63];
    logic [31:0] m_ky_idx   [0:63];
    logic [31:0] input_addrs  [0:63];
    logic [31:0] weight_addrs [0:63];

    always_comb begin
        for (int m = 0; m < NUM_MACS; m++) begin
            input_addrs[m]  = '0;
            weight_addrs[m] = '0;
            flat_idx[m]  = chunk * NUM_MACS + m;
            m_c_idx[m]   = '0;
            m_kx_idx[m]  = '0;
            m_ky_idx[m]  = '0;
            if (flat_idx[m] < total_macs) begin
                m_c_idx[m]  = flat_idx[m] % C_in;
                m_kx_idx[m] = (flat_idx[m] / C_in) % K;
                m_ky_idx[m] = (flat_idx[m] / C_in) / K;
                input_addrs[m]  = calc_input_addr(
                    32'(y_out), 32'(x_out),
                    m_ky_idx[m], m_kx_idx[m], m_c_idx[m],
                    stride, W_in, C_in);
                weight_addrs[m] = calc_weight_addr(
                    32'(c_out),
                    m_ky_idx[m], m_kx_idx[m], m_c_idx[m],
                    K, C_in, weight_base);
            end
        end
    end

    // ----------------------------------------------------------------
    // Registered buffer lookup (1-cycle latency)
    // ----------------------------------------------------------------
    localparam WADDR_BITS = $clog2(WBUF_DEPTH);
    localparam IADDR_BITS = $clog2(IBUF_DEPTH);

    logic signed [7:0] weight_vals [0:63];
    logic signed [7:0] input_vals  [0:63];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_MACS; i++) begin
                weight_vals[i] <= '0;
                input_vals[i]  <= '0;
            end
        end else begin
            for (int i = 0; i < NUM_MACS; i++) begin
                weight_vals[i] <= weight_buffer[weight_addrs[i][WADDR_BITS-1:0]];
                input_vals[i]  <= i_buffer[input_addrs[i][IADDR_BITS-1:0]];
            end
        end
    end

    // ----------------------------------------------------------------
    // MAC array + accumulator
    // ----------------------------------------------------------------
    logic signed [31:0] conv_res;
    logic conv_in_valid;

    assign conv_in_valid = pixel_done;

    conv_iteration conv_iter (
        .clk(clk), .rst_n(rst_n),
        .in_valid(conv_in_valid),
        .accum_clr(accum_clr),
        .weights(weight_vals),
        .ifmap(input_vals),
        .result(conv_res),
        .out_valid()   // unused; FSM owns output timing
    );

    // ----------------------------------------------------------------
    // Bias + leaky relu + int8 clip
    // ----------------------------------------------------------------
    logic signed [31:0] biased, relu_out;
    logic signed [7:0]  clipped;

    assign biased  = conv_res + bias[c_out[4:0]];

    leaky_relu lrelu (.x(biased), .y(relu_out));

    assign clipped = (relu_out > 32'sd127)  ?  8'sd127 :
                     (relu_out < -32'sd128) ? -8'sd128 :
                      relu_out[7:0];

    // ----------------------------------------------------------------
    // Output buffer write (driven by FSM ADVANCE state)
    // ----------------------------------------------------------------
    localparam OADDR_BITS = $clog2(OBUF_DEPTH);
    logic [31:0] output_addr;
    assign output_addr = 32'(y_out) * W_out * C_out + 32'(x_out) * C_out + 32'(c_out);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < OBUF_DEPTH; i++)
                output_buffer[i] <= '0;
        end else if (output_write) begin
            output_buffer[output_addr[OADDR_BITS-1:0]] <= clipped;
        end
    end

    // ----------------------------------------------------------------
    // FSM
    // ----------------------------------------------------------------
    conv_chunk_fsm chunk_fsm (
        .clk(clk), .rst_n(rst_n),
        .start_process(start_process),
        .is_last_chunk(is_last_chunk),
        .is_c_out_max(is_c_out_max),
        .is_x_out_max(is_x_out_max),
        .is_y_out_max(is_y_out_max),
        .clr_chunk(clr_chunk),   .inc_chunk(inc_chunk),
        .clr_c_out(clr_c_out),  .inc_c_out(inc_c_out),
        .clr_x_out(clr_x_out),  .inc_x_out(inc_x_out),
        .clr_y_out(clr_y_out),  .inc_y_out(inc_y_out),
        .accum_clr(accum_clr),
        .pixel_done(pixel_done),
        .output_write(output_write),
        .done(done)
    );

endmodule


// FSM: for each output pixel (y, x, c_out), iterates through chunks then
// writes output. ADVANCE reads the accumulator, clears it, and advances
// to the next output position — all in one cycle.
module conv_chunk_fsm (
    input  logic clk,
    input  logic rst_n,
    input  logic start_process,
    input  logic is_last_chunk,
    input  logic is_c_out_max,
    input  logic is_x_out_max,
    input  logic is_y_out_max,
    output logic clr_chunk,    inc_chunk,
    output logic clr_c_out,    inc_c_out,
    output logic clr_x_out,    inc_x_out,
    output logic clr_y_out,    inc_y_out,
    output logic accum_clr,
    output logic pixel_done,
    output logic output_write,
    output logic done
);
    typedef enum logic [1:0] {
        IDLE       = 2'd0,
        CHUNK_LOAD = 2'd1,
        ADVANCE    = 2'd2,
        ALL_DONE   = 2'd3
    } state_t;

    state_t current_state, next_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) current_state <= IDLE;
        else        current_state <= next_state;
    end

    always_comb begin
        // defaults
        clr_chunk  = 1'b0; inc_chunk  = 1'b0;
        clr_c_out  = 1'b0; inc_c_out  = 1'b0;
        clr_x_out  = 1'b0; inc_x_out  = 1'b0;
        clr_y_out  = 1'b0; inc_y_out  = 1'b0;
        accum_clr  = 1'b0;
        pixel_done = 1'b0;
        output_write = 1'b0;
        done       = 1'b0;
        next_state = current_state;

        case (current_state)
            IDLE: begin
                if (start_process) begin
                    clr_c_out = 1'b1; clr_x_out = 1'b1; clr_y_out = 1'b1;
                    clr_chunk = 1'b1;
                    accum_clr = 1'b1;
                    next_state = CHUNK_LOAD;
                end
            end

            // Feed one chunk per cycle to the MAC array.
            // Because the buffer lookup has 1-cycle latency, the data loaded
            // into weight_vals/input_vals this cycle reflects the addresses
            // that were computed last cycle (i.e., the current chunk index).
            // out_valid = in_valid, so the accumulator captures each chunk
            // result in the same cycle pixel_done is high.
            CHUNK_LOAD: begin
                pixel_done = 1'b1;
                if (is_last_chunk) begin
                    next_state = ADVANCE;
                end else begin
                    inc_chunk  = 1'b1;
                    next_state = CHUNK_LOAD;
                end
            end

            // Last chunk result is now in the accumulator (captured at the
            // clock edge that brought us here). Write output, clear, advance.
            ADVANCE: begin
                output_write = 1'b1;
                clr_chunk    = 1'b1;
                accum_clr    = 1'b1;

                if (!is_c_out_max) begin
                    inc_c_out  = 1'b1;
                    next_state = CHUNK_LOAD;
                end else if (!is_x_out_max) begin
                    clr_c_out  = 1'b1; inc_x_out = 1'b1;
                    next_state = CHUNK_LOAD;
                end else if (!is_y_out_max) begin
                    clr_c_out  = 1'b1; clr_x_out = 1'b1; inc_y_out = 1'b1;
                    next_state = CHUNK_LOAD;
                end else begin
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
