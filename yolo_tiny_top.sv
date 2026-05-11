`timescale 1ns / 1ps

// Pure FSM sequencer — no buffers, no submodule instances.
// yolo_axi_top owns memory and compute; this module only drives sequencing.
//
// mem_stall: when high, FSM holds in current state and all start pulses are suppressed.
// The wrapper asserts this while loading BRAM or storing output to DDR.
module yolo_tiny_top (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic mem_stall,   // hold FSM while wrapper loads/stores DDR

    // Submodule done signals (from wrapper, which manages the actual compute)
    input  logic conv_done,
    input  logic pool_done,
    input  logic up_done,
    input  logic cat_done,

    // Layer start pulses (to wrapper — one cycle, suppressed when mem_stall=1)
    output logic conv_start,
    output logic pool_start,
    output logic up_start,
    output logic cat_start,

    // Config — valid whenever the FSM is in a _START or _WAIT state for that layer
    output logic [31:0] cfg_H_in,  cfg_W_in,  cfg_C_in,
    output logic [31:0] cfg_H_out, cfg_W_out, cfg_C_out,
    output logic [31:0] cfg_stride,
    output logic [31:0] cfg_weight_base,
    output logic [31:0] cfg_bias_base,
    output logic [31:0] cfg_K,        // conv kernel size (1 or 3)

    // Ping-pong state — tells wrapper which DDR side is currently the input
    output logic        ping,

    // One-cycle notification pulses (not gated by mem_stall)
    output logic        save_route,    // SAVE_ROUTE state — wrapper records route DDR addr
    output logic        save_route8,   // conv_done in CONV8_WAIT — wrapper records route8 DDR addr
    output logic        save_det1,     // YOLO1 — wrapper copies output to det1 DDR
    output logic        save_det2,     // YOLO2 — wrapper copies output to det2 DDR
    output logic        route_restore, // ROUTE_RESTORE — wrapper switches ibram source to route8

    output logic [5:0]  fsm_state,
    output logic        done
);

    // ----------------------------------------------------------------
    // State enum
    // ----------------------------------------------------------------
    typedef enum logic [5:0] {
        IDLE,
        CONV1_START,  CONV1_WAIT,
        POOL1_START,  POOL1_WAIT,
        CONV2_START,  CONV2_WAIT,
        POOL2_START,  POOL2_WAIT,
        CONV3_START,  CONV3_WAIT,
        POOL3_START,  POOL3_WAIT,
        CONV4_START,  CONV4_WAIT,
        POOL4_START,  POOL4_WAIT,
        CONV5_START,  CONV5_WAIT,
        SAVE_ROUTE,
        POOL5_START,  POOL5_WAIT,
        CONV6_START,  CONV6_WAIT,
        POOL6_START,  POOL6_WAIT,
        CONV7_START,  CONV7_WAIT,
        CONV8_START,  CONV8_WAIT,
        CONV9_START,  CONV9_WAIT,
        YOLO1,
        ROUTE_RESTORE,
        CONV10_START, CONV10_WAIT,
        UPSAMP_START, UPSAMP_WAIT,
        CONCAT_START, CONCAT_WAIT,
        CONV11_START, CONV11_WAIT,
        YOLO2,
        ALL_DONE
    } state_t;

    state_t current_state, next_state;
    assign fsm_state = current_state;

    // ----------------------------------------------------------------
    // State register
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) current_state <= IDLE;
        else        current_state <= next_state;
    end

    // ----------------------------------------------------------------
    // Ping tracking — flips after every layer completes
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ping <= 1'b0;
        end else begin
            if ((conv_done && (current_state == CONV1_WAIT  || current_state == CONV2_WAIT  ||
                               current_state == CONV3_WAIT  || current_state == CONV4_WAIT  ||
                               current_state == CONV5_WAIT  || current_state == CONV6_WAIT  ||
                               current_state == CONV7_WAIT  || current_state == CONV8_WAIT  ||
                               current_state == CONV9_WAIT  || current_state == CONV10_WAIT ||
                               current_state == CONV11_WAIT)) ||
                (pool_done && (current_state == POOL1_WAIT  || current_state == POOL2_WAIT  ||
                               current_state == POOL3_WAIT  || current_state == POOL4_WAIT  ||
                               current_state == POOL5_WAIT  || current_state == POOL6_WAIT)) ||
                (cat_done  &&  current_state == CONCAT_WAIT))
                ping <= ~ping;
        end
    end

    // ----------------------------------------------------------------
    // Notification pulses (combinational)
    // ----------------------------------------------------------------
    assign save_route    = (current_state == SAVE_ROUTE);
    assign save_route8   = (current_state == CONV8_WAIT  && conv_done);
    assign save_det1     = (current_state == YOLO1);
    assign save_det2     = (current_state == YOLO2);
    assign route_restore = (current_state == ROUTE_RESTORE);

    // ----------------------------------------------------------------
    // Combinational FSM — all transitions gated by mem_stall
    // ----------------------------------------------------------------
    always_comb begin
        next_state       = current_state;
        conv_start       = 1'b0;
        pool_start       = 1'b0;
        up_start         = 1'b0;
        cat_start        = 1'b0;
        done             = 1'b0;
        cfg_weight_base  = 32'd0;
        cfg_bias_base    = 32'd0;
        cfg_K            = 32'd3;
        cfg_H_in  = 32'd0; cfg_W_in  = 32'd0; cfg_C_in  = 32'd0;
        cfg_H_out = 32'd0; cfg_W_out = 32'd0; cfg_C_out = 32'd0;
        cfg_stride = 32'd1;

        case (current_state)
            IDLE: begin
                if (start && !mem_stall) next_state = CONV1_START;
            end

            // --- Conv1: 416x416x3 → 416x416x16, K=3 ---
            CONV1_START: begin
                cfg_weight_base = 32'd0; cfg_bias_base = 32'd0; cfg_K = 32'd3;
                cfg_H_in = 416; cfg_W_in = 416; cfg_C_in = 3;
                cfg_H_out = 416; cfg_W_out = 416; cfg_C_out = 16;
                if (!mem_stall) begin next_state = CONV1_WAIT; conv_start = 1; end
            end
            CONV1_WAIT: begin
                cfg_H_in = 416; cfg_W_in = 416; cfg_C_in = 3;
                cfg_H_out = 416; cfg_W_out = 416; cfg_C_out = 16;
                if (!mem_stall && conv_done) next_state = POOL1_START;
            end

            // --- Pool1: 416x416x16 → 208x208x16, stride=2 ---
            POOL1_START: begin
                cfg_stride = 2; cfg_H_in = 416; cfg_W_in = 416; cfg_C_in = 16;
                cfg_H_out = 208; cfg_W_out = 208; cfg_C_out = 16;
                if (!mem_stall) begin next_state = POOL1_WAIT; pool_start = 1; end
            end
            POOL1_WAIT: begin
                cfg_H_in = 416; cfg_W_in = 416; cfg_C_in = 16;
                cfg_H_out = 208; cfg_W_out = 208; cfg_C_out = 16;
                if (!mem_stall && pool_done) next_state = CONV2_START;
            end

            // --- Conv2: 208x208x16 → 208x208x32, K=3 ---
            CONV2_START: begin
                cfg_weight_base = 32'd432; cfg_bias_base = 32'd16; cfg_K = 32'd3;
                cfg_H_in = 208; cfg_W_in = 208; cfg_C_in = 16;
                cfg_H_out = 208; cfg_W_out = 208; cfg_C_out = 32;
                if (!mem_stall) begin next_state = CONV2_WAIT; conv_start = 1; end
            end
            CONV2_WAIT: begin
                cfg_H_in = 208; cfg_W_in = 208; cfg_C_in = 16;
                cfg_H_out = 208; cfg_W_out = 208; cfg_C_out = 32;
                if (!mem_stall && conv_done) next_state = POOL2_START;
            end

            // --- Pool2: 208x208x32 → 104x104x32, stride=2 ---
            POOL2_START: begin
                cfg_stride = 2; cfg_H_in = 208; cfg_W_in = 208; cfg_C_in = 32;
                cfg_H_out = 104; cfg_W_out = 104; cfg_C_out = 32;
                if (!mem_stall) begin next_state = POOL2_WAIT; pool_start = 1; end
            end
            POOL2_WAIT: begin
                cfg_H_in = 208; cfg_W_in = 208; cfg_C_in = 32;
                cfg_H_out = 104; cfg_W_out = 104; cfg_C_out = 32;
                if (!mem_stall && pool_done) next_state = CONV3_START;
            end

            // --- Conv3: 104x104x32 → 104x104x64, K=3 ---
            CONV3_START: begin
                cfg_weight_base = 32'd5040; cfg_bias_base = 32'd48; cfg_K = 32'd3;
                cfg_H_in = 104; cfg_W_in = 104; cfg_C_in = 32;
                cfg_H_out = 104; cfg_W_out = 104; cfg_C_out = 64;
                if (!mem_stall) begin next_state = CONV3_WAIT; conv_start = 1; end
            end
            CONV3_WAIT: begin
                cfg_H_in = 104; cfg_W_in = 104; cfg_C_in = 32;
                cfg_H_out = 104; cfg_W_out = 104; cfg_C_out = 64;
                if (!mem_stall && conv_done) next_state = POOL3_START;
            end

            // --- Pool3: 104x104x64 → 52x52x64, stride=2 ---
            POOL3_START: begin
                cfg_stride = 2; cfg_H_in = 104; cfg_W_in = 104; cfg_C_in = 64;
                cfg_H_out = 52; cfg_W_out = 52; cfg_C_out = 64;
                if (!mem_stall) begin next_state = POOL3_WAIT; pool_start = 1; end
            end
            POOL3_WAIT: begin
                cfg_H_in = 104; cfg_W_in = 104; cfg_C_in = 64;
                cfg_H_out = 52; cfg_W_out = 52; cfg_C_out = 64;
                if (!mem_stall && pool_done) next_state = CONV4_START;
            end

            // --- Conv4: 52x52x64 → 52x52x128, K=3 ---
            CONV4_START: begin
                cfg_weight_base = 32'd23472; cfg_bias_base = 32'd112; cfg_K = 32'd3;
                cfg_H_in = 52; cfg_W_in = 52; cfg_C_in = 64;
                cfg_H_out = 52; cfg_W_out = 52; cfg_C_out = 128;
                if (!mem_stall) begin next_state = CONV4_WAIT; conv_start = 1; end
            end
            CONV4_WAIT: begin
                cfg_H_in = 52; cfg_W_in = 52; cfg_C_in = 64;
                cfg_H_out = 52; cfg_W_out = 52; cfg_C_out = 128;
                if (!mem_stall && conv_done) next_state = POOL4_START;
            end

            // --- Pool4: 52x52x128 → 26x26x128, stride=2 ---
            POOL4_START: begin
                cfg_stride = 2; cfg_H_in = 52; cfg_W_in = 52; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 128;
                if (!mem_stall) begin next_state = POOL4_WAIT; pool_start = 1; end
            end
            POOL4_WAIT: begin
                cfg_H_in = 52; cfg_W_in = 52; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 128;
                if (!mem_stall && pool_done) next_state = CONV5_START;
            end

            // --- Conv5: 26x26x128 → 26x26x256, K=3 ---
            CONV5_START: begin
                cfg_weight_base = 32'd97200; cfg_bias_base = 32'd240; cfg_K = 32'd3;
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 256;
                if (!mem_stall) begin next_state = CONV5_WAIT; conv_start = 1; end
            end
            CONV5_WAIT: begin
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 256;
                if (!mem_stall && conv_done) next_state = SAVE_ROUTE;
            end

            // save_route pulse fires; wrapper records current output DDR as route_buf
            SAVE_ROUTE: begin
                if (!mem_stall) next_state = POOL5_START;
            end

            // --- Pool5: 26x26x256 → 13x13x256, stride=2 ---
            POOL5_START: begin
                cfg_stride = 2; cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 256;
                if (!mem_stall) begin next_state = POOL5_WAIT; pool_start = 1; end
            end
            POOL5_WAIT: begin
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 256;
                if (!mem_stall && pool_done) next_state = CONV6_START;
            end

            // --- Conv6: 13x13x256 → 13x13x512, K=3 ---
            CONV6_START: begin
                cfg_weight_base = 32'd392112; cfg_bias_base = 32'd496; cfg_K = 32'd3;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                if (!mem_stall) begin next_state = CONV6_WAIT; conv_start = 1; end
            end
            CONV6_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                if (!mem_stall && conv_done) next_state = POOL6_START;
            end

            // --- Pool6: 13x13x512 → 13x13x512, stride=1 ---
            POOL6_START: begin
                cfg_stride = 1; cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 512;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                if (!mem_stall) begin next_state = POOL6_WAIT; pool_start = 1; end
            end
            POOL6_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 512;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                if (!mem_stall && pool_done) next_state = CONV7_START;
            end

            // --- Conv7: 13x13x512 → 13x13x1024, K=3 ---
            CONV7_START: begin
                cfg_weight_base = 32'd1571760; cfg_bias_base = 32'd1008; cfg_K = 32'd3;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 512;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 1024;
                if (!mem_stall) begin next_state = CONV7_WAIT; conv_start = 1; end
            end
            CONV7_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 512;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 1024;
                if (!mem_stall && conv_done) next_state = CONV8_START;
            end

            // --- Conv8: 13x13x1024 → 13x13x256, K=1  (route8 save) ---
            CONV8_START: begin
                cfg_weight_base = 32'd6290352; cfg_bias_base = 32'd2032; cfg_K = 32'd1;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 1024;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 256;
                if (!mem_stall) begin next_state = CONV8_WAIT; conv_start = 1; end
            end
            CONV8_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 1024;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 256;
                // save_route8 fires combinationally on conv_done — wrapper records route8 DDR addr
                if (!mem_stall && conv_done) next_state = CONV9_START;
            end

            // --- Conv9: 13x13x256 → 13x13x512, K=3 ---
            CONV9_START: begin
                cfg_weight_base = 32'd6552496; cfg_bias_base = 32'd2288; cfg_K = 32'd3;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                if (!mem_stall) begin next_state = CONV9_WAIT; conv_start = 1; end
            end
            CONV9_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                if (!mem_stall && conv_done) next_state = YOLO1;
            end

            // save_det1 fires; wrapper copies current output DDR → det1_addr, stalls until done
            YOLO1: begin
                if (!mem_stall) next_state = ROUTE_RESTORE;
            end

            // route_restore fires; wrapper switches ibram source to route8_addr for Conv10
            ROUTE_RESTORE: begin
                if (!mem_stall) next_state = CONV10_START;
            end

            // --- Conv10: 13x13x256 → 13x13x128, K=1 ---
            CONV10_START: begin
                cfg_weight_base = 32'd7732144; cfg_bias_base = 32'd2800; cfg_K = 32'd1;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 128;
                if (!mem_stall) begin next_state = CONV10_WAIT; conv_start = 1; end
            end
            CONV10_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 128;
                if (!mem_stall && conv_done) next_state = UPSAMP_START;
            end

            // --- Upsample: 13x13x128 → 26x26x128 ---
            UPSAMP_START: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 128;
                if (!mem_stall) begin next_state = UPSAMP_WAIT; up_start = 1; end
            end
            UPSAMP_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 128;
                if (!mem_stall && up_done) next_state = CONCAT_START;
            end

            // --- Concat: 26x26x128 ++ 26x26x256 → 26x26x384 ---
            CONCAT_START: begin
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 384;
                if (!mem_stall) begin next_state = CONCAT_WAIT; cat_start = 1; end
            end
            CONCAT_WAIT: begin
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 384;
                if (!mem_stall && cat_done) next_state = CONV11_START;
            end

            // --- Conv11: 26x26x384 → 26x26x256, K=3 ---
            CONV11_START: begin
                cfg_weight_base = 32'd7764912; cfg_bias_base = 32'd2928; cfg_K = 32'd3;
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 384;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 256;
                if (!mem_stall) begin next_state = CONV11_WAIT; conv_start = 1; end
            end
            CONV11_WAIT: begin
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 384;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 256;
                if (!mem_stall && conv_done) next_state = YOLO2;
            end

            // save_det2 fires; wrapper copies current output DDR → det2_addr, stalls until done
            YOLO2: begin
                if (!mem_stall) next_state = ALL_DONE;
            end

            ALL_DONE: begin
                done = 1'b1;
                if (!mem_stall) next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

endmodule
