`timescale 1ns / 1ps

module yolo_tiny_top #(
    parameter WEIGHT_DEPTH  = 8649648,   // all layer weights concatenated
    parameter BIAS_DEPTH    = 4096,       // enough for all C_out across all layers
    parameter IMG_DEPTH     = 519168,     // 416x416x3
    parameter BUF_DEPTH     = 2752512,    // 416x416x16 — largest intermediate
    parameter ROUTE_DEPTH   = 172032,     // 26x26x256 — saved feature map
    parameter UP_DEPTH      = 87360,      // 26x26x128 — upsample output
    parameter CAT_DEPTH     = 258048,     // 26x26x384 — concat output
    parameter DET1_DEPTH    = 43095,      // 13x13x255 (3 anchors × 85)
    parameter DET2_DEPTH    = 172380      // 26x26x255
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic signed [7:0]  weight_buf [WEIGHT_DEPTH-1:0],
    input  logic signed [31:0] bias_buf   [BIAS_DEPTH-1:0],
    input  logic signed [7:0]  image_buf  [IMG_DEPTH-1:0],
    output logic signed [7:0]  det1_buf   [DET1_DEPTH-1:0],
    output logic signed [7:0]  det2_buf   [DET2_DEPTH-1:0],
    output logic done
);

   

    // ----------------------------------------------------------------
    // Feature map ping-pong buffers + special-purpose buffers
    // ----------------------------------------------------------------
    logic signed [7:0] buf_a      [BUF_DEPTH-1:0];   // ping-pong side A
    logic signed [7:0] buf_b      [BUF_DEPTH-1:0];   // ping-pong side B
    logic signed [7:0] route_buf  [ROUTE_DEPTH-1:0]; // saved 26x26x256
    logic signed [7:0] route8_buf [BUF_DEPTH-1:0];   // saved 13x13x256 (conv8 out)
    logic signed [7:0] up_buf     [BUF_DEPTH-1:0];   // upsample output
    logic signed [7:0] cat_obuf   [BUF_DEPTH-1:0];   // concat output

    // ping=0: submodule reads buf_a, writes result into buf_b
    // ping=1: submodule reads buf_b, writes result into buf_a
    logic ping;

    // ----------------------------------------------------------------
    // Layer configuration registers (set in each _START state)
    // ----------------------------------------------------------------
    logic [31:0] cfg_H_in, cfg_W_in, cfg_C_in;
    logic [31:0] cfg_H_out, cfg_W_out, cfg_C_out;
    logic [31:0] cfg_stride;
    logic [31:0] cfg_weight_base; // K * K * C_In * C_out
    logic [31:0] cfg_bias_base;

    // ----------------------------------------------------------------
    // Conv submodule
    // ----------------------------------------------------------------
    logic signed [7:0]  conv_ibuf [BUF_DEPTH-1:0];
    logic signed [7:0]  conv_obuf [BUF_DEPTH-1:0];
    logic signed [31:0] conv_bias [31:0];
    logic               conv_start, conv_done;

    assign conv_ibuf = ping ? buf_b : buf_a;

    always_comb begin
        for(int i = 0; i < 32; i++) begin
            conv_bias[i] = bias_buf[cfg_bias_base + i];
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if(rst_n && conv_done) begin
            if (ping) buf_a <= conv_obuf;
            else      buf_b <= conv_obuf;
        end
    end

    conv_2d #(
        .K(3), .NUM_MACS(64),
        .IBUF_DEPTH(BUF_DEPTH),
        .WBUF_DEPTH(WEIGHT_DEPTH),
        .OBUF_DEPTH(BUF_DEPTH)
    ) u_conv (
        .clk(clk), .rst_n(rst_n),
        .weight_buffer(weight_buf),
        .i_buffer(conv_ibuf),
        .bias(conv_bias),
        .C_out(cfg_C_out), .H_out(cfg_H_out), .W_out(cfg_W_out),
        .C_in(cfg_C_in),   .stride(cfg_stride), .W_in(cfg_W_in),
        .weight_base(cfg_weight_base),
        .start_process(conv_start),
        .output_buffer(conv_obuf),
        .done(conv_done)
    );

    // ----------------------------------------------------------------
    // MaxPool submodule
    // ----------------------------------------------------------------
    logic signed [7:0] pool_ibuf [BUF_DEPTH-1:0];
    logic signed [7:0] pool_obuf [BUF_DEPTH-1:0];
    logic              pool_start, pool_done;

    always_comb begin
        pool_ibuf = ping ? buf_b : buf_a;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (rst_n && pool_done) begin
            if (ping) buf_a <= pool_obuf;
            else      buf_b <= pool_obuf;
        end
    end

    maxpool2d #(
        .IBUF_DEPTH(BUF_DEPTH),
        .OBUF_DEPTH(BUF_DEPTH)
    ) u_pool (
        .clk(clk), .rst_n(rst_n),
        .i_buffer(pool_ibuf),
        .H_in(cfg_H_in), .W_in(cfg_W_in), .C(cfg_C_in),
        .start(pool_start),
        .o_buffer(pool_obuf),
        .stride(cfg_stride),
        .done(pool_done)
    );

    logic signed [7:0] up_ibuf [BUF_DEPTH-1:0];
    logic signed [7:0] up_obuf [BUF_DEPTH-1:0];
    logic              up_start, up_done;

    always_comb begin
       up_ibuf = ping ? buf_b : buf_a;
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (rst_n && up_done) begin
            up_buf <= up_obuf;
        end
    end

    upsample #(
        .IBUF_DEPTH(BUF_DEPTH),
        .OBUF_DEPTH(BUF_DEPTH)
    ) u_up (
        .clk(clk), .rst_n(rst_n),
        .i_buffer(up_ibuf),
        .H_in(cfg_H_in), .W_in(cfg_W_in), .C(cfg_C_in),
        .start(up_start),
        .o_buffer(up_obuf),
        .done(up_done)
    );

    // ----------------------------------------------------------------
    // Route/concat submodule
    // ----------------------------------------------------------------
    logic cat_start, cat_done;
    route_concat #(
        .ABUF_DEPTH(BUF_DEPTH),
        .BBUF_DEPTH(ROUTE_DEPTH),
        .OBUF_DEPTH(BUF_DEPTH)
    ) u_concat (
        .clk(clk), .rst_n(rst_n),
        .a_buffer(up_buf),
        .b_buffer(route_buf),
        .H(cfg_H_out), .W(cfg_W_out),
        .C_a(cfg_C_in), .C_b(32'd256),
        .start(cat_start),
        .o_buffer(cat_obuf),
        .done(cat_done)
    );

    // ----------------------------------------------------------------
    // FSM
    // ----------------------------------------------------------------
     typedef enum logic [5:0] {
        IDLE,
        CONV1_START,  CONV1_WAIT,   // 416x416x3  → 416x416x16,  K=3
        POOL1_START,  POOL1_WAIT,   // 416x416x16 → 208x208x16
        CONV2_START,  CONV2_WAIT,   // 208x208x16 → 208x208x32,  K=3
        POOL2_START,  POOL2_WAIT,   // 208x208x32 → 104x104x32
        CONV3_START,  CONV3_WAIT,   // 104x104x32 → 104x104x64,  K=3
        POOL3_START,  POOL3_WAIT,   // 104x104x64 → 52x52x64
        CONV4_START,  CONV4_WAIT,   // 52x52x64   → 52x52x128,   K=3
        POOL4_START,  POOL4_WAIT,   // 52x52x128  → 26x26x128
        CONV5_START,  CONV5_WAIT,   // 26x26x128  → 26x26x256,   K=3
        SAVE_ROUTE,                 // snapshot conv5 output → route_buf
        POOL5_START,  POOL5_WAIT,   // 26x26x256  → 13x13x256
        CONV6_START,  CONV6_WAIT,   // 13x13x256  → 13x13x512,   K=3
        POOL6_START,  POOL6_WAIT,   // 13x13x512  → 13x13x512  (stride-1 pool)
        CONV7_START,  CONV7_WAIT,   // 13x13x512  → 13x13x1024, K=3
        CONV8_START,  CONV8_WAIT,   // 13x13x1024 → 13x13x256,  K=1  ← route src
        CONV9_START,  CONV9_WAIT,   // 13x13x256  → 13x13x512,  K=3
        YOLO1,                      // det1_buf = current output buf
        ROUTE_RESTORE,              // reload conv8 output as input
        CONV10_START, CONV10_WAIT,  // 13x13x256  → 13x13x128,  K=1
        UPSAMP_START, UPSAMP_WAIT,  // 13x13x128  → 26x26x128
        CONCAT_START, CONCAT_WAIT,  // 26x26x128 ++ 26x26x256 → 26x26x384
        CONV11_START, CONV11_WAIT,  // 26x26x384  → 26x26x256,  K=3
        YOLO2,                      // det2_buf = current output buf
        ALL_DONE
    } state_t;

    state_t current_state, next_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ping       <= 0;
            route_buf  <= '{default: '0};
            route8_buf <= '{default: '0};
            det1_buf   <= '{default: '0};
            det2_buf   <= '{default: '0};
        end else begin
            // flip ping when a conv or pool layer finishes
            if ((conv_done && (current_state == CONV1_WAIT  || current_state == CONV2_WAIT  ||
                               current_state == CONV3_WAIT  || current_state == CONV4_WAIT  ||
                               current_state == CONV5_WAIT  || current_state == CONV6_WAIT  ||
                               current_state == CONV7_WAIT  || current_state == CONV9_WAIT  ||
                               current_state == CONV10_WAIT || current_state == CONV11_WAIT)) ||
                (pool_done && (current_state == POOL1_WAIT  || current_state == POOL2_WAIT  ||
                               current_state == POOL3_WAIT  || current_state == POOL4_WAIT  ||
                               current_state == POOL5_WAIT  || current_state == POOL6_WAIT)) ||
                (cat_done  &&  current_state == CONCAT_WAIT))
            begin
                ping <= ~ping;
            end

            if(current_state == SAVE_ROUTE) begin
                route_buf <= ping ? buf_b: buf_a;
            end
            if(current_state == CONV8_WAIT && conv_done) begin
                route8_buf <= ping ? buf_b: buf_a;
            end
            if(current_state == YOLO1) begin
                det1_buf <= ping ? buf_b: buf_a;
            end
            if(current_state == YOLO2) begin
                det2_buf <= ping ? buf_b: buf_a;
            end
            if(cat_done && current_state == CONCAT_WAIT) begin
                if(ping) buf_a <= cat_obuf;
                else     buf_b <= cat_obuf;
            end
            if (current_state == ROUTE_RESTORE) begin
                if (ping) buf_b <= route8_buf;
                else      buf_a <= route8_buf;
            end
        end
    end

    always_comb begin
        next_state = current_state;
        conv_start = 0;
        pool_start = 0;
        up_start = 0;
        cat_start = 0;
        done = 0;
        cfg_weight_base = 0;
        cfg_bias_base   = 0;
        cfg_H_in  = 0; cfg_W_in  = 0; cfg_C_in  = 0;
        cfg_H_out = 0; cfg_W_out = 0; cfg_C_out = 0;
        cfg_stride = 1;
        case(current_state)
            IDLE: begin
                if(start) begin
                    next_state = CONV1_START;
                end
            end
            CONV1_START: begin
                next_state = CONV1_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd0;
                cfg_bias_base   = 32'd0;
                cfg_H_in = 416; cfg_W_in = 416; cfg_C_in = 3;
                cfg_H_out = 416; cfg_W_out = 416; cfg_C_out = 16;
                cfg_stride = 1;
            end
            CONV1_WAIT: begin
                cfg_H_in = 416; cfg_W_in = 416; cfg_C_in = 3;
                cfg_H_out = 416; cfg_W_out = 416; cfg_C_out = 16;
                if(conv_done) next_state = POOL1_START;
            end
            POOL1_START: begin
               pool_start = 1;
               next_state = POOL1_WAIT;
               cfg_stride = 2;
               cfg_H_in = 416; cfg_W_in = 416; cfg_C_in = 16;
            end
            POOL1_WAIT: begin
                cfg_H_in = 416; cfg_W_in = 416; cfg_C_in = 16;
                if(pool_done) next_state = CONV2_START;
            end
            CONV2_START: begin
                next_state = CONV2_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd432;
                cfg_bias_base   = 32'd16;
                cfg_H_in = 208; cfg_W_in = 208; cfg_C_in = 16;
                cfg_H_out = 208; cfg_W_out = 208; cfg_C_out = 32;
                cfg_stride = 1;
            end
            CONV2_WAIT: begin
                cfg_H_in = 208; cfg_W_in = 208; cfg_C_in = 16;
                cfg_H_out = 208; cfg_W_out = 208; cfg_C_out = 32;
                if(conv_done) next_state = POOL2_START;
            end
            POOL2_START: begin
               pool_start = 1;
               next_state = POOL2_WAIT;
               cfg_stride = 2;
               cfg_H_in = 208; cfg_W_in = 208; cfg_C_in = 32;
            end
            POOL2_WAIT: begin
                cfg_H_in = 208; cfg_W_in = 208; cfg_C_in = 32;
                if(pool_done) next_state = CONV3_START;
            end
            CONV3_START: begin
                next_state = CONV3_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd5040;
                cfg_bias_base   = 32'd48;
                cfg_H_in = 104; cfg_W_in = 104; cfg_C_in = 32;
                cfg_H_out = 104; cfg_W_out = 104; cfg_C_out = 64;
                cfg_stride = 1;
            end
            CONV3_WAIT: begin
                cfg_H_in = 104; cfg_W_in = 104; cfg_C_in = 32;
                cfg_H_out = 104; cfg_W_out = 104; cfg_C_out = 64;
                if(conv_done) next_state = POOL3_START;
            end
            POOL3_START: begin
               pool_start = 1;
               cfg_stride = 2;
               next_state = POOL3_WAIT;
               cfg_H_in = 104; cfg_W_in = 104; cfg_C_in = 64;
            end
            POOL3_WAIT: begin
                cfg_H_in = 104; cfg_W_in = 104; cfg_C_in = 64;
                if(pool_done) next_state = CONV4_START;
            end
            CONV4_START: begin
                next_state = CONV4_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd23472;
                cfg_bias_base   = 32'd112;
                cfg_H_in = 52; cfg_W_in = 52; cfg_C_in = 64;
                cfg_H_out = 52; cfg_W_out = 52; cfg_C_out = 128;
                cfg_stride = 1;
            end
            CONV4_WAIT: begin
                cfg_H_in = 52; cfg_W_in = 52; cfg_C_in = 64;
                cfg_H_out = 52; cfg_W_out = 52; cfg_C_out = 128;
                if(conv_done) next_state = POOL4_START;
            end
            POOL4_START: begin
               pool_start = 1;
               cfg_stride = 2;
               next_state = POOL4_WAIT;
               cfg_H_in = 52; cfg_W_in = 52; cfg_C_in = 128;
            end
            POOL4_WAIT: begin
                cfg_H_in = 52; cfg_W_in = 52; cfg_C_in = 128;
                if(pool_done) next_state = CONV5_START;
            end
            CONV5_START: begin
                next_state = CONV5_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd97200;
                cfg_bias_base   = 32'd240;
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 256;
                cfg_stride = 1;
            end
            CONV5_WAIT: begin
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 128;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 256;
                if(conv_done) next_state = SAVE_ROUTE;
            end
            SAVE_ROUTE: begin
                next_state = POOL5_START;
            end
            POOL5_START: begin
               pool_start = 1;
               cfg_stride = 2;
               next_state = POOL5_WAIT;
               cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 256;
            end
            POOL5_WAIT: begin
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 256;
                if(pool_done) next_state = CONV6_START;
            end
            CONV6_START: begin
                next_state = CONV6_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd392112;
                cfg_bias_base   = 32'd496;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                cfg_stride = 1;
            end
            CONV6_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                if(conv_done) next_state = POOL6_START;
            end
            POOL6_START: begin
               pool_start = 1;
               cfg_stride = 1;
               next_state = POOL6_WAIT;
               cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 512;
            end
            POOL6_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 512;
                if(pool_done) next_state = CONV7_START;
            end
            CONV7_START: begin
                next_state = CONV7_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd1571760;
                cfg_bias_base   = 32'd1008;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 512;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 1024;
                cfg_stride = 1;
            end
            CONV7_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 512;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 1024;
                if(conv_done) next_state = CONV8_START;
            end
            CONV8_START: begin
                next_state = CONV8_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd6290352;
                cfg_bias_base   = 32'd2032;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 1024;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 256;
                cfg_stride = 1;
            end
            CONV8_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 1024;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 256;
                if(conv_done) next_state = CONV9_START;
            end
            CONV9_START: begin
                next_state = CONV9_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd6552496;
                cfg_bias_base   = 32'd2288;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                cfg_stride = 1;
            end
            CONV9_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 512;
                if(conv_done) next_state = YOLO1;
            end
            YOLO1: begin
                next_state = ROUTE_RESTORE;
            end
            ROUTE_RESTORE: begin
                next_state = CONV10_START;
            end
            CONV10_START: begin
                next_state = CONV10_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd7732144;
                cfg_bias_base   = 32'd2800;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 128;
                cfg_stride = 1;
            end
            CONV10_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 256;
                cfg_H_out = 13; cfg_W_out = 13; cfg_C_out = 128;
                if(conv_done) next_state = UPSAMP_START;
            end
            UPSAMP_START: begin
                up_start = 1;
                next_state = UPSAMP_WAIT;
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 128;
            end
            UPSAMP_WAIT: begin
                cfg_H_in = 13; cfg_W_in = 13; cfg_C_in = 128;
                if(up_done) next_state = CONCAT_START;
            end
            CONCAT_START: begin
                cat_start = 1;
                next_state = CONCAT_WAIT;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_in = 128;
            end
            CONCAT_WAIT: begin
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_in = 128;
                if(cat_done) next_state = CONV11_START;
            end
            CONV11_START: begin
                next_state = CONV11_WAIT;
                conv_start = 1;
                cfg_weight_base = 32'd7764912;
                cfg_bias_base   = 32'd2928;
                cfg_H_in = 26; cfg_W_in = 26; cfg_C_in = 384;
                cfg_H_out = 26; cfg_W_out = 26; cfg_C_out = 256;
                cfg_stride = 1;
            end
            CONV11_WAIT: begin
                if(conv_done) begin
                    next_state = YOLO2;
                end
            end
            YOLO2: begin
                next_state = ALL_DONE;
            end
            ALL_DONE: begin
                done = 1;
                next_state = IDLE;
            end
            
            

        endcase
    end
endmodule


