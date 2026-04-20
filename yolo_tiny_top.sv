`timescale 1ns / 1ps

module yolo_tiny_top #(
    parameter WEIGHT_DEPTH  = 1 << 20,   // all layer weights concatenated
    parameter BIAS_DEPTH    = 256,        // enough for all C_out across all layers
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

    state_t state;

endmodule


