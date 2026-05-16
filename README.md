# YOLOv3-Tiny FPGA Accelerator

A complete RTL implementation of the YOLOv3-Tiny object detection network in SystemVerilog, targeting FPGA deployment. The design processes 416×416 RGB images using int8 quantized weights and produces two detection head outputs at 13×13 and 26×26 spatial scales.

## Architecture

The network is sequenced by a 40+ state FSM in `yolo_tiny_top.sv`. A ping-pong buffer scheme (`buf_a`/`buf_b`) carries feature maps between layers, flipping ownership after each completed layer.

```
image_buf (416×416×3)
    │
    ▼
Conv1 (416×416×16)  →  Pool1 (208×208×16)
Conv2 (208×208×32)  →  Pool2 (104×104×32)
Conv3 (104×104×64)  →  Pool3  (52×52×64)
Conv4  (52×52×128)  →  Pool4  (26×26×128)
Conv5  (26×26×256)  →  [save → route_buf]  →  Pool5  (13×13×256)
Conv6  (13×13×512)  →  Pool6  (13×13×512, stride=1)
Conv7  (13×13×1024) →  Conv8  (13×13×256)  →  [save → route8_buf]
Conv9  (13×13×512)  →  YOLO Head 1  →  det1_buf (13×13×255)
                                         ↓
                        ROUTE_RESTORE (reload route8_buf)
Conv10 (13×13×128)  →  Upsample (26×26×128)
Concat (26×26×384)  ←  route_buf (26×26×256)
Conv11 (26×26×256)  →  YOLO Head 2  →  det2_buf (26×26×255)
```

## Modules

| File | Module(s) | Description |
|---|---|---|
| `yolo_tiny_top.sv` | `yolo_tiny_top` | Top-level FSM sequencer, ping-pong buffers, layer wiring |
| `yolo_v1.sv` | `conv_2d`, `conv_chunk_fsm` | 2D convolution with 64-wide MAC array, bias add, leaky ReLU, int8 clip |
| `mac.sv` | `mac_array`, `accumulator`, `conv_iteration` | 64-parallel int8 MAC tree, accumulator register |
| `maxpool.sv` | `maxpool2d` | 2×2 max pool with configurable stride (stride=2 for Pool1–5, stride=1 for Pool6) |
| `upsample.sv` | `upsample` | Nearest-neighbor 2× spatial upsample |
| `routeconcat.sv` | `route_concat` | Channel-wise concatenation of two feature maps (HWC layout) |
| `op_units.sv` | `int_mul`, `int_add`, `bias_add` | Signed int8×int8 multiply, int32 add |
| `leakyRELU.sv` | `leaky_relu` | Leaky ReLU: α=0.125 (arithmetic right-shift by 3) for negative values |
| `fp_op.sv` | `fp_add`, `fp_mul` | 4-stage pipelined FP16 adder and multiplier (used offline, not in inference path) |

## Data Format

- **Weights**: signed int8, stored flat in HWC order, all layers concatenated
- **Biases**: signed int32, all layers concatenated
- **Activations**: signed int8, HWC layout `[y * W * C + x * C + c]`
- **Detection output**: raw int8 feature map at each scale (255 = 3 anchors × 85 values)

### Weight and bias base offsets

| Layer | K | C_in | C_out | Weight base | Bias base |
|---|---|---|---|---|---|
| Conv1  | 3 | 3    | 16   | 0         | 0    |
| Conv2  | 3 | 16   | 32   | 432       | 16   |
| Conv3  | 3 | 32   | 64   | 5040      | 48   |
| Conv4  | 3 | 64   | 128  | 23472     | 112  |
| Conv5  | 3 | 128  | 256  | 97200     | 240  |
| Conv6  | 3 | 256  | 512  | 392112    | 496  |
| Conv7  | 3 | 512  | 1024 | 1571760   | 1008 |
| Conv8  | 1 | 1024 | 256  | 6290352   | 2032 |
| Conv9  | 3 | 256  | 512  | 6552496   | 2288 |
| Conv10 | 1 | 256  | 128  | 7732144   | 2800 |
| Conv11 | 3 | 384  | 256  | 7764912   | 2928 |

## Top-Level Parameters

```systemverilog
parameter WEIGHT_DEPTH = 8649648   // all layer weights concatenated
parameter BIAS_DEPTH   = 4096      // padded to power of 2
parameter IMG_DEPTH    = 519168    // 416×416×3
parameter BUF_DEPTH    = 2752512   // 416×416×16 (largest intermediate)
parameter ROUTE_DEPTH  = 172032    // 26×26×256 (saved route)
parameter DET1_DEPTH   = 43095     // 13×13×255
parameter DET2_DEPTH   = 172380    // 26×26×255
```

## Simulation

Individual module tests run fine. The full-network testbench (`tb_yolo_tiny_top.sv`) elaborates and compiles correctly but full 416×416 behavioral simulation is not practical — the complete forward pass takes hundreds of millions of cycles. Use it for FSM sanity checks with a modified (reduced-dimension) build, or test submodules individually.

The existing `top.sv` is a working conv unit test: 5×5 input, K=3, C_in=1, C_out=1, all weights=1, all inputs=1 → expected output is all 9s.

## FPGA Deployment (Ultra96v2)

The compute core and FSM are synthesizable as written. The flat array ports (`weight_buf`, `image_buf`, `det1_buf`, `det2_buf`) are not directly usable as FPGA I/O and must be replaced with an AXI wrapper for real deployment.

**Memory constraints on ZU3EG (Ultra96v2):**
- On-chip BRAM: ~972 KB — cannot hold any of the large buffers
- LPDDR4: 2 GB — all weight and feature map buffers must live here

**Planned AXI interface:**

| Interface | Type | Purpose |
|---|---|---|
| `S_AXI_LITE` | AXI-Lite slave | `start`, `done`, buffer base addresses |
| `M_AXI_HP0` | AXI4 master | Feature map reads/writes in DDR |
| `M_AXI_HP1` | AXI4 master | Weight/bias reads from DDR |

**Python (PYNQ) workflow:**
WIP
> The AXI wrapper is not yet implemented — the current RTL is the compute core only.
