module route_concat #(
    parameter ABUF_DEPTH = 4096,   // 26*26*128 upsampled
    parameter BBUF_DEPTH = 4096,   // 26*26*256 saved route
    parameter OBUF_DEPTH = 16384   // 26*26*384 output
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic signed [7:0] a_buffer [ABUF_DEPTH-1:0],  // upsample output
    input  logic signed [7:0] b_buffer [BBUF_DEPTH-1:0],  // saved feature map
    input  logic [31:0] H,
    input  logic [31:0] W,
    input  logic [31:0] C_a,   // channels from buffer a
    input  logic [31:0] C_b,   // channels from buffer b
    output logic signed [7:0] o_buffer [OBUF_DEPTH-1:0],
    output logic done
);



 
    // Output pixel counter
    logic [31:0] y_out, x_out, c_out;
    logic running;

    logic signed [7:0] v00,v01;

    localparam A_ADDR_BITS = $clog2(ABUF_DEPTH);
    localparam B_ADDR_BITS = $clog2(BBUF_DEPTH);
    localparam OADDR_BITS  =  $clog2(OBUF_DEPTH);

   
    logic [31:0] a_addr, b_addr;
    assign a_addr = ((y_out * W) * C_a) + (x_out * C_a + c_out);
    assign b_addr = y_out * W * C_b + x_out * C_b + (c_out - C_a);

    assign v00 = a_buffer[a_addr[A_ADDR_BITS-1:0]];
    assign v01 = b_buffer[b_addr[B_ADDR_BITS-1:0]];
    logic [31:0] out_addr, out_addr_r, c_out_r;
    assign out_addr = (y_out * W) * (C_a + C_b) + x_out * (C_a + C_b) + c_out;
    
    always_ff @(posedge clk or negedge rst_n) begin
         if(!rst_n) begin
            out_addr_r <= '0;
            c_out_r <= '0;
         end else begin
            out_addr_r <= out_addr; // have this for a one cycle read latency possibly
            c_out_r <= c_out;
         end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out   <= '0; x_out   <= '0; c_out <= '0;
            running <= 1'b0;
            done    <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start && !running) begin
                y_out <= '0; x_out <= '0; c_out <= '0;
                running <= 1'b1;
            end else if (running) begin
                if(c_out_r < C_a) begin
                    o_buffer[out_addr_r[OADDR_BITS-1:0]] <= v00;
                end else begin
                    o_buffer[out_addr_r[OADDR_BITS-1:0]] <= v01;
                end
                if (c_out < C_a + C_b - 1) begin
                    c_out <= c_out + 1;
                end else begin
                    c_out <= '0;
                    if (x_out < W - 1) begin
                        x_out <= x_out + 1;
                    end else begin
                        x_out <= '0;
                        if (y_out < H - 1) begin
                            y_out <= y_out + 1;
                        end else begin
                            running <= 1'b0;
                            done    <= 1'b1;
                        end
                    end
                end
            end
        end
    end





endmodule