/**
This is the upsample module which is responsible for upsample inputs from
a certain layer it does this by taking inputs from thhe input buffer such as

AB
CD 

and copying like this upscaling it to the new layers output depth like this

AABB
CCDD


**/
module upsample
#(parameter IBUF_DEPTH=1024,
 parameter OBUF_DEPTH=4096)
(   
    input logic clk,
    input logic rst_n,
    input logic start,
    input logic signed [7:0] i_buffer [IBUF_DEPTH-1:0],
    input logic [31:0] H_in,
    input logic [31:0] W_in,
    input logic [31:0] C,
    output logic signed [7:0] o_buffer [OBUF_DEPTH-1:0],
    output logic done
);


    logic [31:0] H_out, W_out;
    assign H_out = H_in * 2;
    assign W_out = W_in * 2;

    // Output pixel counter
    logic [31:0] y_out, x_out, c_out;
    logic running;

    logic [31:0] addr00, out_addr_r;
    logic signed [7:0] v00;

    localparam IADDR_BITS = $clog2(IBUF_DEPTH);
    localparam OADDR_BITS = $clog2(OBUF_DEPTH);

    // Address of the 2x2 window top-left = (y_out*2, x_out*2)
    assign addr00 = (((y_out/2) * W_in * C) + ((x_out/2) * C + c_out));
    assign v00 = i_buffer[addr00[IADDR_BITS-1:0]];


    logic [31:0] out_addr;
    assign out_addr = ((y_out*W_out*C) + (x_out*C) + (c_out));
    
    always_ff @(posedge clk) out_addr_r <= out_addr; // have this for a one cycle read latency possibly

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
                o_buffer[out_addr_r[OADDR_BITS-1:0]] <= v00;
                if (c_out < C - 1) begin
                    c_out <= c_out + 1;
                end else begin
                    c_out <= '0;
                    if (x_out < W_out - 1) begin
                        x_out <= x_out + 1;
                    end else begin
                        x_out <= '0;
                        if (y_out < H_out - 1) begin
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






