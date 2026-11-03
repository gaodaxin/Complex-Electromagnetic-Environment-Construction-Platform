// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_div_q32.v
// 功能：以多拍比较/减法实现配置路径的 64/32 无符号除法，保留调频斜率的 Q32 小数精度。
// ============================================================================
`timescale 1ns/1ps
// Configuration-only 64/32 unsigned division, three clocks per quotient bit.
// A 33-bit remainder keeps compare/subtract out of the real-time DDS path.
module da_sig_div_q32 (
    input wire clk, rst, start,
    input wire [63:0] dividend,
    input wire [31:0] divisor,
    output reg [63:0] quot,
    output reg done
);
    localparam IDLE=0, SHIFT=1, COMPARE=2, UPDATE=3;
    reg [1:0] state=IDLE;
    reg [63:0] num;
    reg [31:0] den;
    reg [32:0] rem, shifted, difference;
    reg ge;
    reg [5:0] bit_index;
    always @(posedge clk) begin
        if (rst) begin
            state<=IDLE; quot<=0; done<=0; num<=0; den<=0;
            rem<=0; shifted<=0; difference<=0; ge<=0; bit_index<=0;
        end else begin
            done<=0;
            case (state)
                IDLE: if (start) begin
                    num<=dividend; den<=divisor; rem<=0; quot<=0;
                    bit_index<=63;
                    if (divisor==0) done<=1;
                    else state<=SHIFT;
                end
                SHIFT: begin shifted<={rem[31:0],num[63]}; state<=COMPARE; end
                COMPARE: begin
                    ge<=shifted>={1'b0,den};
                    difference<=shifted-{1'b0,den}; state<=UPDATE;
                end
                UPDATE: begin
                    rem<=ge ? difference : shifted;
                    quot<={quot[62:0],ge}; num<={num[62:0],1'b0};
                    if (bit_index==0) begin done<=1; state<=IDLE; end
                    else begin bit_index<=bit_index-1'b1; state<=SHIFT; end
                end
            endcase
        end
    end
endmodule
