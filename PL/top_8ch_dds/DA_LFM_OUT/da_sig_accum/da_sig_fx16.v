// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_fx16.v
// 功能：用移位和加法生成十六路 i×F 与 16F 常数倍，减少通用乘法器使用。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_fx16
// 功能: 为 16 路并行样点生成 i·F（i=0..15）及 16F。
// 实现: 使用移位和加法构造常数倍，避免 16 个通用乘法器。
// 用途: 配合 da_sig_tri_pack 形成同一 fabric 拍内的 16 个连续 DDS 相位。
// =============================================================================

module da_sig_fx16 (

    input  wire               clk,       // FPGA 工作时钟
    input  wire               rst_sync,  // 同步复位，高有效
    input  wire               start_edge,// 启动沿；16 路乘积清零
    input  wire signed [31:0] f_in,      // 当前锁存的瞬时 FTW
    input  wire signed [31:0] s120,      // 120S；无斜率时为 0
    output reg  signed [31:0] f_emit,    // f_in 再打一拍，给对照
    output reg         [31:0] f16,       // 16F = f_in<<4
    output reg         [31:0] inc,       // 16F+120S，与 fx 同拍
    output reg         [31:0] fx0,       // 0·F
    output reg         [31:0] fx1,       // 1·F
    output reg         [31:0] fx2,
    output reg         [31:0] fx3,
    output reg         [31:0] fx4,
    output reg         [31:0] fx5,
    output reg         [31:0] fx6,
    output reg         [31:0] fx7,
    output reg         [31:0] fx8,
    output reg         [31:0] fx9,
    output reg         [31:0] fx10,
    output reg         [31:0] fx11,
    output reg         [31:0] fx12,
    output reg         [31:0] fx13,
    output reg         [31:0] fx14,
    output reg         [31:0] fx15       // 15·F
);

// 因 i 固定为 0~15，不用通用乘法器，移位+加法即可

always @(posedge clk) begin

    if (rst_sync || start_edge) begin

        f_emit <= 32'sd0;
        f16    <= 32'd0;
        inc    <= 32'd0;
        fx0  <= 32'd0; fx1  <= 32'd0; fx2  <= 32'd0; fx3  <= 32'd0;
        fx4  <= 32'd0; fx5  <= 32'd0; fx6  <= 32'd0; fx7  <= 32'd0;
        fx8  <= 32'd0; fx9  <= 32'd0; fx10 <= 32'd0; fx11 <= 32'd0;
        fx12 <= 32'd0; fx13 <= 32'd0; fx14 <= 32'd0; fx15 <= 32'd0;

    end else begin

        f_emit <= f_in;                  // 再打一拍
        f16    <= f_in <<< 4;            // 16F，给相位基
        inc    <= (f_in <<< 4) + s120;   // 16F+120S；phase_base 只做 2 输入加

        fx0  <= 32'd0;                   // 0·F
        fx1  <= f_in;                    // 1·F
        fx2  <= f_in <<< 1;              // 2·F
        fx3  <= f_in + (f_in <<< 1);     // 3·F = F+2F
        fx4  <= f_in <<< 2;              // 4·F
        fx5  <= f_in + (f_in <<< 2);     // 5·F
        fx6  <= (f_in <<< 1) + (f_in <<< 2);
                                         // 6·F = 2F+4F
        fx7  <= (f_in + (f_in <<< 1)) + (f_in <<< 2);
                                         // 7·F = 3F+4F
        fx8  <= f_in <<< 3;              // 8·F
        fx9  <= f_in + (f_in <<< 3);     // 9·F
        fx10 <= (f_in <<< 1) + (f_in <<< 3);
                                         // 10·F
        fx11 <= (f_in + (f_in <<< 1)) + (f_in <<< 3);
        fx12 <= (f_in <<< 2) + (f_in <<< 3);
                                         // 12·F
        fx13 <= (f_in + (f_in <<< 2)) + (f_in <<< 3);
        fx14 <= ((f_in <<< 1) + (f_in <<< 2)) + (f_in <<< 3);
        fx15 <= (f_in + (f_in <<< 1)) +
                ((f_in <<< 2) + (f_in <<< 3));
                                         // 15·F = 7F+8F

    end

end

endmodule
