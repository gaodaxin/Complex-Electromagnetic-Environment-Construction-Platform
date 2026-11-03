// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_tri_pack.v
// 功能：用移位和分级加法生成 i(i−1)/2×S，以及 16S、120S 等二次项。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_tri_pack
// 功能: 为 16 路并行 LFM 样点构造 i(i-1)/2×S，并同时给出 16S、120S。
// 实现: 使用 S 的二进制移位项分级相加，避免通用乘法和超长组合进位链。
// =============================================================================

module da_sig_tri_pack (

    input  wire               clk,           // FPGA 工作时钟，与 accum 同钟
    input  wire               rst_sync,      // 同步复位，高有效
    input  wire signed [31:0] slope,         // 每样点斜率 S（FTW 域），上游已打拍

    output wire signed [31:0] t0,            // i=0 二次项，恒 0
    output wire signed [31:0] t1,            // i=1 二次项，恒 0
    output wire signed [31:0] t2,            // i=2：1S
    output wire signed [31:0] t3,            // i=3：3S
    output wire signed [31:0] t4,            // i=4：6S
    output wire signed [31:0] t5,            // i=5：10S
    output wire signed [31:0] t6,            // i=6：15S
    output wire signed [31:0] t7,            // i=7：21S
    output wire signed [31:0] t8,            // i=8：28S
    output wire signed [31:0] t9,            // i=9：36S
    output wire signed [31:0] t10,           // i=10：45S
    output wire signed [31:0] t11,           // i=11：55S
    output wire signed [31:0] t12,           // i=12：66S
    output wire signed [31:0] t13,           // i=13：78S
    output wire signed [31:0] t14,           // i=14：91S
    output wire signed [31:0] t15,           // i=15：105S

    output wire signed [31:0] slope_x16,     // 16S：本拍 16 样点一次斜升
    output wire signed [31:0] slope_x120     // 120S：16 样点二次项之和，加到相位基
);

reg signed [31:0] s;                     // S
reg signed [31:0] s2;                    // 2S
reg signed [31:0] s4;                    // 4S
reg signed [31:0] s8;                    // 8S
reg signed [31:0] s16;                   // 16S
reg signed [31:0] s32;                   // 32S
reg signed [31:0] s64;                   // 64S
reg signed [31:0] s128;                  // 128S

reg signed [31:0] t0_r;                  // i=0：0
reg signed [31:0] t1_r;                  // i=1：0
reg signed [31:0] t2_r;                  // i=2：1S
reg signed [31:0] t3_r;                  // i=3：3S
reg signed [31:0] t4_r;                  // i=4：6S
reg signed [31:0] t5_r;                  // i=5：10S
reg signed [31:0] t6_r;                  // i=6：15S
reg signed [31:0] t7_r;                  // i=7：21S
reg signed [31:0] t8_r;                  // i=8：28S
reg signed [31:0] t9_r;                  // i=9：36S
reg signed [31:0] t10_r;                 // i=10：45S
reg signed [31:0] t11_r;                 // i=11：55S
reg signed [31:0] t12_r;                 // i=12：66S
reg signed [31:0] t13_r;                 // i=13：78S
reg signed [31:0] t14_r;                 // i=14：91S
reg signed [31:0] t15_r;                 // i=15：105S
reg signed [31:0] sx16_r;                // 16S
reg signed [31:0] sx120_r;               // 120S


// =============================================================================
// 拍1：移位项先寄存，避免 S→105S 进同一拍 CARRY 链
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        s    <= 32'sd0;                  // S 清零
        s2   <= 32'sd0;                  // 2S 清零
        s4   <= 32'sd0;                  // 4S 清零
        s8   <= 32'sd0;                  // 8S 清零
        s16  <= 32'sd0;                  // 16S 清零
        s32  <= 32'sd0;                  // 32S 清零
        s64  <= 32'sd0;                  // 64S 清零
        s128 <= 32'sd0;                  // 128S 清零

    end else begin

        s    <= slope;                   // 锁存 S
        s2   <= slope <<< 1;             // 2S
        s4   <= slope <<< 2;             // 4S
        s8   <= slope <<< 3;             // 8S
        s16  <= slope <<< 4;             // 16S
        s32  <= slope <<< 5;             // 32S
        s64  <= slope <<< 6;             // 64S
        s128 <= slope <<< 7;             // 128S

    end

end


// =============================================================================
// 拍2：短加法拼出 t[i]=i*(i-1)/2*S，以及 16S、120S
// 端口不叫 tri0：tri0 是 Verilog-2001 关键字
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        t0_r    <= 32'sd0;               // i=0 清零
        t1_r    <= 32'sd0;               // i=1 清零
        t2_r    <= 32'sd0;               // i=2 清零
        t3_r    <= 32'sd0;               // i=3 清零
        t4_r    <= 32'sd0;               // i=4 清零
        t5_r    <= 32'sd0;               // i=5 清零
        t6_r    <= 32'sd0;               // i=6 清零
        t7_r    <= 32'sd0;               // i=7 清零
        t8_r    <= 32'sd0;               // i=8 清零
        t9_r    <= 32'sd0;               // i=9 清零
        t10_r   <= 32'sd0;               // i=10 清零
        t11_r   <= 32'sd0;               // i=11 清零
        t12_r   <= 32'sd0;               // i=12 清零
        t13_r   <= 32'sd0;               // i=13 清零
        t14_r   <= 32'sd0;               // i=14 清零
        t15_r   <= 32'sd0;               // i=15 清零
        sx16_r  <= 32'sd0;               // 16S 清零
        sx120_r <= 32'sd0;               // 120S 清零

    end else begin

        t0_r    <= 32'sd0;               // i=0：0
        t1_r    <= 32'sd0;               // i=1：0
        t2_r    <= s;                    // i=2：1S
        t3_r    <= s2 + s;               // i=3：2S+S=3S
        t4_r    <= s4 + s2;              // i=4：4S+2S=6S
        t5_r    <= s8 + s2;              // i=5：8S+2S=10S
        t6_r    <= (s8 + s4) + (s2 + s); // i=6：8+4+2+1=15S
        t7_r    <= s16 + s4 + s;         // i=7：16+4+1=21S
        t8_r    <= s16 + s8 + s4;        // i=8：16+8+4=28S
        t9_r    <= s32 + s4;             // i=9：32+4=36S
        t10_r   <= (s32 + s8) + (s4 + s);            // i=10：32+8+4+1=45S
        t11_r   <= (s32 + s16) + (s4 + s2) + s;      // i=11：32+16+4+2+1=55S
        t12_r   <= s64 + s2;             // i=12：64+2=66S
        t13_r   <= s64 + (s8 + s4 + s2); // i=13：64+8+4+2=78S
        t14_r   <= (s64 + s16) + (s8 + s2) + s;      // i=14：64+16+8+2+1=91S
        t15_r   <= (s64 + s32) + (s8 + s);           // i=15：64+32+8+1=105S
        sx16_r  <= s16;                  // 16S：本拍 16 样点一次斜升
        sx120_r <= s128 - s8;            // 120S=128S-8S，二次项之和

    end

end

assign t0  = t0_r;                       // i=0，恒 0
assign t1  = t1_r;                       // i=1，恒 0
assign t2  = t2_r;                       // i=2：1S
assign t3  = t3_r;                       // i=3：3S
assign t4  = t4_r;                       // i=4：6S
assign t5  = t5_r;                       // i=5：10S
assign t6  = t6_r;                       // i=6：15S
assign t7  = t7_r;                       // i=7：21S
assign t8  = t8_r;                       // i=8：28S
assign t9  = t9_r;                       // i=9：36S
assign t10 = t10_r;                      // i=10：45S
assign t11 = t11_r;                      // i=11：55S
assign t12 = t12_r;                      // i=12：66S
assign t13 = t13_r;                      // i=13：78S
assign t14 = t14_r;                      // i=14：91S
assign t15 = t15_r;                      // i=15：105S
assign slope_x16  = sx16_r;              // 16S
assign slope_x120 = sx120_r;             // 120S

endmodule
