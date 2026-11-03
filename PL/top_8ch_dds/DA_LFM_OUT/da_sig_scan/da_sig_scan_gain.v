// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_scan_gain.v
// 功能：对十六个有符号样点乘方向图增益并右移八位，或执行等延迟直通，保持数据与 valid 对齐。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_scan_gain
// 功能: 对 16 路有符号样点施加方向图增益，y=(x×G)>>>8。
// 范围: G=0 完全静音，G=255 约等于满幅；扫描关闭时走等延迟直通支路。
// 时序: 输入、DSP MREG/PREG 和输出共用固定流水，valid 与样点严格对齐。
//
// 数值格式:
//   x 为有符号 int16，G 先补 0 扩成正数 signed[8:0]，乘积使用 signed[23:0]。
//   取乘积 [23:8] 等效算术右移 8 bit；最坏 int16×255 不溢出 24 bit，
//   因而无需额外饱和。满幅系数为 255/256，而不是精确的 1.0。
//
// 流水（从输入采样边沿起算）:
//   第 1 拍: 锁存 din/G/on/valid，同时维护直通延迟链。
//   第 2 拍: 16 个 DSP 形成 MREG 乘积。
//   第 3 拍: 乘积进入 PREG，直通数据/on/valid 到达对应延迟级。
//   第 4 个边沿: dout 在乘法与直通结果间选择，dout_valid 同拍更新。
// DSP 乘积寄存器不复位；复位时 valid 流水清零，因此未初始化值不会被标为有效。
// =============================================================================

module da_sig_scan_gain (

    input  wire         clk,             // FPGA 工作时钟
    input  wire         rst_sync,        // 同步复位，高有效
    input  wire         on,              // 扫描打开；0 时直通对齐到同一拍
    input  wire [7:0]   g,               // 当前天线增益 G，0~255

    input  wire [255:0] din,             // 16×int16 正弦
    input  wire         din_valid,       // 正弦有效

    output reg  [255:0] dout,            // y=(x*G)>>>8；关扫描时为对齐后的 din
    output reg          dout_valid       // 与 dout 对齐的有效
);

reg         on_d;                        // 扫描开关延迟 1 拍
reg         on_d1;                       // 扫描开关延迟 2 拍
reg         on_d2;                       // 扫描开关延迟 3 拍，与输出选择数据对齐
reg [255:0] din_d;                       // 原始样点延迟 1 拍，直通支路使用
reg [255:0] din_d1;                      // 原始样点延迟 2 拍
reg [255:0] din_d2;                      // 原始样点延迟 3 拍，与 PREG 乘积对齐
reg [255:0] din_dsp;                     // DSP AREG 专用输入副本，16 路各取 16 bit
reg signed [8:0] g_d;                    // DSP BREG 输入；最高补 0 保证 128..255 仍为正数
reg         v_d;                         // din_valid 延迟 1 拍
reg         v_d1;                        // din_valid 延迟 2 拍
reg         v_d2;                        // din_valid 延迟 3 拍，与 dout 数据对齐


(* use_dsp = "yes" *)
reg signed [23:0] m0_m,  m1_m,  m2_m,  m3_m;   // 路 0~3 乘积 MREG
(* use_dsp = "yes" *)
reg signed [23:0] m4_m,  m5_m,  m6_m,  m7_m;   // 路 4~7 乘积 MREG
(* use_dsp = "yes" *)
reg signed [23:0] m8_m,  m9_m,  m10_m, m11_m;  // 路 8~11 乘积 MREG
(* use_dsp = "yes" *)
reg signed [23:0] m12_m, m13_m, m14_m, m15_m;  // 路 12~15 乘积 MREG
(* use_dsp = "yes" *)
reg signed [23:0] m0, m1, m2, m3, m4, m5, m6, m7;          // 路 0~7 乘积 PREG
(* use_dsp = "yes" *)
reg signed [23:0] m8, m9, m10, m11, m12, m13, m14, m15;    // 路 8~15 乘积 PREG


// =============================================================================
// DSP 乘法支路：16 路中间量 y_full=x×G，先 MREG 再 PREG
// 每路乘法彼此独立，属性提示 Vivado 使用 DSP48；不在这里做右移和饱和。
// =============================================================================

always @(posedge clk) begin

    m0_m  <= $signed(din_dsp[ 15:  0]) * g_d;  // 路 0：x*G，A/B 已寄存
    m1_m  <= $signed(din_dsp[ 31: 16]) * g_d;
    m2_m  <= $signed(din_dsp[ 47: 32]) * g_d;
    m3_m  <= $signed(din_dsp[ 63: 48]) * g_d;
    m4_m  <= $signed(din_dsp[ 79: 64]) * g_d;
    m5_m  <= $signed(din_dsp[ 95: 80]) * g_d;
    m6_m  <= $signed(din_dsp[111: 96]) * g_d;
    m7_m  <= $signed(din_dsp[127:112]) * g_d;
    m8_m  <= $signed(din_dsp[143:128]) * g_d;
    m9_m  <= $signed(din_dsp[159:144]) * g_d;
    m10_m <= $signed(din_dsp[175:160]) * g_d;
    m11_m <= $signed(din_dsp[191:176]) * g_d;
    m12_m <= $signed(din_dsp[207:192]) * g_d;
    m13_m <= $signed(din_dsp[223:208]) * g_d;
    m14_m <= $signed(din_dsp[239:224]) * g_d;
    m15_m <= $signed(din_dsp[255:240]) * g_d;

    m0  <= m0_m;                         // 路 0：MREG→PREG
    m1  <= m1_m;                         // 路 1：MREG→PREG
    m2  <= m2_m;                         // 路 2：MREG→PREG
    m3  <= m3_m;                         // 路 3：MREG→PREG
    m4  <= m4_m;                         // 路 4：MREG→PREG
    m5  <= m5_m;                         // 路 5：MREG→PREG
    m6  <= m6_m;                         // 路 6：MREG→PREG
    m7  <= m7_m;                         // 路 7：MREG→PREG
    m8  <= m8_m;                         // 路 8：MREG→PREG
    m9  <= m9_m;                         // 路 9：MREG→PREG
    m10 <= m10_m;                        // 路 10：MREG→PREG
    m11 <= m11_m;                        // 路 11：MREG→PREG
    m12 <= m12_m;                        // 路 12：MREG→PREG
    m13 <= m13_m;                        // 路 13：MREG→PREG
    m14 <= m14_m;                        // 路 14：MREG→PREG
    m15 <= m15_m;                        // 路 15：MREG→PREG

end


// =============================================================================
// 控制、直通数据和 valid 对齐流水
// on_d2=0 选择未缩放的 din_d2；on_d2=1 选择 PREG 乘积 [23:8]。
// 两条支路延迟相同，因此运行中切换扫描开关不会造成样点相位跳拍。
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        on_d       <= 1'b0;              // 扫描开关拍 1 清零
        on_d1      <= 1'b0;              // 扫描开关拍 2 清零
        on_d2      <= 1'b0;              // 扫描开关拍 3 清零
        din_d      <= 256'd0;            // 正弦拍 1 清零
        din_dsp    <= 256'd0;
        g_d        <= 9'sd0;
        din_d1     <= 256'd0;            // 正弦拍 2 清零
        din_d2     <= 256'd0;            // 正弦拍 3 清零
        v_d        <= 1'b0;              // 有效拍 1 清零
        v_d1       <= 1'b0;              // 有效拍 2 清零
        v_d2       <= 1'b0;              // 有效拍 3 清零
        dout       <= 256'd0;            // 输出样点清零
        dout_valid <= 1'b0;              // 输出有效清零

    end else begin

        din_d   <= din;                  // 正弦打拍 1，直通对齐
        din_dsp <= din;                  // DSP A 口专用，与 din_d 同拍
        g_d     <= {1'b0, g};            // 明确补零后再按 signed 乘，防止 G[7]=1 被当负数
        din_d1  <= din_d;                // 正弦打拍 2
        din_d2  <= din_d1;               // 正弦打拍 3，与 PREG 对齐

        on_d   <= on;                    // 扫描开关打拍 1
        on_d1  <= on_d;                  // 扫描开关打拍 2
        on_d2  <= on_d1;                 // 扫描开关打拍 3，选通直通/乘法

        v_d    <= din_valid;             // 有效打拍 1
        v_d1   <= v_d;                   // 有效打拍 2
        v_d2   <= v_d1;                  // 有效打拍 3

        dout_valid <= v_d2;              // 与 dout 同一拍出有效

        if (!on_d2)
            dout <= din_d2;              // 关扫描：直通对齐，避免开关错拍
        else
            dout <= {                    // 开扫描：逐路取乘积 [23:8]，再按 lane15..0 拼回
                m15[23:8], m14[23:8], m13[23:8], m12[23:8],
                m11[23:8], m10[23:8], m9[23:8],  m8[23:8],
                m7[23:8],  m6[23:8],  m5[23:8],  m4[23:8],
                m3[23:8],  m2[23:8],  m1[23:8],  m0[23:8]
            };

    end

end

endmodule
