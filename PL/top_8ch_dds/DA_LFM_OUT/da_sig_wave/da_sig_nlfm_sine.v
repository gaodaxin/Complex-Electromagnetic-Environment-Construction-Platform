// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_nlfm_sine.v
// 功能：按配置相位步长推进正弦 NLFM 调制，通过正弦查表和幅度运算生成频偏；本分支原 DDS IP 同样替换为纯 RTL 查找表。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_nlfm_sine
// 功能: 通过相位累加器和正弦查表产生正弦型 NLFM 瞬时频偏。
// 时序: 相位、查表地址和幅度乘法分级流水；输出 valid 与频偏严格对齐。
// 单位: phase_step 为 32 bit 相位步进，输出为有符号 FTW 频偏。
// =============================================================================

module da_sig_nlfm_sine (

    input  wire        clk,                  // FPGA 工作时钟
    input  wire        rst_sync,             // 同步复位，高有效
    input  wire        start_edge,           // 启动脉冲，重新清调制相位和输出流水

    input  wire        active,               // 累加器有效运行时，本模块才更新相位/流水
    input  wire        pulse_rise,           // 新脉冲开始事件，每一发正弦 NLFM 重新从 sin(0) 开始
    input  wire        pulse_gate,           // 脉宽门控，只有脉冲有效区内才推进调制相位
    input  wire        enable,               // 正弦 NLFM 使能，0 时最终 fm_mod 输出 0

    input  wire signed [31:0] amp,           // 正弦频偏峰值 FTW，通常 = |fm_start| = B/2
    input  wire [31:0] phase_step,           // 每个 DAC 样点的 32 bit 调制相位步进 ≈ 2^32/N_sample

    output reg  signed [31:0] fm_mod,        // 当前正弦 NLFM 频偏 F_mod
    output wire signed [31:0] fm_next        // 当前频偏预览，本版本直接等于 fm_mod

);


reg [31:0] p;                                // 当前 32 bit 正弦调制相位，2^32 对应完整 2π

reg [11:0] phase_slice;                      // p 最高 12 bit，作为 4096 点 dds_sin_lut 地址

reg        phase_v;                          // LUT 输入 phase_tvalid
reg        phase_en;                         // 跟随相位数据进入 LUT 的使能标记

reg [1:0]  rst_hold;                         // LUT 独立复位保持计数，start/reset 后保持 aresetn 低若干拍

wire       lut_aresetn;                      // DDS LUT 低有效复位，1 表示正常工作

wire [15:0] sine_tdata;                      // LUT 输出的 16 bit 正弦数据
wire        sine_tvalid;                     // LUT 输出数据有效
wire        sine_tuser;                      // LUT 透传的用户标志，即前面的 phase_en

wire signed [15:0] sine_s;                   // 将 LUT 输出解释为 16 bit 有符号正弦值


reg signed [31:0] amp_d0;                    // amp 延迟第 0 级
reg signed [31:0] amp_d1;                    // amp 延迟第 1 级
reg signed [31:0] amp_d2;                    // amp 延迟第 2 级
reg signed [31:0] amp_d3;                    // amp 延迟第 3 级
reg signed [31:0] amp_d4;                    // amp 延迟第 4 级
reg signed [31:0] amp_d5;                    // amp 延迟第 5 级
reg signed [31:0] amp_d6;                    // amp 延迟第 6 级，送 DSP 乘法器


reg en_d0;                                   // enable 延迟第 0 级
reg en_d1;                                   // enable 延迟第 1 级
reg en_d2;                                   // enable 延迟第 2 级
reg en_d3;                                   // enable 延迟第 3 级
reg en_d4;                                   // enable 延迟第 4 级
reg en_d5;                                   // enable 延迟第 5 级
reg en_d6;                                   // enable 延迟第 6 级
reg en_d7;                                   // enable 延迟第 7 级
reg en_d8;                                   // enable 延迟第 8 级，与乘法/右移流水对齐


reg tv_d1;                                   // LUT tvalid 再延迟 1 拍
reg tv_d2;                                   // LUT tvalid 再延迟 2 拍，对齐 DSP 乘积


(* use_dsp = "yes" *)
reg signed [47:0] prod_m;                    // DSP 乘法第 1 级：amp × sine

(* use_dsp = "yes" *)
reg signed [47:0] prod;                      // DSP 乘法第 2 级寄存，提高高频时序性能


assign lut_aresetn = (rst_hold == 2'd0);     // rst_hold 减到 0 后释放 DDS LUT 复位

assign sine_s = sine_tdata;                  // 16 bit 总线按 signed 形式参与乘法

assign fm_next = fm_mod;                     // 当前版本不额外预测，直接输出已寄存的 fm_mod


// =============================================================================
// DSP 独立乘法流水
// 不加 reset、不加 CE，便于推断 DSP 内部 MREG/PREG
// =============================================================================

always @(posedge clk) begin

    prod_m <= amp_d6 * sine_s;               // 32×16 bit 有符号乘法，结果 48 bit

    prod   <= prod_m;                        // 乘积再打一拍，提高 368 MHz 附近时序裕量

end


// =============================================================================
// LUT 复位延时控制
// reset/start 后 rst_hold=3→2→1→0，期间 lut_aresetn=0
// =============================================================================

always @(posedge clk) begin

    if (rst_sync || start_edge)              // 复位或整机重新启动

        rst_hold <= 2'd3;                    // reset/start 后保持 LUT 复位 3 个时钟

    else if (rst_hold != 2'd0)               // 复位保持计数尚未到 0

        rst_hold <= rst_hold - 2'd1;         // 每拍减 1，直到 0 后释放 LUT

end


// =============================================================================
// 正弦 NLFM 主流水
// =============================================================================

always @(posedge clk) begin

    if (rst_sync || start_edge) begin        // 复位或整机重新启动

        p           <= 32'd0;                // 调制相位从 0 开始，即 sin(0)

        phase_slice <= 12'd0;                // LUT 初始地址 0
        phase_v     <= 1'b0;                 // LUT 输入暂时无效
        phase_en    <= 1'b0;                 // LUT 用户使能清零

        fm_mod      <= 32'sd0;               // 启动期间 NLFM 频偏先输出 0


        amp_d0 <= 32'sd0;                    // amp 流水第 0 级清零
        amp_d1 <= 32'sd0;                    // amp 流水第 1 级清零
        amp_d2 <= 32'sd0;                    // amp 流水第 2 级清零
        amp_d3 <= 32'sd0;                    // amp 流水第 3 级清零
        amp_d4 <= 32'sd0;                    // amp 流水第 4 级清零
        amp_d5 <= 32'sd0;                    // amp 流水第 5 级清零
        amp_d6 <= 32'sd0;                    // amp 流水第 6 级清零


        en_d0 <= 1'b0;                       // enable 流水第 0 级清零
        en_d1 <= 1'b0;                       // enable 流水第 1 级清零
        en_d2 <= 1'b0;                       // enable 流水第 2 级清零
        en_d3 <= 1'b0;                       // enable 流水第 3 级清零
        en_d4 <= 1'b0;                       // enable 流水第 4 级清零
        en_d5 <= 1'b0;                       // enable 流水第 5 级清零
        en_d6 <= 1'b0;                       // enable 流水第 6 级清零
        en_d7 <= 1'b0;                       // enable 流水第 7 级清零
        en_d8 <= 1'b0;                       // enable 流水第 8 级清零


        tv_d1 <= 1'b0;                       // LUT valid 后级第 1 拍清零
        tv_d2 <= 1'b0;                       // LUT valid 后级第 2 拍清零

    end


    else if (active) begin                   // 仅在 accum 有效运行时推进整条流水

        if (pulse_rise && enable)            // 新脉冲且使能：相位回 0

            p <= 32'd0;                      // 每一发从 sin(0) 重新开始正弦 NLFM

        else if (pulse_gate && enable)       // 脉冲内部且使能：推进调制相位

            p <= p + (phase_step << 4);      // 一拍跨 16 样点，调制相位一次前进 16×phase_step


        phase_slice <= p[31:20];             // 取更新前旧 p 的高 12 bit，量化成 4096 点地址


        phase_v  <= lut_aresetn;             // LUT 退出复位后，每个 active 拍送一个有效相位地址

        phase_en <= enable;                  // enable 通过 LUT 的 tuser 通道同步传递


        amp_d0 <= amp;                       // amp 与 LUT 多拍延迟对齐，第 0 拍
        amp_d1 <= amp_d0;                    // amp 延迟第 1 拍
        amp_d2 <= amp_d1;                    // amp 延迟第 2 拍
        amp_d3 <= amp_d2;                    // amp 延迟第 3 拍
        amp_d4 <= amp_d3;                    // amp 延迟第 4 拍
        amp_d5 <= amp_d4;                    // amp 延迟第 5 拍
        amp_d6 <= amp_d5;                    // amp 延迟第 6 拍，送 DSP 乘法


        en_d0 <= enable;                     // enable 沿数据通路延迟，第 0 拍
        en_d1 <= en_d0;                      // enable 延迟第 1 拍
        en_d2 <= en_d1;                      // enable 延迟第 2 拍
        en_d3 <= en_d2;                      // enable 延迟第 3 拍
        en_d4 <= en_d3;                      // enable 延迟第 4 拍
        en_d5 <= en_d4;                      // enable 延迟第 5 拍
        en_d6 <= en_d5;                      // enable 延迟第 6 拍
        en_d7 <= en_d6;                      // enable 延迟第 7 拍
        en_d8 <= en_d7;                      // enable 延迟第 8 拍，判断当前 prod 是否有效


        tv_d1 <= sine_tvalid;                // LUT 输出 valid 再延迟，对齐 DSP 乘法第 1 级
        tv_d2 <= tv_d1;                      // 再延迟一拍，对齐 DSP 乘法第 2 级 prod


        fm_mod <= (en_d8 && tv_d2) ?         // 使能与 LUT valid 都对齐后才出频偏
                  (prod >>> 15) :            // Q1.15：amp×sin(θ)×2^15 再算术右移 15
                  32'sd0;                    // 无效时频偏强制为 0

    end

end


// =============================================================================
// 4096 点正弦 DDS LUT
// =============================================================================

vu13p_sine_lut u_nlfm_sin_lut (

    .aclk                   (clk),                           // DDS LUT 工作时钟

    .aresetn                (lut_aresetn),                  // DDS LUT 低有效复位

    .s_axis_phase_tvalid    (phase_v),                      // 输入相位地址有效
    .s_axis_phase_tdata     ({4'b0000, phase_slice}),       // 12 bit 地址扩展成 16 bit 输入
    .s_axis_phase_tuser     (phase_en),                     // 携带 enable 标记

    .m_axis_data_tvalid     (sine_tvalid),                  // LUT 正弦输出有效
    .m_axis_data_tdata      (sine_tdata),                   // 16 bit 正弦值
    .m_axis_data_tuser      (sine_tuser)                    // 输出对应的 user 标记

);

endmodule
