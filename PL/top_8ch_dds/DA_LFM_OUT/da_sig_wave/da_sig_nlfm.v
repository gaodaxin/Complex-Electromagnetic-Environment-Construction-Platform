// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_nlfm.v
// 功能：装配三角、正弦和锯齿 NLFM 频偏支路，对齐支路延迟后选择输出。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_nlfm
// 功能: NLFM 顶层选择器，统一装配三角、正弦和锯齿三种频偏发生器。
// 时序: 对各支路延迟做对齐后再选择，避免切换类型造成 valid 与数据错拍。
// =============================================================================

module da_sig_nlfm (

    input  wire        clk,                  // FPGA 工作时钟
    input  wire        rst_sync,             // 同步复位，高有效
    input  wire        start_edge,           // 启动脉冲，一般保持 1 个 clk；本拍输出先清 0

    input  wire        active,               // 相位累加器有效运行时，NLFM 子模块才推进内部状态
    input  wire        pulse_rise,           // 新脉冲开始事件，每一发重新从 NLFM 起点开始
    input  wire        pulse_gate,           // 脉宽门控，1 时位于脉冲内部，子模块才继续更新频偏
    input  wire        enable,               // 总 NLFM 使能，再根据 nlfm_type 拆成三路独立 enable

    input  wire [1:0]  nlfm_type,            // NLFM 类型：0 三角 / 1 正弦 / 2 锯齿

    input  wire signed [31:0] fm_start,      // 扫频起始频偏 FTW；正扫通常 -B/2，负扫通常 +B/2
                                             // 三角/锯齿直接作为起点；正弦取 |fm_start| 作为峰值 amp

    input  wire signed [31:0] slope_x16,     // 每个 fabric 拍的频率步进 = 16*S
                                             // 三角 NLFM 若需要 2S，已在上级 da_sig_wave 中提前调整

    input  wire [31:0] n_sample,             // 一个完整 NLFM 调制周期的总 DAC 样点数

    input  wire [31:0] phase_step,           // 正弦 NLFM 专用相位步进 ≈ 2^32/n_sample
                                             // 三角/锯齿支路不使用

    output wire signed [31:0] fm_mod,        // 当前选中 NLFM 支路的频率偏移 FTW
    output wire signed [31:0] fm_next        // 当前版本等于 fm_mod，不是真正提前预测下一值

);


// BEGIN INLINE HEADER: da_sig_common.vh (module内局部作用域)
// =============================================================================
// 文件: da_sig_common.vh
// 功能: 新链时钟/位宽/协议枚举和 Hz→FTW 对照函数。
// 用法: 必须在 module 内部 `include（Verilog 函数不能放在 module 外）。
// 约定:
//   目标板：Fs=4.8 GSPS，fabric=300 MHz，16 lane
//   协议 1 格 = 10 ns；FPGA 1 tick = 3 clk = 10 ns
//   600 MHz 只用于 ADC 8 样点并位，DSP 不放在 600 MHz
//   频率范围 0 ~ Fs。上位机把 Hz 换成 32-bit FTW 再下发：
//     FTW = floor(f_Hz * 2^32 / Fs)，不减 3.6 GHz
//   FPGA 只在 FTW 域做 hop 步长 / LFM 斜率的整数除法，不再除 Fs
// 关联:
//   da_sig_tick        TICK_CLKS
//   da_sig_cfg_calc    ERR_*、PW_MAX_TICK、FTW / Fs 常数
//   da_sig_wave        SIG_* / FM_* / PM_* / NLFM_*
//   da_sig_pcm_ctrl    PM_P3 / PM_P4
// =============================================================================

// -----------------------------------------------------------------------------
// 时钟网格：1 tick = TICK_CLKS 个 fabric clk = LANE_N*TICK_CLKS 个 DAC 样点
// -----------------------------------------------------------------------------
localparam integer TICK_CLKS     = 3;            // 1 tick 的 clk 数；tick 脉冲每 3 拍一次
localparam integer LANE_N        = 16;           // 每拍并行样点数（RFSoC DAC lane）
localparam integer SAMP_PER_TICK = 48;           // 16*3，一格里的样点数
localparam integer DA_SIG_CLK_HZ = 300000000;    // fabric 300 MHz；DSP 不要放到 600 MHz
localparam integer DA_SIG_F_TICK = 100000000;    // clk/3 = 100 MHz；T_scan = round(F_TICK*360/ω)
localparam integer MAX_HOP       = 32;           // 捷变频表 / 相位表项数上限
localparam integer TICK_W        = 20;           // PW / PRI / LFM 时长位宽（tick）
localparam integer FTW_W         = 32;           // 频率字位宽；0x64/0x65/0x69/0x77 低 32 bit

localparam [63:0] DA_SIG_FS_HZ   = 64'd4800000000; // DAC 采样率 4.8 GSPS
localparam [63:0] DA_SIG_POW2_32 = 64'd4294967296; // 2^32，Hz→FTW 分子

// -----------------------------------------------------------------------------
// cfg_calc 合法性窗口（协议 10 ns 格）。寄存器仍是 TICK_W bit，这里是检查上限
// -----------------------------------------------------------------------------
localparam integer PW_MIN_TICK   = 10;           // 脉宽下限 0.1 us
localparam integer PW_MAX_TICK   = 200000;       // 脉宽/LFM 时长检查上限 2 ms
localparam integer PRI_MAX_TICK  = 1000000;      // PRI 检查上限 10 ms

// -----------------------------------------------------------------------------
// cfg_calc 出错码，非法只报错、不改用户数
// -----------------------------------------------------------------------------
localparam [7:0] ERR_OK           = 8'd0;        // 无错
localparam [7:0] ERR_PW_RANGE     = 8'd1;        // 脉宽超 PW_MIN/MAX
localparam [7:0] ERR_PRI_LT_PW    = 8'd2;        // PRI ≤ 脉宽
localparam [7:0] ERR_LFM_TIME     = 8'd3;        // LFM 时长为 0 或超过 PW_MAX_TICK
localparam [7:0] ERR_FREQ_RANGE   = 8'd4;        // 频率越界
localparam [7:0] ERR_SLOPE        = 8'd5;        // 调频斜率超协议 40 MHz/us
localparam [7:0] ERR_HOP          = 8'd6;        // 捷变参数非法

// -----------------------------------------------------------------------------
// 0x61[1:0] sig_type、0x62 调制子类型
// -----------------------------------------------------------------------------
localparam [1:0] SIG_CW           = 2'd0;        // 常规脉冲 / 定频
localparam [1:0] SIG_FM           = 2'd1;        // 调频
localparam [1:0] SIG_PM           = 2'd2;        // 调相
localparam [1:0] FM_LFM           = 2'd0;        // 0x62[1:0] FM 时：线性
localparam [1:0] FM_NLFM          = 2'd1;        // 0x62[1:0] FM 时：非线性
localparam [1:0] NLFM_TRI         = 2'd0;        // 0x62[4:3]：三角
localparam [1:0] NLFM_SINE        = 2'd1;        // 0x62[4:3]：正弦
localparam [1:0] NLFM_SAW         = 2'd2;        // 0x62[4:3]：锯齿
localparam [1:0] PM_BIPHASE       = 2'd0;        // 0x62[1:0] PM 时：二相
localparam [1:0] PM_QUAD          = 2'd1;        // 四相
localparam [1:0] PM_P3            = 2'd2;        // P3
localparam [1:0] PM_P4            = 2'd3;        // P4

// Hz → 32-bit FTW。组合对照用：上位机与 TB 核对；RTL 出波路径不再调用
// hz≥Fs 钳到全 1；否则 floor(hz * 2^32 / Fs)，与 host_sig_gen.hz_to_ftw 相同
function [31:0] da_sig_hz_to_ftw;
    input [47:0] hz;                             // 输入频率，单位 Hz
    reg [95:0] num;                              // hz * 2^32，96 bit 防溢出
    begin
        if (hz >= DA_SIG_FS_HZ[47:0])
            da_sig_hz_to_ftw = 32'hFFFF_FFFF;    // 达到或超过 Fs，FTW 饱和
        else begin
            num = {48'd0, hz} * {32'd0, DA_SIG_POW2_32};
            da_sig_hz_to_ftw = num[95:0] / DA_SIG_FS_HZ;
        end
    end
endfunction

// END INLINE HEADER: da_sig_common.vh



wire signed [31:0] fm_tri;                   // 三角 NLFM 当前频偏
wire signed [31:0] fm_tri_n;                 // 三角 NLFM 预览/next 接口

wire signed [31:0] fm_sin;                   // 正弦 NLFM 当前频偏
wire signed [31:0] fm_sin_n;                 // 正弦 NLFM 预览/next 接口

wire signed [31:0] fm_saw;                   // 锯齿 NLFM 当前频偏
wire signed [31:0] fm_saw_n;                 // 锯齿 NLFM 预览/next 接口


reg signed [31:0] amp_r;                     // |fm_start| 打一拍后送给正弦 NLFM 作为频偏峰值
reg signed [31:0] fm_mod_r;                  // 三路 NLFM 三选一后的最终输出频偏

reg [1:0] typ_r;                             // nlfm_type 打一拍，与子模块输出流水对齐

reg en_tri;                                  // 三角 NLFM 子模块使能
reg en_sin;                                  // 正弦 NLFM 子模块使能
reg en_saw;                                  // 锯齿 NLFM 子模块使能


wire signed [31:0] amp =                     // 正弦 NLFM 频偏峰值 = |fm_start|
        (fm_start[31]) ?
        -fm_start :                          // 负起始频偏时取相反数
         fm_start;                           // 正起始频偏直接作为幅度


da_sig_nlfm_tri u_tri (

    .clk         (clk),                      // 工作时钟
    .rst_sync    (rst_sync),                 // 同步复位
    .start_edge  (start_edge),               // 全局启动时内部重新初始化

    .active      (active),                   // 只有相位引擎运行时才推进
    .pulse_rise  (pulse_rise),               // 每个脉冲重新开始一次三角扫描
    .pulse_gate  (pulse_gate),               // 脉冲内部才更新频偏

    .enable      (en_tri),                   // nlfm_type=三角时使能

    .fm_start    (fm_start),                 // 三角扫描起始频偏
    .slope_x16   (slope_x16),                // 每拍频率增量
    .n_sample    (n_sample),                 // 一个完整三角调制周期总样点数

    .fm_mod      (fm_tri),                   // 当前三角 NLFM 频偏
    .fm_next     (fm_tri_n)                  // 三角支路 next/预览接口

);


da_sig_nlfm_sine u_sine (

    .clk         (clk),                      // 工作时钟
    .rst_sync    (rst_sync),                 // 同步复位
    .start_edge  (start_edge),               // 启动时内部正弦相位重新开始

    .active      (active),                   // 相位引擎有效时才运行
    .pulse_rise  (pulse_rise),               // 每一发重新开始一个正弦 NLFM 周期
    .pulse_gate  (pulse_gate),               // 只在有效脉冲内更新

    .enable      (en_sin),                   // nlfm_type=正弦时使能

    .amp         (amp_r),                    // 正弦频偏峰值 = |fm_start|
    .phase_step  (phase_step),               // 每样点调制相位步进

    .fm_mod      (fm_sin),                   // 当前正弦 NLFM 频偏
    .fm_next     (fm_sin_n)                  // 正弦支路 next/预览接口

);


da_sig_nlfm_saw u_saw (

    .clk         (clk),                      // 工作时钟
    .rst_sync    (rst_sync),                 // 同步复位
    .start_edge  (start_edge),               // 启动时重新装载起始频偏

    .active      (active),                   // 相位引擎有效时才运行
    .pulse_rise  (pulse_rise),               // 每一发重新从 fm_start 开始
    .pulse_gate  (pulse_gate),               // 只在有效脉宽内扫频

    .enable      (en_saw),                   // nlfm_type=锯齿时使能

    .fm_start    (fm_start),                 // 锯齿扫描起始频偏
    .slope_x16   (slope_x16),                // 每 fabric 拍频率步进
    .n_sample    (n_sample),                 // 一个完整锯齿扫描周期样点数

    .fm_mod      (fm_saw),                   // 当前锯齿 NLFM 频偏
    .fm_next     (fm_saw_n)                  // 锯齿支路 next/预览接口

);


// =============================================================================
// 类型选择和输出流水
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin                      // 同步复位

        amp_r     <= 32'sd0;                 // 正弦频偏幅度清零
        fm_mod_r  <= 32'sd0;                 // 最终 NLFM 频偏清零

        typ_r     <= 2'd0;                   // 默认类型置 0，即三角类型编码

        en_tri    <= 1'b0;                   // 关闭三角 NLFM 子模块
        en_sin    <= 1'b0;                   // 关闭正弦 NLFM 子模块
        en_saw    <= 1'b0;                   // 关闭锯齿 NLFM 子模块

    end

    else if (start_edge) begin               // 全局启动沿

        fm_mod_r <= 32'sd0;                  // 启动本拍输出先清 0，等待子模块重新装载

    end

    else begin                               // 正常工作

        amp_r <= amp;                        // |fm_start| 打一拍后送正弦 NLFM

        typ_r <= nlfm_type;                  // 类型打一拍，与子模块输出数据对齐


        en_tri <= enable &&                  // enable=1 且类型为三角时打开三角支路
                  (nlfm_type == NLFM_TRI);   // 三角 NLFM 使能

        en_sin <= enable &&                  // enable=1 且类型为正弦时打开正弦支路
                  (nlfm_type == NLFM_SINE);  // 正弦 NLFM 使能

        en_saw <= enable &&                  // enable=1 且类型为锯齿时打开锯齿支路
                  (nlfm_type == NLFM_SAW);   // 锯齿 NLFM 使能


        if (typ_r == NLFM_SINE)              // 使用已打拍类型，与子模块流水对齐

            fm_mod_r <= fm_sin;              // 选择正弦 NLFM 频偏

        else if (typ_r == NLFM_SAW)          // 锯齿类型

            fm_mod_r <= fm_saw;              // 选择锯齿 NLFM 频偏

        else                                 // 其余情况默认三角

            fm_mod_r <= fm_tri;              // 默认选择三角 NLFM 频偏

    end

end


assign fm_mod = fm_mod_r;                    // 当前最终 NLFM 频率偏移 FTW

assign fm_next = fm_mod_r;                   // 当前版本 fm_next 只是 fm_mod 副本，并未提前预测下一值


endmodule
