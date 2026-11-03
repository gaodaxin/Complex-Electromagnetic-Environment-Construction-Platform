// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_pcm_n.v
// 功能：装配码元控制器和 BPSK/QPSK/P3/P4 相位映射，输出要叠加到载波的相位字。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_pcm_n
// 功能: 相位编码顶层，组合码元控制器与 BPSK/QPSK/P3/P4 相位公式模块。
// 输出: phi_code 为叠加到载波相位上的 32 bit 周相位字，未使能时恒为 0。
// =============================================================================

module da_sig_pcm_n (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        start_edge,       // 启动沿；码元改抽随机 m
    input  wire        tick,             // 码元宽度计数时间基准
    input  wire        pulse_gate,       // 脉宽门；只在脉冲内推进码元
    input  wire        pulse_rise,       // 新脉冲开始；非 CW 时重抽随机 m
    input  wire        cw_en,            // 连续波；1 时 pulse_rise 不重置码
    input  wire        enable,           // 当前为 PM 时使能
    input  wire [1:0]  pm_type,          // 0 二相 / 1 四相 / 2 P3 / 3 P4
    input  wire [15:0] nc,               // 一个码元持续多少 tick
    input  wire [15:0] ncode,            // 一帧码长
    output reg  [31:0] phi_code          // 当前码元对应的 32 bit 附加相位
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



wire [15:0] chip;                        // 当前码元编号 m
wire [1:0]  pm_r;                        // 打拍后的 PM 类型
wire [31:0] phi_bi;                      // 二相：0 或 π
wire [31:0] phi_q;                       // 四相：m·π/2
wire [31:0] phi_pn;                      // P3/P4 多相码
wire [8:0]  n_pn;                        // P3/P4 分档后的码长


da_sig_pm_biphase u_bi (

    .chip (chip),                        // 码元编号
    .phi  (phi_bi)                       // 二相相位字

);

da_sig_pm_quad u_q (

    .chip (chip),                        // 码元编号
    .phi  (phi_q)                        // 四相相位字

);

da_sig_pm_pn u_pn (

    .clk     (clk),                      // 工作时钟
    .chip    (chip),                     // 当前码元 m
    .n_host  (ncode),                    // 上位机码长，内部靠档
    .is_p4   (pm_type == PM_P4),         // 1=P4，0=P3
    .n_use   (n_pn),                     // 实际采用的 N
    .phi     (phi_pn)                    // P3/P4 相位字

);

da_sig_pcm_ctrl u_ctrl (

    .clk         (clk),                  // 工作时钟
    .rst_sync    (rst_sync),             // 同步复位
    .start_edge  (start_edge),           // 启动
    .tick        (tick),                 // 码元计时
    .pulse_gate  (pulse_gate),           // 只在脉内推进
    .pulse_rise  (pulse_rise),           // 新脉冲可重置 chip
    .cw_en       (cw_en),                // CW 不因 PRI 复位码
    .enable      (enable),               // PM 总使能
    .pm_type     (pm_type),              // 编码类型
    .nc          (nc),                   // 码元宽度
    .ncode       (ncode),                // 码长
    .n_pn        (n_pn),                 // P3/P4 分档码长
    .chip        (chip),                 // 输出当前 m
    .pm_r        (pm_r)                  // 打拍后的类型

);


always @(posedge clk) begin

    if (rst_sync || start_edge || !enable)
        phi_code <= 32'd0;               // 非 PM / 复位 / 启动时不加相位
    else begin
        case (pm_r)
            PM_QUAD: phi_code <= phi_q;  // 四相
            PM_P3:   phi_code <= phi_pn; // P3
            PM_P4:   phi_code <= phi_pn; // P4
            default: phi_code <= phi_bi; // 默认二相
        endcase
    end

end

endmodule
