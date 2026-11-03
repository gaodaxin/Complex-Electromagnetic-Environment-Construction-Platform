// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_tick.v
// 功能：每三个波形时钟产生一次时间使能 tick，供 PW、PRI、跳频驻留、码元和扫描计数使用。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_tick
// 功能: 将 300 MHz fabric 时钟按 3 分频产生协议时间基准 tick。
// 用途: PRI、PW、码元、跳频驻留和扫描计数使用 tick；DDS 相位仍每 clk 推进。
// 注意: 当前实际 tick 约 8.14 ns，主机协议参数仍按约定的 10 ns 格量化。
// =============================================================================

module da_sig_tick (

    input  wire       clk,               // FPGA 工作时钟，当前 300 MHz
    input  wire       rst_sync,          // 同步复位，高有效
    input  wire       running,           // 运行使能；0 时冻结计数、tick=0
    input  wire       start_edge,        // 启动沿；计数回 0
    output reg  [1:0] tick_div,          // 格内拍号 0/1/2
    output reg        tick               // 1 拍脉冲，每 3 个 clk 一次
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



// 协议 1 格按 10 ns 填；FPGA 1 tick = 3 clk = 10 ns
// tick 只给 PRI / 脉宽 / 捷变 / 码元边界
// 不推进 DDS 相位，也不做 LFM 斜升

always @(posedge clk) begin

    if (rst_sync || start_edge) begin

        tick_div <= 2'd0;                // 格内计数从 0 开始
        tick     <= 1'b0;                // 复位/启动本拍不产生 tick

    end else if (running) begin

        if (tick_div == TICK_CLKS-1)
            tick_div <= 2'd0;            // 0,1,2 循环，到 2 后回 0
        else
            tick_div <= tick_div + 2'd1;

        tick <= (tick_div == TICK_CLKS-2);
                                         // div==1 的下一拍 tick=1
                                         // 即 div 从 2 回到 0 的那一拍

    end else
        tick <= 1'b0;                    // 停止时 tick 拉低，相位保持

end

endmodule
