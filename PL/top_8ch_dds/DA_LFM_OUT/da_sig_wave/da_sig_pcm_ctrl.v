// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_pcm_ctrl.v
// 功能：依据码元宽度和码长推进码索引，并提供随机码元及 P3/P4 对应控制。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_pcm_ctrl
// 功能: 按 nc 控制码元驻留时间，并为 BPSK/QPSK/P3/P4 产生当前码元编号。
// 时间基: 码元计数只在 tick 且 pulse_gate 有效时推进；非 CW 的新脉冲重抽码元。
// 随机性: 使用独立 LFSR，经乘高映射到 [0,N-1]，避免取模除法长路径。
// =============================================================================

module da_sig_pcm_ctrl (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        start_edge,       // 启动沿；chip/sub 重装，不清 LFSR
    input  wire        tick,             // 码元计时基准
    input  wire        pulse_gate,       // 脉宽门；只在脉冲内推进
    input  wire        pulse_rise,       // 新脉冲开始
    input  wire        cw_en,            // 连续波；1 时 pulse_rise 不重置码
    input  wire        enable,           // PM 使能
    input  wire [1:0]  pm_type,          // 编码类型
    input  wire [15:0] nc,               // 一个码元持续多少 tick
    input  wire [15:0] ncode,            // 一帧码长
    input  wire [8:0]  n_pn,             // P3/P4 分档后的码长
    output reg  [15:0] chip,             // 当前码元编号 m，随机 ∈ [0, N)
    output reg  [1:0]  pm_r              // 打拍后的 pm_type
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



reg [15:0] nc_use;                       // 实际码元宽度，最小 1
reg [15:0] ncode_use;                    // 实际码长 N
reg [15:0] ncode_r;                      // 锁存 N，送给随机乘
reg [15:0] sub;                          // 当前码元内部已经走过的 tick

wire [15:0] pcm_lfsr;                    // 16 bit 伪随机，独立种子
(* use_dsp = "yes" *)
reg [31:0]  chip_prod;                   // lfsr * N，DSP MREG
reg [31:0]  chip_prod_p;                 // 再打一拍 PREG
wire [15:0] chip_rand = chip_prod_p[31:16];
                                         // (lfsr*N)>>16 → 0..N-1

wire p34 =                               // 是否为 P3/P4
    (pm_type == PM_P3) ||
    (pm_type == PM_P4);


da_sig_lfsr16 u_lfsr (

    .clk      (clk),                     // 工作时钟
    .rst_sync (rst_sync),                // 只在复位装种子，启动不打断序列
    .seed     (16'hC0DE),                // 与 PRI/hop 种子错开
    .lfsr     (pcm_lfsr)                 // 每拍左移 1 bit

);


always @(posedge clk) begin

    ncode_r     <= ncode_use;            // N 打拍后再乘，避开组合
    chip_prod   <= pcm_lfsr * ncode_r;   // 16×16，高 16 bit 落在 0..N-1
    chip_prod_p <= chip_prod;            // 拆开 DSP，368 MHz 留余量

end


always @(posedge clk) begin

    if (rst_sync) begin

        sub       <= 16'd0;              // 码元内部计时清零
        chip      <= 16'd0;              // 复位时 m=0，启动后再抽随机
        nc_use    <= 16'd1;
        ncode_use <= 16'd16;
        pm_r      <= 2'd0;

    end else if (!enable) begin

        sub  <= 16'd0;                   // 未使能时不保留码元位置
        chip <= 16'd0;
        nc_use <= (nc == 16'd0) ?
                  16'd1 :                // nc=0 非法，最小 1
                  nc;
        ncode_use <= p34 ?
                     {7'd0, n_pn} :      // P3/P4 用分档结果
                     16'd2;              // 普通 PM 预置 2
        pm_r <= pm_type;                 // 提前锁类型，便于使能后立刻对齐

    end else begin

        nc_use <= (nc == 16'd0) ? 16'd1 : nc;
        pm_r   <= pm_type;               // 类型打一拍

        if (p34)
            ncode_use <= {7'd0, n_pn};   // P3/P4：16/32/64/128/256
        else if (ncode < 16'd2)
            ncode_use <= 16'd2;          // 普通 PM 最少 2 个码元
        else if (ncode > 16'd1024)
            ncode_use <= 16'd1024;       // 普通 PM 最多 1024
        else
            ncode_use <= ncode;


        if (start_edge || (pulse_rise && !cw_en)) begin

            sub  <= 16'd0;               // 新一发 / 启动：重装码元计时
            chip <= chip_rand;           // 随机抽一个 m，不再固定从 0 起

        end else if (tick && pulse_gate) begin

            if ((sub + 16'd1) >= nc_use) begin
                sub  <= 16'd0;           // 当前码元宽度已满
                chip <= chip_rand;       // 下一码元随机播放，不按 0,1,2… 顺序
            end else
                sub <= sub + 16'd1;      // 当前码元还没结束

        end

    end

end

endmodule
