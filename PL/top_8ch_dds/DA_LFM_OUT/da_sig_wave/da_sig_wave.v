// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_wave.v
// 功能：组合载波、频率/相位表、CW/LFM/NLFM/PCM 调制与相位累加，选择每路最终并行相位。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// da_sig_wave：波形频率/相位合成
// 功能：F_inst = carrier + F_mod；φ 累加在 da_sig_accum。
// 本文件只装配：carrier、CW/LFM/NLFM、PCM、相位表、累加器。
// =============================================================================
module da_sig_wave (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        running,          // 波形运行使能
    input  wire        start_edge,       // 启动脉冲，一般保持 1 个 clk
    input  wire        tick,             // PRI 时间基准，捷变/码元只在 tick 上走

    input  wire        pulse_rise,       // 新脉冲开始事件，脉内 hop/LFM/码元重扫
    input  wire        period_rise,      // 新 PRI 周期事件，脉间 hop 用
    input  wire        group_rise,       // 频率捷变脉组切换事件
    input  wire        wave_ready,       // PRI 时间轴已经正式启动
    input  wire        pulse_gate,       // 脉冲有效门控
    input  wire        cw_en,            // CW 连续波使能，1 时门控恒开

    input  wire [1:0]  sig_type,         // 信号类型：0 CW / 1 FM / 2 PM
    input  wire [1:0]  freq_type,        // 0 单载频 / 1 脉内 / 2 脉间 / 3 脉组
    input  wire [1:0]  fm_pm_type,       // FM 时为 fm_type，PM 时为 pm_type
    input  wire [1:0]  nlfm_type,        // NLFM 形状：0 三角 / 1 正弦 / 2 锯齿

    input  wire [5:0]  hop_n,            // 捷变频点数 N，限制为 1~32
    input  wire [15:0] hop_ticks,        // 脉内每点持续 tick 数
    input  wire [31:0] hop_start,        // 捷变起点 FTW，单载频时就是 fc
    input  wire [31:0] hop_step,         // 相邻 hop 间隔 FTW
    input  wire        hop_rand,         // 1=随机抽点，0=按序号顺序跳
    input  wire        freq_tab_en,      // 1=查频表，0=算术 hop_start+j*step
    input  wire [1023:0] freq_tab_bus,   // 频表总线，32 个 × 32 bit FTW

    input  wire        phase_tab_en,     // 1=叠加 hop 相位表，0=附加相位为 0
    input  wire [1023:0] phase_tab_bus,  // 相位表总线，32 个 × 32 bit

    input wire signed [63:0] lfm_slope_q32, lfm_start_q32,
    input  wire signed [31:0] slope_word, // 每样点斜率 S，三角 NLFM 在本模块乘 2
    input  wire [31:0] n_sample,         // 一次扫频总样点数，48×lfm_ticks
    input  wire signed [31:0] fm_start,  // 调制起点频偏 ±B/2
    input  wire [31:0] phase_step,       // 正弦 NLFM：2^32/N_sample

    input  wire [15:0] nc,               // 一个调相码元持续多少 tick
    input  wire [15:0] ncode,            // 一帧调相码长
    input  wire [31:0] phase_init,       // DDS 初相
    input  wire        clear_phase_edge, // 清相沿，只改相位初值不停跑

    output wire        phase_valid,      // 16 路相位是否有效
    output wire        phase_user,       // 与相位对齐的脉宽门
    output wire [31:0] phase_r_0,        // 第 0 路 DDS 相位
    output wire [31:0] phase_r_1,        // 第 1 路 DDS 相位
    output wire [31:0] phase_r_2,        // 第 2 路 DDS 相位
    output wire [31:0] phase_r_3,        // 第 3 路 DDS 相位
    output wire [31:0] phase_r_4,        // 第 4 路 DDS 相位
    output wire [31:0] phase_r_5,        // 第 5 路 DDS 相位
    output wire [31:0] phase_r_6,        // 第 6 路 DDS 相位
    output wire [31:0] phase_r_7,        // 第 7 路 DDS 相位
    output wire [31:0] phase_r_8,        // 第 8 路 DDS 相位
    output wire [31:0] phase_r_9,        // 第 9 路 DDS 相位
    output wire [31:0] phase_r_10,       // 第 10 路 DDS 相位
    output wire [31:0] phase_r_11,       // 第 11 路 DDS 相位
    output wire [31:0] phase_r_12,       // 第 12 路 DDS 相位
    output wire [31:0] phase_r_13,       // 第 13 路 DDS 相位
    output wire [31:0] phase_r_14,       // 第 14 路 DDS 相位
    output wire [31:0] phase_r_15        // 第 15 路 DDS 相位
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



wire        active;                     // accum 活跃，LFM/NLFM 才推进
wire        cfg_busy;                   // accum 恒 0，本模块未再使用
wire signed [31:0] slope_x16;           // 16S，accum 回给 LFM/NLFM 每拍加一次
wire [31:0] carrier_emit;
wire [31:0] carrier;                    // 载波 FTW，不含 F_mod
wire signed [31:0] fm_cw, fm_cw_n;      // CW 频偏，恒 0
wire signed [31:0] fm_lfm, fm_lfm_n;    // LFM 频偏
wire signed [31:0] fm_nlfm, fm_nlfm_n;  // NLFM 频偏
wire [31:0] phi_code;                   // PCM 当前码元附加相位
wire [4:0]  hop_idx;                    // 当前 hop 下标，频表/相位表共用
wire [31:0] phi_hop;                    // 相位表按 hop_idx 读出的附加相位


wire use_nlfm = (sig_type == SIG_FM) && (fm_pm_type == FM_NLFM);
                                        // 非线性调频使能
wire use_precision;
wire use_lfm  = (sig_type == SIG_FM) && !use_nlfm;
                                        // 线性调频使能，FM 且非 NLFM
assign use_precision=use_lfm || sig_type==SIG_CW;
wire use_pcm  = (sig_type == SIG_PM);   // 调相使能
wire need_tri = use_lfm || (use_nlfm && (nlfm_type != NLFM_SINE));
                                        // LFM 或非正弦 NLFM 才需要二次项


reg signed [31:0] slope_use_r;          // 实际送给 accum 的斜率；三角 NLFM 为 2S
reg               need_tri_w_r;         // 二次项选通打一拍
reg        use_nlfm_r, use_lfm_r, use_pcm_r;
                                        // 类型打拍，与子模块输出对齐
reg [1:0]  nlfm_type_r, pm_type_r;      // NLFM/PM 子类型打拍
reg signed [31:0] fm_r;                 // 选中的 F_mod
reg        [31:0] phi_r;                // PCM 相位 + hop 相位
reg signed [31:0] f_sum;                // F_inst = carrier + F_mod


// =============================================================================
// 调制类型锁存与 F_inst 合成
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        slope_use_r  <= 32'sd0;          // 默认斜率 0
        need_tri_w_r <= 1'b0;            // 默认不加二次项
        use_nlfm_r   <= 1'b0;            // 默认非 NLFM
        use_lfm_r    <= 1'b0;            // 默认非 LFM
        use_pcm_r    <= 1'b0;            // 默认非调相
        nlfm_type_r  <= 2'd0;            // 默认三角
        pm_type_r    <= 2'd0;            // 默认二相
        fm_r         <= 32'sd0;          // F_mod 清零
        phi_r        <= 32'd0;           // 附加相位清零
        f_sum        <= 32'sd0;          // 瞬时频率清零

    end else begin

        slope_use_r  <= (use_nlfm && (nlfm_type == NLFM_TRI)) ?
                        (slope_word <<< 1) : slope_word;
                                         // 三角 NLFM 一个周期要走 +B 再 −B
                                         // 每样点用 2S，算术左移保持符号

        need_tri_w_r <= need_tri;        // 二次项选通打拍后进 accum
        use_nlfm_r   <= use_nlfm;        // 与 nlfm 模块输出对齐
        use_lfm_r    <= use_lfm;         // 与 lfm 模块输出对齐
        use_pcm_r    <= use_pcm;         // 与 pcm 模块输出对齐
        nlfm_type_r  <= nlfm_type;       // 锁存 NLFM 形状
        pm_type_r    <= fm_pm_type;      // 锁存调相类型

        if (use_nlfm_r)                  // 用上一拍类型选当前 F_mod
            fm_r <= fm_nlfm;             // NLFM 频偏
        else if (use_lfm_r)
            fm_r <= fm_lfm;              // LFM 频偏
        else
            fm_r <= fm_cw;               // CW / 调相：F_mod = 0

        phi_r <= (use_pcm_r ? phi_code : 32'd0) + phi_hop;
                                         // 调相字与 hop 相位表相加
                                         // 非 PM 时码元相位强制为 0

        f_sum <= $signed(carrier) + fm_r;
                                         // F_inst = carrier + F_mod

    end

end


// =============================================================================
// 载波捷变与 hop 相位表
// =============================================================================

da_sig_carrier u_car (

    .clk          (clk),                 // 工作时钟
    .rst_sync     (rst_sync),            // 同步复位
    .start_edge   (start_edge),          // 启动时装 hop_start / 表第 0 项
    .active       (active),              // accum 在跑才换频
    .tick         (tick),                // 脉内 hop 时间基准
    .pulse_gate   (pulse_gate),          // 脉外不走脉内 hop
    .pulse_rise   (pulse_rise),          // 脉内 hop 回 0
    .period_rise  (period_rise),         // 脉间 hop
    .group_rise   (group_rise),          // 脉组 hop
    .freq_type    (freq_type),           // 0 单载频 / 1 脉内 / 2 脉间 / 3 脉组
    .hop_n        (hop_n),               // 点数 N
    .hop_ticks    (hop_ticks),           // 脉内每点 tick
    .hop_start    (hop_start),           // 算术捷变起点
    .hop_step     (hop_step),            // 算术捷变步长
    .hop_rand     (hop_rand),            // 随机抽点
    .freq_tab_en  (freq_tab_en && (freq_type != 2'd0)),
                                         // 单载频不查表；0x77 粘滞使能不能盖住 0x64 的 fc
    .freq_tab_bus (freq_tab_bus),        // 32 个载波 FTW
    .carrier_emit (carrier_emit),
    .carrier      (carrier),             // 输出当前载波
    .hop_idx      (hop_idx)              // 输出当前 hop 下标

);


da_sig_phase_tab u_ptab (

    .clk           (clk),                // 工作时钟
    .rst_sync      (rst_sync),           // 同步复位
    .en            (phase_tab_en && (freq_type != 2'd0)),
                                         // 单载频不叠加 hop 相位表
    .hop_idx       (hop_idx),            // 与载波 hop 下标对齐
    .phase_tab_bus (phase_tab_bus),      // 32 个相位字
    .phi_hop       (phi_hop)             // 当前 hop 的附加相位

);


// =============================================================================
// CW / LFM / NLFM 频偏
// =============================================================================

da_sig_cw u_cw (

    .fm_mod  (fm_cw),                    // 当前频偏，恒 0
    .fm_next (fm_cw_n)                   // 预览频偏，恒 0

);


da_sig_lfm u_lfm (

    .clk        (clk),                   // 工作时钟
    .rst_sync   (rst_sync),              // 同步复位
    .start_edge (start_edge),            // 启动时回到 fm_start
    .active     (active),                // accum 有效才扫频
    .pulse_rise (pulse_rise),            // 新脉冲重新从 fm_start 开始
    .pulse_gate (pulse_gate),            // 只在脉内继续扫
    .enable     (use_lfm_r),             // 选中 LFM 时为 1
    .fm_start   (fm_start),              // 起点频偏 ±B/2
    .slope_x16  (slope_x16),             // 每拍加 16S
    .n_sample   (n_sample),              // 一次 LFM 总样点数
    .fm_offset  (fm_lfm),                // 当前 LFM 频偏
    .fm_next    (fm_lfm_n)               // 预览，本版等于 fm_offset

);


da_sig_nlfm u_nlfm (

    .clk        (clk),                   // 工作时钟
    .rst_sync   (rst_sync),              // 同步复位
    .start_edge (start_edge),            // 启动时各支路清 0
    .active     (active),                // accum 有效才推进
    .pulse_rise (pulse_rise),            // 新脉冲重新从起点开始
    .pulse_gate (pulse_gate),            // 只在脉内继续
    .enable     (use_nlfm_r),            // 选中 NLFM 时为 1
    .nlfm_type  (nlfm_type_r),           // 0 三角 / 1 正弦 / 2 锯齿
    .fm_start   (fm_start),              // 起点频偏；正弦用绝对值作峰值
    .slope_x16  (slope_x16),             // 每拍频率步进 16S
    .n_sample   (n_sample),              // 一个调制周期总样点数
    .phase_step (phase_step),            // 正弦 NLFM：2^32/N_sample
    .fm_mod     (fm_nlfm),               // 当前选中支路的频偏
    .fm_next    (fm_nlfm_n)              // 预览，本版等于 fm_mod

);


// =============================================================================
// PCM 调相
// =============================================================================

da_sig_pcm_n u_pcm (

    .clk        (clk),                   // 工作时钟
    .rst_sync   (rst_sync),              // 同步复位
    .start_edge (start_edge),            // 启动后码元随机抽 m
    .tick       (tick),                  // 码元宽度计数时间基准
    .pulse_gate (pulse_gate),            // 只在脉内推进码元
    .pulse_rise (pulse_rise),            // 新脉冲；非 CW 时重抽随机 m
    .cw_en      (cw_en),                 // CW 时 pulse_rise 不重置码
    .enable     (use_pcm_r),             // 当前为 PM 时使能
    .pm_type    (pm_type_r),             // 0 二相 / 1 四相 / 2 P3 / 3 P4
    .nc         (nc),                    // 一个码元持续 tick 数
    .ncode      (ncode),                 // 一帧码长
    .phi_code   (phi_code)               // 当前码元 32 bit 附加相位

);


// =============================================================================
// 16 路相位累加：φ_i = φ_base + i·F + i(i-1)/2·S
// =============================================================================

wire old_phase_valid,old_phase_user;
wire [511:0] old_phase_bus,lfm_phase_bus;
wire lfm_phase_valid,lfm_phase_user;
da_sig_accum u_accum (

    .clk              (clk),             // 工作时钟
    .rst_sync         (rst_sync),        // 同步复位
    .start_edge       (start_edge),      // 启动时相位装 phase_init
    .clear_phase_edge (clear_phase_edge),// 清相沿，不停跑
    .running          (running),         // 运行使能
    .wave_ready       (wave_ready),      // PRI 第一 tick 之后才出波
    .pulse_gate       (pulse_gate),      // 脉宽门；脉间不推进相位基
    .need_tri         (need_tri_w_r),    // 二次项选通，已打拍
    .slope_word       (slope_use_r),     // 每样点斜率；三角 NLFM 已乘 2
    .phi_code         (phi_r),           // 调相 + hop 相位
    .phase_init       (phase_init),      // 初相
    .f_next           (f_sum),           // 下一拍瞬时频率 F_inst
    .f_idle           (f_sum),           // 空闲时仍跟上 F_inst，避免再启动跳频
    .active           (active),          // 回给 LFM/NLFM/carrier
    .cfg_busy         (cfg_busy),        // 恒 0
    .phase_valid      (old_phase_valid),     // 16 路相位有效
    .phase_user       (old_phase_user),      // 与相位对齐的门控
    .slope_x16        (slope_x16),       // 16S，回给 LFM/NLFM
    .phase_r_0        (old_phase_bus[0+:32]),       // 第 0 路
    .phase_r_1        (old_phase_bus[32+:32]),       // 第 1 路
    .phase_r_2        (old_phase_bus[64+:32]),       // 第 2 路
    .phase_r_3        (old_phase_bus[96+:32]),       // 第 3 路
    .phase_r_4        (old_phase_bus[128+:32]),       // 第 4 路
    .phase_r_5        (old_phase_bus[160+:32]),       // 第 5 路
    .phase_r_6        (old_phase_bus[192+:32]),       // 第 6 路
    .phase_r_7        (old_phase_bus[224+:32]),       // 第 7 路
    .phase_r_8        (old_phase_bus[256+:32]),       // 第 8 路
    .phase_r_9        (old_phase_bus[288+:32]),       // 第 9 路
    .phase_r_10       (old_phase_bus[320+:32]),      // 第 10 路
    .phase_r_11       (old_phase_bus[352+:32]),      // 第 11 路
    .phase_r_12       (old_phase_bus[384+:32]),      // 第 12 路
    .phase_r_13       (old_phase_bus[416+:32]),      // 第 13 路
    .phase_r_14       (old_phase_bus[448+:32]),      // 第 14 路
    .phase_r_15       (old_phase_bus[480+:32])       // 第 15 路

);

da_sig_lfm_q32 u_lfm_q32(
 .clk(clk),.rst_sync(rst_sync),.start_edge(start_edge),.clear_phase_edge(clear_phase_edge),
 .running(running && use_precision),.wave_ready(wave_ready),.pulse_gate(pulse_gate),.pulse_rise(pulse_rise),
 .carrier(carrier_emit),.phase_init(phase_init),.n_sample(n_sample),
 .slope_q32(use_lfm?lfm_slope_q32:64'd0),.fm_start_q32(use_lfm?lfm_start_q32:64'd0),
 .phase_bus(lfm_phase_bus),.phase_valid(lfm_phase_valid),.phase_user(lfm_phase_user));
assign phase_valid=use_precision?lfm_phase_valid:old_phase_valid;
assign phase_user=use_precision?lfm_phase_user:old_phase_user;
assign phase_r_0=use_precision?lfm_phase_bus[0+:32]:old_phase_bus[0+:32];
assign phase_r_1=use_precision?lfm_phase_bus[32+:32]:old_phase_bus[32+:32];
assign phase_r_2=use_precision?lfm_phase_bus[64+:32]:old_phase_bus[64+:32];
assign phase_r_3=use_precision?lfm_phase_bus[96+:32]:old_phase_bus[96+:32];
assign phase_r_4=use_precision?lfm_phase_bus[128+:32]:old_phase_bus[128+:32];
assign phase_r_5=use_precision?lfm_phase_bus[160+:32]:old_phase_bus[160+:32];
assign phase_r_6=use_precision?lfm_phase_bus[192+:32]:old_phase_bus[192+:32];
assign phase_r_7=use_precision?lfm_phase_bus[224+:32]:old_phase_bus[224+:32];
assign phase_r_8=use_precision?lfm_phase_bus[256+:32]:old_phase_bus[256+:32];
assign phase_r_9=use_precision?lfm_phase_bus[288+:32]:old_phase_bus[288+:32];
assign phase_r_10=use_precision?lfm_phase_bus[320+:32]:old_phase_bus[320+:32];
assign phase_r_11=use_precision?lfm_phase_bus[352+:32]:old_phase_bus[352+:32];
assign phase_r_12=use_precision?lfm_phase_bus[384+:32]:old_phase_bus[384+:32];
assign phase_r_13=use_precision?lfm_phase_bus[416+:32]:old_phase_bus[416+:32];
assign phase_r_14=use_precision?lfm_phase_bus[448+:32]:old_phase_bus[448+:32];
assign phase_r_15=use_precision?lfm_phase_bus[480+:32]:old_phase_bus[480+:32];

endmodule
