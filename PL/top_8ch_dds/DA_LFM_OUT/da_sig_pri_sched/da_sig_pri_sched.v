// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_pri_sched.v
// 功能：统一调度固定、脉间参差、脉组参差、抖动和滑变 PRI，并输出脉冲开始、周期和组边界事件。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_pri_sched
// 功能: 统一调度固定、脉间参差、脉组参差、抖动和滑变 PRI。
// 输出事件: pulse_rise 表示脉冲开始，period_rise 表示 PRI 回绕，group_rise
//           表示频率捷变脉组边界；三个事件均为单个 fabric 时钟脉冲。
// 时间基: cnt 只在 da_sig_tick 的 tick 上推进，PW/PRI 参数单位均为 tick。
// =============================================================================

module da_sig_pri_sched (

    input  wire         clk,             // FPGA 工作时钟
    input  wire         rst_sync,        // 同步复位，高有效
    input  wire         running,         // 波形运行使能
    input  wire         start_edge,      // 启动脉冲，一般保持 1 个 clk
    input  wire         tick,            // PRI 时间基准脉冲，cnt 只在 tick 到来时推进

    input  wire         cw_en,           // CW 连续波使能，1 时 pulse_gate 恒为 1
    input  wire [19:0]  pw_ticks,        // 脉宽，单位为 tick

    input  wire [1:0]   pri_type,        // PRI 类型：固定/参差/抖动等
    input  wire [5:0]   pri_m,           // 参差 PRI 个数，限制为 1~32
    input  wire [5:0]   pri_group_size,  // 脉组参差时，每种 PRI 连续使用的脉冲数
    input  wire [5:0]   hop_group_size,  // 频率捷变时，每个频率组包含的 PRI 数

    input  wire [639:0] pri_tab_bus,     // PRI 表，32 个 × 20 bit = 640 bit

    input  wire [19:0]  jit_p0,          // 抖动模式平均 PRI
    input  wire [19:0]  jit_n,           // 正弦一周的墙钟 tick；每发转角 = 360°×P0/N
    input  wire [6:0]   jit_rho,         // 抖动深度，一般表示百分比
    input  wire         jit_mode,        // 抖动模式选择：正弦/伪随机等

    input  wire [19:0]  slide_min,       // PRI 滑变最小值
    input  wire [19:0]  slide_max,       // PRI 滑变最大值
    input  wire [19:0]  slide_stp,       // PRI 滑变步长
    input  wire         slide_md,        // 滑变模式：固定步长/随机步长
    input  wire         slide_run,       // 1=允许滑变；0=即使 Pmin/Pmax/δ 残留也不滑

    output reg          pulse_gate,      // 脉冲有效门控
    output reg          pulse_rise,      // 新脉冲开始事件，保持 1 个 clk
    output reg          period_rise,     // 新 PRI 周期事件，保持 1 个 clk
    output reg          group_rise,      // 频率捷变脉组切换事件
    output reg          wave_ready,      // PRI 时间轴已经正式启动
    output reg  [19:0]  pulse_idx,       // 当前脉冲/PRI 编号
    output reg  [4:0]   pri_ptr,         // 当前参差 PRI 表索引
    output wire        cfg_ready        // 正弦抖动计算就绪，其他模式不等待该除法
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


// BEGIN INLINE HEADER: da_sig_mod.vh (module内局部作用域)
// =============================================================================
// 文件: da_sig_mod.vh
// 功能: 新链公共常数、枚举和纯函数。被 PRI / 载波 / 调相 / 扫描 / 抖动 / 滑变 include。
// 约定:
//   时间  协议 1 格 = 10 ns = 1 tick；FPGA 1 tick = 3 clk @ 300 MHz = 10 ns
//   频率  32-bit FTW = floor(f_Hz * 2^32 / 4.8e9)，上位机已换好，RTL 不再除 Fs
//   分工  参数库只存；PRI 算周期，wave/carrier 算频率，scan 算幅度
// 关联:
//   da_sig_host_regs   表长/位宽与影子总线一致
//   da_sig_pri_*       PRI_TAB_*、PRI_* 枚举、da_sig_pri_clamp、da_sig_sin8
//   da_sig_carrier     FREQ_* 枚举、FREQ_TAB_*
//   da_sig_pm_pn       da_sig_pm_nclass / da_sig_pm_k
//   da_sig_scan_nco    SCAN_* 枚举、SCAN_IDX_W
// =============================================================================

// -----------------------------------------------------------------------------
// PRI 表：0x6D 下标 + 0x6E 写 tab[idx]。项宽 20 bit tick，上限 (2^20-1)*10 ns ≈ 10.49 ms
// -----------------------------------------------------------------------------
localparam integer PRI_TAB_N      = 32;      // 参差表项数 M 上限，下标 0..31
localparam integer PRI_TAB_W      = 20;      // 每项 PRI 位宽，单位 tick
localparam integer PRI_TAB_BUS_W  = 640;     // 单通道扁平总线：32*20

// -----------------------------------------------------------------------------
// 捷变频表：0x76 下标 + 0x77 写 freq_tab[idx]。每项 32-bit FTW
// -----------------------------------------------------------------------------
localparam integer FREQ_TAB_N     = 32;      // 频点数 N_hop 上限，下标 0..31
localparam integer FREQ_TAB_W     = 32;      // 每项 FTW 位宽
localparam integer FREQ_TAB_BUS_W = 1024;    // 单通道扁平总线：32*32

// -----------------------------------------------------------------------------
// 相位表：0x78 addr 0..31，低 32 bit 为相位字（deg/360 * 2^32）
// -----------------------------------------------------------------------------
localparam integer PHASE_TAB_N     = 32;     // 相位表项数，与 hop 点对齐
localparam integer PHASE_TAB_W     = 32;     // 每项相位字位宽
localparam integer PHASE_TAB_BUS_W = 1024;   // 单通道扁平总线：32*32

// -----------------------------------------------------------------------------
// 扫描方向图：0x79 逐点写 G；地址 10 bit ↔ 360°，1 LSB = 360/1024 ≈ 0.3516°
//   例：start=896 → −45°，span=256 → 90°，target=57 → 20°
// -----------------------------------------------------------------------------
localparam integer SCAN_TAB_N     = 1024;    // 方向图点数，圆扫一周转一圈
localparam integer SCAN_TAB_W     = 8;       // 增益 G，0..255；y=(x*G)>>>8
localparam integer SCAN_IDX_W     = 10;      // 扫描地址位宽，0~1023

// -----------------------------------------------------------------------------
// 0x61[5:4] pri_type。滑变不占此枚举，走 0x72/0x80，pri_type 仍为 PRI_FIXED
// -----------------------------------------------------------------------------
localparam [1:0] PRI_FIXED  = 2'd0;          // 固定 PRI，用 tab[0] / 默认 PRI
localparam [1:0] PRI_STAG_P = 2'd1;          // 脉间参差：每个脉冲换一项
localparam [1:0] PRI_STAG_G = 2'd2;          // 脉组参差：每 pri_group 个脉冲换一项
localparam [1:0] PRI_JITTER = 2'd3;          // 抖动：0x70 P0、0x7F N_jit、0x71 ρ/模式

// -----------------------------------------------------------------------------
// 0x61[3:2] freq_type。载波 hop 事件：脉内 tick、脉间 period_rise、脉组 group_rise
// -----------------------------------------------------------------------------
localparam [1:0] FREQ_CW    = 2'd0;          // 单载频，fc 固定
localparam [1:0] FREQ_INTRA = 2'd1;          // 脉内捷变，hop_ticks 驻留
localparam [1:0] FREQ_INTER = 2'd2;          // 脉间捷变，每个脉冲换一点
localparam [1:0] FREQ_GROUP = 2'd3;          // 脉组捷变，每 hop_group 个脉冲换一点

// -----------------------------------------------------------------------------
// 0x73 scan_type。真正打开还要 scan_en=1 且 T_scan≠0
// -----------------------------------------------------------------------------
localparam [1:0] SCAN_OFF   = 2'd0;          // 关扫描，增益直通
localparam [1:0] SCAN_CIRC  = 2'd1;          // 圆扫，addr=θ[47:38]
localparam [1:0] SCAN_SEC   = 2'd2;          // 扇扫，start..start+span 折返

// P3/P4 码长靠档：主机 Ncode 向下收到 16/32/64/128/256，保证 N=2^k
function [8:0] da_sig_pm_nclass;
    input [15:0] n_host;                     // 0x6C 下发的码长
    begin
        if (n_host >= 16'd256)
            da_sig_pm_nclass = 9'd256;       // ≥256 → 256
        else if (n_host >= 16'd128)
            da_sig_pm_nclass = 9'd128;
        else if (n_host >= 16'd64)
            da_sig_pm_nclass = 9'd64;
        else if (n_host >= 16'd32)
            da_sig_pm_nclass = 9'd32;
        else
            da_sig_pm_nclass = 9'd16;        // <32 一律 16
    end
endfunction

// N=2^k → k，给 P3/P4 相位左移 31-k 用。16→4 … 256→8
function [3:0] da_sig_pm_k;
    input [8:0] nclass;                      // da_sig_pm_nclass 的输出
    begin
        case (nclass)
            9'd256:  da_sig_pm_k = 4'd8;
            9'd128:  da_sig_pm_k = 4'd7;
            9'd64:   da_sig_pm_k = 4'd6;
            9'd32:   da_sig_pm_k = 4'd5;
            default: da_sig_pm_k = 4'd4;     // N=16
        endcase
    end
endfunction

// PRI 合法化：raw<PW 或 raw=0 时强制成 PW，避免门控脉宽大于周期
function [19:0] da_sig_pri_clamp;
    input [19:0] raw;                        // 调度算出的 PRI（tick）
    input [19:0] pw;                         // 当前脉宽（tick）
    begin
        if ((raw < pw) || (raw == 20'd0))
            da_sig_pri_clamp = pw;
        else
            da_sig_pri_clamp = raw;
    end
endfunction

// 128 点正弦 LUT，步进 360°/128。m=round(127*sin(2π·k/128))，k=ph[7:1]
// 抖动 P=P0*(1+ρ*m/100)，m∈[-127,127]
function signed [7:0] da_sig_sin8;
    input [7:0] ph;                          // 高 7 bit 选点，最低 bit 不用
    begin
        case (ph[7:1])
            7'd0:   da_sig_sin8 =  8'sd0;
            7'd1:   da_sig_sin8 =  8'sd6;
            7'd2:   da_sig_sin8 =  8'sd12;
            7'd3:   da_sig_sin8 =  8'sd19;
            7'd4:   da_sig_sin8 =  8'sd25;
            7'd5:   da_sig_sin8 =  8'sd31;
            7'd6:   da_sig_sin8 =  8'sd37;
            7'd7:   da_sig_sin8 =  8'sd43;
            7'd8:   da_sig_sin8 =  8'sd49;
            7'd9:   da_sig_sin8 =  8'sd54;
            7'd10:  da_sig_sin8 =  8'sd60;
            7'd11:  da_sig_sin8 =  8'sd65;
            7'd12:  da_sig_sin8 =  8'sd71;
            7'd13:  da_sig_sin8 =  8'sd76;
            7'd14:  da_sig_sin8 =  8'sd81;
            7'd15:  da_sig_sin8 =  8'sd85;
            7'd16:  da_sig_sin8 =  8'sd90;
            7'd17:  da_sig_sin8 =  8'sd94;
            7'd18:  da_sig_sin8 =  8'sd98;
            7'd19:  da_sig_sin8 =  8'sd102;
            7'd20:  da_sig_sin8 =  8'sd106;
            7'd21:  da_sig_sin8 =  8'sd109;
            7'd22:  da_sig_sin8 =  8'sd112;
            7'd23:  da_sig_sin8 =  8'sd115;
            7'd24:  da_sig_sin8 =  8'sd117;
            7'd25:  da_sig_sin8 =  8'sd120;
            7'd26:  da_sig_sin8 =  8'sd122;
            7'd27:  da_sig_sin8 =  8'sd123;
            7'd28:  da_sig_sin8 =  8'sd125;
            7'd29:  da_sig_sin8 =  8'sd126;
            7'd30:  da_sig_sin8 =  8'sd126;
            7'd31:  da_sig_sin8 =  8'sd127;
            7'd32:  da_sig_sin8 =  8'sd127;
            7'd33:  da_sig_sin8 =  8'sd127;
            7'd34:  da_sig_sin8 =  8'sd126;
            7'd35:  da_sig_sin8 =  8'sd126;
            7'd36:  da_sig_sin8 =  8'sd125;
            7'd37:  da_sig_sin8 =  8'sd123;
            7'd38:  da_sig_sin8 =  8'sd122;
            7'd39:  da_sig_sin8 =  8'sd120;
            7'd40:  da_sig_sin8 =  8'sd117;
            7'd41:  da_sig_sin8 =  8'sd115;
            7'd42:  da_sig_sin8 =  8'sd112;
            7'd43:  da_sig_sin8 =  8'sd109;
            7'd44:  da_sig_sin8 =  8'sd106;
            7'd45:  da_sig_sin8 =  8'sd102;
            7'd46:  da_sig_sin8 =  8'sd98;
            7'd47:  da_sig_sin8 =  8'sd94;
            7'd48:  da_sig_sin8 =  8'sd90;
            7'd49:  da_sig_sin8 =  8'sd85;
            7'd50:  da_sig_sin8 =  8'sd81;
            7'd51:  da_sig_sin8 =  8'sd76;
            7'd52:  da_sig_sin8 =  8'sd71;
            7'd53:  da_sig_sin8 =  8'sd65;
            7'd54:  da_sig_sin8 =  8'sd60;
            7'd55:  da_sig_sin8 =  8'sd54;
            7'd56:  da_sig_sin8 =  8'sd49;
            7'd57:  da_sig_sin8 =  8'sd43;
            7'd58:  da_sig_sin8 =  8'sd37;
            7'd59:  da_sig_sin8 =  8'sd31;
            7'd60:  da_sig_sin8 =  8'sd25;
            7'd61:  da_sig_sin8 =  8'sd19;
            7'd62:  da_sig_sin8 =  8'sd12;
            7'd63:  da_sig_sin8 =  8'sd6;
            7'd64:  da_sig_sin8 =  8'sd0;
            7'd65:  da_sig_sin8 = -8'sd6;
            7'd66:  da_sig_sin8 = -8'sd12;
            7'd67:  da_sig_sin8 = -8'sd19;
            7'd68:  da_sig_sin8 = -8'sd25;
            7'd69:  da_sig_sin8 = -8'sd31;
            7'd70:  da_sig_sin8 = -8'sd37;
            7'd71:  da_sig_sin8 = -8'sd43;
            7'd72:  da_sig_sin8 = -8'sd49;
            7'd73:  da_sig_sin8 = -8'sd54;
            7'd74:  da_sig_sin8 = -8'sd60;
            7'd75:  da_sig_sin8 = -8'sd65;
            7'd76:  da_sig_sin8 = -8'sd71;
            7'd77:  da_sig_sin8 = -8'sd76;
            7'd78:  da_sig_sin8 = -8'sd81;
            7'd79:  da_sig_sin8 = -8'sd85;
            7'd80:  da_sig_sin8 = -8'sd90;
            7'd81:  da_sig_sin8 = -8'sd94;
            7'd82:  da_sig_sin8 = -8'sd98;
            7'd83:  da_sig_sin8 = -8'sd102;
            7'd84:  da_sig_sin8 = -8'sd106;
            7'd85:  da_sig_sin8 = -8'sd109;
            7'd86:  da_sig_sin8 = -8'sd112;
            7'd87:  da_sig_sin8 = -8'sd115;
            7'd88:  da_sig_sin8 = -8'sd117;
            7'd89:  da_sig_sin8 = -8'sd120;
            7'd90:  da_sig_sin8 = -8'sd122;
            7'd91:  da_sig_sin8 = -8'sd123;
            7'd92:  da_sig_sin8 = -8'sd125;
            7'd93:  da_sig_sin8 = -8'sd126;
            7'd94:  da_sig_sin8 = -8'sd126;
            7'd95:  da_sig_sin8 = -8'sd127;
            7'd96:  da_sig_sin8 = -8'sd127;
            7'd97:  da_sig_sin8 = -8'sd127;
            7'd98:  da_sig_sin8 = -8'sd126;
            7'd99:  da_sig_sin8 = -8'sd126;
            7'd100: da_sig_sin8 = -8'sd125;
            7'd101: da_sig_sin8 = -8'sd123;
            7'd102: da_sig_sin8 = -8'sd122;
            7'd103: da_sig_sin8 = -8'sd120;
            7'd104: da_sig_sin8 = -8'sd117;
            7'd105: da_sig_sin8 = -8'sd115;
            7'd106: da_sig_sin8 = -8'sd112;
            7'd107: da_sig_sin8 = -8'sd109;
            7'd108: da_sig_sin8 = -8'sd106;
            7'd109: da_sig_sin8 = -8'sd102;
            7'd110: da_sig_sin8 = -8'sd98;
            7'd111: da_sig_sin8 = -8'sd94;
            7'd112: da_sig_sin8 = -8'sd90;
            7'd113: da_sig_sin8 = -8'sd85;
            7'd114: da_sig_sin8 = -8'sd81;
            7'd115: da_sig_sin8 = -8'sd76;
            7'd116: da_sig_sin8 = -8'sd71;
            7'd117: da_sig_sin8 = -8'sd65;
            7'd118: da_sig_sin8 = -8'sd60;
            7'd119: da_sig_sin8 = -8'sd54;
            7'd120: da_sig_sin8 = -8'sd49;
            7'd121: da_sig_sin8 = -8'sd43;
            7'd122: da_sig_sin8 = -8'sd37;
            7'd123: da_sig_sin8 = -8'sd31;
            7'd124: da_sig_sin8 = -8'sd25;
            7'd125: da_sig_sin8 = -8'sd19;
            7'd126: da_sig_sin8 = -8'sd12;
            default: da_sig_sin8 = -8'sd6;   // k=127
        endcase
    end
endfunction

// END INLINE HEADER: da_sig_mod.vh



reg [19:0] cnt;                         // 当前 PRI 内部 tick 计数，范围 0~P-1
reg [19:0] P;                           // 当前正在使用的 PRI 长度，单位 tick
reg        first_tick_pending;          // start 后等待第一个真实 tick

reg [4:0]  ptr;                         // 当前 PRI 表指针，范围 0~31

reg [5:0]  grp_pri;                     // 脉组参差内部脉冲计数器
reg [5:0]  grp_hop;                     // 频率捷变脉组内部计数器

reg [19:0] pw_r;                        // 锁存后的有效脉宽
reg [19:0] pw_m1_r;                     // pw_r-1，用于 tick 到来时提前判断门控
reg [19:0] P_m1_r;                      // P-1，用于判断当前 PRI 是否结束

reg [5:0]  m_ok_r;                      // 限幅后的 PRI 参差数量
reg [5:0]  pg_ok_r;                     // 限幅后的 PRI 组大小
reg [5:0]  hg_ok_r;                     // 限幅后的 hop 组大小

reg [1:0]  pty_r;                       // 锁存后的 pri_type
reg        grp_last_r;                  // 当前是否为 PRI 参差组最后一个脉冲

reg [19:0] P_nxt_r;                     // 下一周期即将使用的 PRI
reg [19:0] P_0_r;                       // start 时使用的第一个 PRI

reg [19:0] slide_min_r;                 // 锁存后的滑变最小 PRI
reg [19:0] slide_max_r;                 // 锁存后的滑变最大 PRI
reg [19:0] slide_stp_r;                 // 锁存后的滑变步长


wire [19:0] pw_ok =                     // 对脉宽进行保护，最小为 1 tick
    (pw_ticks == 20'd0) ? 20'd1 :
                          pw_ticks;

wire [5:0] m_ok =                       // PRI 参差数量限制到 1~32
    (pri_m < 6'd1)  ? 6'd1  :
    (pri_m > 6'd32) ? 6'd32 :
                       pri_m;

wire [5:0] pg_ok =                      // PRI 脉组大小限制到 1~32
    (pri_group_size < 6'd1)  ? 6'd1  :
    (pri_group_size > 6'd32) ? 6'd32 :
                                pri_group_size;

wire [5:0] hg_ok =                      // 频率捷变脉组大小限制到 1~32
    (hop_group_size < 6'd1)  ? 6'd1  :
    (hop_group_size > 6'd32) ? 6'd32 :
                                hop_group_size;


wire [5:0] ptr_p1 =                     // 将 5 bit ptr 扩展成 6 bit 后加 1
    {1'b0, ptr} + 6'd1;                 // 这样可以正确表示 31+1=32

wire [4:0] ptr_inc =                    // 计算 PRI 表循环后的下一个索引
    (ptr_p1 >= m_ok_r) ? 5'd0 :         // 已到最后一个 PRI，则回到索引 0
                          ptr_p1[4:0];   // 否则索引 +1


wire stag_p =                           // 脉间参差标志
    (pty_r == PRI_STAG_P);              // 脉间参差时每个 PRI 都切换表项

wire stag_g =                           // 脉组参差切表标志
    (pty_r == PRI_STAG_G) &&            // 必须处于脉组参差模式
    grp_last_r;                         // 并且当前已经到达组内最后一个脉冲

wire [4:0] ptr_nxt =                    // 下一个 PRI 应该使用哪个表索引
    (stag_p || stag_g) ? ptr_inc :      // 脉间参差或组结束时索引 +1
                         ptr;           // 其他情况下保持当前索引


wire slide_en =                         // PRI 滑变有效条件
    slide_run &&                        // 0x7C[4]；关滑变必须为 0，避免 Pmin/Pmax/δ 粘滞
    (pty_r == PRI_FIXED) &&             // 滑变与固定 PRI 共用 pri_type=0
    (slide_max_r > slide_min_r) &&
    (slide_stp_r != 20'd0);

wire jit_en =                           // PRI 抖动有效条件
    (pty_r == PRI_JITTER);              // PRI 类型选择为抖动模式


wire wrap =                             // 当前 PRI 周期结束标志
    tick &&                             // 必须有新的时间 tick
    !first_tick_pending &&              // 启动后的第一个 tick 不参与 wrap
    (cnt == P_m1_r);                    // 当前计数已经到 P-1


wire [15:0] pri_lfsr;                   // 16 bit LFSR 伪随机序列

wire [19:0] P_tab;                      // PRI 表模块计算出的下一 PRI
wire [19:0] P_0_tab;                    // PRI 表模块计算出的初始 PRI
wire [4:0]  ptr_qq;                     // PRI 表模块流水后的实际表指针

wire [19:0] P_jit;                      // PRI 抖动模块计算出的 PRI

wire [19:0] P_slide;                    // PRI 滑变模块计算出的下一 PRI
wire [19:0] P_0_slide;                  // PRI 滑变模块计算出的初始 PRI
wire jitter_ready;
assign cfg_ready = cw_en || (pri_type != PRI_JITTER) || jit_mode || jitter_ready;


da_sig_lfsr16 u_lfsr (

    .clk       (clk),                   // 工作时钟
    .rst_sync  (rst_sync),              // 同步复位
    .seed      (16'hBEEF),              // 固定随机种子，方便仿真和结果复现
    .lfsr      (pri_lfsr)               // 输出 16 bit 伪随机序列

);


da_sig_pri_tab u_tab (

    .clk          (clk),                // 工作时钟
    .rst_sync     (rst_sync),           // 同步复位
    .running      (running),            // 运行使能
    .ptr_nxt      (ptr_nxt),            // 下一 PRI 表索引
    .pw           (pw_r),               // 当前脉宽，用于限制 PRI >= PW
    .pri_tab_bus  (pri_tab_bus),        // 32 个 PRI 构成的总线
    .P_tab        (P_tab),              // 输出选中的 PRI
    .P_0          (P_0_tab),            // 输出启动时第一个 PRI
    .ptr_qq       (ptr_qq)              // 与 P_tab 流水延迟对应的表索引

);


da_sig_pri_jitter u_jit (

    .clk         (clk),                 // 工作时钟
    .rst_sync    (rst_sync),            // 同步复位
    .running     (running),             // 运行使能
    .wrap        (wrap),                // PRI 回绕时正弦转固定角 Δθ
    .start_edge  (start_edge),          // 启动时初始化抖动序列
    .jit_p0      (jit_p0),              // 平均 PRI
    .jit_n       (jit_n),               // 抖动周期（墙钟 tick）
    .jit_rho     (jit_rho),             // 抖动深度
    .jit_mode    (jit_mode),            // 正弦或伪随机模式
    .pw          (pw_r),                // 用于保证生成的 PRI 不小于脉宽
    .lfsr        (pri_lfsr),            // 随机抖动使用的伪随机数
    .P_jit       (P_jit),               // 输出计算后的抖动 PRI
    .cfg_ready   (jitter_ready)

);


da_sig_pri_slide u_slide (

    .clk         (clk),                 // 工作时钟
    .rst_sync    (rst_sync),            // 同步复位
    .running     (running),             // 运行使能
    .wrap        (wrap),                // 当前 PRI 结束时更新滑变值
    .start_edge  (start_edge),          // 启动时初始化滑变过程
    .P_cur       (P),                   // 当前正在使用的 PRI
    .slide_min   (slide_min),           // 滑变下限
    .slide_max   (slide_max),           // 滑变上限
    .slide_stp   (slide_stp),           // 滑变步长
    .slide_md    (slide_md),            // 固定步长/随机步长选择
    .pw          (pw_r),                // 用于保证 PRI 不小于 PW
    .lfsr        (pri_lfsr),            // 随机滑变使用的伪随机序列
    .P_slide     (P_slide),             // 输出下一滑变 PRI
    .P_0         (P_0_slide)            // 输出启动时第一个滑变 PRI

);


// =============================================================================
// 参数锁存与下一 PRI 计算
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        pw_r        <= 20'd1;            // 默认脉宽为 1 tick
        pw_m1_r     <= 20'd0;            // 默认 PW-1=0
        P_m1_r      <= 20'd9;            // 默认 P=10，所以 P-1=9

        m_ok_r      <= 6'd1;             // 默认只有 1 个 PRI
        pg_ok_r     <= 6'd1;             // 默认 PRI group 大小为 1
        hg_ok_r     <= 6'd1;             // 默认 hop group 大小为 1

        pty_r       <= 2'd0;             // 默认 PRI 类型为 0
        grp_last_r  <= 1'b0;             // 默认不是 PRI 组最后一个脉冲

        P_nxt_r     <= 20'd10;           // 默认下一 PRI 为 10 tick
        P_0_r       <= 20'd10;           // 默认初始 PRI 为 10 tick

        slide_min_r <= 20'd0;            // 默认滑变下限为 0
        slide_max_r <= 20'd0;            // 默认滑变上限为 0
        slide_stp_r <= 20'd0;            // 默认滑变步长为 0，相当于关闭滑变

    end else begin

        slide_min_r <= slide_min;        // 锁存滑变最小 PRI
        slide_max_r <= slide_max;        // 锁存滑变最大 PRI
        slide_stp_r <= slide_stp;        // 锁存滑变步长

        pw_r <= pw_ok;                   // 锁存合法化后的脉宽

        pw_m1_r <=                       // 提前计算 PW-1
            (pw_ok <= 20'd1) ?
            20'd0 :                     // PW<=1 时直接使用 0，防止无符号下溢
            (pw_ok - 20'd1);            // 正常情况下得到 PW-1

        P_m1_r <=                        // 提前计算当前 PRI 的 P-1
            (P <= 20'd1) ?
            20'd0 :                     // P<=1 时保护为 0
            (P - 20'd1);                // 正常情况下得到 P-1

        m_ok_r  <= m_ok;                 // 锁存限制后的 PRI 数量
        pg_ok_r <= pg_ok;                // 锁存限制后的 PRI group 大小
        hg_ok_r <= hg_ok;                // 锁存限制后的 hop group 大小
        pty_r   <= pri_type;             // 锁存 PRI 类型

        grp_last_r <=                    // 判断当前是不是 PRI group 的最后一个脉冲
            ((grp_pri + 6'd1) >= pg_ok);// grp_pri 从 0 开始，所以需要先 +1 再比较


        if (jit_en)                      // PRI 抖动优先级最高
            P_nxt_r <= P_jit;            // 下一 PRI 来自抖动模块

        else if (slide_en)               // 未抖动且满足滑变条件
            P_nxt_r <= P_slide;          // 下一 PRI 来自滑变模块

        else
            P_nxt_r <= P_tab;            // 其余情况从 PRI 表模块取得


        if (jit_en)                      // 抖动模式启动时
            P_0_r <= da_sig_pri_clamp(   // 以基准 PRI jit_p0 作为初始值
                        jit_p0,           // 基准 PRI
                        pw_r              // 同时确保 PRI 不小于脉宽
                     );

        else if (slide_en)               // 滑变模式启动时
            P_0_r <= P_0_slide;          // 使用滑变模块产生的初始 PRI

        else
            P_0_r <= P_0_tab;            // 固定/参差模式使用 PRI 表初始值

    end

end


// =============================================================================
// PRI 主时间轴
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        cnt                 <= 20'd0;     // PRI 内部 tick 计数清零
        P                   <= 20'd10;    // 默认 PRI 为 10 tick

        ptr                 <= 5'd0;      // PRI 表内部指针清零
        pri_ptr             <= 5'd0;      // 输出 PRI 表指针清零

        grp_pri             <= 6'd0;      // PRI 参差 group 计数清零
        grp_hop             <= 6'd0;      // 频率 hop group 计数清零

        first_tick_pending  <= 1'b0;      // 当前不等待第一个 tick

        wave_ready          <= 1'b0;      // 波形时间轴未准备好
        pulse_gate          <= 1'b0;      // 脉冲输出关闭
        pulse_rise          <= 1'b0;      // 脉冲开始事件清零
        period_rise         <= 1'b0;      // PRI 周期事件清零
        group_rise          <= 1'b0;      // 频率组事件清零

        pulse_idx           <= 20'd0;     // 脉冲编号清零

    end else if (start_edge) begin

        cnt                 <= 20'd0;     // 启动后从 PRI 的第 0 tick 开始

        ptr                 <= 5'd0;      // PRI 表重新从第 0 项开始
        pri_ptr             <= 5'd0;      // 输出索引同步清零

        P                   <= P_0_r;     // 装载当前模式对应的启动 PRI

        grp_pri             <= 6'd0;      // PRI 参差 group 从头开始
        grp_hop             <= 6'd0;      // hop group 从头开始

        first_tick_pending  <= 1'b1;      // start 本拍不出波，等待第一个真实 tick

        wave_ready          <= 1'b0;      // 此时波形还没有正式开始
        pulse_gate          <= 1'b0;      // start 本拍门控关闭
        pulse_rise          <= 1'b0;      // start 本拍不认为是脉冲上升沿
        period_rise         <= 1'b0;      // start 本拍没有 PRI 周期事件
        group_rise          <= 1'b0;      // start 本拍没有 group 事件

        pulse_idx           <= 20'd0;     // 第一个脉冲编号重新从 0 开始

    end else if (!running) begin

        wave_ready  <= 1'b0;              // 停止运行后波形无效
        pulse_gate  <= 1'b0;              // 停止输出脉冲
        pulse_rise  <= 1'b0;              // 清除脉冲开始事件
        period_rise <= 1'b0;              // 清除 PRI 周期事件
        group_rise  <= 1'b0;              // 清除 group 事件

                                             // 注意：这里不清 cnt/P/ptr/pulse_idx
                                             // 因此 running=0 更类似关闭输出，而非复位

    end else begin

        pulse_rise  <= 1'b0;              // rise 信号默认只保持 1 个 clk
        period_rise <= 1'b0;              // 每个 clk 先默认清零
        group_rise  <= 1'b0;              // 条件满足后再重新置 1


        if (tick) begin                   // 只有 tick 到来时才推进 PRI 时间轴

            if (first_tick_pending) begin

                first_tick_pending <= 1'b0;      // 已收到启动后的第一个真实 tick

                wave_ready         <= 1'b1;      // 从这一刻开始 PRI 时间轴有效

                pulse_rise         <= ~cw_en;    // 非 CW 时，第一个脉冲从这里开始

                cnt                <= 20'd0;      // 第一个有效 tick 定义为 cnt=0

                pulse_gate         <=             // 第一个 tick 打开脉冲门
                                      cw_en ||    // CW 模式一直开门
                                      (pw_r != 20'd0);

                pri_ptr            <= ptr;        // 输出当前 PRI 表索引

            end else if (wrap) begin

                cnt         <= 20'd0;             // 当前 PRI 结束，新 PRI 从 cnt=0 开始

                pulse_idx   <= pulse_idx +        // 脉冲/PRI 编号加 1
                               20'd1;

                period_rise <= 1'b1;              // 产生 PRI 边界事件

                pulse_rise  <= ~cw_en;            // 非 CW 时，新 PRI 同时开始新脉冲

                pulse_gate  <=                     // 新 PRI 的第 0 tick 重新开门
                               cw_en ||
                               (pw_r != 20'd0);

                ptr         <= ptr_qq;             // 更新内部 PRI 表索引

                pri_ptr     <= ptr_qq;             // 更新输出 PRI 表索引

                P           <= P_nxt_r;            // 装载提前计算好的下一 PRI


                if (pty_r == PRI_STAG_G) begin     // 只有脉组参差需要维护 grp_pri

                    if (grp_last_r)
                        grp_pri <= 6'd0;            // 当前组结束，下一组从 0 开始

                    else
                        grp_pri <= grp_pri + 6'd1; // 当前组继续，下一个脉冲计数 +1

                end


                if ((grp_hop + 6'd1) >=            // 判断是否已经完成一个 hop group
                    hg_ok_r) begin

                    grp_hop    <= 6'd0;             // 当前 hop group 结束，计数重新归零

                    group_rise <= 1'b1;             // 通知频率捷变模块可以切换频点

                end else begin

                    grp_hop <= grp_hop + 6'd1;     // 当前 hop group 尚未结束

                end

            end else begin

                cnt <= cnt + 20'd1;                // PRI 中间普通 tick，计数加 1

                pulse_gate <=                      // 根据脉宽决定下一状态是否继续出波
                              cw_en ||
                              (cnt < pw_m1_r);      // 使用旧 cnt 与 PW-1 比较

                                                    // 例如 PW=3：
                                                    // cnt=0 -> 下一拍 cnt=1，gate=1
                                                    // cnt=1 -> 下一拍 cnt=2，gate=1
                                                    // cnt=2 -> 下一拍 cnt=3，gate=0
                                                    // 因此有效时间正好为 3 tick

            end

        end else if (wave_ready) begin

            pulse_gate <=                           // 没有 tick 时 cnt 保持不变
                          cw_en ||                  // CW 始终保持输出
                          (cnt < pw_r);             // 普通脉冲则维持当前门控状态

        end

    end

end

endmodule
