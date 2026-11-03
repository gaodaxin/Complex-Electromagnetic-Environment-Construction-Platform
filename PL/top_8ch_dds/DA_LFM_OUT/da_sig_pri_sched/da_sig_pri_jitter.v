// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_pri_jitter.v
// 功能：根据正弦或伪随机调制量计算 PRI 抖动，按脉冲周期边界提交更新。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_pri_jitter
// 功能: 计算正弦或伪随机抖动 PRI：P=P0×(1+rho×m/100)。
// 正弦: 每发固定转角 Δθ=360°×P0/N_jit，再按该角和平均 PRI 算当前 P。
//       相位只在 PRI wrap 推进，与上一拍实际 PRI 无关。
// 时序: P0×rho、再乘调制量及比例换算分级进入 DSP，满足 300 MHz。
// =============================================================================

module da_sig_pri_jitter (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        running,          // 运行使能；停止时允许重算 2^32×P0/N_jit
    input  wire        wrap,             // PRI 回绕；正弦相位每发推一次固定角
    input  wire        start_edge,       // 启动沿；当前发用 P0，相位直接到 Δθ
    input  wire [19:0] jit_p0,           // 抖动中心 P0（平均 PRI），单位 tick
    input  wire [19:0] jit_n,            // 抖动调制周期 N_jit，单位墙钟 tick
    input  wire [6:0]  jit_rho,          // 抖动深度 ρ，百分数 0..50 → ±50%
    input  wire        jit_mode,         // 0 正弦 / 1 伪随机
    input  wire [19:0] pw,               // 当前脉宽，用来保证 PRI>=PW
    input  wire [15:0] lfsr,             // 外部 16 bit LFSR，伪随机模式用低 8 bit
    output reg  [19:0] P_jit,            // 计算后的抖动 PRI（正弦时是下一发）
    output reg         cfg_ready        // 当前 P0/N 转角已算好，参数流水已稳定
);


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



reg [19:0]        jit_p0_r;              // 锁存后的中心 PRI，0 时改成 1
reg [19:0]        jit_n_r;               // 锁存后的 N_jit
reg [6:0]         jit_rho_r;             // 锁存后的 ρ
reg               jit_mode_r;            // 锁存后的抖动模式
reg [31:0]        jit_ph;                // 正弦抖动相位：下一发的角度
reg [31:0]        jit_ftw;               // 2^32×P0/N_jit，每发加到 jit_ph
reg [19:0]        jit_n_hold;            // 上次启动除法时的 N_jit
reg [19:0]        jit_p0_hold;           // 上次启动除法时的 P0
reg               jit_div_go;            // 启动除法的 1 拍脉冲
reg               jit_div_cancel;        // 取消旧除法，下一拍才发起新任务
reg               jit_div_pending;
reg               jit_div_busy;
reg [3:0]         jit_settle;
wire [31:0]       jit_ftw_w;             // 除法商
wire              jit_div_done;          // 除法完成
wire [19:0]       jit_p0_ok =            // P0=0 非法，除法分子按 1
    (jit_p0 == 20'd0) ? 20'd1 : jit_p0;
wire jit_params_changed = (jit_n != jit_n_hold) || (jit_p0_ok != jit_p0_hold);

reg signed [7:0]  m_s8;                  // 调制系数 m∈[-127,127]
reg [6:0]         rho_ok_r;              // 钳位后的 ρ，最大 50
(* use_dsp = "yes" *)
reg [26:0]        jit_ur;                // P0*ρ，进第一颗 DSP
reg [26:0]        jit_ur_p;              // 再锁一拍，拆开两颗 DSP
(* use_dsp = "yes" *)
reg signed [35:0] jit_sm;                // (P0*ρ)*m
reg signed [35:0] jit_sm7;               // 右移 7 bit，相当于 /128
// 保住常数乘法后的流水寄存器。若允许跨级重定时，Vivado 会把该寄存器
// 推到 DSP 输入侧，使两颗级联 DSP 与下一拍 P0+Δ 落进同一条组合路径。
(* use_dsp = "yes", dont_touch = "yes" *)
reg signed [35:0] jit_dlt;               // Δ = (sm7*41)>>>12 ≈ P0*ρ*m/100
reg signed [21:0] jit_psum_r;            // P0+Δ
reg [19:0]        pw_r;                  // 锁存后的脉宽


da_sig_div_u u_jit_div (

    .clk       (clk),                    // 工作时钟
    .rst       (rst_sync || jit_div_cancel), // 新任务取消旧计算
    .start     (jit_div_go),             // 启动脉冲
    .shl32     (1'b1),                   // (P0<<32)/N_jit → 每发 2^32×P0/N
    .dividend  ({28'd0, jit_p0_hold}),   // 冻结本次计算的分子
    .divisor   ({44'd0, jit_n_hold}),    // 冻结本次计算的分母
    .quot      (jit_ftw_w),              // 商 → 每发固定转角
    .done      (jit_div_done)            // 完成脉冲

);


// =============================================================================
// 抖动 PRI 流水
// P = P0*(1+ρ*m/100) = P0 + P0*ρ*m/100
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        jit_p0_r    <= 20'd1000;         // 默认中心 PRI=1000 tick
        jit_n_r     <= 20'd0;            // 默认无调制周期
        jit_rho_r   <= 7'd0;             // 默认深度 0，即不抖
        jit_mode_r  <= 1'b0;             // 默认正弦
        jit_ph      <= 32'd0;            // 正弦相位清零
        jit_ftw     <= 32'd0;
        jit_n_hold  <= 20'd0;
        jit_p0_hold <= 20'd0;
        jit_div_go  <= 1'b0;
        jit_div_cancel <= 1'b0;
        jit_div_pending <= 1'b0;
        jit_div_busy <= 1'b0;
        jit_settle <= 4'd10;
        cfg_ready <= 1'b0;
        m_s8        <= 8'sd0;
        rho_ok_r    <= 7'd0;
        jit_ur      <= 27'd0;
        jit_ur_p    <= 27'd0;
        jit_sm      <= 36'sd0;
        jit_sm7     <= 36'sd0;
        jit_dlt     <= 36'sd0;
        jit_psum_r  <= 22'sd0;
        pw_r        <= 20'd1;
        P_jit       <= 20'd10;           // 默认输出 PRI=10

    end else begin

        jit_div_go <= 1'b0;              // 除法启动默认只保持 1 拍
        // Parameter capture enters the global loading state before this
        // registered ready is consumed. The extra stage breaks the wide
        // P0/N comparison -> PRI mode -> run-control setup path.
        cfg_ready <= !jit_params_changed && !jit_div_busy && !jit_div_cancel &&
                     !jit_div_go && (jit_settle == 0);
        jit_div_cancel <= 1'b0;
        if (jit_settle != 0)
            jit_settle <= jit_settle - 1'b1;

        jit_p0_r   <= (jit_p0 == 20'd0) ?
                      20'd1 :            // P0=0 非法，最小按 1 tick
                      jit_p0;
        jit_n_r    <= jit_n;             // 锁存 N_jit
        jit_rho_r  <= jit_rho;           // 锁存 ρ
        jit_mode_r <= jit_mode;          // 锁存模式
        pw_r       <= pw;                // 锁存脉宽

        if (start_edge)
            jit_ph <= jit_ftw;           // 第一发用 P0（sin 0）；相位到 Δθ 给下一发
        else if (running && wrap && !jit_mode_r && (jit_n_r > 20'd1))
            jit_ph <= jit_ph + jit_ftw;  // 每发按 2^32×P0/N 转固定角


        // 参数变更优先于旧任务完成。取消/启动分两拍，除法器始终使用冻结
        // 操作数；新配置在旧任务计算期间到达时，旧结果不能覆盖新任务。
        if (!running && jit_params_changed) begin
            jit_n_hold <= jit_n;
            jit_p0_hold <= jit_p0_ok;
            jit_div_cancel <= 1'b1;
            jit_div_pending <= (jit_n > 20'd1) && (jit_n > jit_p0_ok);
            jit_div_busy <= (jit_n > 20'd1) && (jit_n > jit_p0_ok);
            jit_settle <= 4'd10;
            if ((jit_n <= 20'd1) || (jit_n <= jit_p0_ok))
                jit_ftw <= 32'd0;
        end else if (jit_div_cancel) begin
            jit_div_go <= jit_div_pending;
            jit_div_pending <= 1'b0;
        end else if (jit_div_done && jit_div_busy) begin
            // 完成脉冲不能被 running 吞掉；全局启动屏障检查 cfg_ready。
            jit_ftw <= jit_ftw_w;
            jit_div_busy <= 1'b0;
        end


        m_s8 <= jit_mode_r ?
                $signed(lfsr[7:0]) :     // 伪随机：LFSR 低 8 bit 当有符号 m
                da_sig_sin8(jit_ph[31:24]);
                                         // 正弦：高 7 bit 查 128 点，步进 360°/128

        rho_ok_r <= (jit_rho_r > 7'd50) ?
                    7'd50 :              // ρ 最大 50，对应 ±50%
                    jit_rho_r;

        jit_ur   <= jit_p0_r * rho_ok_r; // 第一颗 DSP：P0*ρ
        jit_ur_p <= jit_ur;              // 再锁一拍，避免 P 直灌下一颗 DSP 的 A
        jit_sm   <= $signed({1'b0, jit_ur_p}) * m_s8;
                                         // 第二颗 DSP：(P0*ρ)*m
        jit_sm7  <= jit_sm >>> 7;        // /128，m 满幅约 127
        jit_dlt  <= (jit_sm7 * 36'sd41) >>> 12;
                                         // 41/4096 ≈ 1/100 的定点近似
        jit_psum_r <= $signed({2'b00, jit_p0_r}) +
                      jit_dlt[21:0];     // P0+Δ，扩成 22 bit 看符号


        if (jit_psum_r[21] ||            // 结果为负
            (jit_psum_r[19:0] == 20'd0)) // 或算成 0
            P_jit <= pw_r;               // 强制成脉宽，避免非法 PRI
        else
            P_jit <= da_sig_pri_clamp(   // 正常情况再保证 PRI>=PW
                         jit_psum_r[19:0],
                         pw_r
                     );

    end

end

endmodule
