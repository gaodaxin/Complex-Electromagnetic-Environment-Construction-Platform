// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_pri_slide.v
// 功能：在 PRI 上下限间三角折返，支持固定或随机步长；只在周期边界更新方向。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_pri_slide
// 功能: 在 Pmin/Pmax 之间产生三角折返 PRI，支持固定步长和随机步长。
// 时序: 参数锁存、随机乘法、步长选择、上下限判断和 PW 钳位分级流水。
// 更新规则: 方向只在 PRI wrap 时提交，避免一个 PRI 内出现多次方向改变。
// =============================================================================

module da_sig_pri_slide (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        running,          // 运行使能
    input  wire        wrap,             // PRI 回绕；此时才切换滑变方向
    input  wire        start_edge,       // 启动沿；方向改成向上
    input  wire [19:0] P_cur,            // 当前正在跑的 PRI
    input  wire [19:0] slide_min,        // 滑变下限 Pmin，单位 tick
    input  wire [19:0] slide_max,        // 滑变上限 Pmax
    input  wire [19:0] slide_stp,        // 滑变步长 δ
    input  wire        slide_md,         // 0 固定步长 / 1 随机步长
    input  wire [19:0] pw,               // 当前脉宽，用来保证 PRI>=PW
    input  wire [15:0] lfsr,             // 随机步长用的 16 bit LFSR
    output reg  [19:0] P_slide,          // 下一滑变 PRI
    output reg  [19:0] P_0               // start 时装载的初始 PRI，取 Pmin
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



reg [19:0] slide_min_r;                  // 锁存后的 Pmin
reg [19:0] slide_max_r;                  // 锁存后的 Pmax
reg [19:0] slide_stp_r;                  // 锁存后的 δ
reg        slide_md_r;                   // 锁存后的步长模式
reg        slide_dir_r;                  // 1=向上滑，0=向下滑
reg        slide_dir_nxt;                // 下一回绕时要用的方向
reg [19:0] P_slide_raw;                  // 尚未做 PW 钳位的下一 PRI
reg [35:0] slide_prod_r;                 // LFSR×δ，单独进 DSP
reg [19:0] d_use_r;                      // 本拍真正使用的步长


// =============================================================================
// 滑变流水：乘法、取步长、P±d、clamp 各一拍
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        slide_min_r   <= 20'd0;
        slide_max_r   <= 20'd0;
        slide_stp_r   <= 20'd0;
        slide_md_r    <= 1'b0;
        slide_dir_r   <= 1'b1;           // 复位后默认向上滑
        slide_dir_nxt <= 1'b1;
        P_slide_raw   <= 20'd10;
        P_slide       <= 20'd10;
        P_0           <= 20'd10;
        slide_prod_r  <= 36'd0;
        d_use_r       <= 20'd1;          // 步长最小按 1，避免卡住

    end else begin

        slide_min_r <= slide_min;        // 锁存下限
        slide_max_r <= slide_max;        // 锁存上限
        slide_stp_r <= slide_stp;        // 锁存步长
        slide_md_r  <= slide_md;         // 锁存模式
        P_0         <= da_sig_pri_clamp( // start 用 Pmin，并保证 >=PW
                           slide_min_r,
                           pw
                       );

        if (start_edge)
            slide_dir_r <= 1'b1;         // 每次启动都从向上开始
        else if (running && wrap)
            slide_dir_r <= slide_dir_nxt;// 回绕时才换方向，避免同一 PRI 内抖动


        // 拍1：乘法单独进 DSP，禁止 LFSR→DSP→P 同一拍
        slide_prod_r <= lfsr * slide_stp_r;


        // 拍2：选出本拍步长
        if (slide_md_r) begin            // 随机步长：取乘积高 20 bit
            if (slide_prod_r[35:16] == 20'd0)
                d_use_r <= 20'd1;        // 随机结果为 0 时仍走 1 tick
            else
                d_use_r <= slide_prod_r[35:16];
        end else begin                   // 固定步长
            d_use_r <= (slide_stp_r == 20'd0) ?
                       20'd1 :           // δ=0 时按 1 处理，避免冻住
                       slide_stp_r;
        end


        // 拍3：三角折返，用 21 bit 比较防止溢出
        if (slide_dir_r) begin           // 向上

            if (({1'b0, P_cur} + {1'b0, d_use_r}) >=
                {1'b0, slide_max_r}) begin
                P_slide_raw   <= slide_max_r; // 碰到上限，钉在 Pmax
                slide_dir_nxt <= 1'b0;        // 下一回绕改为向下
            end else begin
                P_slide_raw   <= P_cur + d_use_r;
                slide_dir_nxt <= 1'b1;        // 还没到顶，继续向上
            end

        end else begin                   // 向下

            if ({1'b0, P_cur} <=
                ({1'b0, slide_min_r} + {1'b0, d_use_r})) begin
                P_slide_raw   <= slide_min_r; // 碰到下限，钉在 Pmin
                slide_dir_nxt <= 1'b1;        // 下一回绕改为向上
            end else begin
                P_slide_raw   <= P_cur - d_use_r;
                slide_dir_nxt <= 1'b0;        // 还没到底，继续向下
            end

        end


        // 拍4：PW 钳位
        P_slide <= da_sig_pri_clamp(P_slide_raw, pw);

    end

end

endmodule
