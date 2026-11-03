// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_scan_nco.v
// 功能：按 tick 推进扫描角地址，支持整周圆扫和扇区往返；加载时计算扫描频率字。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_scan_nco
// 功能: 产生 0..1023 扫描角地址，支持圆周连续扫描和扇区往返扫描。
// 时间基: 角度只在 tick 更新；t_scan 表示转过 360 度所需的 tick 数。
// 配置: load 时锁存类型/起点/跨度，并通过迭代除法计算扫描频率字。
//
// 圆扫数学关系:
//   scan_ftw = floor(2^48/t_scan)；除法保留有效低位，不在截断后补零
//   theta[n+1] = theta[n] + scan_ftw，scan_idx = theta[47:38]
//   48 bit theta 自然溢出即完成一周，输出高 10 bit 对应 1024 个角度格。
//
// 扇扫数学关系:
//   theta 仍按圆扫速率积累，但只在其高 10 bit 变化时让 sec_pos 走一步。
//   地址序列为 start,start+1,...,start+span,start+span-1,...,start，端点不重复。
//   start+sec_pos 使用 10 bit 自然截断，因此扇区允许跨越 1023->0。
//
// 控制优先级:
//   cfg_load 清旧 FTW并发起除法；div_done 装新 FTW；普通 tick 最后才推进角度。
//   start_edge 位于 always 块末尾，对当拍位置更新具有最终覆盖权，但不会清 scan_ftw。
// =============================================================================

module da_sig_scan_nco (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        start_edge,       // 启动沿：theta/sec_pos 回 0，不清已算好的 FTW
    input  wire        cfg_load,         // 加载沿：锁存配置、暂时清 FTW、启动重算
    input  wire        tick,             // 扫描角只在 tick 推进，1 tick = 3 clk
    input  wire        scan_en,          // 扫描使能，cfg_load 时锁存
    input  wire [1:0]  scan_type,        // 0 关 / 1 圆扫 / 2 扇扫
    input  wire [9:0]  scan_start_idx,   // 扇扫起始地址，0~1023
    input  wire [9:0]  scan_span_idx,    // 扇扫跨度，sec_pos ∈ [0,span]
    input  wire [31:0] t_scan,           // 扫 360 度的周期 T；0 表示关闭扫描

    output reg config_busy=0,
    output reg  [9:0]  scan_idx,         // 当前扫描地址 0~1023
    output reg         on_r              // 锁存后的扫描开关
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



reg [47:0]  theta;                       // Q0.48 周相位累加器；圆扫取高 10 bit
reg [47:0]  scan_ftw;                    // 每 tick 的相位增量 floor(2^48/T)
reg [31:0]  t_scan_r;                    // cfg_load 锁存的 T，只给除法器
reg         div_go;                      // 启动除法的 1 拍脉冲
reg         div_pend;                    // cfg_load 后推迟 1 拍再 div_go
wire [63:0] div_q;                       // 完整除法商：2^48/T
wire        div_done;                    // 除法完成脉冲

reg [9:0]   start_r;                     // 锁存后的扇扫起点
reg [9:0]   span_r;                      // 锁存后的扇扫跨度
reg [1:0]   sty_r;                       // 锁存后的扫描类型
reg [9:0]   sec_pos;                     // 扇扫相对位置，[0, span_r]
reg         sec_down;                    // 0 往 span 走，1 往 0 走

reg [47:0]  theta_sum_r;                 // 预计算下一 theta，供 tick 拍直接装载
reg         step10_r;                    // 预计算前后高 10 bit 不同，扇扫位置走一步
reg         span0_r;                     // 锁存 span==0，单点扇扫钉在起点


// =============================================================================
// 无符号迭代除法器
// 使用现有 64/32 配置除法器直接计算 floor(2^48/t_scan_r)。
// 除法只在 cfg_load 后运行，商就绪后一次装入，不进入逐样点出波路径。
// cfg_load 同时复位除法器，取消旧任务；div_pend 在锁存新 T 后发起计算。
// =============================================================================

da_sig_div_q32 u_div (
    .clk       (clk),
    .rst       (rst_sync || cfg_load),   // 新 load 取消旧任务
    .start     (div_go),
    .dividend  (64'h0001_0000_0000_0000), // 2^48，常量必须足够宽
    .divisor   (t_scan_r),               // 使用已锁存的新配置
    .quot      (div_q),
    .done      (div_done)
);


// =============================================================================
// cfg_load 配置锁存 + 实时预计算
// theta_sum_r/step10_r 每个 clk 更新，tick 到来时只经过寄存器选择路径，
// 避免 48 bit 加法器直接落在扫描状态更新的关键路径上。
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        start_r      <= 10'd0;           // 扇扫起点清零
        span_r       <= 10'd0;           // 扇扫跨度清零
        sty_r        <= 2'd0;            // 扫描类型清零
        on_r         <= 1'b0;            // 扫描关闭
        t_scan_r     <= 32'd0;           // T 清零
        theta_sum_r  <= 48'd0;           // θ+FTW 清零
        step10_r     <= 1'b0;            // 无高 10 bit 进位
        span0_r      <= 1'b1;            // 默认单点

    end else begin

        if (cfg_load) begin              // 只在加载沿锁存，运行中不跟输入
            start_r  <= scan_start_idx;  // 扇扫起点
            span_r   <= scan_span_idx;   // 扇扫跨度
            sty_r    <= scan_type;       // 圆扫/扇扫
            t_scan_r <= t_scan;          // 给除法器的 T
            on_r     <= scan_en &&
                        (scan_type != SCAN_OFF) &&
                        (t_scan != 32'd0);
                                         // 用当拍新值，不用旧 on_r
        end

        theta_sum_r <= theta + scan_ftw; // 提前算下一 theta，48 bit 溢出即一周回绕
        step10_r    <= (theta_sum_r[47:38] != theta[47:38]);
                                         // 地址量化格发生变化，扇扫才移动一步
        span0_r     <= (span_r == 10'd0); // 跨度 0 钉在起点

    end

end


// =============================================================================
// 扫描角 NCO 和扇扫位置状态机
//   cfg_load : 清 FTW，推迟 1 拍启动 2^48/T，避免除数仍是旧 t_scan_r
//   start    : theta/sec_pos/sec_down 回 0，scan_idx 回起点，保留 FTW
//   div_done : 装 scan_ftw
//   tick     : 圆扫取 theta[47:38]；扇扫 start+sec_pos，10 bit 自然回绕
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        theta     <= 48'd0;              // 扫描角清零
        scan_ftw  <= 48'd0;              // 扫描 FTW 清零
        div_go    <= 1'b0;               // 除法启动清零
        div_pend  <= 1'b0;               // 推迟启动清零
        scan_idx  <= 10'd0;              // 表地址回 0
        sec_pos   <= 10'd0;              // 扇扫相对位置回 0
        sec_down  <= 1'b0;               // 默认往 span 走

    end else begin

        div_go <= 1'b0;                  // 默认清除启动脉冲

        if (cfg_load) begin

            scan_ftw <= 48'd0;           // 先清 FTW，等除法完成
            div_pend <= scan_en &&
                        (scan_type != SCAN_OFF) &&
                        (t_scan != 32'd0);
                                         // 下一拍再 div_go，divisor 已是 t_scan_r

        end else if (div_pend) begin

            div_go   <= 1'b1;            // t_scan_r 已锁存，启动除法
            div_pend <= 1'b0;

        end else if (div_done) begin

            scan_ftw <= div_q[47:0];     // floor(2^48/T)，保留实际低位

        end else if (on_r && tick && (scan_ftw != 48'd0)) begin

            theta <= theta_sum_r;        // 使用预计算结果，不在本状态路径现场做 48 bit 加法

            if (sty_r == SCAN_CIRC)
                scan_idx <= theta_sum_r[47:38];
                                         // 圆扫：高 10 bit 直接当地址

            else if (sty_r == SCAN_SEC && step10_r) begin

                if (span0_r) begin       // 特例：跨度 0，不允许 sec_pos 做减一
                    sec_pos  <= 10'd0;   // 单点扇区钉在起点
                    sec_down <= 1'b0;
                    scan_idx <= start_r;
                end else if (!sec_down) begin
                    if (sec_pos == span_r) begin
                        sec_down <= 1'b1; // 已在上界，下一步往回
                        sec_pos  <= span_r - 10'd1;
                        scan_idx <= start_r + (span_r - 10'd1);
                    end else begin
                        sec_pos  <= sec_pos + 10'd1;
                        scan_idx <= start_r + (sec_pos + 10'd1);
                                         // 10 bit 溢出：896+256 → 128
                    end
                end else begin
                    if (sec_pos == 10'd0) begin
                        sec_down <= 1'b0; // 已在起点，下一步往上
                        sec_pos  <= 10'd1;
                        scan_idx <= start_r + 10'd1;
                    end else begin
                        sec_pos  <= sec_pos - 10'd1;
                        scan_idx <= start_r + (sec_pos - 10'd1);
                    end
                end

            end

        end

        if (start_edge) begin            // 末尾独立 if：覆盖当拍 tick 造成的位置变化
            theta    <= 48'd0;
            sec_pos  <= 10'd0;
            sec_down <= 1'b0;
            scan_idx <= ((cfg_load ? scan_type : sty_r) == SCAN_SEC) ?
                        (cfg_load ? scan_start_idx : start_r) :
                        10'd0;           // 与 cfg_load 同拍时旁路旧 sty_r/start_r
        end

    end

end

always @(posedge clk) begin
 if(rst_sync) config_busy<=0;
 else if(cfg_load) config_busy<=scan_en && scan_type!=SCAN_OFF && t_scan!=0;
 else if(div_done) config_busy<=0;
end

endmodule
