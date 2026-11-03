// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_carrier.v
// 功能：根据单载频、脉内/脉间/脉组事件安排载波频点，并支持频表和随机跳频下标。
// ============================================================================
`timescale 1ns/1ps
// Carrier scheduler. pulse_rise is already a one-clock event from the PRI
// scheduler, so it must not be qualified with that scheduler's earlier tick.
module da_sig_carrier (
 input wire clk,rst_sync,start_edge,active,tick,pulse_gate,pulse_rise,period_rise,group_rise,
 input wire [1:0] freq_type,
 input wire [5:0] hop_n,
 input wire [15:0] hop_ticks,
 input wire [31:0] hop_start,hop_step,
 input wire hop_rand,freq_tab_en,
 input wire [1023:0] freq_tab_bus,
 output reg [31:0] carrier,
 output reg [4:0] hop_idx,
 output wire [31:0] carrier_emit
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

 reg [4:0] hop=0;
 reg [15:0] hop_sub=0;
 wire [5:0] n_use=(hop_n<1)?6'd1:(hop_n>32)?6'd32:hop_n;
 wire [4:0] last=n_use-1'b1;
 wire [15:0] span=(hop_ticks==0)?16'd1:hop_ticks;
 wire intra=freq_type==FREQ_INTRA;
 wire hop_rst=intra && pulse_rise;
 reg hop_ge=0;
 always @(posedge clk) begin
  if(rst_sync || start_edge) hop_ge<=0;
  else hop_ge<=({1'b0,hop_sub}+17'd1>={1'b0,span});
 end
 wire hop_adv=intra ? (tick && pulse_gate && !pulse_rise && hop_ge) :
              (freq_type==FREQ_INTER)?period_rise:(freq_type==FREQ_GROUP)?group_rise:1'b0;
 wire [4:0] ordinal_next=(hop>=last)?5'd0:hop+1'b1;
 wire use_tab=freq_tab_en && freq_type!=FREQ_CW;
 wire [4:0] random_index;
 da_sig_hop_rand u_rand(.clk(clk),.rst_sync(rst_sync),.start_edge(start_edge),
                       .hop_n(hop_n),.hop_rand_n(random_index));
 reg [4:0] random_tag0=0,random_tag1=0,random_tag2=0;
 (* use_dsp="yes" *) reg [31:0] product0=0;
 reg [31:0] product1=0,random_arith2=0;
 reg [31:0] random_tab0=0,random_tab1=0,random_tab2=0;
 reg [31:0] ordinal_ftw=0;
 always @(posedge clk) begin
  if(rst_sync || start_edge) begin
   random_tag0<=0;random_tag1<=0;random_tag2<=0;
   product0<=0;product1<=0;random_arith2<=hop_start;
   random_tab0<=freq_tab_bus[31:0];random_tab1<=freq_tab_bus[31:0];random_tab2<=freq_tab_bus[31:0];
   ordinal_ftw<=hop_start;
  end else begin
   random_tag0<=random_index;random_tag1<=random_tag0;random_tag2<=random_tag1;
   product0<=hop_step*random_index;product1<=product0;random_arith2<=hop_start+product1;
   random_tab0<=freq_tab_bus[random_index*32+:32];random_tab1<=random_tab0;random_tab2<=random_tab1;
   ordinal_ftw<=use_tab ? freq_tab_bus[ordinal_next*32+:32] :
                         ((hop>=last)?hop_start:carrier+hop_step);
  end
 end
 wire [4:0] selected_index=hop_rand?random_tag2:(hop_rst?5'd0:ordinal_next);
 wire [31:0] selected_ftw=hop_rand?(use_tab?random_tab2:random_arith2):
                         hop_rst?(use_tab?freq_tab_bus[31:0]:hop_start):ordinal_ftw;
 // A new pulse sees its new start frequency immediately. Ordinary intra-pulse
 // changes take effect after the last vector of the previous dwell interval.
 wire pulse_inter_update=pulse_rise && active &&
    ((freq_type==FREQ_INTER && period_rise)||(freq_type==FREQ_GROUP && group_rise));
 assign carrier_emit=(hop_rst || pulse_inter_update)?selected_ftw:carrier;
 always @(posedge clk) begin
  if(rst_sync || start_edge) begin
   hop<=0;hop_idx<=0;hop_sub<=0;carrier<=use_tab?freq_tab_bus[31:0]:hop_start;
  end else if(hop_rst || (active && hop_adv)) begin
   hop<=selected_index;hop_idx<=selected_index;carrier<=selected_ftw;hop_sub<=0;
  end else if(active && intra && tick && pulse_gate) begin
   hop_sub<=hop_sub+1'b1;
  end
 end
endmodule
