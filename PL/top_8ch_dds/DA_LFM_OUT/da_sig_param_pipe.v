// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_param_pipe.v
// 功能：通过保持式请求/应答跨域传递参数快照，目的域原子捕获宽参数，并报告源域是否可接收下一次配置。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_param_pipe
// 功能: 命令域冻结提交快照，DAC域一次捕获；捕获应答后才接受下一次load。
// 时钟: 输出全部稳定在 DAC fabric 时钟域，供配置计算和实时波形链使用。
// 设计目的: 隔离跨域组合路径，并保证 PRI/频率/相位表在一次运行期间不撕裂。
// =============================================================================

module da_sig_param_pipe (

    input wire cfg_clk, cfg_rst, cfg_load_tgl, capture_release,
    output wire cfg_ready,
    output reg cfg_captured=0,
    input wire [31:0] phase_init_word,
    output wire [31:0] phase_active,
    input  wire          clk,            // FPGA 工作时钟，当前 300 MHz
    input  wire          rst_sync,       // 同步复位，高有效
    input  wire          cfg_load_edge,  // 加载沿，1 拍；仅此时快照宽表

    input  wire          sig_use_new,    // 新链开关，1 时走本链路
    input  wire [1:0]    sig_type,       // 信号类型：0 CW / 1 FM / 2 PM
    input  wire [1:0]    sig_freq_type,  // 捷变类型：0 单载频 / 1 脉内 / 2 脉间 / 3 脉组
    input  wire          sig_cw_en,      // CW 连续波使能，1 时 pulse_gate 恒为 1
    input  wire          sig_lfm_neg,    // LFM 负扫，1 时斜率与 fm_start 取反
    input  wire [19:0]   sig_pw_ticks,   // 脉宽，单位 tick
    input  wire [19:0]   sig_pri_ticks,  // 默认 PRI；表项 0 为 0 时用来顶上
    input  wire [47:0]   sig_fc_hz,      // 中心频率，48 bit，低 32 bit 为 FTW
    input  wire [31:0]   sig_hop_bw_hz,  // 捷变带宽 FTW
    input  wire [5:0]    sig_hop_n,      // 捷变频点数，一般 1~32
    input  wire [15:0]   sig_hop_ticks,  // 脉内 hop 间隔，单位 tick
    input  wire [31:0]   sig_lfm_bw_hz,  // 调频带宽 FTW
    input  wire [19:0]   sig_lfm_ticks,  // 调频时长，单位 tick
    input  wire [1:0]    sig_pm_type,    // FM 子类型或 PM 子类型
    input  wire [1:0]    sig_nlfm_type,  // NLFM 形状：三角/正弦/锯齿
    input  wire [15:0]   sig_nc,         // 码元宽度，单位 tick
    input  wire [15:0]   sig_ncode,      // 码长
    input  wire [1:0]    sig_pri_type,   // PRI 类型：固定/参差/抖动等
    input  wire [5:0]    sig_pri_m,      // 参差 PRI 个数
    input  wire [5:0]    sig_pri_grp,    // 脉组参差时每组脉冲数
    input  wire [5:0]    sig_hop_grp,    // 频率捷变时每组 PRI 数
    input  wire [639:0]  sig_pri_tab,    // PRI 表，32 个 × 20 bit = 640 bit
    input  wire          sig_scan_en,    // 天线扫描使能
    input  wire [1:0]    sig_scan_type,  // 扫描类型：0 关 / 1 圆扫 / 2 扇扫
    input  wire [31:0]   sig_scan_end,   // [9:0] start [19:10] span [29:20] target
    input  wire [31:0]   sig_scan_t,     // 扫描一周周期，单位 tick
    input  wire          sig_hop_rand,   // 捷变随机抽点使能
    input  wire          sig_freq_tab_en, // 1=使用频表，0=等差栅
    input  wire [1023:0] sig_freq_tab,   // 频表，32 个 × 32 bit = 1024 bit
    input  wire          sig_phase_tab_en, // 1=使用相位表
    input  wire [1023:0] sig_phase_tab,  // 相位表，32 个 × 32 bit = 1024 bit
    input  wire [19:0]   sig_jit_p0,     // 抖动模式基准 PRI
    input  wire [19:0]   sig_jit_n,      // 抖动序列长度或正弦调制周期
    input  wire [6:0]    sig_jit_rho,    // 抖动深度
    input  wire          sig_jit_mode,   // 抖动模式：0 正弦 / 1 伪随机
    input  wire [19:0]   sig_slide_min,  // PRI 滑变最小值
    input  wire [19:0]   sig_slide_max,  // PRI 滑变最大值
    input  wire [19:0]   sig_slide_stp,  // PRI 滑变步长
    input  wire          sig_slide_md,   // 滑变模式：0 固定步长 / 1 随机步长
    input  wire          sig_slide_en,   // 滑变总开关

    output wire use_new,        // 打拍后的新链开关
    output wire  [1:0] st,             // 打拍后的信号类型
    output wire  [1:0] ft,             // 打拍后的捷变类型
    output wire cw,             // 打拍后的连续波使能
    output wire ln,             // 打拍后的 LFM 负扫
    output wire  [19:0] pwt,            // 打拍后的脉宽
    output wire  [19:0] prt,            // 打拍后的默认 PRI
    output wire  [31:0] fch,            // 打拍后的中心 FTW，取 sig_fc_hz 低 32 bit
    output wire  [31:0] hbw,            // 打拍后的捷变带宽 FTW
    output wire  [5:0] hn,             // 打拍后的捷变点数
    output wire  [15:0] ht,             // 打拍后的脉内 hop 间隔
    output wire  [31:0] lbw,            // 打拍后的调频带宽 FTW
    output wire  [19:0] lt,             // 打拍后的调频时长
    output wire  [1:0] pm,             // 打拍后的 FM/PM 子类型
    output wire  [1:0] nl,             // 打拍后的 NLFM 形状
    output wire  [15:0] nc,             // 打拍后的码元宽度
    output wire  [15:0] nk,             // 打拍后的码长
    output wire  [1:0] pty,            // 打拍后的 PRI 类型
    output wire  [5:0] prm,            // 打拍后的参差项数
    output wire  [5:0] pgr,            // 打拍后的 PRI 脉组大小
    output wire  [5:0] hgr,            // 打拍后的 hop 脉组大小
    output wire sen,            // 打拍后的扫描使能
    output wire  [1:0] sty,            // 打拍后的扫描类型
    output wire  [31:0] sed,            // 打拍后的 start/span/target
    output wire  [31:0] stm,            // 打拍后的扫描周期
    output wire hr,             // 打拍后的随机捷变使能
    output wire fte,            // 打拍后的频表使能
    output wire pte,            // 打拍后的相位表使能
    output wire jm,             // 打拍后的抖动模式
    output wire smd,            // 打拍后的滑变步长模式
    output wire slen,           // 打拍后的滑变总开关
    output wire  [19:0] jp0,            // 打拍后的抖动基准 PRI
    output wire  [19:0] jn,             // 打拍后的抖动周期
    output wire  [6:0] jrh,            // 打拍后的抖动深度
    output wire  [19:0] smin,           // 打拍后的滑变下限
    output wire  [19:0] smax,           // 打拍后的滑变上限
    output wire  [19:0] sst,            // 打拍后的滑变步长
    output wire  [639:0] pri_tab_s,      // cfg_load 快照后的 PRI 表
    output wire  [1023:0] freq_tab_s,     // cfg_load 快照后的频表
    output wire  [1023:0] phase_tab_s     // cfg_load 快照后的相位表
);
// The source snapshot is immutable until the complete configuration has been
// accepted and derived values/table copies are ready in the destination.
localparam integer CFG_W=3141;
(* DONT_TOUCH="TRUE" *) reg [CFG_W-1:0] src_snapshot=0;
(* DONT_TOUCH="TRUE" *) reg [CFG_W-1:0] dst_snapshot=0;
assign {use_new,st,ft,cw,ln,pwt,prt,fch,hbw,hn,ht,lbw,lt,pm,nl,nc,nk,pty,prm,pgr,hgr,sen,sty,sed,stm,hr,fte,pte,jm,smd,slen,jp0,jn,jrh,smin,smax,sst,pri_tab_s,freq_tab_s,phase_tab_s,phase_active}=dst_snapshot;
reg src_req=0, src_seen=0;
(* ASYNC_REG="TRUE" *) reg [2:0] ack_sync=0, req_sync=0;
reg dst_seen=0, dst_ack=0, capture_pending=0;
reg [1:0] release_delay=0;
assign cfg_ready=(src_req==ack_sync[2]) && (src_seen==cfg_load_tgl);
always @(posedge cfg_clk) begin
 if(cfg_rst) begin src_snapshot<=0;src_req<=0;src_seen<=0;ack_sync<=0;end
 else begin
  ack_sync<={ack_sync[1:0],dst_ack};
  if((src_seen!=cfg_load_tgl) && (src_req==ack_sync[2])) begin
   src_snapshot<={sig_use_new,sig_type,sig_freq_type,sig_cw_en,sig_lfm_neg,sig_pw_ticks,sig_pri_ticks,sig_fc_hz[31:0],sig_hop_bw_hz,sig_hop_n,sig_hop_ticks,sig_lfm_bw_hz,sig_lfm_ticks,sig_pm_type,sig_nlfm_type,sig_nc,sig_ncode,sig_pri_type,sig_pri_m,sig_pri_grp,sig_hop_grp,sig_scan_en,sig_scan_type,sig_scan_end,sig_scan_t,sig_hop_rand,sig_freq_tab_en,sig_phase_tab_en,sig_jit_mode,sig_slide_md,sig_slide_en,sig_jit_p0,sig_jit_n,sig_jit_rho,sig_slide_min,sig_slide_max,sig_slide_stp,((sig_pri_tab[19:0]==0)?{sig_pri_tab[639:20],sig_pri_ticks}:sig_pri_tab),sig_freq_tab,sig_phase_tab,phase_init_word};
   src_req<=~src_req;src_seen<=cfg_load_tgl;
  end
 end
end
always @(posedge clk) begin
 if(rst_sync) begin
  req_sync<=0;dst_seen<=0;dst_ack<=0;capture_pending<=0;release_delay<=0;cfg_captured<=0;
  dst_snapshot<=0;
 end else begin
  req_sync<={req_sync[1:0],src_req};cfg_captured<=0;
  if(req_sync[2]!=dst_seen) begin
   dst_snapshot<=src_snapshot;
   dst_seen<=req_sync[2];cfg_captured<=1;capture_pending<=1;release_delay<=2;
  end else if(release_delay!=0) release_delay<=release_delay-1'b1;
  else if(capture_pending && capture_release) begin dst_ack<=dst_seen;capture_pending<=0;end
 end
end
endmodule
