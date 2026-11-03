// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/top_8ch_dds.v
// 功能：拆分八通道控制总线，实例化八个 DA_LFM_OUT，并寄存每路 256 bit 样点和 valid。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 8 路 DAC 信号产生顶层：切总线、例化 8 个 DA_LFM_OUT、出口打拍
// =============================================================================

module top_8ch_dds (

    input wire cfg_clk,
    output wire [7:0] sig_cfg_ready,
    input  wire          clk_100Mhz,         // 端口名仍叫 clk_100Mhz，实际 300 MHz
    input  wire          rst,                // 系统复位，命令域电平，本域两级后用
    input  wire [7:0]    ch_en,              // 通道使能，0 则该路保持复位静音

    input  wire          dds_rst_0,          // 通道 0 复位电平
    input  wire [31:0]   phase_init_word_0,  // 通道 0 初相
    input  wire          cfg_load_tgl_0,     // 通道 0 加载翻转
    input  wire          start_tgl_0,        // 通道 0 启动翻转
    input  wire          stop_tgl_0,         // 通道 0 停止翻转
    input  wire          clear_phase_tgl_0,  // 通道 0 清相翻转

    input  wire          dds_rst_1,          // 通道 1 复位电平
    input  wire [31:0]   phase_init_word_1,  // 通道 1 初相
    input  wire          cfg_load_tgl_1,     // 通道 1 加载翻转
    input  wire          start_tgl_1,        // 通道 1 启动翻转
    input  wire          stop_tgl_1,         // 通道 1 停止翻转
    input  wire          clear_phase_tgl_1,  // 通道 1 清相翻转

    input  wire          dds_rst_2,          // 通道 2 复位电平
    input  wire [31:0]   phase_init_word_2,  // 通道 2 初相
    input  wire          cfg_load_tgl_2,     // 通道 2 加载翻转
    input  wire          start_tgl_2,        // 通道 2 启动翻转
    input  wire          stop_tgl_2,         // 通道 2 停止翻转
    input  wire          clear_phase_tgl_2,  // 通道 2 清相翻转

    input  wire          dds_rst_3,          // 通道 3 复位电平
    input  wire [31:0]   phase_init_word_3,  // 通道 3 初相
    input  wire          cfg_load_tgl_3,     // 通道 3 加载翻转
    input  wire          start_tgl_3,        // 通道 3 启动翻转
    input  wire          stop_tgl_3,         // 通道 3 停止翻转
    input  wire          clear_phase_tgl_3,  // 通道 3 清相翻转

    input  wire          dds_rst_4,          // 通道 4 复位电平
    input  wire [31:0]   phase_init_word_4,  // 通道 4 初相
    input  wire          cfg_load_tgl_4,     // 通道 4 加载翻转
    input  wire          start_tgl_4,        // 通道 4 启动翻转
    input  wire          stop_tgl_4,         // 通道 4 停止翻转
    input  wire          clear_phase_tgl_4,  // 通道 4 清相翻转

    input  wire          dds_rst_5,          // 通道 5 复位电平
    input  wire [31:0]   phase_init_word_5,  // 通道 5 初相
    input  wire          cfg_load_tgl_5,     // 通道 5 加载翻转
    input  wire          start_tgl_5,        // 通道 5 启动翻转
    input  wire          stop_tgl_5,         // 通道 5 停止翻转
    input  wire          clear_phase_tgl_5,  // 通道 5 清相翻转

    input  wire          dds_rst_6,          // 通道 6 复位电平
    input  wire [31:0]   phase_init_word_6,  // 通道 6 初相
    input  wire          cfg_load_tgl_6,     // 通道 6 加载翻转
    input  wire          start_tgl_6,        // 通道 6 启动翻转
    input  wire          stop_tgl_6,         // 通道 6 停止翻转
    input  wire          clear_phase_tgl_6,  // 通道 6 清相翻转

    input  wire          dds_rst_7,          // 通道 7 复位电平
    input  wire [31:0]   phase_init_word_7,  // 通道 7 初相
    input  wire          cfg_load_tgl_7,     // 通道 7 加载翻转
    input  wire          start_tgl_7,        // 通道 7 启动翻转
    input  wire          stop_tgl_7,         // 通道 7 停止翻转
    input  wire          clear_phase_tgl_7,  // 通道 7 清相翻转

    input  wire [7:0]    sig_use_new,        // 8 路新链开关，bitN=通道 N
    input  wire [15:0]   sig_type,           // 8×2 信号类型：0 CW / 1 FM / 2 PM
    input  wire [15:0]   sig_freq_type,      // 8×2 捷变类型
    input  wire [7:0]    sig_cw_en,          // 8 路连续波
    input  wire [7:0]    sig_lfm_neg,        // 8 路 LFM 负扫
    input  wire [159:0]  sig_pw_ticks,       // 8×20 脉宽 tick
    input  wire [159:0]  sig_pri_ticks,      // 8×20 默认 PRI tick
    input  wire [383:0]  sig_fc_hz,          // 8×48 中心频率，低 32=FTW
    input  wire [255:0]  sig_hop_bw_hz,      // 8×32 捷变带宽 FTW
    input  wire [47:0]   sig_hop_n,          // 8×6 捷变点数
    input  wire [127:0]  sig_hop_ticks,      // 8×16 脉内 hop 间隔
    input  wire [255:0]  sig_lfm_bw_hz,      // 8×32 调频带宽 FTW
    input  wire [159:0]  sig_lfm_ticks,      // 8×20 调频时长 tick
    input  wire [15:0]   sig_pm_type,        // 8×2 调相/FM 子类型
    input  wire [15:0]   sig_nlfm_type,      // 8×2 NLFM 形状
    input  wire [127:0]  sig_nc,             // 8×16 码元宽
    input  wire [127:0]  sig_ncode,          // 8×16 码长
    input  wire [15:0]   sig_pri_type,       // 8×2 PRI 类型
    input  wire [47:0]   sig_pri_m,          // 8×6 参差项数
    input  wire [47:0]   sig_pri_grp,        // 8×6 脉组大小
    input  wire [47:0]   sig_hop_grp,        // 8×6 hop 脉组
    input  wire [5119:0] sig_pri_tab,        // 8×32×20 PRI 表
    input  wire [7:0]    sig_scan_en,        // 8 路扫描使能
    input  wire [15:0]   sig_scan_type,      // 8×2 扫描类型
    input  wire [255:0]  sig_scan_end,       // 8×32：start/span/target
    input  wire [255:0]  sig_scan_t,         // 8×32 扫描周期
    input  wire [7:0]    sig_beam_we_tgl,    // 8 路方向图写翻转
    input  wire [79:0]   sig_beam_waddr,     // 8×10 compact 写偏移
    input  wire [255:0]  sig_beam_wdata,     // 8×32 {G0,G1,G2,G3}
    input  wire [7:0]    sig_beam_range_tgl, // 8 路 0x7B 区间翻转
    input  wire [79:0]   sig_beam_a0,        // 8×10 非零区起始
    input  wire [79:0]   sig_beam_a1,        // 8×10 非零区结束
    input  wire [7:0]    sig_hop_rand,       // 8 路随机捷变
    input  wire [7:0]    sig_freq_tab_en,    // 8 路频表使能
    input  wire [8191:0] sig_freq_tab,       // 8×32×32 FTW 频表
    input  wire [7:0]    sig_phase_tab_en,   // 8 路相位表使能
    input  wire [8191:0] sig_phase_tab,      // 8×32×32 相位字
    input  wire [159:0]  sig_jit_p0,         // 8×20 抖动中心
    input  wire [159:0]  sig_jit_n,          // 8×20 抖动周期
    input  wire [55:0]   sig_jit_rho,        // 8×7 抖动深度 ρ
    input  wire [7:0]    sig_jit_mode,       // 8 路抖动模式
    input  wire [159:0]  sig_slide_min,      // 8×20 滑变下限
    input  wire [159:0]  sig_slide_max,      // 8×20 滑变上限
    input  wire [159:0]  sig_slide_stp,      // 8×20 滑变步长
    input  wire [7:0]    sig_slide_md,       // 8 路滑变步长模式
    input  wire [7:0]    sig_slide_en,       // 8 路滑变总开关

    output wire [255:0]  dds_sin_0,          // 通道 0 打拍后 16×int16
    output wire          dds_valid_0,        // 通道 0 有效
    output wire [255:0]  dds_sin_1,          // 通道 1 打拍后样点
    output wire          dds_valid_1,        // 通道 1 有效
    output wire [255:0]  dds_sin_2,          // 通道 2 打拍后样点
    output wire          dds_valid_2,        // 通道 2 有效
    output wire [255:0]  dds_sin_3,          // 通道 3 打拍后样点
    output wire          dds_valid_3,        // 通道 3 有效
    output wire [255:0]  dds_sin_4,          // 通道 4 打拍后样点
    output wire          dds_valid_4,        // 通道 4 有效
    output wire [255:0]  dds_sin_5,          // 通道 5 打拍后样点
    output wire          dds_valid_5,        // 通道 5 有效
    output wire [255:0]  dds_sin_6,          // 通道 6 打拍后样点
    output wire          dds_valid_6,        // 通道 6 有效
    output wire [255:0]  dds_sin_7,          // 通道 7 打拍后样点
    output wire          dds_valid_7         // 通道 7 有效
);

localparam ENABLE_DDS_ILA = 1'b0;            // 1=例化 ila_8ch_dds 看 8 路出口


// =============================================================================
// 系统复位两级同步：rst(命令域) → rst_sync1 → rst_sync2=sys_rst
// =============================================================================

(* ASYNC_REG="TRUE" *) reg rst_sync1 = 1'b0;                        // 复位第一级
(* ASYNC_REG="TRUE" *) reg rst_sync2 = 1'b0;                        // 复位第二级

always @(posedge clk_100Mhz) begin           // 实际 300 MHz

    rst_sync1 <= rst;                        // 命令域 rst 第一拍
    rst_sync2 <= rst_sync1;                  // 命令域 rst 第二拍

end

wire sys_rst = rst_sync2;                    // 本域使用的系统复位


// =============================================================================
// 8 路控制打包，后面 generate 按通道切开
// =============================================================================

wire [7:0]   dds_rst_b =                     // 8 路复位电平打包，bitN=通道 N
    {dds_rst_7, dds_rst_6, dds_rst_5, dds_rst_4,
     dds_rst_3, dds_rst_2, dds_rst_1, dds_rst_0};

(* ASYNC_REG="TRUE" *) reg [7:0] ch_enable_meta=0,ch_enable_sync=0;
(* ASYNC_REG="TRUE" *) reg [7:0] channel_reset_meta=8'hff,channel_reset_sync=8'hff;
always @(posedge clk_100Mhz) begin
 ch_enable_meta<=ch_en;ch_enable_sync<=ch_enable_meta;
 channel_reset_meta<=dds_rst_b;channel_reset_sync<=channel_reset_meta;
end

wire [255:0] phase_init_b =                  // 8×32 初相打包
    {phase_init_word_7, phase_init_word_6,
     phase_init_word_5, phase_init_word_4,
     phase_init_word_3, phase_init_word_2,
     phase_init_word_1, phase_init_word_0};

wire [7:0] cfg_load_b =                      // 8 路加载翻转打包
    {cfg_load_tgl_7, cfg_load_tgl_6, cfg_load_tgl_5, cfg_load_tgl_4,
     cfg_load_tgl_3, cfg_load_tgl_2, cfg_load_tgl_1, cfg_load_tgl_0};

wire [7:0] start_b    =                      // 8 路启动翻转打包
    {start_tgl_7, start_tgl_6, start_tgl_5, start_tgl_4,
     start_tgl_3, start_tgl_2, start_tgl_1, start_tgl_0};

wire [7:0] stop_b     =                      // 8 路停止翻转打包
    {stop_tgl_7, stop_tgl_6, stop_tgl_5, stop_tgl_4,
     stop_tgl_3, stop_tgl_2, stop_tgl_1, stop_tgl_0};

wire [7:0] clr_b      =                      // 8 路清相翻转打包
    {clear_phase_tgl_7, clear_phase_tgl_6, clear_phase_tgl_5, clear_phase_tgl_4,
     clear_phase_tgl_3, clear_phase_tgl_2, clear_phase_tgl_1, clear_phase_tgl_0};

wire [255:0] dds_sin_w [0:7];                // 8 路 DA_LFM_OUT 直接出口样点
wire [7:0]   dds_valid_w;                    // 8 路 DA_LFM_OUT 直接出口有效
reg  [255:0] dds_sin_r [0:7];                // 8 路打拍后样点，送给 BD
reg  [7:0]   dds_valid_r;                    // 8 路打拍后有效


// =============================================================================
// 出口映射：打拍寄存器 → 顶层端口
// =============================================================================

assign dds_sin_0   = dds_sin_r[0];           // 通道 0 样点
assign dds_valid_0 = dds_valid_r[0];         // 通道 0 有效
assign dds_sin_1   = dds_sin_r[1];           // 通道 1 样点
assign dds_valid_1 = dds_valid_r[1];         // 通道 1 有效
assign dds_sin_2   = dds_sin_r[2];           // 通道 2 样点
assign dds_valid_2 = dds_valid_r[2];         // 通道 2 有效
assign dds_sin_3   = dds_sin_r[3];           // 通道 3 样点
assign dds_valid_3 = dds_valid_r[3];         // 通道 3 有效
assign dds_sin_4   = dds_sin_r[4];           // 通道 4 样点
assign dds_valid_4 = dds_valid_r[4];         // 通道 4 有效
assign dds_sin_5   = dds_sin_r[5];           // 通道 5 样点
assign dds_valid_5 = dds_valid_r[5];         // 通道 5 有效
assign dds_sin_6   = dds_sin_r[6];           // 通道 6 样点
assign dds_valid_6 = dds_valid_r[6];         // 通道 6 有效
assign dds_sin_7   = dds_sin_r[7];           // 通道 7 样点
assign dds_valid_7 = dds_valid_r[7];         // 通道 7 有效


// =============================================================================
// 8 路出口再打一拍，送给 BD data_sel
// =============================================================================

integer oi;                                  // 输出打拍循环变量

always @(posedge clk_100Mhz) begin           // 实际 300 MHz

    if (sys_rst) begin

        for (oi = 0; oi < 8; oi = oi + 1)
            dds_sin_r[oi] <= 256'd0;         // 各路样点清零
        dds_valid_r <= 8'd0;                 // 8 路有效清零

    end else begin

        for (oi = 0; oi < 8; oi = oi + 1)
            dds_sin_r[oi] <= dds_sin_w[oi];  // 各路样点打一拍
        dds_valid_r <= dds_valid_w;          // 8 路有效打一拍

    end

end


// =============================================================================
// 8 通道例化。切片：1-bit bus[ch]；W-bit bus[ch*W +: W]
// Fs=4.8 GSPS，clk=300 MHz，16 样点/拍，1 tick = 3 clk
// =============================================================================

genvar gi;                                   // 通道号 0..7

generate
    for (gi = 0; gi < 8; gi = gi + 1) begin : g_ch

        DA_LFM_OUT u_dds (

            .cfg_clk(cfg_clk),.cfg_rst(rst),.cfg_ready(sig_cfg_ready[gi]),
            .clk              (clk_100Mhz),                  // 实际 300 MHz 织物时钟
            .dds_rst          (sys_rst | channel_reset_sync[gi] | ~ch_enable_sync[gi]), // 系统复位或通道复位或通道关闭
            .phase_init_word  (phase_init_b[gi*32 +: 32]),    // 本通道 32 bit 初相
            .cfg_load_tgl     (cfg_load_b[gi]),              // 本通道加载翻转
            .start_tgl        (start_b[gi]),                 // 本通道启动翻转
            .stop_tgl         (stop_b[gi]),                  // 本通道停止翻转
            .clear_phase_tgl  (clr_b[gi]),                   // 本通道清相翻转
            .sig_use_new      (sig_use_new[gi]),             // 本通道新链开关
            .sig_type         (sig_type[gi*2 +: 2]),         // 本通道 2 bit 信号类型
            .sig_freq_type    (sig_freq_type[gi*2 +: 2]),    // 本通道 2 bit 捷变类型
            .sig_cw_en        (sig_cw_en[gi]),               // 本通道连续波
            .sig_lfm_neg      (sig_lfm_neg[gi]),             // 本通道 LFM 负扫
            .sig_pw_ticks     (sig_pw_ticks[gi*20 +: 20]),   // 本通道 20 bit 脉宽
            .sig_pri_ticks    (sig_pri_ticks[gi*20 +: 20]),  // 本通道 20 bit 默认 PRI
            .sig_fc_hz        (sig_fc_hz[gi*48 +: 48]),      // 本通道 48 bit 中心，低 32=FTW
            .sig_hop_bw_hz    (sig_hop_bw_hz[gi*32 +: 32]),  // 本通道 32 bit 捷变带宽
            .sig_hop_n        (sig_hop_n[gi*6 +: 6]),        // 本通道 6 bit 捷变点数
            .sig_hop_ticks    (sig_hop_ticks[gi*16 +: 16]),  // 本通道 16 bit hop 间隔
            .sig_lfm_bw_hz    (sig_lfm_bw_hz[gi*32 +: 32]),  // 本通道 32 bit 调频带宽
            .sig_lfm_ticks    (sig_lfm_ticks[gi*20 +: 20]),  // 本通道 20 bit 调频时长
            .sig_pm_type      (sig_pm_type[gi*2 +: 2]),      // 本通道 2 bit FM/PM 子类型
            .sig_nlfm_type    (sig_nlfm_type[gi*2 +: 2]),    // 本通道 2 bit NLFM 形状
            .sig_nc           (sig_nc[gi*16 +: 16]),         // 本通道 16 bit 码元宽
            .sig_ncode        (sig_ncode[gi*16 +: 16]),      // 本通道 16 bit 码长
            .sig_pri_type     (sig_pri_type[gi*2 +: 2]),     // 本通道 2 bit PRI 类型
            .sig_pri_m        (sig_pri_m[gi*6 +: 6]),        // 本通道 6 bit 参差项数
            .sig_pri_grp      (sig_pri_grp[gi*6 +: 6]),      // 本通道 6 bit PRI 脉组
            .sig_hop_grp      (sig_hop_grp[gi*6 +: 6]),      // 本通道 6 bit hop 脉组
            .sig_pri_tab      (sig_pri_tab[gi*640 +: 640]),  // 本通道 32×20 PRI 表
            .sig_scan_en      (sig_scan_en[gi]),             // 本通道扫描使能
            .sig_scan_type    (sig_scan_type[gi*2 +: 2]),    // 本通道 2 bit 扫描类型
            .sig_scan_end     (sig_scan_end[gi*32 +: 32]),   // start/span/target
            .sig_scan_t       (sig_scan_t[gi*32 +: 32]),     // 本通道 32 bit 扫描周期
            .sig_beam_we_tgl     (sig_beam_we_tgl[gi]),
            .sig_beam_waddr      (sig_beam_waddr[gi*10 +: 10]),
            .sig_beam_wdata      (sig_beam_wdata[gi*32 +: 32]),
            .sig_beam_range_tgl  (sig_beam_range_tgl[gi]),
            .sig_beam_a0         (sig_beam_a0[gi*10 +: 10]),
            .sig_beam_a1         (sig_beam_a1[gi*10 +: 10]),
            .sig_hop_rand     (sig_hop_rand[gi]),            // 本通道随机捷变
            .sig_freq_tab_en  (sig_freq_tab_en[gi]),         // 本通道频表使能
            .sig_freq_tab     (sig_freq_tab[gi*1024 +: 1024]), // 本通道 32×32 FTW 频表
            .sig_phase_tab_en (sig_phase_tab_en[gi]),        // 本通道相位表使能
            .sig_phase_tab    (sig_phase_tab[gi*1024 +: 1024]), // 本通道 32×32 相位表
            .sig_jit_p0       (sig_jit_p0[gi*20 +: 20]),     // 本通道 20 bit 抖动中心
            .sig_jit_n        (sig_jit_n[gi*20 +: 20]),      // 本通道 20 bit 抖动周期
            .sig_jit_rho      (sig_jit_rho[gi*7 +: 7]),      // 本通道 7 bit 抖动深度
            .sig_jit_mode     (sig_jit_mode[gi]),            // 本通道抖动模式
            .sig_slide_min    (sig_slide_min[gi*20 +: 20]),  // 本通道 20 bit 滑变下限
            .sig_slide_max    (sig_slide_max[gi*20 +: 20]),  // 本通道 20 bit 滑变上限
            .sig_slide_stp    (sig_slide_stp[gi*20 +: 20]),  // 本通道 20 bit 滑变步长
            .sig_slide_md     (sig_slide_md[gi]),
            .sig_slide_en     (sig_slide_en[gi]),
            .dds_sin          (dds_sin_w[gi]),               // 本通道 16×int16 样点
            .dds_valid        (dds_valid_w[gi])              // 本通道有效
        );

    end
endgenerate


// =============================================================================
// 可选 ILA：抓 8 路打拍后的样点。默认关，避免占 BRAM/时钟扇出
// =============================================================================

generate
    if (ENABLE_DDS_ILA) begin : gen_ila_8ch_dds

        // VU13P: 原调试IP已禁用；保留连线注释供以后自行添加调试。
// ila_8ch_dds u_ila_8ch_dds (
// 
//             .clk    (clk_100Mhz),            // 实际 300 MHz
//             .probe0 (dds_sin_r[0]),          // 通道 0 打拍后样点
//             .probe1 (dds_sin_r[1]),          // 通道 1 打拍后样点
//             .probe2 (dds_sin_r[2]),          // 通道 2 打拍后样点
//             .probe3 (dds_sin_r[3]),          // 通道 3 打拍后样点
//             .probe4 (dds_sin_r[4]),          // 通道 4 打拍后样点
//             .probe5 (dds_sin_r[5]),          // 通道 5 打拍后样点
//             .probe6 (dds_sin_r[6]),          // 通道 6 打拍后样点
//             .probe7 (dds_sin_r[7])           // 通道 7 打拍后样点
// 
//         );

    end
endgenerate

endmodule
