// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_scan.v
// 功能：串接扫描角度 NCO、相对目标方向图查表和十六路增益乘法，关闭扫描时等延迟直通。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_scan
// 功能: 扫描链顶层，串接扫描角 NCO、compact 方向图查表和 16 路增益乘法。
// 数据流: scan_idx -> 相对目标角 -> G -> 16×样点缩放；关闭扫描时等延迟直通。
//
// 配置来源（由 da_sig_host_regs 解码）:
//   0x73 : scan_en/scan_type；0x74 : start/span/target；0x75 : t_scan。
//   0x7B : 方向图非零区间 a0/a1；0x79 : 每次写入连续 4 个增益 G。
//
// 坐标约定:
//   scan_idx/target_idx 均为一周 1024 点的绝对方位地址，地址加减按模 1024。
//   relative_idx=(scan_idx-target_idx) mod 1024，方向图表使用该相对地址。
//   1 个地址格对应 360/1024 度，G 为 0..255 的线性幅度系数。
//
// 时序约定:
//   配置和方向图应在停止状态写完，最后用 cfg_load 锁存，再发 start_edge。
//   NCO/方向图查表存在控制流水，增益模块再统一延迟数据、on 和 valid；
//   因而 dout_valid 始终只描述同拍 dout，不能拿 din_valid 直接解释 dout。
// =============================================================================

module da_sig_scan (

    output wire config_busy, beam_busy,
    input  wire         clk,             // FPGA 工作时钟
    input  wire         rst_sync,        // 同步复位，高有效
    input  wire         start_edge,      // 启动沿；扫描位置回起点，保持已算好的 FTW
    input  wire         cfg_load,        // 加载沿；锁存扫描配置并启动 2^32/T 除法
    input  wire         tick,            // 慢时间基准，当前每 3 个 300 MHz clk 一次
    input  wire         scan_en,         // 扫描总使能；仅 cfg_load 时采纳
    input  wire [1:0]   scan_type,       // 0 关 / 1 圆扫 / 2 扇扫
    input  wire [9:0]   scan_start_idx,  // 扇扫绝对起始地址，10 bit 加法自然跨零回绕
    input  wire [9:0]   scan_span_idx,   // 扇扫跨度；0 表示固定在起始地址
    input  wire [9:0]   target_idx,      // 目标绝对方位，cfg_load 时锁存到查表模块
    input  wire [31:0]  t_scan,          // 扫描一周的周期 T，单位 tick
    input  wire         beam_range_we,   // 0x7B 提交脉冲：锁存区间并清空 compact 表
    input  wire [9:0]   beam_a0,         // 非零区起始相对地址
    input  wire [9:0]   beam_a1,         // 非零区结束相对地址
    input  wire         beam_tab_we,     // 0x79 提交脉冲：一次写连续 4 个 G
    input  wire [9:0]   beam_tab_waddr,  // compact 写偏移，真正存储范围 0..255
    input  wire [31:0]  beam_tab_wdata,  // {G0,G1,G2,G3}
    input  wire [255:0] din,             // 16×int16 正弦
    input  wire         din_valid,       // 正弦有效
    output wire [255:0] dout,            // 乘 G 后的样点
    output wire         dout_valid,      // 与 dout 对齐的有效
    output wire [9:0]   scan_idx,        // 观察：当前扫描地址
    output wire [9:0]   relative_idx,    // 观察：相对目标的地址
    output wire [7:0]   g,               // 观察：当前增益
    output wire         beam_valid       // 观察：方向图已加载
);

wire on_r;                               // load 锁存后的扫描开关，送增益支路选择


// =============================================================================
// 第 1 级：扫描角发生器
// 圆扫直接输出 theta 高 10 bit；扇扫输出 start+sec_pos 的往返序列。
// =============================================================================

da_sig_scan_nco u_nco (

    .clk             (clk),              // 工作时钟
    .rst_sync        (rst_sync),         // 同步复位
    .start_edge      (start_edge),       // 启动：θ 回 0
    .cfg_load        (cfg_load),         // 加载：锁存并重算 FTW
    .tick            (tick),             // NCO 时间基准
    .scan_en         (scan_en),          // 扫描使能，load 时锁存
    .scan_type       (scan_type),        // 圆扫/扇扫
    .scan_start_idx  (scan_start_idx),   // 扇扫起点
    .scan_span_idx   (scan_span_idx),    // 扇扫跨度
    .t_scan          (t_scan),           // 扫描周期 T
    .scan_idx        (scan_idx),         // 当前扫描地址
    .config_busy(config_busy),
    .on_r            (on_r)              // 锁存后的开关

);


// =============================================================================
// 第 2 级：相对角换算和方向图查表
// 主机只下载非零主瓣，区间外自动输出 G=0，减少 RS422 表传输量。
// =============================================================================

da_sig_scan_beam u_beam (

    .clk             (clk),              // 工作时钟
    .rst_sync        (rst_sync),         // 同步复位，不复位 BRAM
    .cfg_load        (cfg_load),         // 最后 load 时锁存目标并确认表有效
    .target_idx      (target_idx),       // 目标绝对方位
    .scan_idx        (scan_idx),         // 当前波束绝对方位
    .beam_range_we   (beam_range_we),    // 锁存 a0/a1
    .beam_a0         (beam_a0),          // 主瓣起始相对地址
    .beam_a1         (beam_a1),          // 主瓣结束相对地址
    .beam_tab_we     (beam_tab_we),      // 4 点写使能
    .beam_tab_waddr  (beam_tab_waddr),   // compact 偏移
    .beam_tab_wdata  (beam_tab_wdata),   // 四个连续 G
    .relative_idx    (relative_idx),     // scan - target
    .g               (g),                // 查表 G，未加载为 255
    .config_busy(beam_busy),
    .beam_valid      (beam_valid)        // 表已加载

);


// =============================================================================
// 第 3 级：16 路样点增益
// 开扫描走 x*G/256，关扫描走等延迟直通，保证切换时 AXIS 时序不变化。
// =============================================================================

da_sig_scan_gain u_gain (

    .clk        (clk),                   // 工作时钟
    .rst_sync   (rst_sync),              // 同步复位
    .on         (on_r),                  // 关扫描时直通对齐
    .g          (g),                     // 当前 G；gain 内与 din 对齐进 DSP
    .din        (din),                   // 16 路正弦
    .din_valid  (din_valid),             // 正弦有效
    .dout       (dout),                  // y = (x*G)>>>8
    .dout_valid (dout_valid)             // 与 dout 对齐

);

endmodule
