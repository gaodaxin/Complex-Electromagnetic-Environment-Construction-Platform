// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_nlfm_saw.v
// 功能：复用线性调频模块构造周期回绕的锯齿 NLFM 频偏支路。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_nlfm_saw
// 功能: 产生锯齿型非线性调频频偏；每次脉冲开始时重装起始频偏。
// 单位: 频偏和步进均在有符号 32 bit FTW 域中运算。
// =============================================================================

module da_sig_nlfm_saw (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        start_edge,       // 启动沿；重新装 fm_start
    input  wire        active,           // 相位累加器工作时才推进
    input  wire        pulse_rise,       // 新脉冲；锯齿从 fm_start 再开始
    input  wire        pulse_gate,       // 脉宽门；只在脉内扫频
    input  wire        enable,           // nlfm_type=锯齿时使能
    input  wire signed [31:0] fm_start,  // 起始频偏，通常 −B/2
    input  wire signed [31:0] slope_x16, // 每拍频偏增量 16S
    input  wire [31:0] n_sample,         // 一个完整锯齿的总样点数
    output wire signed [31:0] fm_mod,    // 当前锯齿频偏
    output wire signed [31:0] fm_next    // 下一拍频偏预览
);

// 锯齿与 LFM 同一套斜升回绕：−B/2 → +B/2 后直接回到 −B/2

da_sig_lfm u_saw (

    .clk         (clk),                  // 工作时钟
    .rst_sync    (rst_sync),             // 同步复位
    .start_edge  (start_edge),           // 启动
    .active      (active),               // 累加器有效才扫
    .pulse_rise  (pulse_rise),           // 每发重新开始
    .pulse_gate  (pulse_gate),           // 只在脉内
    .enable      (enable),               // 锯齿使能
    .fm_start    (fm_start),             // 起点频偏
    .slope_x16   (slope_x16),            // 每拍 +16S
    .n_sample    (n_sample),             // 周期样点数
    .fm_offset   (fm_mod),               // 当前频偏
    .fm_next     (fm_next)               // 下一拍频偏

);

endmodule
