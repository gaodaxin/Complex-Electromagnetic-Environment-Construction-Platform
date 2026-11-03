// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_lfm_input_sync.v
// 功能：将命令域电平及 load/start/stop/clear toggle 同步到波形域，并把动作还原为单拍脉冲。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_lfm_input_sync
// 功能: 完成命令域到 DAC fabric 域的控制跨时钟处理。
// 规则: 电平参数采用两级同步；load/start/stop/clear 使用 toggle 同步后异或
//       还原为单拍脉冲，保证慢命令域事件不会在快时钟域丢失。
// 注意: 上游必须保证同一 toggle 的两次翻转间隔大于本模块同步链延迟。
// =============================================================================

module da_lfm_input_sync (

    input  wire        clk,              // FPGA 工作时钟，当前 300 MHz
    input  wire        dds_rst,          // 复位电平，来自命令域约 100 MHz
    input  wire [31:0] phase_init_word,  // 初相，命令域
    input  wire        cfg_load_tgl,     // 加载翻转
    input  wire        start_tgl,        // 启动翻转
    input  wire        stop_tgl,         // 停止翻转
    input  wire        clear_phase_tgl,  // 清相翻转
    output wire        dds_rst_sync,     // 本域两级同步后的复位
    output wire [31:0] phase_init_sync,  // 本域两级同步后的初相
    output wire        cfg_load_edge,    // 加载沿，1 拍
    output wire        start_edge,       // 启动沿，1 拍
    output wire        stop_edge,        // 停止沿，1 拍
    output wire        clear_phase_edge  // 清相沿，1 拍
);

(* ASYNC_REG="TRUE" *) reg rst1 = 1'b0, rst2 = 1'b0;            // 复位两级同步
reg [31:0] ph1 = 32'd0, ph2 = 32'd0;     // 初相两级同步

(* ASYNC_REG="TRUE" *) reg load_m  = 1'b0, load_s  = 1'b0, load_l  = 1'b0;
(* ASYNC_REG="TRUE" *) reg start_m = 1'b0, start_s = 1'b0, start_l = 1'b0;
(* ASYNC_REG="TRUE" *) reg stop_m  = 1'b0, stop_s  = 1'b0, stop_l  = 1'b0;
(* ASYNC_REG="TRUE" *) reg clr_m   = 1'b0, clr_s   = 1'b0, clr_l   = 1'b0;
                                         // *_m 亚稳态采样，*_s 同步后，*_l 上一拍


assign dds_rst_sync    = rst2;           // 两级后的复位电平
assign phase_init_sync = ph2;            // 两级后的初相
assign cfg_load_edge    = load_s  ^ load_l;  // toggle 变 1 拍沿
assign start_edge       = start_s ^ start_l;
assign stop_edge        = stop_s  ^ stop_l;
assign clear_phase_edge = clr_s   ^ clr_l;


always @(posedge clk) begin

    rst1 <= dds_rst;                     // 复位第一级
    rst2 <= rst1;                        // 复位第二级

    ph1  <= phase_init_word;             // 初相第一级
    ph2  <= ph1;                         // 初相第二级

    load_m  <= cfg_load_tgl;  load_s  <= load_m;  load_l  <= load_s;
    start_m <= start_tgl;     start_s <= start_m; start_l <= start_s;
    stop_m  <= stop_tgl;      stop_s  <= stop_m;  stop_l  <= stop_s;
    clr_m   <= clear_phase_tgl; clr_s <= clr_m;   clr_l   <= clr_s;

end

endmodule
