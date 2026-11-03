`timescale 1ns / 1ps
// ============================================================================
// 模块：signal_core_io
// 功能：100 MHz域的复位释放、UART RX同步和RS422字节收发。
// rst异步置位、三拍同步释放；RX两级同步及初始空闲电平沿用原顶层。
// tx_override=1时发送命令应答；=0时由RS422按原协议逐字节回显。
// UART收发器的上电初始化及约115207 baud分频保持原实现。
// ============================================================================
module signal_core_io (
    input wire clk_cfg_100m,
    input wire rst,
    input wire uart_rx,
    output wire uart_tx,
    output wire command_reset,
    output wire [7:0] rx_data,
    output wire rx_valid,
    output wire tx_done,
    input wire tx_req,
    input wire tx_override,
    input wire [7:0] tx_data
);
(* ASYNC_REG="TRUE" *) reg [2:0] command_reset_pipe = 3'b111;
always @(posedge clk_cfg_100m or posedge rst)
    if (rst) command_reset_pipe <= 3'b111;
    else command_reset_pipe <= {command_reset_pipe[1:0],1'b0};
assign command_reset = command_reset_pipe[2];

(* ASYNC_REG="TRUE" *) reg [1:0] uart_rx_sync = 2'b11;
always @(posedge clk_cfg_100m)
    if (command_reset) uart_rx_sync <= 2'b11;
    else uart_rx_sync <= {uart_rx_sync[0],uart_rx};

RS422 u_rs422 (
    .clk_100Mhz(clk_cfg_100m),
    .rs422_rxd(uart_rx_sync[1]), .rs422_txd(uart_tx),
    .rs422_re(), .rs422_de(),
    .rxd_data(rx_data), .rxd_valid(rx_valid),
    .txd_done(tx_done), .txd_en(tx_req),
    .txd_ad_flag(tx_override), .txd_data(tx_data)
);
endmodule
