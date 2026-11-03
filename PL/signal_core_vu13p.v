`timescale 1ns / 1ps
// ============================================================================
// 功能：串口控制的八通道波形发生核心，目标 xcvu13p-flga2577-2-i。
// 层级：signal_core_io 负责复位/UART；signal_core_waveform 负责命令到波形连接。
// 时钟：命令域100 MHz；波形域300 MHz，每通道16样点/拍，等效4.8 GSPS。
// 输出：dds_data_0..7 / dds_data_vld_0..7，最低16位为最早样点。
// 启动：释放rst后先发送0x4A=0解除DDS全局复位，再配置、load、start。
// 集成：递归添加本目录全部.v，或在新工程中source ../add_sources.tcl。
// 本顶层只保留接口与功能模块实例，内部参数总线收在signal_core_waveform。
// 2026-10-20：移除ADC上传和BRAM/HMC分支，时钟/波形参数与扫描精度修复沿用。
// ============================================================================
module signal_core_vu13p (
    input  wire           clk_cfg_100m          , // 100 MHz 命令/串口时钟
    input  wire           clk_dds_300m          , // 300 MHz 波形时钟；每通道每拍16个样点
    input  wire           rst                   , // 高有效系统复位；两个时钟稳定后释放
    input  wire           uart_rx               , // RS422逻辑接收，空闲高
    output wire           uart_tx               , // RS422逻辑发送
    output wire [255:0]   dds_data_0            , // 通道0：16×signed int16，lane0在[15:0]
    output wire [255:0]   dds_data_1            , // 通道1：16×signed int16，lane0在[15:0]
    output wire [255:0]   dds_data_2            , // 通道2：16×signed int16，lane0在[15:0]
    output wire [255:0]   dds_data_3            , // 通道3：16×signed int16，lane0在[15:0]
    output wire [255:0]   dds_data_4            , // 通道4：16×signed int16，lane0在[15:0]
    output wire [255:0]   dds_data_5            , // 通道5：16×signed int16，lane0在[15:0]
    output wire [255:0]   dds_data_6            , // 通道6：16×signed int16，lane0在[15:0]
    output wire [255:0]   dds_data_7            , // 通道7：16×signed int16，lane0在[15:0]
    output wire           dds_data_vld_0        , // 通道0：与数据同拍，无ready回压
    output wire           dds_data_vld_1        , // 通道1：与数据同拍，无ready回压
    output wire           dds_data_vld_2        , // 通道2：与数据同拍，无ready回压
    output wire           dds_data_vld_3        , // 通道3：与数据同拍，无ready回压
    output wire           dds_data_vld_4        , // 通道4：与数据同拍，无ready回压
    output wire           dds_data_vld_5        , // 通道5：与数据同拍，无ready回压
    output wire           dds_data_vld_6        , // 通道6：与数据同拍，无ready回压
    output wire           dds_data_vld_7          // 通道7：与数据同拍，无ready回压
);

// 两个子模块之间仅交换串口字节和同步后的命令域复位。
wire command_reset;
wire [7:0] rx_data, tx_data;
wire rx_valid, tx_done, tx_req, tx_override;

signal_core_io u_io (
    .clk_cfg_100m           (clk_cfg_100m),
    .rst                    (rst),
    .uart_rx                (uart_rx),
    .uart_tx                (uart_tx),
    .command_reset          (command_reset),
    .rx_data                (rx_data),
    .rx_valid               (rx_valid),
    .tx_done                (tx_done),
    .tx_req                 (tx_req),
    .tx_override            (tx_override),
    .tx_data                (tx_data)
);

signal_core_waveform u_waveform (
    .clk_cfg_100m           (clk_cfg_100m),
    .clk_dds_300m           (clk_dds_300m),
    .command_reset          (command_reset),
    .rx_data                (rx_data),
    .rx_valid               (rx_valid),
    .tx_done                (tx_done),
    .tx_req                 (tx_req),
    .tx_override            (tx_override),
    .tx_data                (tx_data),
    .dds_data_0             (dds_data_0),
    .dds_data_1             (dds_data_1),
    .dds_data_2             (dds_data_2),
    .dds_data_3             (dds_data_3),
    .dds_data_4             (dds_data_4),
    .dds_data_5             (dds_data_5),
    .dds_data_6             (dds_data_6),
    .dds_data_7             (dds_data_7),
    .dds_data_vld_0         (dds_data_vld_0),
    .dds_data_vld_1         (dds_data_vld_1),
    .dds_data_vld_2         (dds_data_vld_2),
    .dds_data_vld_3         (dds_data_vld_3),
    .dds_data_vld_4         (dds_data_vld_4),
    .dds_data_vld_5         (dds_data_vld_5),
    .dds_data_vld_6         (dds_data_vld_6),
    .dds_data_vld_7         (dds_data_vld_7)
);
endmodule
