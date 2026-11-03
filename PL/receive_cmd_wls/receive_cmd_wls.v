// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/RS422/receive_cmd_wls.v
// 功能：仲裁旧 7E 7E 短帧和 AA55 长帧，生成八通道参数及动作 toggle，安排短帧回显和长帧ACK。
// 2026-10-20：删除采集/上传支路；0x10/0x11仅消费旧帧并回显。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: receive_cmd_wls
// 功能: RS422 命令域总控制器，兼容旧 7E 7E 短帧并接入 AA55/256B 长帧。
// 数据流: 串口字节 -> 协议仲裁 -> 旧命令状态机或 da_sig_excel_decode ->
//         da_sig_host_regs -> 8 通道参数总线及 load/start/stop toggle。
// 仲裁: 长帧被认领后冻结旧状态机并关闭逐字节回显；解码结束后独占发送 4B ACK。
// 时钟域: 本模块及寄存器工作在 100 MHz 命令域，DDS 控制通过 toggle 跨到
//         300 MHz，各宽参数由 DA_LFM_OUT 在 load 前后稳定采样。
//
// 两套接收协议:
//   旧短帧  7E 7E | CMD | [CH] | DATA | 00 00 00 | AA AA
//             - s1..s10 顺序接收；每个 rxd_valid 字节由 RS422 顶层直接回显。
//             - 参数先进入 temp_*，只有完整帧尾通过后才在 s10 生效。
//   新长帧  AA 55 | 固定 256 字节 | checksum | CC DD
//             - 仅当旧状态机位于 s1 时允许 decoder 认领 AA。
//             - 接收、校验和内部 commit 期间禁止旧状态机解释载荷中的 7E。
//             - 不逐字节回显，结束后返回 AA 55 | seq | 00/01。
//
// 写入与动作分离:
//   sig_commit/xl_commit 只更新 da_sig_host_regs 的命令域影子寄存器；
//   cfg_load/start/stop/clear 使用“每发生一次动作就翻转一次”的 toggle。下游同步器
//   检测 toggle 变化得到单拍脉冲，因此 toggle 不能在复位之外被强制清零代替翻转。
//   长帧 on=FF 由 decoder 先产生 load，再隔一个 100 MHz 周期产生 start。
//
// UART 发送优先级（高到低）:
//   长帧 ACK > 旧命令发送口/默认回显。
//   tx_override 告诉 RS422.v 当前由本模块接管 TX，否则 RS422.v 回显 RX 字节。
//
// 注意:
//   信号链参数输出采用 sig_* 扁平总线，
//   第 ch 路 W 位字段由下游使用 bus[ch*W +: W] 取出。
// =============================================================================

module receive_cmd_wls (
    input wire [7:0] sig_cfg_ready,
    input wire         clk_100Mhz,
    input wire         rst,
    input wire [7:0]   rxd_data,
    input wire         txd_done,
    input wire         rxd_valid,
    output wire        tx_req,
    output wire        tx_override,
    output wire [7:0]   txd_data_out,
    output reg         dds_rst_0= 1'b0,
    output reg         dds_rst_1= 1'b0,
    output reg         dds_rst_2= 1'b0,
    output reg         dds_rst_3= 1'b0,
    output reg         dds_rst_4= 1'b0,
    output reg         dds_rst_5= 1'b0,
    output reg         dds_rst_6= 1'b0,
    output reg         dds_rst_7= 1'b0,
    output reg [31:0]  phase_init_word_0= 32'd0,
    output reg         cfg_load_tgl_0= 1'b0,
    output reg         start_tgl_0= 1'b0,
    output reg         stop_tgl_0= 1'b0,
    output reg         clear_phase_tgl_0= 1'b0,
    output reg [31:0]  phase_init_word_1= 32'd0,
    output reg         cfg_load_tgl_1= 1'b0,
    output reg         start_tgl_1= 1'b0,
    output reg         stop_tgl_1= 1'b0,
    output reg         clear_phase_tgl_1= 1'b0,
    output reg [31:0]  phase_init_word_2= 32'd0,
    output reg         cfg_load_tgl_2= 1'b0,
    output reg         start_tgl_2= 1'b0,
    output reg         stop_tgl_2= 1'b0,
    output reg         clear_phase_tgl_2= 1'b0,
    output reg [31:0]  phase_init_word_3= 32'd0,
    output reg         cfg_load_tgl_3= 1'b0,
    output reg         start_tgl_3= 1'b0,
    output reg         stop_tgl_3= 1'b0,
    output reg         clear_phase_tgl_3= 1'b0,
    output reg [31:0]  phase_init_word_4= 32'd0,
    output reg         cfg_load_tgl_4= 1'b0,
    output reg         start_tgl_4= 1'b0,
    output reg         stop_tgl_4= 1'b0,
    output reg         clear_phase_tgl_4= 1'b0,
    output reg [31:0]  phase_init_word_5= 32'd0,
    output reg         cfg_load_tgl_5= 1'b0,
    output reg         start_tgl_5= 1'b0,
    output reg         stop_tgl_5= 1'b0,
    output reg         clear_phase_tgl_5= 1'b0,
    output reg [31:0]  phase_init_word_6= 32'd0,
    output reg         cfg_load_tgl_6= 1'b0,
    output reg         start_tgl_6= 1'b0,
    output reg         stop_tgl_6= 1'b0,
    output reg         clear_phase_tgl_6= 1'b0,
    output reg [31:0]  phase_init_word_7= 32'd0,
    output reg         cfg_load_tgl_7= 1'b0,
    output reg         start_tgl_7= 1'b0,
    output reg         stop_tgl_7= 1'b0,
    output reg         clear_phase_tgl_7= 1'b0,
    output wire [7:0]   sig_use_new,
    output wire [15:0]  sig_type,
    output wire [15:0]  sig_freq_type,
    output wire [7:0]   sig_cw_en,
    output wire [7:0]   sig_lfm_neg,
    output wire [159:0] sig_pw_ticks,
    output wire [159:0] sig_pri_ticks,
    output wire [383:0] sig_fc_hz,
    output wire [255:0] sig_hop_bw_hz,
    output wire [47:0]  sig_hop_n,
    output wire [127:0] sig_hop_ticks,
    output wire [255:0] sig_lfm_bw_hz,
    output wire [159:0] sig_lfm_ticks,
    output wire [15:0]  sig_pm_type,
    output wire [15:0]  sig_nlfm_type,
    output wire [127:0] sig_nc,
    output wire [127:0] sig_ncode,
    output wire [15:0]  sig_pri_type,
    output wire [47:0]  sig_pri_m,
    output wire [47:0]  sig_pri_grp,
    output wire [47:0]  sig_hop_grp,
    output wire [5119:0] sig_pri_tab,
    output wire [7:0]   sig_scan_en,
    output wire [15:0]  sig_scan_type,
    output wire [255:0] sig_scan_end,
    output wire [255:0] sig_scan_t,
    output wire [7:0]   sig_beam_we_tgl,
    output wire [79:0]  sig_beam_waddr,
    output wire [255:0] sig_beam_wdata,
    output wire [7:0]   sig_beam_range_tgl,
    output wire [79:0]  sig_beam_a0,
    output wire [79:0]  sig_beam_a1,
    output wire [7:0]   sig_hop_rand,
    output wire [7:0]   sig_freq_tab_en,
    output wire [8191:0] sig_freq_tab,
    output wire [7:0]   sig_phase_tab_en,
    output wire [8191:0] sig_phase_tab,
    output wire [159:0] sig_jit_p0,
    output wire [159:0] sig_jit_n,
    output wire [55:0]  sig_jit_rho,
    output wire [7:0]   sig_jit_mode,
    output wire [159:0] sig_slide_min,
    output wire [159:0] sig_slide_max,
    output wire [159:0] sig_slide_stp,
    output wire [7:0]   sig_slide_md,
    output wire [7:0]   sig_slide_en,
    output reg         dds_sys_rst = 1'b1,
    output reg [7:0]   dds_ch_en   = 8'hFF
);

// 历史短帧中的板级字段只在模块内部保存，不再作为移植核端口。
reg [7:0]   rx_cmd = 8'd0;
reg         jamming_mode;
reg [7:0]   Receive_Pulse;
reg [7:0]   Transmit_Pulse;
reg [7:0]   Receive_Win;
reg [7:0]   Transmit_Win;
reg [7:0]   Fixed_RT;
reg [7:0]   Num_Open_Win;
reg [7:0]   baohu_time;
reg [15:0]  f_word;
reg [15:0]  pulse_word;
reg [15:0]  toa_word;
reg [15:0]  dds_swich;
reg         r_sw_bd = 1'b0;
reg         r_gpio_d0 = 1'b0;
reg         r_sw_zd1 = 1'b0;
reg         r_sw_zd2 = 1'b0;
reg         pll_rst1 = 1'b1;
reg         pll_rst2 = 1'b0;
reg [4:0]   xcbd_pll_sel = 5'd0;
reg         xcbd_q_att1 = 1'b0;
reg [5:0]   xcbd_j_att = 6'd0;
reg [4:0]   bd_xbp_att = 5'd0;
reg [3:0]   bd_sbp_att = 4'd0;
reg [1:0]   gr_f_sel = 2'd0;
reg         xcbd_ctrl_tr = 1'b0;
reg         xcbd_ctrl_gf = 1'b0;
reg         jh_sel_ttl = 1'b0;
reg [4:0]   jh_t_ctrl = 5'd0;
reg         iq_phase = 1'd0;
reg         danpin_da_flag = 1'd0;
reg [1:0]   amp_att = 2'd1;
reg [1:0]   qie_JH = 1'd0;

// =============================================================================
// 1. 旧短帧的暂存寄存器
// =============================================================================
// txd_en/txd_data 是旧短帧回显口；长帧ACK期间由末级mux屏蔽。
reg [7:0]   yuliu;                          // 旧协议保留字节
reg         txd_en;                         // 旧命令响应发送使能
reg [7:0]   txd_data;                       // 旧命令响应发送数据

// 通道号和数据必须先暂存。若在收到数据字节时直接改输出，校验失败或断帧也会
// 污染正在运行的配置；现在统一到 s10（完整尾标记之后）提交。
reg [2:0] temp_channel = 3'd0;              // 当前短帧目标通道 0..7
reg temp_dds_rst = 1'b0;                    // 0x48 单通道 DDS 复位暂存
reg temp_dds_sys_rst = 1'b1;                // 0x4A 全局 DDS 复位暂存
reg [7:0] temp_dds_ch_en = 8'hFF;           // 0x4B 八路使能掩码暂存

// CW/LFM 控制暂存。收到完整帧尾后才在 s10 写入指定通道。
reg [31:0] temp_phase_init_word = 32'd0;
reg [7:0]  temp_lfm_ctrl = 8'd0;   // 0x58/0x59 load/start/stop/clear
reg [2:0]  lfm_word_i = 3'd0;      // 多字节计数

// 0x61–0x80 短帧载荷按 MSB first 左移拼接，s10 产生一次 sig_commit。
// payload 进入 host_regs 后，各命令只读取自己定义的低若干位。
reg [63:0]  temp_sig_payload = 64'd0;
reg         sig_commit = 1'b0;

// =============================================================================
// 2. AA55 长帧解码器与统一寄存器写口
// =============================================================================
// xl_commit 与 sig_commit 逻辑互斥：decoder busy 时旧状态机冻结；xl_commit 在 mux
// 中具有显式优先级，作为异常情况下的最后一道保护。
wire        xl_busy;
wire        xl_commit;
wire [7:0]  xl_cmd;
wire [2:0]  xl_ch;
wire [63:0] xl_payload;
wire        xl_act_load;
wire        xl_act_start;
wire        xl_act_stop;
wire        xl_ack_en;
wire [7:0]  xl_ack_data;

function [2:0] sig_cmd_last;
    input [7:0] c;
    begin
        // 返回“最后一个载荷字节的零基下标”，因此 6B 命令返回 5，1B 返回 0。
        // s4 每收到一字节比较该值，决定继续收数还是进入三个 00 填充字节。
        case (c)
            8'h64, 8'h77: sig_cmd_last = 3'd5; // 6B：0x64 低 4B 为中心 FTW；0x77 仍 6B
            8'h78:        sig_cmd_last = 3'd7; // 2B addr + 6B
            8'h72:        sig_cmd_last = 3'd4; // 1B sel + 4B
            8'h63, 8'h65, 8'h69, 8'h6A, 8'h6E, 8'h70, 8'h74, 8'h75, 8'h79, 8'h7B, 8'h7F:
                          sig_cmd_last = 3'd3; // 4B；0x79 四字节 G，0x7B 起止地址
            8'h67, 8'h6B, 8'h6C:
                          sig_cmd_last = 3'd1; // 2B
            default:      sig_cmd_last = 3'd0; // 1B
        endcase
    end
endfunction

function sig_cmd_is_new;
    input [7:0] c;
    begin
        // 0x7E 被旧协议帧头占用，0x7F/0x80 是后补扩展命令，需单独包含。
        sig_cmd_is_new = ((c >= 8'h61) && (c <= 8'h7D)) || (c == 8'h7F) || (c == 8'h80);
    end
endfunction

// 两种协议最终都归一化成 commit/cmd/ch/payload，后续寄存器和 DDS 链无需知道
// 原始帧是 7E7E 还是 AA55。xl_commit 优先，保证长帧连续提交期间写口唯一。
da_sig_host_regs u_sig_host_regs (
    .clk            (clk_100Mhz),
    .rst            (rst),
    .commit         (sig_commit | xl_commit),
    .cmd            (xl_commit ? xl_cmd : rx_cmd),
    .ch             (xl_commit ? xl_ch : temp_channel),
    .payload        (xl_commit ? xl_payload : temp_sig_payload),
    .sig_use_new    (sig_use_new),
    .sig_type       (sig_type),
    .sig_freq_type  (sig_freq_type),
    .sig_cw_en      (sig_cw_en),
    .sig_lfm_neg    (sig_lfm_neg),
    .sig_pw_ticks   (sig_pw_ticks),
    .sig_pri_ticks  (sig_pri_ticks),
    .sig_fc_hz      (sig_fc_hz),
    .sig_hop_bw_hz  (sig_hop_bw_hz),
    .sig_hop_n      (sig_hop_n),
    .sig_hop_ticks  (sig_hop_ticks),
    .sig_lfm_bw_hz  (sig_lfm_bw_hz),
    .sig_lfm_ticks  (sig_lfm_ticks),
    .sig_pm_type    (sig_pm_type),
    .sig_nlfm_type  (sig_nlfm_type),
    .sig_nc         (sig_nc),
    .sig_ncode      (sig_ncode),
    .sig_pri_type   (sig_pri_type),
    .sig_pri_m      (sig_pri_m),
    .sig_pri_grp    (sig_pri_grp),
    .sig_hop_grp    (sig_hop_grp),
    .sig_pri_tab    (sig_pri_tab),
    .sig_scan_en      (sig_scan_en),
    .sig_scan_type    (sig_scan_type),
    .sig_scan_end     (sig_scan_end),
    .sig_scan_t       (sig_scan_t),
    .sig_beam_we_tgl     (sig_beam_we_tgl),
    .sig_beam_waddr      (sig_beam_waddr),
    .sig_beam_wdata      (sig_beam_wdata),
    .sig_beam_range_tgl  (sig_beam_range_tgl),
    .sig_beam_a0         (sig_beam_a0),
    .sig_beam_a1         (sig_beam_a1),
    .sig_hop_rand     (sig_hop_rand),
    .sig_freq_tab_en  (sig_freq_tab_en),
    .sig_freq_tab     (sig_freq_tab),
    .sig_phase_tab_en (sig_phase_tab_en),
    .sig_phase_tab    (sig_phase_tab),
    .sig_jit_p0       (sig_jit_p0),
    .sig_jit_n        (sig_jit_n),
    .sig_jit_rho      (sig_jit_rho),
    .sig_jit_mode     (sig_jit_mode),
    .sig_slide_min    (sig_slide_min),
    .sig_slide_max    (sig_slide_max),
    .sig_slide_stp    (sig_slide_stp),
    .sig_slide_md     (sig_slide_md),
    .sig_slide_en     (sig_slide_en)
);


// =============================================================================
// 3. 旧 7E7E 短帧状态机定义
// =============================================================================
// s0 仅完成一次初始化跳转；正常空闲态是 s1。s3 根据命令是否含通道决定去
// s3_channel 或直接去 s4。s4 接收变长 DATA，s5/s6/s7 固定检查三个 00，
// s8/s9 检查 AA AA，s10 原子提交暂存值。
localparam  s0       = 4'd0,               // 初始状态
            s1       = 4'd1,               // 等待帧头1
            s2       = 4'd2,               // 等待帧头2
            s3       = 4'd3,               // 接收命令码
            s3_channel = 4'd4,             // 通道选择状态
            s4       = 4'd5,               // 命令解析处理
            s5       = 4'd6,               // 参数字节1处理
            s6       = 4'd7,               // 参数字节2处理
            s7       = 4'd8,               // 参数字节3处理
            s8       = 4'd9,               // 等待帧尾1
            s9       = 4'd10,              // 等待帧尾2
            s10      = 4'd11;              // 命令完成状态

reg [3:0]   state = s0;                    // 主状态机当前状态
reg [3:0]   rx_cnt = 4'd5;                 // 历史接收计数器，仅保留兼容/ILA 观察

// 协议帧格式参数
parameter   headByte1 = 8'h7E,             // 帧头字节1
            headByte2 = 8'h7E,             // 帧头字节2
            tailByte1 = 8'hAA,             // 帧尾字节1
            tailByte2 = 8'hAA;             // 帧尾字节2

// =============================================================================
// 4. 长帧认领及 UART 发送仲裁
// =============================================================================
// ``xl_claim`` 补住 decoder 从 ST_IDLE 接收首字节到 busy 变高之间的同一拍空隙。
// 若只使用 xl_busy，首字节 AA 当拍仍可能被 RS422.v 回显，破坏“长帧无回显”约定。
// 只在旧接收机真正空闲时认领AA，保证短帧与长帧不会同时消费载荷。
wire xl_arm     = (state == s1);
wire xl_claim   = xl_arm && rxd_valid && (rxd_data == 8'hAA);
// ACK 最后一字节发送期间 decoder 仍保持 busy/ack_en，从首 AA 到最后 status 的
// txd_done 全程占住发送 mux，避免末字节被旧 txd_data 替换。
wire xl_tx_mode = xl_claim || xl_busy || xl_ack_en;

// 256B 缓冲、校验、类别展开和 4B ACK 均封装在 decoder 内；输出 commit 是
// 单周期寄存器写事务，act_* 是全部 commit 完成后的动作事务。
da_sig_excel_decode u_da_sig_excel_decode (
    .cfg_ready(sig_cfg_ready),
    .clk        (clk_100Mhz),
    .rst        (rst),
    .arm        (xl_arm),
    .rx_valid   (rxd_valid),
    .rx_data    (rxd_data),
    .txd_done   (txd_done),
    .busy       (xl_busy),
    .commit     (xl_commit),
    .cmd        (xl_cmd),
    .ch         (xl_ch),
    .payload    (xl_payload),
    .act_load   (xl_act_load),
    .act_start  (xl_act_start),
    .act_stop   (xl_act_stop),
    .ack_en     (xl_ack_en),
    .ack_data   (xl_ack_data)
);


// 长帧接收/ACK期间取得UART发送权；其他时间沿用短帧逐字节回显。
assign tx_override  = xl_tx_mode;
assign txd_data_out = xl_tx_mode ? xl_ack_data : txd_data;
assign tx_req       = xl_tx_mode ? xl_ack_en   : txd_en;

// =============================================================================
// 5. 旧短帧接收、暂存和原子提交
// =============================================================================
always @(posedge clk_100Mhz) begin
    if (rst) begin
        // 协议状态回到 s0；DDS 保持全局复位但通道掩码默认全开。动作 toggle
        // 全部清零作为两侧共同初始相位，释放复位后第一次翻转才代表事件。
        state <= s0;
        rx_cmd <= 8'd0;
        txd_en <= 1'b0;
        txd_data <= 8'd0;
        rx_cnt <= 4'd5;
        
        // 丢弃尚未通过短帧尾校验的暂存参数。
        temp_channel <= 3'd0;
        temp_dds_rst <= 1'b0;
        temp_dds_sys_rst <= 1'b1;
        temp_dds_ch_en   <= 8'hFF;
        temp_phase_init_word <= 32'd0;
        temp_lfm_ctrl <= 8'd0;
        lfm_word_i <= 3'd0;
        temp_sig_payload <= 64'd0;
        sig_commit <= 1'b0;
        // 全局 DDS 输出和八路动作事件初值。
        dds_sys_rst <= 1'b1;
        dds_ch_en   <= 8'hFF;
        phase_init_word_0 <= 32'd0;
        cfg_load_tgl_0 <= 1'b0;
        start_tgl_0 <= 1'b0;
        stop_tgl_0 <= 1'b0;
        clear_phase_tgl_0 <= 1'b0;
        phase_init_word_1 <= 32'd0;
        cfg_load_tgl_1 <= 1'b0;
        start_tgl_1 <= 1'b0;
        stop_tgl_1 <= 1'b0;
        clear_phase_tgl_1 <= 1'b0;
        phase_init_word_2 <= 32'd0;
        cfg_load_tgl_2 <= 1'b0;
        start_tgl_2 <= 1'b0;
        stop_tgl_2 <= 1'b0;
        clear_phase_tgl_2 <= 1'b0;
        phase_init_word_3 <= 32'd0;
        cfg_load_tgl_3 <= 1'b0;
        start_tgl_3 <= 1'b0;
        stop_tgl_3 <= 1'b0;
        clear_phase_tgl_3 <= 1'b0;
        phase_init_word_4 <= 32'd0;
        cfg_load_tgl_4 <= 1'b0;
        start_tgl_4 <= 1'b0;
        stop_tgl_4 <= 1'b0;
        clear_phase_tgl_4 <= 1'b0;
        phase_init_word_5 <= 32'd0;
        cfg_load_tgl_5 <= 1'b0;
        start_tgl_5 <= 1'b0;
        stop_tgl_5 <= 1'b0;
        clear_phase_tgl_5 <= 1'b0;
        phase_init_word_6 <= 32'd0;
        cfg_load_tgl_6 <= 1'b0;
        start_tgl_6 <= 1'b0;
        stop_tgl_6 <= 1'b0;
        clear_phase_tgl_6 <= 1'b0;
        phase_init_word_7 <= 32'd0;
        cfg_load_tgl_7 <= 1'b0;
        start_tgl_7 <= 1'b0;
        stop_tgl_7 <= 1'b0;
        clear_phase_tgl_7 <= 1'b0;
        
    end else begin
        sig_commit <= 1'b0;

        // 长帧动作在 decoder 完成全部内部提交后到达，转换成原有跨时钟 toggle。
        // decoder 保证 load 与 start 不同拍；这里对通道译码后只翻转对应一路。
        if (xl_act_load || xl_act_start || xl_act_stop) begin
            case (xl_ch)
                3'd0: begin
                    if (xl_act_load)  cfg_load_tgl_0 <= ~cfg_load_tgl_0;
                    if (xl_act_start) start_tgl_0    <= ~start_tgl_0;
                    if (xl_act_stop)  stop_tgl_0     <= ~stop_tgl_0;
                end
                3'd1: begin
                    if (xl_act_load)  cfg_load_tgl_1 <= ~cfg_load_tgl_1;
                    if (xl_act_start) start_tgl_1    <= ~start_tgl_1;
                    if (xl_act_stop)  stop_tgl_1     <= ~stop_tgl_1;
                end
                3'd2: begin
                    if (xl_act_load)  cfg_load_tgl_2 <= ~cfg_load_tgl_2;
                    if (xl_act_start) start_tgl_2    <= ~start_tgl_2;
                    if (xl_act_stop)  stop_tgl_2     <= ~stop_tgl_2;
                end
                3'd3: begin
                    if (xl_act_load)  cfg_load_tgl_3 <= ~cfg_load_tgl_3;
                    if (xl_act_start) start_tgl_3    <= ~start_tgl_3;
                    if (xl_act_stop)  stop_tgl_3     <= ~stop_tgl_3;
                end
                3'd4: begin
                    if (xl_act_load)  cfg_load_tgl_4 <= ~cfg_load_tgl_4;
                    if (xl_act_start) start_tgl_4    <= ~start_tgl_4;
                    if (xl_act_stop)  stop_tgl_4     <= ~stop_tgl_4;
                end
                3'd5: begin
                    if (xl_act_load)  cfg_load_tgl_5 <= ~cfg_load_tgl_5;
                    if (xl_act_start) start_tgl_5    <= ~start_tgl_5;
                    if (xl_act_stop)  stop_tgl_5     <= ~stop_tgl_5;
                end
                3'd6: begin
                    if (xl_act_load)  cfg_load_tgl_6 <= ~cfg_load_tgl_6;
                    if (xl_act_start) start_tgl_6    <= ~start_tgl_6;
                    if (xl_act_stop)  stop_tgl_6     <= ~stop_tgl_6;
                end
                default: begin
                    if (xl_act_load)  cfg_load_tgl_7 <= ~cfg_load_tgl_7;
                    if (xl_act_start) start_tgl_7    <= ~start_tgl_7;
                    if (xl_act_stop)  stop_tgl_7     <= ~stop_tgl_7;
                end
            endcase
        end

        if (xl_busy || xl_claim) begin
            // 屏蔽长帧载荷中的 7E，且不让旧回显发送使能粘住。将 state 固定
            // 在 s1，待 decoder 完整退出后旧协议可以从干净的帧头状态恢复。
            state  <= s1;
            txd_en <= 1'b0;
        end else begin
        case (state)
            // ==================================================
            // s0: 初始状态
            // ==================================================
            s0: begin
                state <= s1;
                txd_en <= 1'b0;
            end
            
            // ==================================================
            // s1: 等待帧头1
            // ==================================================
            s1: begin
                rx_cnt <= 4'd5;
                rx_cmd <= 8'd0;
                
                // 非 7E 字节在旧协议空闲态直接忽略；AA 会在外层同拍被长帧认领。
                if ((rxd_valid) && (rxd_data == headByte1)) begin
                    state <= s2;
                    txd_en <= 1'b1;
                    txd_data <= headByte1;
                end else begin
                    state <= s1;
                end
            end
            
            // ==================================================
            // s2: 等待帧头2
            // ==================================================
            s2: begin
                if (rxd_valid) begin
                    if (rxd_data == headByte2) begin
                        state <= s3;
                        txd_en <= 1'b1;
                        txd_data <= headByte2;
                    end else begin
                        state <= s1;
                    end
                end else begin
                    state <= s2;
                end
            end
            
            // ==================================================
            // s3: 接收命令码
            // ==================================================
            s3: begin
                if (rxd_valid) begin
                    rx_cmd <= rxd_data;
                    temp_sig_payload <= 64'd0;
                    // 通道类命令：0x40~0x49 旧 DDS；0x50~0x58 单段 LFM；
                    // 0x5A~0x60 分段；0x61–0x80 新协议。全局 0x4A/0x4B/0x59
                    // 没有 CH 字节，直接把下一字节当 DATA。
                    if (((rxd_data >= 8'h40) && (rxd_data <= 8'h49)) ||
                        ((rxd_data >= 8'h50) && (rxd_data <= 8'h58)) ||
                        ((rxd_data >= 8'h5A) && (rxd_data <= 8'h7D)) ||
                        (rxd_data == 8'h7F) || (rxd_data == 8'h80)) begin
                        state <= s3_channel;  // DDS命令需要选择通道
                    end else begin
                        state <= s4;          // 非DDS命令直接进入参数处理
                    end
                    txd_en <= 1'b1;
                    txd_data <= rxd_data;
                end else begin
                    state <= s3;
                end
            end
            
            // ==================================================
            // s3_channel: 通道选择状态 - 只暂存不立即使用
            // ==================================================
            s3_channel: begin
                if (rxd_valid) begin
                    temp_channel <= rxd_data[2:0];  // 暂存通道号到临时寄存器
                    state <= s4;                   // 进入参数处理
                    txd_en <= 1'b1;
                    txd_data <= rxd_data;          // 回送通道号
                end else begin
                    state <= s3_channel;
                end
            end
            
            // ==================================================
            // s4: 命令解析处理 - 8通道DDS独立控制
            // ==================================================
            s4: begin
                if (rxd_valid) begin
                    case (rx_cmd)
                        // ==================================================
                        // DDS控制命令 (0x40-0x49)
                        // ==================================================
                        
                      
                        // 0x40–0x47/0x49 的旧 DDS 数据口已删除，但仍必须按原长度
                        // 消耗载荷，才能让后面的 00 00 00 / AA AA 保持字节对齐。
                        8'h40, 8'h41, 8'h42: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            if (lfm_word_i < 3'd1) begin
                                lfm_word_i <= lfm_word_i + 3'd1;
                                state <= s4;
                            end else begin
                                lfm_word_i <= 3'd0;
                                state <= s5;
                            end
                        end
                        8'h43, 8'h44: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            if (lfm_word_i < 3'd2) begin
                                lfm_word_i <= lfm_word_i + 3'd1;
                                state <= s4;
                            end else begin
                                lfm_word_i <= 3'd0;
                                state <= s5;
                            end
                        end
                        8'h45, 8'h46, 8'h47, 8'h49: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            state <= s5;
                        end
                        8'h48: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            state <= s5;
                            temp_dds_rst <= rxd_data[0];
                        end
                        8'h4A: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            state <= s5;
                            temp_dds_sys_rst <= rxd_data[0];
                        end
                        8'h4B: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            state <= s5;
                            temp_dds_ch_en <= rxd_data;
                        end
                        // 0x50/0x51–0x54/0x5A–0x60 旧链参数：吃字节，不落寄存器
                        8'h50, 8'h5A, 8'h5B, 8'h60: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            state <= s5;
                        end
                        8'h51, 8'h52, 8'h53, 8'h54, 8'h5C, 8'h5D, 8'h5E, 8'h5F: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            if (lfm_word_i < 3'd3) begin
                                lfm_word_i <= lfm_word_i + 3'd1;
                                state <= s4;
                            end else begin
                                lfm_word_i <= 3'd0;
                                state <= s5;
                            end
                        end
                        8'h55: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            case (lfm_word_i)
                                3'd0: temp_phase_init_word[31:24] <= rxd_data;
                                3'd1: temp_phase_init_word[23:16] <= rxd_data;
                                3'd2: temp_phase_init_word[15:8]  <= rxd_data;
                                default: temp_phase_init_word[7:0] <= rxd_data;
                            endcase
                            if (lfm_word_i < 3'd3) begin
                                lfm_word_i <= lfm_word_i + 3'd1;
                                state <= s4;
                            end else begin
                                lfm_word_i <= 3'd0;
                                state <= s5;
                            end
                        end
                        8'h58, 8'h59: begin
                            txd_data <= rxd_data; txd_en <= 1'b1;
                            state <= s5;
                            temp_lfm_ctrl <= rxd_data;
                        end
                        8'h61, 8'h62, 8'h63, 8'h64, 8'h65, 8'h66, 8'h67, 8'h68,
                        8'h69, 8'h6A, 8'h6B, 8'h6C, 8'h6D, 8'h6E, 8'h6F, 8'h70,
                        8'h71, 8'h72, 8'h73, 8'h74, 8'h75, 8'h76, 8'h77, 8'h78,
                        8'h79, 8'h7A, 8'h7B, 8'h7C, 8'h7D, 8'h7F, 8'h80: begin
                            // 大端线序逐字节左移；收到最后一字节后，完整载荷位于
                            // temp_sig_payload 低端，并在 s10 与命令/通道一起提交。
                            txd_data <= rxd_data;
                            txd_en   <= 1'b1;
                            temp_sig_payload <= {temp_sig_payload[55:0], rxd_data};
                            if (lfm_word_i < sig_cmd_last(rx_cmd)) begin
                                lfm_word_i <= lfm_word_i + 3'd1;
                                state <= s4;
                            end else begin
                                lfm_word_i <= 3'd0;
                                state <= s5;
                            end
                        end

            // 已移除的旧采集/上传命令：消费一字节载荷和帧尾，保持回显。
            // 不设置事务标志，也不阻塞后续短帧、长帧或ACK。
            8'h10, 8'h11: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
            end

            // 0x12: 系统工作模式设置
            8'h12: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                jamming_mode <= rxd_data[0];
            end
            
            // 0x13: 接收脉冲宽度设置
            8'h13: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                Receive_Pulse <= rxd_data;
            end
            
            // 0x14: 发射脉冲宽度设置
            8'h14: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                Transmit_Pulse <= rxd_data;
            end
            
            // 0x15: 接收窗口宽度设置
            8'h15: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                Receive_Win <= rxd_data;
            end
            
            // 0x16: 发射窗口宽度设置
            8'h16: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                Transmit_Win <= rxd_data;
            end
            
            // 0x17: 固定重复周期设置
            8'h17: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                Fixed_RT <= rxd_data;
            end
            
            // 0x18: 开窗数量设置
            8'h18: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                Num_Open_Win <= rxd_data;
            end
            
            // 0x19: 保护时间设置
            8'h19: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                baohu_time <= rxd_data;
            end
            
            // 0x1A: 频率控制字设置
            8'h1A: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                f_word <= {f_word[7:0], rxd_data};  // 组合成16位
            end
            
            // 0x1B: 脉冲控制字设置
            8'h1B: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                pulse_word <= {pulse_word[7:0], rxd_data};  // 组合成16位
            end
            
            // 0x1C: 到达时间控制字设置
            8'h1C: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                toa_word <= {toa_word[7:0], rxd_data};  // 组合成16位
            end
            
            // 0x1D: DDS切换控制字设置
            8'h1D: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                dds_swich <= {dds_swich[7:0], rxd_data};  // 组合成16位
            end
            
            // ==================================================
            // 硬件控制命令 (0x20-0x3F)
            // ==================================================
            
            // 0x20: 波段开关控制
            8'h20: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                r_sw_bd <= rxd_data[0];
            end
            
            // 0x21: GPIO D0控制
            8'h21: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                r_gpio_d0 <= rxd_data[0];
            end
            
            // 0x22: 通道极1开关控制
            8'h22: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                r_sw_zd1 <= rxd_data[0];
            end
            
            // 0x23: 通道2开关控制
            8'h23: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                r_sw_zd2 <= rxd_data[0];
            end
            
            // 0x24: PLL复位信号1
            8'h24: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                pll_rst1 <= rxd_data[0];
            end
            
            // 0x25: PLL复位信号2
            8'h25: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                pll_rst2 <= rxd_data[0];
            end
            
            // 0x26: 变频器PLL选择
            8'h26: begin
                txd_data <= rxd_data;
                txd_en <= 1'b1;
                state <= s5;
                xcbd_pll_sel <= rxd_data[4:0];
            end
            
            // 其他硬件控制命令...
            
            // ==================================================
            // 默认命令处理
            // ==================================================
                    endcase
                end else begin
                    state <= s4;
                end
            end
            
            // ==================================================
            // s5-s10: 协议帧后续处理
            // ==================================================
            // s5/s6/s7 严格要求三个保留字节为 00；任何不匹配均静默丢帧回 s1，
            // 不产生 sig_commit，也不翻转动作 toggle。
            s5: begin
                if (rxd_valid) begin
                    if (rxd_data == 8'b0) begin
                        state <= s6;
                        txd_data <= 8'h00;
                        txd_en <= 1'b1;
                    end else begin
                        state <= s1;
                    end
                end else begin
                    state <= s5;
                end
            end
            
            s6: begin
                if (rxd_valid) begin
                    if (rxd_data == 8'b0) begin
                        state <= s7;
                        txd_data <= 8'h00;
                        txd_en <= 1'b1;
                    end else begin
                        state <= s1;
                    end
                end else begin
                    state <= s6;
                end
            end
            
            s7: begin
                if (rxd_valid) begin
                    if (rxd_data == 8'b0) begin
                        state <= s8;
                        txd_data <= 8'h00;
                        txd_en <= 1'b1;
                    end else begin
                        state <= s1;
                    end
                end else begin
                    state <= s7;
                end
            end
            
            s8: begin
                if (rxd_valid) begin
                    if (rxd_data == tailByte1) begin
                        state <= s9;
                        txd_data <= tailByte1;
                        txd_en <= 1'b1;
                    end else begin
                        state <= s1;
                    end
                end else begin
                    state <= s8;
                end
            end
            
            s9: begin
                if (rxd_valid) begin
                    if (rxd_data == tailByte2) begin
                        state <= s10;
                        txd_data <= tailByte2;
                        txd_en <= 1'b1;
                    end else begin
                        state <= s1;
                    end
                end else begin
                    state <= s9;
                end
            end
            
            // ==================================================
            // s10: 命令完成状态 - 应用暂存的参数到指定通道
            // ==================================================
            s10: begin
                if (((rx_cmd==8'h58 && temp_lfm_ctrl[0]) || rx_cmd==8'h79 || rx_cmd==8'h7B) && !sig_cfg_ready[temp_channel]) state<=s10;
                else if(rx_cmd==8'h59 && temp_lfm_ctrl[0] && !(&(sig_cfg_ready | ~dds_ch_en))) state<=s10;
                else begin
                // 只有到达这里才说明 7E7E、三个 00 和 AAAA 均完整。历史控制输出、
                // 新链寄存器 commit 和动作 toggle 都集中在此拍原子生效。
                if (rx_cmd == 8'h48) begin
                    case (temp_channel)
                        3'd0: dds_rst_0 <= temp_dds_rst;
                        3'd1: dds_rst_1 <= temp_dds_rst;
                        3'd2: dds_rst_2 <= temp_dds_rst;
                        3'd3: dds_rst_3 <= temp_dds_rst;
                        3'd4: dds_rst_4 <= temp_dds_rst;
                        3'd5: dds_rst_5 <= temp_dds_rst;
                        3'd6: dds_rst_6 <= temp_dds_rst;
                        3'd7: dds_rst_7 <= temp_dds_rst;
                    endcase
                end
                
                if (rx_cmd >= 8'h50 && rx_cmd <= 8'h58) begin
                    case (rx_cmd)
                        8'h55: begin
                            case (temp_channel)
                                3'd0: phase_init_word_0 <= temp_phase_init_word;
                                3'd1: phase_init_word_1 <= temp_phase_init_word;
                                3'd2: phase_init_word_2 <= temp_phase_init_word;
                                3'd3: phase_init_word_3 <= temp_phase_init_word;
                                3'd4: phase_init_word_4 <= temp_phase_init_word;
                                3'd5: phase_init_word_5 <= temp_phase_init_word;
                                3'd6: phase_init_word_6 <= temp_phase_init_word;
                                3'd7: phase_init_word_7 <= temp_phase_init_word;
                            endcase
                        end
                        8'h58: begin
                            case (temp_channel)
                                3'd0: begin
                                    if (temp_lfm_ctrl[0]) cfg_load_tgl_0 <= ~cfg_load_tgl_0;
                                    if (temp_lfm_ctrl[1]) start_tgl_0 <= ~start_tgl_0;
                                    if (temp_lfm_ctrl[2]) stop_tgl_0 <= ~stop_tgl_0;
                                    if (temp_lfm_ctrl[3]) clear_phase_tgl_0 <= ~clear_phase_tgl_0;
                                end
                                3'd1: begin
                                    if (temp_lfm_ctrl[0]) cfg_load_tgl_1 <= ~cfg_load_tgl_1;
                                    if (temp_lfm_ctrl[1]) start_tgl_1 <= ~start_tgl_1;
                                    if (temp_lfm_ctrl[2]) stop_tgl_1 <= ~stop_tgl_1;
                                    if (temp_lfm_ctrl[3]) clear_phase_tgl_1 <= ~clear_phase_tgl_1;
                                end
                                3'd2: begin
                                    if (temp_lfm_ctrl[0]) cfg_load_tgl_2 <= ~cfg_load_tgl_2;
                                    if (temp_lfm_ctrl[1]) start_tgl_2 <= ~start_tgl_2;
                                    if (temp_lfm_ctrl[2]) stop_tgl_2 <= ~stop_tgl_2;
                                    if (temp_lfm_ctrl[3]) clear_phase_tgl_2 <= ~clear_phase_tgl_2;
                                end
                                3'd3: begin
                                    if (temp_lfm_ctrl[0]) cfg_load_tgl_3 <= ~cfg_load_tgl_3;
                                    if (temp_lfm_ctrl[1]) start_tgl_3 <= ~start_tgl_3;
                                    if (temp_lfm_ctrl[2]) stop_tgl_3 <= ~stop_tgl_3;
                                    if (temp_lfm_ctrl[3]) clear_phase_tgl_3 <= ~clear_phase_tgl_3;
                                end
                                3'd4: begin
                                    if (temp_lfm_ctrl[0]) cfg_load_tgl_4 <= ~cfg_load_tgl_4;
                                    if (temp_lfm_ctrl[1]) start_tgl_4 <= ~start_tgl_4;
                                    if (temp_lfm_ctrl[2]) stop_tgl_4 <= ~stop_tgl_4;
                                    if (temp_lfm_ctrl[3]) clear_phase_tgl_4 <= ~clear_phase_tgl_4;
                                end
                                3'd5: begin
                                    if (temp_lfm_ctrl[0]) cfg_load_tgl_5 <= ~cfg_load_tgl_5;
                                    if (temp_lfm_ctrl[1]) start_tgl_5 <= ~start_tgl_5;
                                    if (temp_lfm_ctrl[2]) stop_tgl_5 <= ~stop_tgl_5;
                                    if (temp_lfm_ctrl[3]) clear_phase_tgl_5 <= ~clear_phase_tgl_5;
                                end
                                3'd6: begin
                                    if (temp_lfm_ctrl[0]) cfg_load_tgl_6 <= ~cfg_load_tgl_6;
                                    if (temp_lfm_ctrl[1]) start_tgl_6 <= ~start_tgl_6;
                                    if (temp_lfm_ctrl[2]) stop_tgl_6 <= ~stop_tgl_6;
                                    if (temp_lfm_ctrl[3]) clear_phase_tgl_6 <= ~clear_phase_tgl_6;
                                end
                                3'd7: begin
                                    if (temp_lfm_ctrl[0]) cfg_load_tgl_7 <= ~cfg_load_tgl_7;
                                    if (temp_lfm_ctrl[1]) start_tgl_7 <= ~start_tgl_7;
                                    if (temp_lfm_ctrl[2]) stop_tgl_7 <= ~stop_tgl_7;
                                    if (temp_lfm_ctrl[3]) clear_phase_tgl_7 <= ~clear_phase_tgl_7;
                                end
                            endcase
                        end
                        default: ;
                    endcase
                end

                if (rx_cmd == 8'h59) begin
                    // 全局控制把同一个动作同时翻转到八路，供同步启动/停止使用。
                    if (temp_lfm_ctrl[0]) begin
                        if(dds_ch_en[0]) cfg_load_tgl_0 <= ~cfg_load_tgl_0; if(dds_ch_en[1]) cfg_load_tgl_1 <= ~cfg_load_tgl_1;
                        if(dds_ch_en[2]) cfg_load_tgl_2 <= ~cfg_load_tgl_2; if(dds_ch_en[3]) cfg_load_tgl_3 <= ~cfg_load_tgl_3;
                        if(dds_ch_en[4]) cfg_load_tgl_4 <= ~cfg_load_tgl_4; if(dds_ch_en[5]) cfg_load_tgl_5 <= ~cfg_load_tgl_5;
                        if(dds_ch_en[6]) cfg_load_tgl_6 <= ~cfg_load_tgl_6; if(dds_ch_en[7]) cfg_load_tgl_7 <= ~cfg_load_tgl_7;
                    end
                    if (temp_lfm_ctrl[1]) begin
                        start_tgl_0 <= ~start_tgl_0; start_tgl_1 <= ~start_tgl_1;
                        start_tgl_2 <= ~start_tgl_2; start_tgl_3 <= ~start_tgl_3;
                        start_tgl_4 <= ~start_tgl_4; start_tgl_5 <= ~start_tgl_5;
                        start_tgl_6 <= ~start_tgl_6; start_tgl_7 <= ~start_tgl_7;
                    end
                    if (temp_lfm_ctrl[2]) begin
                        stop_tgl_0 <= ~stop_tgl_0; stop_tgl_1 <= ~stop_tgl_1;
                        stop_tgl_2 <= ~stop_tgl_2; stop_tgl_3 <= ~stop_tgl_3;
                        stop_tgl_4 <= ~stop_tgl_4; stop_tgl_5 <= ~stop_tgl_5;
                        stop_tgl_6 <= ~stop_tgl_6; stop_tgl_7 <= ~stop_tgl_7;
                    end
                    if (temp_lfm_ctrl[3]) begin
                        clear_phase_tgl_0 <= ~clear_phase_tgl_0; clear_phase_tgl_1 <= ~clear_phase_tgl_1;
                        clear_phase_tgl_2 <= ~clear_phase_tgl_2; clear_phase_tgl_3 <= ~clear_phase_tgl_3;
                        clear_phase_tgl_4 <= ~clear_phase_tgl_4; clear_phase_tgl_5 <= ~clear_phase_tgl_5;
                        clear_phase_tgl_6 <= ~clear_phase_tgl_6; clear_phase_tgl_7 <= ~clear_phase_tgl_7;
                    end
                end

            if (rx_cmd == 8'h4A) begin
                dds_sys_rst <= temp_dds_sys_rst;
            end else if (rx_cmd == 8'h4B) begin
                dds_ch_en   <= temp_dds_ch_en;
            end

                // 新协议提交：0x61–0x80 写入 host_regs；0x50/0x5A 清 sig_use_new。
                // sig_commit 默认每拍拉低，所以这里只形成一个 100 MHz 周期脉冲。
                if (sig_cmd_is_new(rx_cmd) || (rx_cmd == 8'h50) || (rx_cmd == 8'h5A))
                    sig_commit <= 1'b1;
            
                // 完整帧提交结束，重新允许下一帧。
                state <= s1;
                end
            end
            
            default: state <= s0;
        endcase
        end
    end
end

// =============================================================================
// 6. 在线逻辑分析探针
// =============================================================================
// ILA 仅观察命令域信号，不参与任何控制和握手。修改探针宽度/顺序时必须同步
// 更新对应 .xci，否则 Vivado elaboration 会报端口宽度不匹配。
// VU13P: 原调试IP已禁用；保留连线注释供以后自行添加调试。
// ila_cmd_receive_0 my_ila_cmd_receive_0 (
//     .clk(clk_100Mhz),
//     
//     // 探针0-9: 基本通信信号
//     .probe0(rxd_data),// 接收数据
//     .probe1(rxd_valid), //接收数据有效标志
//     .probe2(rx_cmd),//解析出的命令码
//     .probe3(txd_data_out),//发送数据输出
//     .probe4(state),//主状态机当前状态
//     .probe5(txd_en),//发送使能
//     .probe9(jamming_mode),//干扰模式使能
//     
//     // 探针10-19: 状态和控制信号
//     .probe10(rst),//DDS通道选择(0-7)
//     .probe19(rxd_valid),//
//     
//     // 探针20-29: 通道0参数
//     .probe26(dds_rst_0),
//     
//     // 探针30-39: 通道1参数
//     .probe36(dds_rst_1),
//     
//     // 探针40-49: 其他通道参数
//     .probe46(dds_rst_2)
// );
// ILA核实例化
// VU13P: 原调试IP已禁用；保留连线注释供以后自行添加调试。
// ila_cmd_receive my_ila_cmd_receive (
//     .clk(clk_100Mhz),
//     
//     // 探针0-6: 基本通信信号
//     .probe0(rxd_data),          // 接收数据
//     .probe1(rxd_valid),         // 接收数据有效标志
//     .probe2(rx_cmd),            // 解析出的命令码
//     .probe3(txd_data_out),      // 发送数据输出
//     .probe4(state),             // 主状态机当前状态 (4位)
//     .probe5(txd_en),            // 发送使能
//     
//     // 探针7-14: 通道0关键参数（原14-21，现前移）
//     
//     // 探针15-25: 临时参数信号（原50-60，现重新编号为15-25）
//     .probe15(temp_channel),           // 临时通道选择
//     .probe24(temp_dds_rst),           // 临时DDS复位
//    
//     .probe26(dds_rst_0),
//     .probe27(1'b0),
//     .probe28(rst),            // rst
//    
//    
//     .probe29(dds_sys_rst),
//     .probe30(dds_ch_en),
//     .probe31(temp_dds_sys_rst),
//     .probe32(temp_dds_ch_en)
// );



endmodule
