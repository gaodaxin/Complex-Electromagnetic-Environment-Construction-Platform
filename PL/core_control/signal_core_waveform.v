`timescale 1ns / 1ps
// ============================================================================
// 模块：signal_core_waveform
// 功能：装配命令解析与八通道信号发生器，集中管理二者之间的参数/事件总线。
// receive_cmd_wls在100 MHz域提交参数和load/start/stop/clear toggle；
// top_8ch_dds内部完成跨域、八路波形生成及300 MHz出口寄存。
// 字段保持具名wire连接，便于核对位宽；不额外打拍，不改变原控制/样点延迟。
// 0x10/0x11旧采集/上传命令只消费帧并回显，不占用应答发送链。
// ============================================================================
module signal_core_waveform (
    input  wire           clk_cfg_100m          , // 100 MHz命令时钟
    input  wire           clk_dds_300m          , // 300 MHz波形时钟
    input  wire           command_reset         , // 100 MHz域同步释放的高有效复位
    input  wire [7:0]     rx_data               , // 已接收字节
    input  wire           rx_valid              , // 接收字节有效
    input  wire           tx_done               , // UART一字节发送完成
    output wire           tx_req                , // 应答字节发送请求
    output wire           tx_override           , // 应答链取得发送权
    output wire [7:0]     tx_data               , // 应答/回显字节
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

// 参数与动作：100 MHz命令寄存器 → 八通道内部CDC。
wire [7:0] sig_cfg_ready;
wire          dds_rst_0;                     // 通道 0 DDS 复位，命令域电平
wire          dds_rst_1;                     // 通道 1 DDS 复位
wire          dds_rst_2;                     // 通道 2 DDS 复位
wire          dds_rst_3;                     // 通道 3 DDS 复位
wire          dds_rst_4;                     // 通道 4 DDS 复位
wire          dds_rst_5;                     // 通道 5 DDS 复位
wire          dds_rst_6;                     // 通道 6 DDS 复位
wire          dds_rst_7;                     // 通道 7 DDS 复位

wire [31:0]   phase_init_word_0;             // 通道 0 初相
wire          cfg_load_tgl_0;                // 通道 0 加载翻转，变沿变 1 拍
wire          start_tgl_0;                   // 通道 0 启动翻转
wire          stop_tgl_0;                    // 通道 0 停止翻转
wire          clear_phase_tgl_0;             // 通道 0 清相翻转
wire [31:0]   phase_init_word_1;             // 通道 1 初相
wire          cfg_load_tgl_1;                // 通道 1 加载翻转
wire          start_tgl_1;                   // 通道 1 启动翻转
wire          stop_tgl_1;                    // 通道 1 停止翻转
wire          clear_phase_tgl_1;             // 通道 1 清相翻转
wire [31:0]   phase_init_word_2;             // 通道 2 初相
wire          cfg_load_tgl_2;                // 通道 2 加载翻转
wire          start_tgl_2;                   // 通道 2 启动翻转
wire          stop_tgl_2;                    // 通道 2 停止翻转
wire          clear_phase_tgl_2;             // 通道 2 清相翻转
wire [31:0]   phase_init_word_3;             // 通道 3 初相
wire          cfg_load_tgl_3;                // 通道 3 加载翻转
wire          start_tgl_3;                   // 通道 3 启动翻转
wire          stop_tgl_3;                    // 通道 3 停止翻转
wire          clear_phase_tgl_3;             // 通道 3 清相翻转
wire [31:0]   phase_init_word_4;             // 通道 4 初相
wire          cfg_load_tgl_4;                // 通道 4 加载翻转
wire          start_tgl_4;                   // 通道 4 启动翻转
wire          stop_tgl_4;                    // 通道 4 停止翻转
wire          clear_phase_tgl_4;             // 通道 4 清相翻转
wire [31:0]   phase_init_word_5;             // 通道 5 初相
wire          cfg_load_tgl_5;                // 通道 5 加载翻转
wire          start_tgl_5;                   // 通道 5 启动翻转
wire          stop_tgl_5;                    // 通道 5 停止翻转
wire          clear_phase_tgl_5;             // 通道 5 清相翻转
wire [31:0]   phase_init_word_6;             // 通道 6 初相
wire          cfg_load_tgl_6;                // 通道 6 加载翻转
wire          start_tgl_6;                   // 通道 6 启动翻转
wire          stop_tgl_6;                    // 通道 6 停止翻转
wire          clear_phase_tgl_6;             // 通道 6 清相翻转
wire [31:0]   phase_init_word_7;             // 通道 7 初相
wire          cfg_load_tgl_7;                // 通道 7 加载翻转
wire          start_tgl_7;                   // 通道 7 启动翻转
wire          stop_tgl_7;                    // 通道 7 停止翻转
wire          clear_phase_tgl_7;             // 通道 7 清相翻转
wire          dds_sys_rst;                   // DDS 全局复位，1=静音
wire [7:0]    dds_ch_en;                     // 8 路通道使能，1=开

wire [7:0]    sig_use_new;                   // 新链开关，bit[ch]
wire [15:0]   sig_type;                      // 8×2：0 CW / 1 FM / 2 PM
wire [15:0]   sig_freq_type;                 // 8×2：0 单载频 / 1 脉内 / 2 脉间 / 3 脉组
wire [7:0]    sig_cw_en;                     // 连续波，1=门恒开
wire [7:0]    sig_lfm_neg;                   // LFM 负扫
wire [159:0]  sig_pw_ticks;                  // 8×20 脉宽 tick
wire [159:0]  sig_pri_ticks;                 // 8×20 默认 PRI tick
wire [383:0]  sig_fc_hz;                     // 8×48 中心频率，低 32=FTW
wire [255:0]  sig_hop_bw_hz;                 // 8×32 捷变带宽 FTW
wire [47:0]   sig_hop_n;                     // 8×6 捷变频点数 1..32
wire [127:0]  sig_hop_ticks;                 // 8×16 脉内 hop 间隔 tick
wire [255:0]  sig_lfm_bw_hz;                 // 8×32 调频带宽 FTW
wire [159:0]  sig_lfm_ticks;                 // 8×20 调频时长 tick
wire [15:0]   sig_pm_type;                   // 8×2 FM/PM 子类型
wire [15:0]   sig_nlfm_type;                 // 8×2 NLFM 形状
wire [127:0]  sig_nc;                        // 8×16 码元宽度 tick
wire [127:0]  sig_ncode;                     // 8×16 码长
wire [15:0]   sig_pri_type;                  // 8×2：固定/参差/抖动
wire [47:0]   sig_pri_m;                     // 8×6 参差项数
wire [47:0]   sig_pri_grp;                   // 8×6 脉组参差组大小
wire [47:0]   sig_hop_grp;                   // 8×6 脉组捷变组大小
wire [5119:0] sig_pri_tab;                   // 8×32×20 PRI 表
wire [7:0]    sig_scan_en;                   // 8 路扫描使能
wire [15:0]   sig_scan_type;                 // 8×2：0 关 / 1 圆扫 / 2 扇扫
wire [255:0]  sig_scan_end;                  // 8×32：[9:0] start [19:10] span [29:20] target
wire [255:0]  sig_scan_t;                    // 8×32 扫描一周 T，单位 tick
wire [7:0]    sig_beam_we_tgl;               // 8 路方向图写翻转，每笔 0x79 翻一次
wire [79:0]   sig_beam_waddr;                // 8×10 compact 写偏移
wire [255:0]  sig_beam_wdata;                // 8×32 {G0,G1,G2,G3}
wire [7:0]    sig_beam_range_tgl;            // 8 路 0x7B 区间翻转
wire [79:0]   sig_beam_a0;                   // 8×10 非零区起始
wire [79:0]   sig_beam_a1;                   // 8×10 非零区结束
wire [7:0]    sig_hop_rand;                  // 8 路随机捷变
wire [7:0]    sig_freq_tab_en;               // 8 路频表使能
wire [8191:0] sig_freq_tab;                  // 8×32×32 FTW 频表
wire [7:0]    sig_phase_tab_en;              // 8 路相位表使能
wire [8191:0] sig_phase_tab;                 // 8×32×32 相位字
wire [159:0]  sig_jit_p0;                    // 8×20 抖动中心 P0
wire [159:0]  sig_jit_n;                     // 8×20 抖动周期 N_jit
wire [55:0]   sig_jit_rho;                   // 8×7 抖动深度 ρ
wire [7:0]    sig_jit_mode;                  // 8 路：0 正弦 / 1 伪随机
wire [159:0]  sig_slide_min;                 // 8×20 滑变 Pmin
wire [159:0]  sig_slide_max;                 // 8×20 滑变 Pmax
wire [159:0]  sig_slide_stp;                 // 8×20 滑变步长 δ
wire [7:0]    sig_slide_md;                  // 8 路：0 固定步长 / 1 随机步长
wire [7:0]    sig_slide_en;                  // 8 路滑变总开关


// 命令解析：保留短帧配置/动作、长帧解码与ACK。
receive_cmd_wls u_receive_cmd (
    .sig_cfg_ready          (sig_cfg_ready),
    .clk_100Mhz             (clk_cfg_100m),
    .rst                    (command_reset),
    .rxd_data               (rx_data),
    .txd_done               (tx_done),
    .rxd_valid              (rx_valid),
    .tx_req                 (tx_req),
    .tx_override            (tx_override),
    .txd_data_out           (tx_data),
    .dds_rst_0              (dds_rst_0),
    .dds_rst_1              (dds_rst_1),
    .dds_rst_2              (dds_rst_2),
    .dds_rst_3              (dds_rst_3),
    .dds_rst_4              (dds_rst_4),
    .dds_rst_5              (dds_rst_5),
    .dds_rst_6              (dds_rst_6),
    .dds_rst_7              (dds_rst_7),
    .phase_init_word_0      (phase_init_word_0),
    .cfg_load_tgl_0         (cfg_load_tgl_0),
    .start_tgl_0            (start_tgl_0),
    .stop_tgl_0             (stop_tgl_0),
    .clear_phase_tgl_0      (clear_phase_tgl_0),
    .phase_init_word_1      (phase_init_word_1),
    .cfg_load_tgl_1         (cfg_load_tgl_1),
    .start_tgl_1            (start_tgl_1),
    .stop_tgl_1             (stop_tgl_1),
    .clear_phase_tgl_1      (clear_phase_tgl_1),
    .phase_init_word_2      (phase_init_word_2),
    .cfg_load_tgl_2         (cfg_load_tgl_2),
    .start_tgl_2            (start_tgl_2),
    .stop_tgl_2             (stop_tgl_2),
    .clear_phase_tgl_2      (clear_phase_tgl_2),
    .phase_init_word_3      (phase_init_word_3),
    .cfg_load_tgl_3         (cfg_load_tgl_3),
    .start_tgl_3            (start_tgl_3),
    .stop_tgl_3             (stop_tgl_3),
    .clear_phase_tgl_3      (clear_phase_tgl_3),
    .phase_init_word_4      (phase_init_word_4),
    .cfg_load_tgl_4         (cfg_load_tgl_4),
    .start_tgl_4            (start_tgl_4),
    .stop_tgl_4             (stop_tgl_4),
    .clear_phase_tgl_4      (clear_phase_tgl_4),
    .phase_init_word_5      (phase_init_word_5),
    .cfg_load_tgl_5         (cfg_load_tgl_5),
    .start_tgl_5            (start_tgl_5),
    .stop_tgl_5             (stop_tgl_5),
    .clear_phase_tgl_5      (clear_phase_tgl_5),
    .phase_init_word_6      (phase_init_word_6),
    .cfg_load_tgl_6         (cfg_load_tgl_6),
    .start_tgl_6            (start_tgl_6),
    .stop_tgl_6             (stop_tgl_6),
    .clear_phase_tgl_6      (clear_phase_tgl_6),
    .phase_init_word_7      (phase_init_word_7),
    .cfg_load_tgl_7         (cfg_load_tgl_7),
    .start_tgl_7            (start_tgl_7),
    .stop_tgl_7             (stop_tgl_7),
    .clear_phase_tgl_7      (clear_phase_tgl_7),
    .sig_use_new            (sig_use_new),
    .sig_type               (sig_type),
    .sig_freq_type          (sig_freq_type),
    .sig_cw_en              (sig_cw_en),
    .sig_lfm_neg            (sig_lfm_neg),
    .sig_pw_ticks           (sig_pw_ticks),
    .sig_pri_ticks          (sig_pri_ticks),
    .sig_fc_hz              (sig_fc_hz),
    .sig_hop_bw_hz          (sig_hop_bw_hz),
    .sig_hop_n              (sig_hop_n),
    .sig_hop_ticks          (sig_hop_ticks),
    .sig_lfm_bw_hz          (sig_lfm_bw_hz),
    .sig_lfm_ticks          (sig_lfm_ticks),
    .sig_pm_type            (sig_pm_type),
    .sig_nlfm_type          (sig_nlfm_type),
    .sig_nc                 (sig_nc),
    .sig_ncode              (sig_ncode),
    .sig_pri_type           (sig_pri_type),
    .sig_pri_m              (sig_pri_m),
    .sig_pri_grp            (sig_pri_grp),
    .sig_hop_grp            (sig_hop_grp),
    .sig_pri_tab            (sig_pri_tab),
    .sig_scan_en            (sig_scan_en),
    .sig_scan_type          (sig_scan_type),
    .sig_scan_end           (sig_scan_end),
    .sig_scan_t             (sig_scan_t),
    .sig_beam_we_tgl        (sig_beam_we_tgl),
    .sig_beam_waddr         (sig_beam_waddr),
    .sig_beam_wdata         (sig_beam_wdata),
    .sig_beam_range_tgl     (sig_beam_range_tgl),
    .sig_beam_a0            (sig_beam_a0),
    .sig_beam_a1            (sig_beam_a1),
    .sig_hop_rand           (sig_hop_rand),
    .sig_freq_tab_en        (sig_freq_tab_en),
    .sig_freq_tab           (sig_freq_tab),
    .sig_phase_tab_en       (sig_phase_tab_en),
    .sig_phase_tab          (sig_phase_tab),
    .sig_jit_p0             (sig_jit_p0),
    .sig_jit_n              (sig_jit_n),
    .sig_jit_rho            (sig_jit_rho),
    .sig_jit_mode           (sig_jit_mode),
    .sig_slide_min          (sig_slide_min),
    .sig_slide_max          (sig_slide_max),
    .sig_slide_stp          (sig_slide_stp),
    .sig_slide_md           (sig_slide_md),
    .sig_slide_en           (sig_slide_en),
    .dds_sys_rst            (dds_sys_rst),
    .dds_ch_en              (dds_ch_en)
);

// 波形输出：直接连接公开数据口，出口寄存仍由top_8ch_dds承担。
top_8ch_dds u_top_8ch_dds (
    .cfg_clk                (clk_cfg_100m),
    .sig_cfg_ready          (sig_cfg_ready),
    .clk_100Mhz             (clk_dds_300m),
    .rst                    (dds_sys_rst | command_reset),
    .ch_en                  (dds_ch_en),
    .dds_rst_0              (dds_rst_0),
    .dds_rst_1              (dds_rst_1),
    .dds_rst_2              (dds_rst_2),
    .dds_rst_3              (dds_rst_3),
    .dds_rst_4              (dds_rst_4),
    .dds_rst_5              (dds_rst_5),
    .dds_rst_6              (dds_rst_6),
    .dds_rst_7              (dds_rst_7),
    .phase_init_word_0      (phase_init_word_0),
    .cfg_load_tgl_0         (cfg_load_tgl_0),
    .start_tgl_0            (start_tgl_0),
    .stop_tgl_0             (stop_tgl_0),
    .clear_phase_tgl_0      (clear_phase_tgl_0),
    .phase_init_word_1      (phase_init_word_1),
    .cfg_load_tgl_1         (cfg_load_tgl_1),
    .start_tgl_1            (start_tgl_1),
    .stop_tgl_1             (stop_tgl_1),
    .clear_phase_tgl_1      (clear_phase_tgl_1),
    .phase_init_word_2      (phase_init_word_2),
    .cfg_load_tgl_2         (cfg_load_tgl_2),
    .start_tgl_2            (start_tgl_2),
    .stop_tgl_2             (stop_tgl_2),
    .clear_phase_tgl_2      (clear_phase_tgl_2),
    .phase_init_word_3      (phase_init_word_3),
    .cfg_load_tgl_3         (cfg_load_tgl_3),
    .start_tgl_3            (start_tgl_3),
    .stop_tgl_3             (stop_tgl_3),
    .clear_phase_tgl_3      (clear_phase_tgl_3),
    .phase_init_word_4      (phase_init_word_4),
    .cfg_load_tgl_4         (cfg_load_tgl_4),
    .start_tgl_4            (start_tgl_4),
    .stop_tgl_4             (stop_tgl_4),
    .clear_phase_tgl_4      (clear_phase_tgl_4),
    .phase_init_word_5      (phase_init_word_5),
    .cfg_load_tgl_5         (cfg_load_tgl_5),
    .start_tgl_5            (start_tgl_5),
    .stop_tgl_5             (stop_tgl_5),
    .clear_phase_tgl_5      (clear_phase_tgl_5),
    .phase_init_word_6      (phase_init_word_6),
    .cfg_load_tgl_6         (cfg_load_tgl_6),
    .start_tgl_6            (start_tgl_6),
    .stop_tgl_6             (stop_tgl_6),
    .clear_phase_tgl_6      (clear_phase_tgl_6),
    .phase_init_word_7      (phase_init_word_7),
    .cfg_load_tgl_7         (cfg_load_tgl_7),
    .start_tgl_7            (start_tgl_7),
    .stop_tgl_7             (stop_tgl_7),
    .clear_phase_tgl_7      (clear_phase_tgl_7),
    .sig_use_new            (sig_use_new),
    .sig_type               (sig_type),
    .sig_freq_type          (sig_freq_type),
    .sig_cw_en              (sig_cw_en),
    .sig_lfm_neg            (sig_lfm_neg),
    .sig_pw_ticks           (sig_pw_ticks),
    .sig_pri_ticks          (sig_pri_ticks),
    .sig_fc_hz              (sig_fc_hz),
    .sig_hop_bw_hz          (sig_hop_bw_hz),
    .sig_hop_n              (sig_hop_n),
    .sig_hop_ticks          (sig_hop_ticks),
    .sig_lfm_bw_hz          (sig_lfm_bw_hz),
    .sig_lfm_ticks          (sig_lfm_ticks),
    .sig_pm_type            (sig_pm_type),
    .sig_nlfm_type          (sig_nlfm_type),
    .sig_nc                 (sig_nc),
    .sig_ncode              (sig_ncode),
    .sig_pri_type           (sig_pri_type),
    .sig_pri_m              (sig_pri_m),
    .sig_pri_grp            (sig_pri_grp),
    .sig_hop_grp            (sig_hop_grp),
    .sig_pri_tab            (sig_pri_tab),
    .sig_scan_en            (sig_scan_en),
    .sig_scan_type          (sig_scan_type),
    .sig_scan_end           (sig_scan_end),
    .sig_scan_t             (sig_scan_t),
    .sig_beam_we_tgl        (sig_beam_we_tgl),
    .sig_beam_waddr         (sig_beam_waddr),
    .sig_beam_wdata         (sig_beam_wdata),
    .sig_beam_range_tgl     (sig_beam_range_tgl),
    .sig_beam_a0            (sig_beam_a0),
    .sig_beam_a1            (sig_beam_a1),
    .sig_hop_rand           (sig_hop_rand),
    .sig_freq_tab_en        (sig_freq_tab_en),
    .sig_freq_tab           (sig_freq_tab),
    .sig_phase_tab_en       (sig_phase_tab_en),
    .sig_phase_tab          (sig_phase_tab),
    .sig_jit_p0             (sig_jit_p0),
    .sig_jit_n              (sig_jit_n),
    .sig_jit_rho            (sig_jit_rho),
    .sig_jit_mode           (sig_jit_mode),
    .sig_slide_min          (sig_slide_min),
    .sig_slide_max          (sig_slide_max),
    .sig_slide_stp          (sig_slide_stp),
    .sig_slide_md           (sig_slide_md),
    .sig_slide_en           (sig_slide_en),
    .dds_sin_0              (dds_data_0),
    .dds_valid_0            (dds_data_vld_0),
    .dds_sin_1              (dds_data_1),
    .dds_valid_1            (dds_data_vld_1),
    .dds_sin_2              (dds_data_2),
    .dds_valid_2            (dds_data_vld_2),
    .dds_sin_3              (dds_data_3),
    .dds_valid_3            (dds_data_vld_3),
    .dds_sin_4              (dds_data_4),
    .dds_valid_4            (dds_data_vld_4),
    .dds_sin_5              (dds_data_5),
    .dds_valid_5            (dds_data_vld_5),
    .dds_sin_6              (dds_data_6),
    .dds_valid_6            (dds_data_vld_6),
    .dds_sin_7              (dds_data_7),
    .dds_valid_7            (dds_data_vld_7)
);
endmodule
