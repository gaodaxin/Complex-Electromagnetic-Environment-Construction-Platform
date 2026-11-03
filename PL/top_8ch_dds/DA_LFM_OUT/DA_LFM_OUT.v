// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/DA_LFM_OUT.v
// 功能：单通道装配层：同步配置和动作，安排计算/运行/PRI，合成相位，查正弦表，并施加扫描增益。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 单通道顶层装配：命令域参数 → 300 MHz 出波
// =============================================================================

module DA_LFM_OUT (

    input wire cfg_clk, cfg_rst,
    output wire cfg_ready,
    input  wire          clk,                // FPGA 工作时钟，当前 300 MHz
    input  wire          dds_rst,            // 通道复位电平，来自命令域
    input  wire [31:0]   phase_init_word,    // 初相，命令域
    input  wire          cfg_load_tgl,       // 加载翻转，变沿后变成 1 拍
    input  wire          start_tgl,          // 启动翻转
    input  wire          stop_tgl,           // 停止翻转
    input  wire          clear_phase_tgl,    // 清相翻转

    input  wire          sig_use_new,        // 新链开关，1 时走本链路
    input  wire [1:0]    sig_type,           // 信号类型：0 CW / 1 FM / 2 PM
    input  wire [1:0]    sig_freq_type,      // 捷变类型：0 单载频 / 1 脉内 / 2 脉间 / 3 脉组
    input  wire          sig_cw_en,          // CW 连续波使能
    input  wire          sig_lfm_neg,        // LFM 负扫
    input  wire [19:0]   sig_pw_ticks,       // 脉宽，单位 tick
    input  wire [19:0]   sig_pri_ticks,      // 默认 PRI，单位 tick
    input  wire [47:0]   sig_fc_hz,          // 中心频率，低 32 bit 为 FTW
    input  wire [31:0]   sig_hop_bw_hz,      // 捷变带宽 FTW
    input  wire [5:0]    sig_hop_n,          // 捷变频点数
    input  wire [15:0]   sig_hop_ticks,      // 脉内 hop 间隔
    input  wire [31:0]   sig_lfm_bw_hz,      // 调频带宽 FTW
    input  wire [19:0]   sig_lfm_ticks,      // 调频时长，单位 tick
    input  wire [1:0]    sig_pm_type,        // FM 子类型或 PM 子类型
    input  wire [1:0]    sig_nlfm_type,      // NLFM 形状
    input  wire [15:0]   sig_nc,             // 码元宽度
    input  wire [15:0]   sig_ncode,          // 码长
    input  wire [1:0]    sig_pri_type,       // PRI 类型
    input  wire [5:0]    sig_pri_m,          // 参差 PRI 个数
    input  wire [5:0]    sig_pri_grp,        // 脉组参差组大小
    input  wire [5:0]    sig_hop_grp,        // 频率捷变脉组大小
    input  wire [639:0]  sig_pri_tab,        // PRI 表，32 × 20 bit
    input  wire          sig_scan_en,        // 天线扫描使能
    input  wire [1:0]    sig_scan_type,      // 扫描类型
    input  wire [31:0]   sig_scan_end,       // [9:0] start [19:10] span [29:20] target
    input  wire [31:0]   sig_scan_t,         // 扫描周期，单位 tick
    input  wire          sig_beam_we_tgl,    // 方向图写翻转（命令域）
    input  wire [9:0]    sig_beam_waddr,     // compact 写偏移
    input  wire [31:0]   sig_beam_wdata,     // {G0,G1,G2,G3}
    input  wire          sig_beam_range_tgl, // 0x7B 区间翻转
    input  wire [9:0]    sig_beam_a0,        // 非零区起始
    input  wire [9:0]    sig_beam_a1,        // 非零区结束
    input  wire          sig_hop_rand,       // 捷变随机抽点
    input  wire          sig_freq_tab_en,    // 频表使能
    input  wire [1023:0] sig_freq_tab,       // 频表，32 × 32 bit
    input  wire          sig_phase_tab_en,   // 相位表使能
    input  wire [1023:0] sig_phase_tab,      // 相位表，32 × 32 bit
    input  wire [19:0]   sig_jit_p0,         // 抖动基准 PRI
    input  wire [19:0]   sig_jit_n,          // 抖动周期参数
    input  wire [6:0]    sig_jit_rho,        // 抖动深度
    input  wire          sig_jit_mode,       // 抖动模式
    input  wire [19:0]   sig_slide_min,      // 滑变下限
    input  wire [19:0]   sig_slide_max,      // 滑变上限
    input  wire [19:0]   sig_slide_stp,      // 滑变步长
    input  wire          sig_slide_md,       // 滑变步长模式
    input  wire          sig_slide_en,       // 滑变总开关

    output wire [255:0]  dds_sin,            // 16×int16 正弦样点
    output wire          dds_valid           // 与 dds_sin 对齐的有效
);

(* ASYNC_REG="TRUE" *) reg [1:0] cfg_rst_sync=2'b11;
always @(posedge cfg_clk) cfg_rst_sync<={cfg_rst_sync[0],cfg_rst};
wire        dds_rst_sync;                    // 本域两级同步后的复位
wire [31:0] phase_init_sync;                 // 本域两级同步后的初相
wire        cfg_load_edge;                   // 加载沿，toggle 变 1 拍
wire        start_edge;                      // 启动沿，toggle 变 1 拍
wire        stop_edge;                       // 停止沿，toggle 变 1 拍
wire        clear_phase_edge;                // 清相沿，toggle 变 1 拍


// =============================================================================
// 命令域 → 本域：电平两级同步，toggle → 边沿
// =============================================================================

da_lfm_input_sync u_input_sync (

    .clk               (clk),                // 工作时钟
    .dds_rst           (dds_rst),            // 命令域复位电平
    .phase_init_word   (phase_init_word),    // 命令域初相
    .cfg_load_tgl      (cfg_load_tgl),       // 加载翻转
    .start_tgl         (start_tgl),          // 启动翻转
    .stop_tgl          (stop_tgl),           // 停止翻转
    .clear_phase_tgl   (clear_phase_tgl),    // 清相翻转
    .dds_rst_sync      (dds_rst_sync),       // 同步后复位
    .phase_init_sync   (phase_init_sync),    // 同步后初相
    .cfg_load_edge     (cfg_load_edge),      // 加载沿，1 拍
    .start_edge        (start_edge),         // 启动沿，1 拍
    .stop_edge         (stop_edge),          // 停止沿，1 拍
    .clear_phase_edge  (clear_phase_edge)    // 清相沿，1 拍

);


wire cfg_applied, config_complete;
wire param_source_ready,beam_source_ready;
assign cfg_ready=param_source_ready && beam_source_ready;
wire [31:0] phase_active;
wire scan_busy;
wire        use_new;                         // 打拍后的新链开关
wire        cw;                              // 打拍后的连续波使能
wire        ln;                              // 打拍后的 LFM 负扫
wire        sen;                             // 打拍后的扫描使能
wire        hr;                              // 打拍后的随机捷变
wire        fte;                             // 打拍后的频表使能
wire        pte;                             // 打拍后的相位表使能
wire        jm;                              // 打拍后的抖动模式
wire        smd;                             // 打拍后的滑变模式
wire        slen;                            // 打拍后的滑变总开关
wire [1:0]  st;                              // 打拍后的信号类型
wire [1:0]  ft;                              // 打拍后的捷变类型
wire [1:0]  pm;                              // 打拍后的 FM/PM 子类型
wire [1:0]  nl;                              // 打拍后的 NLFM 形状
wire [1:0]  pty;                             // 打拍后的 PRI 类型
wire [1:0]  sty;                             // 打拍后的扫描类型
wire [19:0] pwt;                             // 打拍后的脉宽
wire [19:0] prt;                             // 打拍后的默认 PRI
wire [19:0] lt;                              // 打拍后的调频时长
wire [19:0] jp0;                             // 打拍后的抖动基准
wire [19:0] jn;                              // 打拍后的抖动周期
wire [19:0] smin;                            // 打拍后的滑变下限
wire [19:0] smax;                            // 打拍后的滑变上限
wire [19:0] sst;                             // 打拍后的滑变步长
wire [31:0] fch;                             // 打拍后的中心 FTW
wire [31:0] hbw;                             // 打拍后的捷变带宽
wire [31:0] lbw;                             // 打拍后的调频带宽
wire [31:0] sed;                             // 打拍后的扫描配置：start/span/target
wire [31:0] stm;                             // 打拍后的扫描周期
wire [5:0]  hn;                              // 打拍后的捷变点数
wire [5:0]  prm;                             // 打拍后的参差项数
wire [5:0]  pgr;                             // 打拍后的 PRI 组大小
wire [5:0]  hgr;                             // 打拍后的 hop 组大小
wire [15:0] ht;                              // 打拍后的 hop 间隔
wire [15:0] nc;                              // 打拍后的码元宽度
wire [15:0] nk;                              // 打拍后的码长
wire [6:0]  jrh;                             // 打拍后的抖动深度
wire [639:0]  pri_tab_s;                     // load 快照后的 PRI 表
wire [1023:0] freq_tab_s;                    // load 快照后的频表
wire [1023:0] phase_tab_s;                   // load 快照后的相位表


// =============================================================================
// 标量两级打拍，宽表仅在 load 沿锁存
// =============================================================================

da_sig_param_pipe u_pipe (

    .cfg_clk(cfg_clk),.cfg_rst(cfg_rst_sync[1]),.cfg_load_tgl(cfg_load_tgl),
    .cfg_ready(param_source_ready),.cfg_captured(cfg_applied),.capture_release(config_complete),
    .phase_init_word(phase_init_word),.phase_active(phase_active),
    .clk              (clk),                 // 工作时钟
    .rst_sync         (dds_rst_sync),        // 同步复位
    .cfg_load_edge    (cfg_load_edge),       // 加载沿，快照宽表
    .sig_use_new      (sig_use_new),         // 新链开关
    .sig_type         (sig_type),            // 信号类型
    .sig_freq_type    (sig_freq_type),       // 捷变类型
    .sig_cw_en        (sig_cw_en),           // 连续波使能
    .sig_lfm_neg      (sig_lfm_neg),         // LFM 负扫
    .sig_pw_ticks     (sig_pw_ticks),        // 脉宽
    .sig_pri_ticks    (sig_pri_ticks),       // 默认 PRI
    .sig_fc_hz        (sig_fc_hz),           // 中心频率，内部取低 32 bit
    .sig_hop_bw_hz    (sig_hop_bw_hz),       // 捷变带宽
    .sig_hop_n        (sig_hop_n),           // 捷变点数
    .sig_hop_ticks    (sig_hop_ticks),       // 脉内 hop 间隔
    .sig_lfm_bw_hz    (sig_lfm_bw_hz),       // 调频带宽
    .sig_lfm_ticks    (sig_lfm_ticks),       // 调频时长
    .sig_pm_type      (sig_pm_type),         // FM/PM 子类型
    .sig_nlfm_type    (sig_nlfm_type),       // NLFM 形状
    .sig_nc           (sig_nc),              // 码元宽度
    .sig_ncode        (sig_ncode),           // 码长
    .sig_pri_type     (sig_pri_type),        // PRI 类型
    .sig_pri_m        (sig_pri_m),           // 参差项数
    .sig_pri_grp      (sig_pri_grp),         // PRI 脉组大小
    .sig_hop_grp      (sig_hop_grp),         // hop 脉组大小
    .sig_pri_tab      (sig_pri_tab),         // PRI 表
    .sig_scan_en      (sig_scan_en),         // 扫描使能
    .sig_scan_type    (sig_scan_type),       // 扫描类型
    .sig_scan_end     (sig_scan_end),        // start/span/target 打包
    .sig_scan_t       (sig_scan_t),          // 扫描周期
    .sig_hop_rand     (sig_hop_rand),        // 随机捷变
    .sig_freq_tab_en  (sig_freq_tab_en),     // 频表使能
    .sig_freq_tab     (sig_freq_tab),        // 频表
    .sig_phase_tab_en (sig_phase_tab_en),    // 相位表使能
    .sig_phase_tab    (sig_phase_tab),       // 相位表
    .sig_jit_p0       (sig_jit_p0),          // 抖动基准
    .sig_jit_n        (sig_jit_n),           // 抖动周期
    .sig_jit_rho      (sig_jit_rho),         // 抖动深度
    .sig_jit_mode     (sig_jit_mode),        // 抖动模式
    .sig_slide_min    (sig_slide_min),       // 滑变下限
    .sig_slide_max    (sig_slide_max),       // 滑变上限
    .sig_slide_stp    (sig_slide_stp),       // 滑变步长
    .sig_slide_md     (sig_slide_md),
    .sig_slide_en     (sig_slide_en),
    .use_new          (use_new),             // 打拍后新链开关
    .st               (st),                  // 打拍后信号类型
    .ft               (ft),                  // 打拍后捷变类型
    .cw               (cw),                  // 打拍后连续波
    .ln               (ln),                  // 打拍后负扫
    .pwt              (pwt),                 // 打拍后脉宽
    .prt              (prt),                 // 打拍后默认 PRI
    .fch              (fch),                 // 打拍后中心 FTW
    .hbw              (hbw),                 // 打拍后捷变带宽
    .hn               (hn),                  // 打拍后捷变点数
    .ht               (ht),                  // 打拍后 hop 间隔
    .lbw              (lbw),                 // 打拍后调频带宽
    .lt               (lt),                  // 打拍后调频时长
    .pm               (pm),                  // 打拍后 FM/PM 子类型
    .nl               (nl),                  // 打拍后 NLFM 形状
    .nc               (nc),                  // 打拍后码元宽度
    .nk               (nk),                  // 打拍后码长
    .pty              (pty),                 // 打拍后 PRI 类型
    .prm              (prm),                 // 打拍后参差项数
    .pgr              (pgr),                 // 打拍后 PRI 组大小
    .hgr              (hgr),                 // 打拍后 hop 组大小
    .sen              (sen),                 // 打拍后扫描使能
    .sty              (sty),                 // 打拍后扫描类型
    .sed              (sed),                 // 打拍后 start/span/target
    .stm              (stm),                 // 打拍后扫描周期
    .hr               (hr),                  // 打拍后随机捷变
    .fte              (fte),                 // 打拍后频表使能
    .pte              (pte),                 // 打拍后相位表使能
    .jm               (jm),                  // 打拍后抖动模式
    .smd              (smd),
    .slen             (slen),
    .jp0              (jp0),                 // 打拍后抖动基准
    .jn               (jn),                  // 打拍后抖动周期
    .jrh              (jrh),                 // 打拍后抖动深度
    .smin             (smin),                // 打拍后滑变下限
    .smax             (smax),                // 打拍后滑变上限
    .sst              (sst),                 // 打拍后滑变步长
    .pri_tab_s        (pri_tab_s),           // 快照 PRI 表
    .freq_tab_s       (freq_tab_s),          // 快照频表
    .phase_tab_s      (phase_tab_s)          // 快照相位表

);


wire        calc_busy;                       // cfg_calc 正在算 hop/斜率
wire        calc_done;                       // 计算完成 1 拍
wire        calc_go;                         // 发给 cfg_calc 的 1 拍启动
wire        running_n;                       // 通道运行；1 时 PRI/波形推进
wire        start_n;                         // 发给 tick/PRI/相位的 1 拍启动
wire [7:0]  calc_err;                        // 0=可 start，非 0 丢掉挂起的 start
wire [31:0] hop_start_n;                     // 捷变起点 FTW
wire [31:0] hop_step_n;                      // 相邻 hop 间隔
wire [31:0] n_samp;                          // N_sample = 48*lfm_ticks
wire [31:0] phase_step_n;                    // 正弦 NLFM：2^32/N_sample
wire signed [63:0] lfm_slope_q32_n, lfm_start_q32_n;
wire beam_busy;
wire pri_ready;
wire signed [31:0] slope_n;                  // LFM 每样点斜率 S
wire signed [31:0] fm_start_n;               // 调制起点 ±B/2


// =============================================================================
// 运行控制：load 后先算，算完才允许 start
// =============================================================================

da_sig_run_ctrl u_run (

    .clk            (clk),                   // 工作时钟
    .rst_sync       (dds_rst_sync),          // 同步复位
    .use_new        (use_new),               // 新链开关；0 时不响应 start/load
    .load_request(cfg_load_edge),.config_complete(config_complete),
    .aux_ready(!scan_busy && !beam_busy && pri_ready),
    .cfg_load_edge  (cfg_applied),         // 加载沿：启动计算并停止出波
    .start_edge     (start_edge),            // 启动沿
    .stop_edge      (stop_edge),             // 停止沿
    .calc_busy      (calc_busy),             // cfg_calc 正在算
    .calc_err       (calc_err),              // 计算错误码
    .calc_done      (calc_done),             // 计算完成 1 拍
    .calc_go        (calc_go),               // 发给 cfg_calc 的 1 拍启动
    .running        (running_n),             // 通道运行；1 时 PRI/波形推进
    .start_n        (start_n)                // 发给 tick/PRI/相位的 1 拍启动

);


// =============================================================================
// load 后计算 hop 起点/步长、LFM 斜率、N_sample
// =============================================================================

da_sig_cfg_calc u_cfg_calc (

    .clk          (clk),                     // 工作时钟
    .rst          (dds_rst_sync),            // 同步复位
    .start_calc   (calc_go),                 // load 后 1 拍启动计算
    .sig_type     (st),                      // 打拍后的信号类型
    .freq_type    (ft),                      // 打拍后的捷变类型
    .lfm_neg      (ln),                      // 打拍后的负扫
    .fc_ftw       (fch),                     // 中心 FTW
    .hop_bw_ftw   (hbw),                     // 捷变带宽 FTW
    .hop_n        (hn),                      // 捷变点数
    .lfm_bw_ftw   (lbw),                     // 调频带宽 FTW
    .lfm_ticks    (lt),                      // 调频时长
    .busy         (calc_busy),               // 计算忙，挡住 start
    .done         (calc_done),               // 计算完成 1 拍
    .err          (calc_err),                // 错误码，0 才可 start
    .hop_start    (hop_start_n),             // 捷变起点 FTW
    .hop_step     (hop_step_n),              // 相邻 hop 间隔
    .slope_word   (slope_n),                 // 每样点斜率 S
    .n_sample     (n_samp),                  // 48*lfm_ticks
    .fm_start     (fm_start_n),              // 调制起点 ±B/2
    .lfm_slope_q32(lfm_slope_q32_n),.lfm_start_q32(lfm_start_q32_n),
    .phase_step   (phase_step_n)             // 正弦 NLFM：2^32/N_sample

);


wire tick_n;                                 // 每 3 个 clk 一次的 1 拍 tick
wire [1:0] tick_div_n;                       // 格内拍号 0/1/2，本层未再使用


// =============================================================================
// PRI 时间基准：1 tick = 3 clk
// =============================================================================

da_sig_tick u_tick (

    .clk         (clk),                      // 工作时钟
    .rst_sync    (dds_rst_sync),             // 同步复位
    .running     (running_n),                // 运行使能；0 时冻结 tick
    .start_edge  (start_n),                  // 启动沿，计数回 0
    .tick_div    (tick_div_n),               // 格内拍号
    .tick        (tick_n)                    // 给 PRI/脉宽/捷变/码元用的 tick

);


wire        gate_n;                          // 脉冲有效门控
wire        rise_n;                          // 新脉冲开始事件
wire        prd_n;                           // 新 PRI 周期事件
wire        grp_n;                           // 频率捷变脉组切换事件
wire        ready_n;                         // PRI 时间轴已经正式启动
wire [19:0] pidx_n;                          // 当前脉冲/PRI 编号，本层未再使用
wire [4:0]  pptr_n;                          // 当前参差 PRI 表索引，本层未再使用


// =============================================================================
// PRI 主时间轴：门控、脉冲事件、参差/抖动/滑变
// =============================================================================

da_sig_pri_sched u_pri (

    .clk             (clk),                  // 工作时钟
    .rst_sync        (dds_rst_sync),         // 同步复位
    .running         (running_n),            // 运行使能
    .start_edge      (start_n),              // 启动沿
    .tick            (tick_n),               // PRI 时间推进 tick
    .cw_en           (cw),                   // CW 时门控恒开
    .pw_ticks        (pwt),                  // 脉宽
    .pri_type        (pty),                  // PRI 类型
    .pri_m           (prm),                  // 参差项数
    .pri_group_size  (pgr),                  // PRI 脉组大小
    .hop_group_size  (hgr),                  // hop 脉组大小
    .pri_tab_bus     (pri_tab_s),            // 快照后的 PRI 表
    .jit_p0          (jp0),                  // 抖动基准
    .jit_n           (jn),                   // 抖动周期
    .jit_rho         (jrh),                  // 抖动深度
    .jit_mode        (jm),                   // 抖动模式
    .slide_min       (smin),                 // 滑变下限
    .slide_max       (smax),                 // 滑变上限
    .slide_stp       (sst),                  // 滑变步长
    .slide_md        (smd),
    .slide_run       (slen),
    .pulse_gate      (gate_n),               // 脉冲有效门控
    .pulse_rise      (rise_n),               // 新脉冲开始事件
    .period_rise     (prd_n),                // 新 PRI 周期事件
    .group_rise      (grp_n),                // 频率捷变脉组切换事件
    .wave_ready      (ready_n),              // PRI 时间轴已经正式启动
    .pulse_idx       (pidx_n),               // 当前脉冲编号
    .pri_ptr         (pptr_n),               // 当前 PRI 表索引
    .cfg_ready       (pri_ready)

);


wire        n_valid;                         // 相位有效
wire        n_user;                          // 脉冲门内用户标志
wire [31:0] np0;                             // 第 0 路瞬时相位
wire [31:0] np1;                             // 第 1 路瞬时相位
wire [31:0] np2;                             // 第 2 路瞬时相位
wire [31:0] np3;                             // 第 3 路瞬时相位
wire [31:0] np4;                             // 第 4 路瞬时相位
wire [31:0] np5;                             // 第 5 路瞬时相位
wire [31:0] np6;                             // 第 6 路瞬时相位
wire [31:0] np7;                             // 第 7 路瞬时相位
wire [31:0] np8;                             // 第 8 路瞬时相位
wire [31:0] np9;                             // 第 9 路瞬时相位
wire [31:0] np10;                            // 第 10 路瞬时相位
wire [31:0] np11;                            // 第 11 路瞬时相位
wire [31:0] np12;                            // 第 12 路瞬时相位
wire [31:0] np13;                            // 第 13 路瞬时相位
wire [31:0] np14;                            // 第 14 路瞬时相位
wire [31:0] np15;                            // 第 15 路瞬时相位


// =============================================================================
// 波形合成：F_inst = 载波 + 调制，输出 16 路相位
// =============================================================================

da_sig_wave u_wave (

    .clk               (clk),                // 工作时钟
    .rst_sync          (dds_rst_sync),       // 同步复位
    .running           (running_n),          // 运行使能
    .start_edge        (start_n),            // 启动沿
    .tick              (tick_n),             // 时间 tick
    .pulse_rise        (rise_n),             // 新脉冲开始
    .period_rise       (prd_n),              // 新 PRI 周期
    .group_rise        (grp_n),              // 脉组切换
    .wave_ready        (ready_n),            // 时间轴就绪
    .pulse_gate        (gate_n),             // 脉冲门控
    .cw_en             (cw),                 // 连续波
    .sig_type          (st),                 // 信号类型
    .freq_type         (ft),                 // 捷变类型
    .fm_pm_type        (pm),                 // FM/PM 子类型
    .nlfm_type         (nl),                 // NLFM 形状
    .hop_n             (hn),                 // 捷变点数
    .hop_ticks         (ht),                 // 脉内 hop 间隔
    .hop_start         (hop_start_n),        // 捷变起点
    .hop_step          (hop_step_n),         // 捷变步长
    .hop_rand          (hr),                 // 随机捷变
    .freq_tab_en       (fte),                // 频表使能
    .freq_tab_bus      (freq_tab_s),         // 快照频表
    .phase_tab_en      (pte),                // 相位表使能
    .phase_tab_bus     (phase_tab_s),        // 快照相位表
    .lfm_slope_q32(lfm_slope_q32_n),.lfm_start_q32(lfm_start_q32_n),
    .slope_word        (slope_n),            // LFM 斜率
    .n_sample          (n_samp),             // 调频样点数
    .fm_start          (fm_start_n),         // 调制起点
    .phase_step        (phase_step_n),       // 正弦 NLFM 相位步长
    .nc                (nc),                 // 码元宽度
    .ncode             (nk),                 // 码长
    .phase_init        (phase_active),    // 同步后初相
    .clear_phase_edge  (clear_phase_edge),   // 清相沿
    .phase_valid       (n_valid),            // 相位有效
    .phase_user        (n_user),             // 门控：0 则后面静音
    .phase_r_0         (np0),                // 第 0 路相位
    .phase_r_1         (np1),                // 第 1 路相位
    .phase_r_2         (np2),                // 第 2 路相位
    .phase_r_3         (np3),                // 第 3 路相位
    .phase_r_4         (np4),                // 第 4 路相位
    .phase_r_5         (np5),                // 第 5 路相位
    .phase_r_6         (np6),                // 第 6 路相位
    .phase_r_7         (np7),                // 第 7 路相位
    .phase_r_8         (np8),                // 第 8 路相位
    .phase_r_9         (np9),                // 第 9 路相位
    .phase_r_10        (np10),               // 第 10 路相位
    .phase_r_11        (np11),               // 第 11 路相位
    .phase_r_12        (np12),               // 第 12 路相位
    .phase_r_13        (np13),               // 第 13 路相位
    .phase_r_14        (np14),               // 第 14 路相位
    .phase_r_15        (np15)                // 第 15 路相位

);


reg         mux_valid;                       // 打拍后的相位有效
reg         mux_user;                        // 打拍后的门控
reg  [31:0] mx0;                             // 打拍后第 0 路相位
reg  [31:0] mx1;                             // 打拍后第 1 路相位
reg  [31:0] mx2;                             // 打拍后第 2 路相位
reg  [31:0] mx3;                             // 打拍后第 3 路相位
reg  [31:0] mx4;                             // 打拍后第 4 路相位
reg  [31:0] mx5;                             // 打拍后第 5 路相位
reg  [31:0] mx6;                             // 打拍后第 6 路相位
reg  [31:0] mx7;                             // 打拍后第 7 路相位
reg  [31:0] mx8;                             // 打拍后第 8 路相位
reg  [31:0] mx9;                             // 打拍后第 9 路相位
reg  [31:0] mx10;                            // 打拍后第 10 路相位
reg  [31:0] mx11;                            // 打拍后第 11 路相位
reg  [31:0] mx12;                            // 打拍后第 12 路相位
reg  [31:0] mx13;                            // 打拍后第 13 路相位
reg  [31:0] mx14;                            // 打拍后第 14 路相位
reg  [31:0] mx15;                            // 打拍后第 15 路相位


// =============================================================================
// 相位出口打一拍，切断 wave → LUT 的长组合
// =============================================================================

always @(posedge clk) begin

    if (dds_rst_sync) begin

        mux_valid <= 1'b0;                   // 复位后相位无效
        mux_user  <= 1'b0;                   // 复位后门控关闭
        mx0  <= 32'd0;                       // 第 0 路相位清零
        mx1  <= 32'd0;                       // 第 1 路相位清零
        mx2  <= 32'd0;                       // 第 2 路相位清零
        mx3  <= 32'd0;                       // 第 3 路相位清零
        mx4  <= 32'd0;                       // 第 4 路相位清零
        mx5  <= 32'd0;                       // 第 5 路相位清零
        mx6  <= 32'd0;                       // 第 6 路相位清零
        mx7  <= 32'd0;                       // 第 7 路相位清零
        mx8  <= 32'd0;                       // 第 8 路相位清零
        mx9  <= 32'd0;                       // 第 9 路相位清零
        mx10 <= 32'd0;                       // 第 10 路相位清零
        mx11 <= 32'd0;                       // 第 11 路相位清零
        mx12 <= 32'd0;                       // 第 12 路相位清零
        mx13 <= 32'd0;                       // 第 13 路相位清零
        mx14 <= 32'd0;                       // 第 14 路相位清零
        mx15 <= 32'd0;                       // 第 15 路相位清零

    end else begin

        mux_valid <= n_valid;                // 相位有效打一拍
        mux_user  <= n_user && running_n;    // 运行中且在门内才允许出样
        mx0  <= np0;                         // 第 0 路相位打一拍
        mx1  <= np1;                         // 第 1 路相位打一拍
        mx2  <= np2;                         // 第 2 路相位打一拍
        mx3  <= np3;                         // 第 3 路相位打一拍
        mx4  <= np4;                         // 第 4 路相位打一拍
        mx5  <= np5;                         // 第 5 路相位打一拍
        mx6  <= np6;                         // 第 6 路相位打一拍
        mx7  <= np7;                         // 第 7 路相位打一拍
        mx8  <= np8;                         // 第 8 路相位打一拍
        mx9  <= np9;                         // 第 9 路相位打一拍
        mx10 <= np10;                        // 第 10 路相位打一拍
        mx11 <= np11;                        // 第 11 路相位打一拍
        mx12 <= np12;                        // 第 12 路相位打一拍
        mx13 <= np13;                        // 第 13 路相位打一拍
        mx14 <= np14;                        // 第 14 路相位打一拍
        mx15 <= np15;                        // 第 15 路相位打一拍

    end

end


wire [255:0] dds_pre;                        // LUT 后、扫描前的 16×int16
wire         dds_pre_v;                      // 与 dds_pre 对齐的有效


// =============================================================================
// 16 路相位 → 正弦 LUT → 256 bit DAC 总线
// =============================================================================

da_lfm_dds_path u_dds_path (

    .clk                         (clk),      // 工作时钟
    .rst_sync                    (dds_rst_sync), // 同步复位
    .stop_edge                   (stop_edge), // 停止沿：拉低 LUT aresetn 排空
    .cfg_load_edge               (cfg_load_edge), // 加载沿：同样排空 LUT
    .clear_phase_edge            (clear_phase_edge), // 清相沿：同样排空 LUT
    .phase_axis_valid            (mux_valid), // 打拍后的相位有效
    .phase_axis_user             (mux_user), // 打拍后的门控
    .phase_r_0                   (mx0),      // 第 0 路相位
    .phase_r_1                   (mx1),      // 第 1 路相位
    .phase_r_2                   (mx2),      // 第 2 路相位
    .phase_r_3                   (mx3),      // 第 3 路相位
    .phase_r_4                   (mx4),      // 第 4 路相位
    .phase_r_5                   (mx5),      // 第 5 路相位
    .phase_r_6                   (mx6),      // 第 6 路相位
    .phase_r_7                   (mx7),      // 第 7 路相位
    .phase_r_8                   (mx8),      // 第 8 路相位
    .phase_r_9                   (mx9),      // 第 9 路相位
    .phase_r_10                  (mx10),     // 第 10 路相位
    .phase_r_11                  (mx11),     // 第 11 路相位
    .phase_r_12                  (mx12),     // 第 12 路相位
    .phase_r_13                  (mx13),     // 第 13 路相位
    .phase_r_14                  (mx14),     // 第 14 路相位
    .phase_r_15                  (mx15),     // 第 15 路相位
    .dds_red_en_sync             (1'b0),     // 幅度衰减关，本通道不用
    .dds_signal_reduction_sync   (11'sd0),   // 衰减系数绑 0
    .dds_sin                     (dds_pre),  // LUT 后正弦
    .dds_valid                   (dds_pre_v) // LUT 后有效

);




//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
wire [9:0] scan_idx_nc;      // 扫描索引
wire [9:0] rel_idx_nc;       // 相对索引
wire [7:0] g_nc;             // 增益
wire beam_valid_nc;          // 波束表有效

//---------------------------------------------------------------------------
// CDC 目的域输出信号
//---------------------------------------------------------------------------
wire        beam_we_r;       // 目的域波束表写脉冲，单拍，来自 CDC
wire        beam_range_we_r; // 目的域波束范围写脉冲，单拍，来自 CDC
wire [9:0]  beam_wa_s2;      // 目的域波束表写地址，同步稳定后
wire [9:0]  beam_a0_s2;      // 目的域波束范围起点 a0，同步稳定后
wire [9:0]  beam_a1_s2;      // 目的域波束范围终点 a1，同步稳定后
wire [31:0] beam_wd_s2;      // 目的域波束表写数据，同步稳定后


// 功能：保持式 bundled-data 请求/应答跨时钟域传输。
//   Kind（range/write）与 payload 同拍捕获，1 拍后本地解码，
//   不作为直接异步脉冲使用。

//=============================================================================
da_sig_beam_cdc u_beam_cdc(
    .src_clk            (cfg_clk),              // 源时钟域时钟
    .src_rst            (cfg_rst_sync[1]),      // 源域复位（同步后）
    .clk                (clk),                  // 目的时钟域时钟
    .rst_sync           (dds_rst_sync),         // 目的域同步复位

    .write_tgl          (sig_beam_we_tgl),      // 波束写请求 toggle
    .range_tgl          (sig_beam_range_tgl),   // 波束范围请求 toggle
    .waddr              (sig_beam_waddr),       // 波束表写地址
    .wdata              (sig_beam_wdata),       // 波束表写数据
    .a0                 (sig_beam_a0),          // 波束范围起点
    .a1                 (sig_beam_a1),          // 波束范围终点

    .destination_ready  (!beam_busy),           // 目的域就绪：波束不忙
    .source_ready       (beam_source_ready),    // 源域可发起新传输

    .write_pulse        (beam_we_r),            // 目的域写脉冲，单拍
    .range_pulse        (beam_range_we_r),      // 目的域范围脉冲，单拍

    .dst_waddr          (beam_wa_s2),           // 目的域写地址
    .dst_wdata          (beam_wd_s2),           // 目的域写数据
    .dst_a0             (beam_a0_s2),           // 目的域范围起点
    .dst_a1             (beam_a1_s2)            // 目的域范围终点
);

// =============================================================================
// 天线扫描：只乘幅度，不改频率/PRI
//   cfg_load 锁存类型/T/起点/跨度/目标，并重算 scan_FTW
//   start_edge 复位 θ、sec_pos，圆扫 0，扇扫回到 start
//   圆扫 addr=θ[47:38]；扇扫 start+sec_pos，10 bit 溢出跨 0°
//   G = beam_tab[scan_idx-target]；y=(x*G)>>>8，关扫描直通
// =============================================================================

da_sig_scan u_scan (

    .config_busy(scan_busy),.beam_busy(beam_busy),
    .clk             (clk),                  // 本域 300 MHz
    .rst_sync        (dds_rst_sync),         // 同步复位；BRAM 内容不复位
    .start_edge      (start_n),              // 启动：θ/sec_pos 回 0，保留 FTW
    .cfg_load        (cfg_applied),        // 加载：锁存配置并启动 2^32/T
    .tick            (tick_n),               // 1 tick=3 clk，扫描角只在此推进
    .scan_en         (sen),                  // 0x61[7]，关则增益直通
    .scan_type       (sty),                  // 0 关 / 1 圆扫 / 2 扇扫
    .scan_start_idx  (sed[9:0]),             // 扇扫起点；例 896 = −45°
    .scan_span_idx   (sed[19:10]),           // 扇扫跨度；例 256 = 90°
    .target_idx      (sed[29:20]),           // 目标方位；例 57 ≈ 20°
    .t_scan          (stm),
    .beam_range_we   (beam_range_we_r),
    .beam_a0         (beam_a0_s2),
    .beam_a1         (beam_a1_s2),
    .beam_tab_we     (beam_we_r),
    .beam_tab_waddr  (beam_wa_s2),
    .beam_tab_wdata  (beam_wd_s2),
    .din             (dds_pre),              // LUT 后 16×int16 正弦
    .din_valid       (dds_pre_v),            // 与 din 对齐的有效
    .dout            (dds_sin),              // 乘 G 后的 16 路样点，送 DAC
    .dout_valid      (dds_valid),            // 与 dout 对齐的有效
    .scan_idx        (scan_idx_nc),          // 观察：当前扫描地址，本层不用
    .relative_idx    (rel_idx_nc),           // 观察：相对目标角，本层不用
    .g               (g_nc),                 // 观察：当前 G，本层不用
    .beam_valid      (beam_valid_nc)         // 观察：表已加载，本层不用

);

endmodule
