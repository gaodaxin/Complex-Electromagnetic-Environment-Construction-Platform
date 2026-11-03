// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_host_regs.v
// 功能：保存八通道协议影子寄存器、PRI/频率/相位表和方向图写参数，按通道输出扁平参数总线。
// ============================================================================
`timescale 1ns / 1ps
// =============================================================================
// 模块: da_sig_host_regs
// 功能: 8 路新协议参数影子。只存、不计算。receive_cmd 在帧尾 s10 拉 commit，
//       按 CMD 写入通道 ch。下游 DA_LFM_OUT 再算周期 / 频率 / 幅度。
// 关联:
//   上游 receive_cmd_wls（解码 7E 7E 帧 → commit/cmd/ch/payload）
//   下游 Rfsoc_Demo_Top → top_8ch_dds 按通道切开 → 8×DA_LFM_OUT
// 数据流:
//   帧 → commit=1 → case(cmd) 写 r_*[ch]
//     → generate 把 r_* 拼成 sig_* 宽总线（组合，寄存器只在 r_*）
//   0x50/0x5A 清 use_new，旧 0x50–0x60 脚本仍走旧链。
//
// 单位（全文件统一）:
//   时间  协议 1 格 = 10 ns = 1 tick；FPGA 1 tick = 3 clk
//   频率  上位机已换成 32-bit FTW：FTW = floor(f_Hz * 2^32 / 4.8e9)
//
//   时间轴（PRI/PW）和频率轴（捷变）分开存，没有「第 j 个 PRI 对应第 j 个频率」。
//
// 出波真正用到: use_new/type/freq_type/cw、PW、PRI 表与类型、fc/hop/lfm、
//               调相、扫描配置、方向图逐点写、32 点频表、32 点相位表、抖动、滑变。
//
// 写口契约:
//   commit=1 的上升沿只执行一条 cmd；cmd/ch/payload 必须在该拍稳定。payload 的
//   有效数据统一右对齐，短帧和长帧解码器已经把线上大端字节重组为整数。
//   本模块没有 ready/busy，因为每个 case 分支都能单拍完成；长表由上游拆成
//   多个 commit。ch 必须是 0..7，上游已完成合法性校验。
//
// 影子与生效:
//   r_* 是 100 MHz 命令域影子，commit 后立即更新 sig_* 组合输出；这不等于
//   DDS 立即采用参数。receive_cmd_wls 另发 cfg_load toggle，下游跨域后才把整组
//   稳定参数装入运行寄存器。表写例外：0x79/0x7B 使用 toggle 逐笔跨域写 RAM。
//
// 地址语义:
//   0x6D 选择 PRI 表地址，下一条 0x6E 写值；0x76/0x77 对频率表同理。
//   0x7B 设置方向图相对区间并把 compact 写指针清零；每条 0x79 打包 4 个 G，
//   对外 waddr 是 compact 偏移 0,4,8...，绝对角地址由 da_sig_scan_beam 重建。
// =============================================================================
module da_sig_host_regs (
    input  wire        clk,                  // 时钟：命令域，约 100 MHz
    input  wire        rst,                  // 复位：清全部影子
    input  wire        commit,               // 写使能：帧校验通过后 1 拍
    input  wire [7:0]  cmd,                  // 命令字：0x50–0x80
    input  wire [2:0]  ch,                   // 通道号：0..7
    input  wire [63:0] payload,              // 帧 DATA，小端位域；多字节与脚本 BE 装填一致

    output wire [7:0]   sig_use_new,         // 新链开关 use_new，bit[ch]
    output wire [15:0]  sig_type,            // 信号类型 sig_type，每路 2 bit
    output wire [15:0]  sig_freq_type,       // 捷变类型 freq_type，每路 2 bit
    output wire [7:0]   sig_cw_en,           // 连续波 cw_en，1=门恒开
    output wire [7:0]   sig_lfm_neg,         // LFM 负扫 lfm_neg
    output wire [159:0] sig_pw_ticks,        // 脉宽 PW（tick），每路 20 bit
    output wire [159:0] sig_pri_ticks,       // 默认 PRI（tick），tab[0]=0 时顶上
    output wire [383:0] sig_fc_hz,           // 中心频率，每路 48 bit，低 32=FTW
    output wire [255:0] sig_hop_bw_hz,       // 捷变带宽 B（FTW）
    output wire [47:0]  sig_hop_n,           // 捷变频点数 N，1..32
    output wire [127:0] sig_hop_ticks,       // 脉内 hop 间隔（tick）
    output wire [255:0] sig_lfm_bw_hz,       // 调频带宽 B（FTW）
    output wire [159:0] sig_lfm_ticks,       // 调频时长 N_lfm（tick）
    output wire [15:0]  sig_pm_type,         // FM 子类型或 PM 子类型
    output wire [15:0]  sig_nlfm_type,       // NLFM 形状
    output wire [127:0] sig_nc,              // 码元宽度 Nc（tick）
    output wire [127:0] sig_ncode,           // 码长 Ncode
    output wire [15:0]  sig_pri_type,        // PRI 变化类型
    output wire [47:0]  sig_pri_m,           // 参差项数 M
    output wire [47:0]  sig_pri_grp,         // 脉组参差：每组脉冲个数
    output wire [47:0]  sig_hop_grp,         // 脉组捷变：每组脉冲个数
    output wire [5119:0] sig_pri_tab,        // 32 组 PRI 表，每项 20 bit tick
    output wire [7:0]   sig_scan_en,         // 天线扫描使能
    output wire [15:0]  sig_scan_type,       // 扫描类型
    output wire [255:0] sig_scan_end,        // [9:0] start [19:10] span [29:20] target
    output wire [255:0] sig_scan_t,          // 扫描周期 T（tick）
    output wire [7:0]   sig_beam_we_tgl,     // 方向图写翻转，每笔 0x79 翻一次
    output wire [79:0]  sig_beam_waddr,      // 8×10 compact 写偏移
    output wire [255:0] sig_beam_wdata,      // 8×32 {G0,G1,G2,G3}
    output wire [7:0]   sig_beam_range_tgl,  // 0x7B 区间翻转
    output wire [79:0]  sig_beam_a0,         // 8×10 非零区起始
    output wire [79:0]  sig_beam_a1,         // 8×10 非零区结束
    output wire [7:0]    sig_hop_rand,       // 捷变随机：0 顺序 / 1 随机抽点
    output wire [7:0]    sig_freq_tab_en,    // 1=使用 0x76/77 频表，0=等差栅
    output wire [8191:0] sig_freq_tab,       // 8×32×32 FTW 频表
    output wire [7:0]    sig_phase_tab_en,   // 1=使用 0x78 相位表
    output wire [8191:0] sig_phase_tab,      // 8×32×32 相位字，与 hop 对齐
    output wire [159:0]  sig_jit_p0,         // 抖动中心 P0（tick）
    output wire [159:0]  sig_jit_n,          // 抖动周期 N_jit（墙钟 tick）
    output wire [55:0]   sig_jit_rho,        // 抖动深度 ρ，0..50
    output wire [7:0]    sig_jit_mode,       // 0 正弦 / 1 伪随机
    output wire [159:0]  sig_slide_min,      // 滑变 Pmin
    output wire [159:0]  sig_slide_max,      // 滑变 Pmax
    output wire [159:0]  sig_slide_stp,      // 滑变步长 δ
    output wire [7:0]    sig_slide_md,       // 0 固定步长 / 1 随机步长
    output wire [7:0]    sig_slide_en        // 1=允许滑变；关模式必须为 0
);
    integer si, pj;

    // =========================================================================
    // 1. 每通道影子寄存器
    // =========================================================================
    reg        r_use_new   [0:7];            // use_new：1=新链 0x61，0=旧 0x50/0x5A
    reg [1:0]  r_sig_type  [0:7];            // sig_type：0 常规脉冲 CW / 1 调频 / 2 调相
    reg [1:0]  r_freq_type [0:7];            // freq_type：0 单载频 / 1 脉内捷变 / 2 脉间 / 3 脉组
    reg [1:0]  r_pri_type  [0:7];            // pri_type：0 固定 / 1 脉间参差 / 2 脉组参差 / 3 抖动
    reg        r_cw        [0:7];            // cw_en：1=连续波，pulse_gate 恒 1
    reg        r_scan_en   [0:7];            // scan_en：天线扫描开
    reg [1:0]  r_fm_pm     [0:7];            // fm_type 或 pm_type：FM 时 0 LFM/1 NLFM；PM 时 0 二相/1 四相/2 P3/3 P4
    reg        r_lfm_neg   [0:7];            // lfm_neg：1=负扫，S 与 fm_start 取反
    reg [1:0]  r_nlfm      [0:7];            // nlfm_type：0 三角 / 1 正弦 / 2 锯齿
    reg [1:0]  r_phase_md  [0:7];            // phase_mode：脉外用哪路 F（只存）
    reg        r_hop_rand  [0:7];            // hop_rand：随机频点捷变
    reg [1:0]  r_lfm_pwm   [0:7];            // lfm_pw_mode：LFM 周期与 PW 关系（只存）
    reg        r_stag      [0:7];            // stag：旧脉组标志（只存；现用 pri_type）
    reg        r_slide_md  [0:7];            // slide_step_mode：0 固定步长 / 1 随机步长
    reg        r_slide_en  [0:7];            // 滑变总开关，0x7C[4]
    reg [19:0] r_pw        [0:7];            // PW：脉宽，门控 cnt<PW，tick
    reg [19:0] r_pri       [0:7];            // PRI_default：最近一次 0x6E，默认周期 tick
    reg [31:0] r_fc        [0:7];            // fc：中心频率 FTW，捷变/LFM 的 f_c
    reg [31:0] r_hop_bw    [0:7];            // B_hop：捷变带宽 FTW；hop_start=fc-B/2，step=B/(N-1)
    reg [5:0]  r_hop_n     [0:7];            // N_hop：捷变频点数 1..32
    reg [15:0] r_hop_t     [0:7];            // hop_ticks：脉内每点持续 tick（跳速）
    reg [5:0]  r_hop_grp   [0:7];            // hop_group_size：脉组捷变，每组脉冲个数
    reg [5:0]  r_pri_grp   [0:7];            // pri_group_size：脉组参差，每组脉冲个数
    reg [31:0] r_lfm_bw    [0:7];            // B_lfm：调频带宽 FTW；S=±B/N_sample
    reg [19:0] r_lfm_t     [0:7];            // N_lfm：调频时长 tick；N_sample=48*N_lfm
    reg [15:0] r_nc        [0:7];            // Nc：一个码元宽度 tick
    reg [15:0] r_ncode     [0:7];            // Ncode：码长；P3/P4 再靠档 16/32/64/128/256
    reg [4:0]  r_pri_idx   [0:7];            // pri_idx：下一笔 0x6E 写入 tab 的下标 0..31
    reg [19:0] r_pri_tab   [0:7][0:31];      // pri_tab[j]：第 j 项 PRI（tick），参差重频 32 个
    reg [5:0]  r_pri_m     [0:7];            // M：参差用几项 1..32
    reg [19:0] r_jit_p0    [0:7];            // P0：抖动中心 PRI（tick）
    reg [19:0] r_jit_n     [0:7];            // N_jit：抖动调制周期（墙钟 tick）
    reg [6:0]  r_jit_rho   [0:7];            // rho：抖动深度百分数 0..50 → ±50%
    reg        r_jit_mode  [0:7];            // jit_mode：0 正弦 / 1 伪随机
    reg [1:0]  r_slide_sel [0:7];            // slide_sel：上次 0x72 写的是 min/max/step
    reg [19:0] r_slide_min [0:7];            // Pmin：滑变下限 tick
    reg [19:0] r_slide_max [0:7];            // Pmax：滑变上限 tick
    reg [19:0] r_slide_stp [0:7];            // delta：滑变步长 tick
    reg [4:0]  r_freq_idx  [0:7];            // 下一笔 0x77 写入 freq_tab 的下标
    reg [31:0] r_freq_tab  [0:7][0:31];      // freq_tab[j]：第 j 个捷变频点 FTW
    reg        r_freq_tab_en [0:7];          // 写过 0x77 后为 1
    reg [31:0] r_phase_tab [0:7][0:31];      // phase_tab[j]：与 hop 对齐的附加相位
    reg        r_phase_tab_en [0:7];         // 写过合法 0x78 后为 1
    reg [7:0]  r_scan_ty   [0:7];            // scan_type：0 关 / 1 圆扫 / 2 扇扫（用低 2 bit）
    reg [31:0] r_scan_end  [0:7];            // [9:0] start [19:10] span [29:20] target
    reg [31:0] r_scan_t    [0:7];            // T_scan：扫一周的周期 tick
    reg [15:0] r_code_addr [0:7];            // code_addr：码 RAM 地址（只存）
    reg [47:0] r_code_word [0:7];            // code_word：码 RAM 数据（只存）
    reg [9:0]  r_scan_idx  [0:7];            // 最近一笔 compact 写偏移
    reg [7:0]  r_scan_val  [0:7];            // 最近一笔 0x79 的 G0（观察）
    reg        r_beam_tgl  [0:7];            // 每笔 0x79 翻转
    reg [31:0] r_beam_pack [0:7];            // 0x79 四字节 G
    reg [7:0]  r_beam_off  [0:7];            // compact 写指针
    reg [9:0]  r_beam_a0   [0:7];            // 0x7B 起始相对地址
    reg [9:0]  r_beam_a1   [0:7];            // 0x7B 结束相对地址
    reg        r_beam_range_tgl [0:7];       // 每笔 0x7B 翻转
    reg [7:0]  r_bank_cmd  [0:7];            // bank：双 bank 命令（只存）

    // =========================================================================
    // 2. 二维寄存器数组到扁平端口总线的纯组合打包
    // =========================================================================
    // r_*[ch] → sig_*；top_8ch_dds 使用 bus[ch*W +: W] 切出每路字段。
    // 这里没有新增寄存器级，因此所有时序边界仍在上面的 r_*。
    genvar gch;
    generate
        for (gch = 0; gch < 8; gch = gch + 1) begin : g_pack
            assign sig_use_new[gch]                    = r_use_new[gch];
            assign sig_type[gch*2+1:gch*2]             = r_sig_type[gch];
            assign sig_freq_type[gch*2+1:gch*2]        = r_freq_type[gch];
            assign sig_cw_en[gch]                      = r_cw[gch];
            assign sig_lfm_neg[gch]                    = r_lfm_neg[gch];
            assign sig_pw_ticks[gch*20+19:gch*20]      = r_pw[gch];
            assign sig_pri_ticks[gch*20+19:gch*20]     = r_pri[gch];
            assign sig_fc_hz[gch*48+47:gch*48]         = {16'd0, r_fc[gch]};
            assign sig_hop_bw_hz[gch*32+31:gch*32]     = r_hop_bw[gch];
            assign sig_hop_n[gch*6+5:gch*6]            = r_hop_n[gch];
            assign sig_pri_type[gch*2+1:gch*2]         = r_pri_type[gch];
            assign sig_pri_m[gch*6+5:gch*6]            = r_pri_m[gch];
            assign sig_pri_grp[gch*6+5:gch*6]          = r_pri_grp[gch];
            assign sig_hop_grp[gch*6+5:gch*6]          = r_hop_grp[gch];
            assign sig_scan_en[gch]                    = r_scan_en[gch];
            assign sig_scan_type[gch*2+1:gch*2]        = r_scan_ty[gch][1:0];
            assign sig_scan_end[gch*32+31:gch*32]      = r_scan_end[gch];
            assign sig_scan_t[gch*32+31:gch*32]        = r_scan_t[gch];
            assign sig_beam_we_tgl[gch]                = r_beam_tgl[gch];
            assign sig_beam_waddr[gch*10+9:gch*10]     = r_scan_idx[gch];
            assign sig_beam_wdata[gch*32+31:gch*32]    = r_beam_pack[gch];
            assign sig_beam_range_tgl[gch]             = r_beam_range_tgl[gch];
            assign sig_beam_a0[gch*10+9:gch*10]        = r_beam_a0[gch];
            assign sig_beam_a1[gch*10+9:gch*10]        = r_beam_a1[gch];
            assign sig_hop_ticks[gch*16+15:gch*16]     = r_hop_t[gch];
            assign sig_lfm_bw_hz[gch*32+31:gch*32]     = r_lfm_bw[gch];
            assign sig_lfm_ticks[gch*20+19:gch*20]     = r_lfm_t[gch];
            assign sig_pm_type[gch*2+1:gch*2]          = r_fm_pm[gch];
            assign sig_nlfm_type[gch*2+1:gch*2]        = r_nlfm[gch];
            assign sig_nc[gch*16+15:gch*16]            = r_nc[gch];
            assign sig_ncode[gch*16+15:gch*16]         = r_ncode[gch];
            assign sig_hop_rand[gch]                   = r_hop_rand[gch];
            assign sig_freq_tab_en[gch]                = r_freq_tab_en[gch];
            assign sig_phase_tab_en[gch]               = r_phase_tab_en[gch];
            assign sig_jit_p0[gch*20+19:gch*20]        = r_jit_p0[gch];
            assign sig_jit_n[gch*20+19:gch*20]         = r_jit_n[gch];
            assign sig_jit_rho[gch*7+6:gch*7]          = r_jit_rho[gch];
            assign sig_jit_mode[gch]                   = r_jit_mode[gch];
            assign sig_slide_min[gch*20+19:gch*20]     = r_slide_min[gch];
            assign sig_slide_max[gch*20+19:gch*20]     = r_slide_max[gch];
            assign sig_slide_stp[gch*20+19:gch*20]     = r_slide_stp[gch];
            assign sig_slide_md[gch]                   = r_slide_md[gch];
            assign sig_slide_en[gch]                   = r_slide_en[gch];
        end
    endgenerate

    // PRI/频率/相位表按“通道优先、表下标次之”展平：
    // flat[(ch*32+idx)*W +: W] = table[ch][idx]。
    genvar gpi, gfi;
    generate
        for (gch = 0; gch < 8; gch = gch + 1) begin : g_tab
            for (gpi = 0; gpi < 32; gpi = gpi + 1) begin : g_pri
                // sig_pri_tab 通道 gch、第 gpi 项：20 bit 周期
                assign sig_pri_tab[(gch*32+gpi)*20+19:(gch*32+gpi)*20] = r_pri_tab[gch][gpi];
            end
            for (gfi = 0; gfi < 32; gfi = gfi + 1) begin : g_freq
                assign sig_freq_tab[(gch*32+gfi)*32+31:(gch*32+gfi)*32] = r_freq_tab[gch][gfi];
                assign sig_phase_tab[(gch*32+gfi)*32+31:(gch*32+gfi)*32] = r_phase_tab[gch][gfi];
            end
        end
    endgenerate

    // =========================================================================
    // 3. 复位默认值与单拍命令写入
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            // 默认保持新链关闭、单载频/固定 PRI/不扫描。表内容清零，但长度类
            // 字段给合法最小值，避免释放复位后出现除零或非法数组下标。
            for (si = 0; si < 8; si = si + 1) begin
                r_use_new[si]   <= 1'b0;
                r_sig_type[si]  <= 2'd0;
                r_freq_type[si] <= 2'd0;
                r_pri_type[si]  <= 2'd0;
                r_cw[si]        <= 1'b0;
                r_scan_en[si]   <= 1'b0;
                r_fm_pm[si]     <= 2'd0;
                r_lfm_neg[si]   <= 1'b0;
                r_nlfm[si]      <= 2'd0;
                r_phase_md[si]  <= 2'd0;
                r_hop_rand[si]  <= 1'b0;
                r_lfm_pwm[si]   <= 2'd0;
                r_stag[si]      <= 1'b0;
                r_slide_md[si]  <= 1'b0;
                r_slide_en[si]  <= 1'b0;
                r_pw[si]        <= 20'd100;
                r_pri[si]       <= 20'd1000;
                r_fc[si]        <= 32'd0;
                r_hop_bw[si]    <= 32'd0;
                r_hop_n[si]     <= 6'd1;
                r_hop_t[si]     <= 16'd1;
                r_hop_grp[si]   <= 6'd1;
                r_pri_grp[si]   <= 6'd1;
                r_lfm_bw[si]    <= 32'd0;
                r_lfm_t[si]     <= 20'd0;
                r_nc[si]        <= 16'd1;
                r_ncode[si]     <= 16'd2;
                r_pri_idx[si]   <= 5'd0;
                r_pri_m[si]     <= 6'd1;
                r_jit_p0[si]    <= 20'd1000;
                r_jit_n[si]     <= 20'd0;
                r_jit_rho[si]   <= 7'd0;
                r_jit_mode[si]  <= 1'b0;
                r_slide_sel[si] <= 2'd0;
                r_slide_min[si] <= 20'd0;
                r_slide_max[si] <= 20'd0;
                r_slide_stp[si] <= 20'd0;
                r_freq_idx[si]  <= 5'd0;
                r_freq_tab_en[si]  <= 1'b0;
                r_phase_tab_en[si] <= 1'b0;
                r_scan_ty[si]   <= 8'd0;
                r_scan_end[si]  <= 32'd0;
                r_scan_t[si]    <= 32'd0;
                r_code_addr[si] <= 16'd0;
                r_code_word[si] <= 48'd0;
                r_scan_idx[si]  <= 10'd0;
                r_scan_val[si]  <= 8'd0;
                r_beam_tgl[si]  <= 1'b0;
                r_beam_pack[si] <= 32'd0;
                r_beam_off[si]  <= 8'd0;
                r_beam_a0[si]   <= 10'd0;
                r_beam_a1[si]   <= 10'd0;
                r_beam_range_tgl[si] <= 1'b0;
                r_bank_cmd[si]  <= 8'd0;
                for (pj = 0; pj < 32; pj = pj + 1) begin
                    r_pri_tab[si][pj]   <= 20'd0;
                    r_freq_tab[si][pj]  <= 32'd0;
                    r_phase_tab[si][pj] <= 32'd0;
                end
            end
        end else if (commit) begin
            // 一拍只写一个 CMD；未命中的寄存器保持。payload 位域见各分支。
            case (cmd)
                // 0x50/0x5A 旧链模式：关掉新链
                8'h50, 8'h5A: r_use_new[ch] <= 1'b0;

                // 0x61 工作模式字（1 字节）
                // [1:0] sig_type  信号类型
                // [3:2] freq_type 捷变：0 单载频 / 1 脉内 / 2 脉间 / 3 脉组
                // [5:4] pri_type  PRI：0 固定 / 1 脉间参差 / 2 脉组参差 / 3 抖动
                // [6]   cw_en     连续波
                // [7]   scan_en   扫描
                8'h61: begin
                    r_use_new[ch]   <= 1'b1;
                    r_sig_type[ch]  <= payload[1:0];
                    r_freq_type[ch] <= payload[3:2];
                    r_pri_type[ch]  <= payload[5:4];
                    r_cw[ch]        <= payload[6];
                    r_scan_en[ch]   <= payload[7];
                    // 回到单载频时清掉粘滞的查表使能，否则 start 仍读 tab[0] 而不是 0x64 的 fc
                    if (payload[3:2] == 2'd0) begin
                        r_freq_tab_en[ch]  <= 1'b0;
                        r_phase_tab_en[ch] <= 1'b0;
                    end
                end

                // 0x62 调制子类型（1 字节）
                // [1:0] fm_type 或 pm_type
                // [2]   lfm_neg
                // [4:3] nlfm_type
                // [6:5] phase_mode（只存）
                // [7]   hop_rand 随机捷变
                8'h62: begin
                    r_fm_pm[ch]    <= payload[1:0];
                    r_lfm_neg[ch]  <= payload[2];
                    r_nlfm[ch]     <= payload[4:3];
                    r_phase_md[ch] <= payload[6:5];
                    r_hop_rand[ch] <= payload[7];
                end

                // 0x7C 附加模式（1 字节）
                // [1:0] lfm_pw_mode  [2] stag  [3] slide_step_mode  [4] slide_en
                8'h7C: begin
                    r_lfm_pwm[ch]  <= payload[1:0];
                    r_stag[ch]     <= payload[2];
                    r_slide_md[ch] <= payload[3];
                    r_slide_en[ch] <= payload[4];
                end

                // 0x63 脉宽 PW（tick）。高电平时长，不是 PRI。每通道 1 个。
                8'h63: r_pw[ch]      <= payload[19:0];

                // 0x64 中心频率 fc。帧 6 字节，这里用低 32 bit FTW。
                8'h64: r_fc[ch]      <= payload[31:0];

                // 0x65 捷变带宽 B_hop（FTW）。栅格 f_j = fc + (2j-(N-1))B/(2(N-1))
                8'h65: r_hop_bw[ch]  <= payload[31:0];

                // 0x66 捷变频点数 N_hop，钳位 1..32
                8'h66: r_hop_n[ch]   <= (payload[7:0] == 8'd0) ? 6'd1 :
                                        (payload[7:0] > 8'd32) ? 6'd32 : payload[5:0];

                // 0x67 脉内跳速 hop_ticks：每个频点持续多少 tick
                8'h67: r_hop_t[ch]   <= (payload[15:0] == 16'd0) ? 16'd1 : payload[15:0];

                // 0x68 脉组捷变 hop_group_size：每组多少个脉冲换一个频点
                8'h68: r_hop_grp[ch] <= (payload[7:0] == 8'd0) ? 6'd1 :
                                        (payload[7:0] > 8'd32) ? 6'd32 : payload[5:0];

                // 0x7D 脉组参差 pri_group_size：每组多少个脉冲换一项 PRI（时间轴，不是频率）
                8'h7D: r_pri_grp[ch] <= (payload[7:0] == 8'd0) ? 6'd1 :
                                        (payload[7:0] > 8'd32) ? 6'd32 : payload[5:0];

                // 0x69 调频带宽 B_lfm（FTW）。S = ±B / (48*N_lfm)
                8'h69: r_lfm_bw[ch]  <= payload[31:0];

                // 0x6A 调频时长 N_lfm（tick）
                8'h6A: r_lfm_t[ch]   <= payload[19:0];

                // 0x6B 码元宽度 Nc（tick）
                8'h6B: r_nc[ch]      <= payload[15:0];

                // 0x6C 码长 Ncode
                8'h6C: r_ncode[ch]   <= payload[15:0];

                // 0x6D PRI 表下标 pri_idx，0..31。下一帧 0x6E 写 tab[idx]
                8'h6D: r_pri_idx[ch] <= payload[4:0];

                // 0x6E PRI 表值（tick）→ tab[pri_idx]，并更新默认 PRI
                8'h6E: begin
                    r_pri_tab[ch][r_pri_idx[ch]] <= payload[19:0];
                    r_pri[ch]                    <= payload[19:0];
                end

                // 0x6F 参差项数 M：用 tab[0..M-1]
                8'h6F: r_pri_m[ch]    <= (payload[7:0] == 8'd0) ? 6'd1 :
                                         (payload[7:0] > 8'd32) ? 6'd32 : payload[5:0];

                // 0x70 抖动中心 P0（tick）。P = P0*(1+ρ*m/100)
                8'h70: r_jit_p0[ch]   <= payload[19:0];

                // 0x7F 抖动周期 N_jit（墙钟 tick）
                8'h7F: r_jit_n[ch]    <= payload[19:0];

                // 0x71 [6:0] ρ 百分数 0..50，[7] 0 正弦 / 1 伪随机
                8'h71: begin
                    r_jit_rho[ch]  <= (payload[6:0] > 7'd50) ? 7'd50 : payload[6:0];
                    r_jit_mode[ch] <= payload[7];
                end

                // 0x72 滑变：payload[33:32]=0 Pmin / 1 Pmax / 2 δ，低 20 bit 为 tick
                8'h72: begin
                    r_slide_sel[ch] <= payload[33:32];
                    case (payload[33:32])
                        2'd0: r_slide_min[ch] <= payload[19:0];
                        2'd1: r_slide_max[ch] <= payload[19:0];
                        default: r_slide_stp[ch] <= payload[19:0];
                    endcase
                end

                // 0x80 滑变步长模式（与 0x7C[3] 同类）
                8'h80: r_slide_md[ch] <= payload[0];

                // 0x73 扫描类型 scan_type
                8'h73: r_scan_ty[ch]  <= payload[7:0];

                // 0x74 [9:0] start [19:10] span [29:20] target
                8'h74: r_scan_end[ch] <= payload[31:0];

                // 0x75 扫描周期 T_scan（tick），scan_FTW=(2^32/T)<<16
                8'h75: r_scan_t[ch]   <= payload[31:0];

                // 0x76 频表下标 0..31。下一帧 0x77 写 freq_tab[idx]
                8'h76: r_freq_idx[ch] <= payload[4:0];

                // 0x77 频表 FTW（6 字节，低 32 bit，语义同 0x64）
                8'h77: begin
                    r_freq_tab[ch][r_freq_idx[ch]] <= payload[31:0];
                    r_freq_tab_en[ch]              <= 1'b1;
                end

                // 0x78 相位表：2B 地址 + 6B 字。addr 0..31 写入 phase_tab，低 32 bit 为相位字
                8'h78: begin
                    r_code_addr[ch] <= payload[63:48];
                    r_code_word[ch] <= payload[47:0];
                    if (payload[63:48] < 16'd32) begin
                        r_phase_tab[ch][payload[52:48]] <= payload[31:0];
                        r_phase_tab_en[ch]              <= 1'b1;
                    end
                end

                // 0x79 4B：紧凑幅度 {G0,G1,G2,G3}，从 r_beam_off 起写入 compact
                8'h79: begin
                    r_beam_pack[ch] <= payload[31:0];
                    r_scan_idx[ch]  <= {2'b00, r_beam_off[ch]};
                    r_scan_val[ch]  <= payload[31:24];
                    r_beam_tgl[ch]  <= ~r_beam_tgl[ch];
                    r_beam_off[ch]  <= r_beam_off[ch] + 8'd4;
                end

                // 0x7B 4B：非零区 [9:0]=a0 [25:16]=a1，清空 compact 写指针
                8'h7B: begin
                    r_beam_a0[ch]        <= payload[9:0];
                    r_beam_a1[ch]        <= payload[25:16];
                    r_beam_off[ch]       <= 8'd0;
                    r_beam_range_tgl[ch] <= ~r_beam_range_tgl[ch];
                end

                // 0x7A 双 bank（只存）
                8'h7A: r_bank_cmd[ch] <= payload[7:0];
                default: ;
            endcase
        end
    end
endmodule
