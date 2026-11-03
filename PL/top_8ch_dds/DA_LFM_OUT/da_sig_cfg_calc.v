// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_cfg_calc.v
// 功能：加载后检查配置并计算跳频步长、调频斜率、样点数和 NLFM 相位步长；除法只位于配置路径。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_cfg_calc
// 功能: cfg_load 后算运行时常数。上位机已给 FTW，这里只做整数 BW/N，不再除 Fs。
// 关联:
//   上游 DA_LFM_OUT     cfg_load_edge → start_calc，标量来自 host_regs 两级打拍
//   同级 da_sig_div_u   每商 bit 3 拍，约 96 clk
//   下游 da_sig_wave / da_sig_carrier / da_sig_accum
//        hop_start/hop_step、slope_word、n_sample、fm_start、phase_step
// 数据流（状态机）:
//   host 0x61..0x6A 标量
//     → S_IDLE  锁存 n_use、n_sample=48*lfm_ticks
//     → S_CHK   合法性；无 hop 则 hop_start=fc、hop_step=0
//     → S_HDIV  启动 hop_bw/(N-1)
//     → S_HWAIT 商 → hop_step；fc−B/2 → hop_start
//     → S_SLOPE 非 FM 则斜率/phase_step=0 直接 DONE
//     → S_SDIV  商 → ±slope；±B/2 → fm_start；再启动 2^32/N
//     → S_PSTEP 商 → phase_step（正弦 NLFM）
//     → S_DONE  done=1、busy=0，DA_LFM_OUT 才允许 start
// =============================================================================
module da_sig_cfg_calc (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst,              // 同步复位，高有效
    input  wire        start_calc,       // cfg_load 后启动计算，保持 1 个 clk

    input  wire [1:0]  sig_type,         // 信号类型：0 CW / 1 FM / 2 PM
    input  wire [1:0]  freq_type,        // 0 单载频 / 1 脉内 / 2 脉间 / 3 脉组
    input  wire        lfm_neg,          // LFM 负扫：1 则 S、fm_start 取反

    input  wire [31:0] fc_ftw,           // 中心频率 FTW，上位机已换算
    input  wire [31:0] hop_bw_ftw,       // 捷变带宽 FTW，整数域的 B
    input  wire [5:0]  hop_n,            // 捷变频点数 N，限制为 1~32
    input  wire [31:0] lfm_bw_ftw,       // 调频带宽 FTW，整数域的 B
    input  wire [19:0] lfm_ticks,        // 调频时长，单位 tick

    output reg         busy,             // 计算中为 1，挡住 DA start
    output reg         done,             // 完成脉冲，保持 1 个 clk
    output reg  [7:0]  err,              // 错误码，0 表示可以 start
    output reg  [31:0] hop_start,        // 捷变起点 FTW = fc − B/2
    output reg  [31:0] hop_step,         // 相邻 hop 间隔 = B/(N-1)
    output reg signed [31:0] slope_word, // 每样点斜率 S = ±B/N_sample
    output reg  [31:0] n_sample,         // 一次扫频总样点数 = 48×lfm_ticks
    output reg signed [31:0] fm_start,   // 调制起点频偏 ±B/2
    output wire signed [63:0] lfm_slope_q32,
    output wire signed [63:0] lfm_start_q32,
    output reg  [31:0] phase_step        // 正弦 NLFM：2^32/N_sample
);


// BEGIN INLINE HEADER: da_sig_common.vh (module内局部作用域)
// =============================================================================
// 文件: da_sig_common.vh
// 功能: 新链时钟/位宽/协议枚举和 Hz→FTW 对照函数。
// 用法: 必须在 module 内部 `include（Verilog 函数不能放在 module 外）。
// 约定:
//   目标板：Fs=4.8 GSPS，fabric=300 MHz，16 lane
//   协议 1 格 = 10 ns；FPGA 1 tick = 3 clk = 10 ns
//   600 MHz 只用于 ADC 8 样点并位，DSP 不放在 600 MHz
//   频率范围 0 ~ Fs。上位机把 Hz 换成 32-bit FTW 再下发：
//     FTW = floor(f_Hz * 2^32 / Fs)，不减 3.6 GHz
//   FPGA 只在 FTW 域做 hop 步长 / LFM 斜率的整数除法，不再除 Fs
// 关联:
//   da_sig_tick        TICK_CLKS
//   da_sig_cfg_calc    ERR_*、PW_MAX_TICK、FTW / Fs 常数
//   da_sig_wave        SIG_* / FM_* / PM_* / NLFM_*
//   da_sig_pcm_ctrl    PM_P3 / PM_P4
// =============================================================================

// -----------------------------------------------------------------------------
// 时钟网格：1 tick = TICK_CLKS 个 fabric clk = LANE_N*TICK_CLKS 个 DAC 样点
// -----------------------------------------------------------------------------
localparam integer TICK_CLKS     = 3;            // 1 tick 的 clk 数；tick 脉冲每 3 拍一次
localparam integer LANE_N        = 16;           // 每拍并行样点数（RFSoC DAC lane）
localparam integer SAMP_PER_TICK = 48;           // 16*3，一格里的样点数
localparam integer DA_SIG_CLK_HZ = 300000000;    // fabric 300 MHz；DSP 不要放到 600 MHz
localparam integer DA_SIG_F_TICK = 100000000;    // clk/3 = 100 MHz；T_scan = round(F_TICK*360/ω)
localparam integer MAX_HOP       = 32;           // 捷变频表 / 相位表项数上限
localparam integer TICK_W        = 20;           // PW / PRI / LFM 时长位宽（tick）
localparam integer FTW_W         = 32;           // 频率字位宽；0x64/0x65/0x69/0x77 低 32 bit

localparam [63:0] DA_SIG_FS_HZ   = 64'd4800000000; // DAC 采样率 4.8 GSPS
localparam [63:0] DA_SIG_POW2_32 = 64'd4294967296; // 2^32，Hz→FTW 分子

// -----------------------------------------------------------------------------
// cfg_calc 合法性窗口（协议 10 ns 格）。寄存器仍是 TICK_W bit，这里是检查上限
// -----------------------------------------------------------------------------
localparam integer PW_MIN_TICK   = 10;           // 脉宽下限 0.1 us
localparam integer PW_MAX_TICK   = 200000;       // 脉宽/LFM 时长检查上限 2 ms
localparam integer PRI_MAX_TICK  = 1000000;      // PRI 检查上限 10 ms

// -----------------------------------------------------------------------------
// cfg_calc 出错码，非法只报错、不改用户数
// -----------------------------------------------------------------------------
localparam [7:0] ERR_OK           = 8'd0;        // 无错
localparam [7:0] ERR_PW_RANGE     = 8'd1;        // 脉宽超 PW_MIN/MAX
localparam [7:0] ERR_PRI_LT_PW    = 8'd2;        // PRI ≤ 脉宽
localparam [7:0] ERR_LFM_TIME     = 8'd3;        // LFM 时长为 0 或超过 PW_MAX_TICK
localparam [7:0] ERR_FREQ_RANGE   = 8'd4;        // 频率越界
localparam [7:0] ERR_SLOPE        = 8'd5;        // 调频斜率超协议 40 MHz/us
localparam [7:0] ERR_HOP          = 8'd6;        // 捷变参数非法

// -----------------------------------------------------------------------------
// 0x61[1:0] sig_type、0x62 调制子类型
// -----------------------------------------------------------------------------
localparam [1:0] SIG_CW           = 2'd0;        // 常规脉冲 / 定频
localparam [1:0] SIG_FM           = 2'd1;        // 调频
localparam [1:0] SIG_PM           = 2'd2;        // 调相
localparam [1:0] FM_LFM           = 2'd0;        // 0x62[1:0] FM 时：线性
localparam [1:0] FM_NLFM          = 2'd1;        // 0x62[1:0] FM 时：非线性
localparam [1:0] NLFM_TRI         = 2'd0;        // 0x62[4:3]：三角
localparam [1:0] NLFM_SINE        = 2'd1;        // 0x62[4:3]：正弦
localparam [1:0] NLFM_SAW         = 2'd2;        // 0x62[4:3]：锯齿
localparam [1:0] PM_BIPHASE       = 2'd0;        // 0x62[1:0] PM 时：二相
localparam [1:0] PM_QUAD          = 2'd1;        // 四相
localparam [1:0] PM_P3            = 2'd2;        // P3
localparam [1:0] PM_P4            = 2'd3;        // P4

// Hz → 32-bit FTW。组合对照用：上位机与 TB 核对；RTL 出波路径不再调用
// hz≥Fs 钳到全 1；否则 floor(hz * 2^32 / Fs)，与 host_sig_gen.hz_to_ftw 相同
function [31:0] da_sig_hz_to_ftw;
    input [47:0] hz;                             // 输入频率，单位 Hz
    reg [95:0] num;                              // hz * 2^32，96 bit 防溢出
    begin
        if (hz >= DA_SIG_FS_HZ[47:0])
            da_sig_hz_to_ftw = 32'hFFFF_FFFF;    // 达到或超过 Fs，FTW 饱和
        else begin
            num = {48'd0, hz} * {32'd0, DA_SIG_POW2_32};
            da_sig_hz_to_ftw = num[95:0] / DA_SIG_FS_HZ;
        end
    end
endfunction

// END INLINE HEADER: da_sig_common.vh



localparam [3:0] S_IDLE  = 4'd0;        // 空闲，等待 start_calc
localparam [3:0] S_CHK   = 4'd1;        // 检查 hop / LFM 时长
localparam [3:0] S_HDIV  = 4'd2;        // 已启动 hop 除法，空 1 拍
localparam [3:0] S_HWAIT = 4'd3;        // 等待 hop_bw/(N-1)
localparam [3:0] S_SLOPE = 4'd4;        // 决定是否算斜率
localparam [3:0] S_SDIV  = 4'd5;        // 等待 lfm_bw/N_sample
localparam [3:0] S_PSTEP = 4'd6;        // 等待 2^32/N_sample
localparam [3:0] S_DONE  = 4'd7;        // 给出 done 脉冲后回 IDLE


wire [63:0] q32_magnitude;
wire q32_done;
reg q32_ready=1'b1;
reg q32_negative=1'b0;
reg [31:0] q32_bandwidth=0;
wire [31:0] q32_samples=({12'd0,lfm_ticks}<<5)+({12'd0,lfm_ticks}<<4);
da_sig_div_q32 u_q32_div(.clk(clk),.rst(rst),
    .start(start_calc && !busy && sig_type==2'd1),
    .dividend({lfm_bw_ftw,32'd0}),.divisor(q32_samples),
    .quot(q32_magnitude),.done(q32_done));
assign lfm_slope_q32=q32_negative ? -$signed(q32_magnitude) : $signed(q32_magnitude);
wire signed [63:0] half_bandwidth=$signed({32'd0,q32_bandwidth}<<31);
assign lfm_start_q32=q32_negative ? half_bandwidth : -half_bandwidth;
always @(posedge clk) begin
    if (rst) begin q32_ready<=1; q32_negative<=0; q32_bandwidth<=0; end
    else begin
        if (start_calc && !busy) begin
            q32_ready<=(sig_type!=2'd1);q32_negative<=lfm_neg;q32_bandwidth<=lfm_bw_ftw;
        end
        if (q32_done) q32_ready<=1;
    end
end

reg [3:0] st;                           // 当前状态
reg [5:0] n_use;                        // 钳位后的 hop 点数
reg       div_start;                    // 除法启动脉冲，保持 1 个 clk
reg       div_shl32;                    // 0=BW/N；1=2^32/N
wire      div_done;                     // 除法完成
wire [31:0] div_quot;                   // 除法商，整数结果
reg [47:0]  div_a;                      // 被除数
reg [63:0]  div_b;                      // 除数


wire [5:0] n_req =                      // hop_n 钳到 1~32
    (hop_n < 6'd1)  ? 6'd1  :
    (hop_n > 6'd32) ? 6'd32 :
                       hop_n;

wire       hop_on = (freq_type != 2'd0);
                                        // 0 单载频不算 hop 表；1/2/3 都要算


// =============================================================================
// 无符号除法器：本模块独占
// HDIV 算 hop_step，SDIV 算 slope，PSTEP 算 phase_step
// =============================================================================

da_sig_div_u u_div (

    .clk      (clk),                     // 工作时钟
    .rst      (rst),                     // 同步复位
    .start    (div_start),               // 启动脉冲
    .shl32    (div_shl32),               // 1=被除数先左移 32 位，算 2^32/N
    .dividend (div_a),                   // 被除数：带宽 FTW 或 1
    .divisor  (div_b),                   // 除数：N-1 或 N_sample
    .quot     (div_quot),                // 整数商
    .done     (div_done)                 // 完成脉冲

);


// =============================================================================
// 配置状态机：hop 步长、斜率、phase_step
// =============================================================================

always @(posedge clk) begin

    if (rst) begin

        st         <= S_IDLE;            // 上电进入空闲
        busy       <= 1'b0;              // 不挡 start
        done       <= 1'b0;              // 无完成脉冲
        err        <= ERR_OK;            // 默认无错
        div_start  <= 1'b0;              // 除法未启动
        slope_word <= 32'sd0;            // 斜率清零
        n_sample   <= 32'd0;             // 样点数清零
        fm_start   <= 32'sd0;            // 调制起点清零
        hop_start  <= 32'd0;             // 捷变起点清零
        hop_step   <= 32'd0;             // 捷变步长清零
        phase_step <= 32'd0;             // 正弦步进清零
        n_use      <= 6'd1;              // 默认 1 个频点
        div_a      <= 48'd0;             // 被除数清零
        div_b      <= 64'd0;             // 除数清零
        div_shl32  <= 1'b0;              // 默认做 BW/N，不左移

    end else begin

        div_start <= 1'b0;               // 启动脉冲默认只保持 1 个 clk
        done      <= 1'b0;               // 完成脉冲默认只保持 1 个 clk

        case (st)

            S_IDLE: begin                // 空闲。start_calc 后锁存并进 CHK

                busy <= 1'b0;            // 空闲时不挡 start

                if (start_calc) begin

                    busy     <= 1'b1;    // 开始计算，挡住 DA start
                    err      <= ERR_OK;  // 新一次计算先清错
                    n_use    <= hop_on ? n_req : 6'd1;
                                         // 单载频强制 N=1，不做 hop 除法
                    n_sample <= {7'd0, lfm_ticks, 5'd0} +
                                {8'd0, lfm_ticks, 4'd0};
                                         // 32T + 16T = 48T
                                         // 每 tick = 16 lane × 3 clk
                    fm_start <= 32'sd0;  // 先清调制起点，FM 路径再写
                    st       <= S_CHK;   // 去检查合法性

                end

            end

            S_CHK: begin                 // 检查。FM 时长非法 → DONE
                                         // 无 hop → hop=fc，进 SLOPE
                                         // 有 hop → 装 hop_bw/(N-1)，进 HDIV

                if ((sig_type == 2'd1) &&
                    ((lfm_ticks == 20'd0) ||
                     (lfm_ticks > PW_MAX_TICK[19:0]))) begin

                    err <= ERR_LFM_TIME; // FM 时长为 0 或超过上限
                    st  <= S_DONE;

                end else if ((n_use <= 6'd1) ||
                             (hop_bw_ftw == 32'd0)) begin

                    hop_start <= fc_ftw; // 单载频 / 点数不足 / 带宽为 0
                    hop_step  <= 32'd0;  // 不跳，步长 0
                    st        <= S_SLOPE;

                end else begin           // 整数 hop_step = hop_bw / (N-1)
                                         // 不再把 Hz 除以 Fs

                    div_a      <= {16'd0, hop_bw_ftw};
                                         // 被除数：捷变带宽 FTW
                    div_b      <= {58'd0, n_use - 6'd1};
                                         // 除数：N-1
                    div_shl32  <= 1'b0;  // 普通除法，不左移
                    div_start  <= 1'b1;  // 启动 hop 除法
                    st         <= S_HDIV;

                end

            end

            S_HDIV: begin                // 除法启动后空 1 拍，等 RUN 开始

                st <= S_HWAIT;

            end

            S_HWAIT: begin               // 等 hop 除法
                                         // 商 → hop_step；fc−B/2 → hop_start

                if (div_done) begin

                    hop_step  <= div_quot;
                                         // 相邻 hop 间隔 B/(N-1)
                    hop_start <= fc_ftw - {1'b0, hop_bw_ftw[31:1]};
                                         // 起点 = fc − B/2，右移 1 位当除 2
                    st        <= S_SLOPE;

                end

            end

            S_SLOPE: begin               // 非 FM：斜率/phase_step 清 0，DONE
                                         // FM：装 lfm_bw/N_sample，进 SDIV

                if (sig_type == 2'd1) begin

                    if (n_sample == 32'd0) begin

                        slope_word <= 32'sd0;
                                         // 样点数为 0 无法除，斜率置 0
                        fm_start   <= 32'sd0;
                        phase_step <= 32'd0;
                        st         <= S_DONE;

                    end else begin       // 整数斜率 = 调频带宽 / 样点数

                        div_a      <= {16'd0, lfm_bw_ftw};
                                         // 被除数：调频带宽 FTW
                        div_b      <= {32'd0, n_sample};
                                         // 除数：N_sample
                        div_shl32  <= 1'b0;
                                         // 普通除法，不左移
                        div_start  <= 1'b1;
                        st         <= S_SDIV;

                    end

                end else begin           // CW / PM：没有调频斜率

                    slope_word <= 32'sd0;
                    fm_start   <= 32'sd0;
                    phase_step <= 32'd0;
                    st         <= S_DONE;

                end

            end

            S_SDIV: begin                // 等 slope 除法。商带符号 → S
                                         // ±B/2 → fm_start
                                         // 再启动 1<<32 / N_sample 给正弦 NLFM

                if (div_done) begin

                    slope_word <= lfm_neg ? -$signed(div_quot)
                                          :  $signed(div_quot);
                                         // 负扫把斜率取反
                    fm_start   <= lfm_neg ?  $signed({1'b0, lfm_bw_ftw[31:1]})
                                          : -$signed({1'b0, lfm_bw_ftw[31:1]});
                                         // 正扫从 −B/2 往上，负扫从 +B/2 往下

                    div_a      <= 48'd1; // 被除数 1，配合 shl32 得到 2^32
                    div_b      <= {32'd0, n_sample};
                    div_shl32  <= 1'b1;  // 算 2^32 / N_sample
                    div_start  <= 1'b1;
                    st         <= S_PSTEP;

                end

            end

            S_PSTEP: begin               // 等 phase_step 除法。商 → phase_step

                if (div_done) begin

                    phase_step <= div_quot;
                                         // 正弦 NLFM 相位步进；其它模式下游可忽略
                    st         <= S_DONE;

                end

            end

            S_DONE: begin                // 完成脉冲，busy 放下，回 IDLE

                if (q32_ready) begin
                    busy <= 1'b0; done <= 1'b1; st <= S_IDLE;
                end

            end

            default: st <= S_IDLE;       // 非法状态回到空闲

        endcase

    end

end

endmodule
