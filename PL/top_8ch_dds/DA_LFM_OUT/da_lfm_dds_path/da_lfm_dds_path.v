// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_lfm_dds_path.v
// 功能：将十六路 32 bit 相位截为 12 bit 正弦表地址，进行幅度缩放并打包十六个 16 bit 样点。
// da_lfm_common.vh的旧链限幅函数为兼容保留，此活动通路仅用相位查表/缩放。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_lfm_dds_path
// 功能: 将 16 路并行 32 bit 相位送入正弦 LUT，完成幅度缩放并打包成
//       16×16 bit DAC 样点。
// 时钟: 全部逻辑位于 DAC fabric 时钟域（当前 300 MHz）。
// 时序: 相位截位、LUT、幅度乘法和出口均有流水；valid/user 与样点等延迟。
// =============================================================================

module da_lfm_dds_path (

    input  wire         clk,                  // FPGA 工作时钟，300 MHz
    input  wire         rst_sync,             // 同步复位，高有效
    input  wire         stop_edge,            // 停止沿，拉低 LUT aresetn 排空流水
    input  wire         cfg_load_edge,        // 加载沿，同样排空 LUT，避免换参后冒出旧正弦
    input  wire         clear_phase_edge,     // 清相沿，同样排空 LUT

    input  wire         phase_axis_valid,     // 与 16 路相位同拍的 AXIS valid
    input  wire         phase_axis_user,      // 脉冲门控，0 则后续输出静音
    input  wire [31:0]  phase_r_0,            // 并行第 0 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_1,            // 并行第 1 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_2,            // 并行第 2 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_3,            // 并行第 3 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_4,            // 并行第 4 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_5,            // 并行第 5 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_6,            // 并行第 6 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_7,            // 并行第 7 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_8,            // 并行第 8 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_9,            // 并行第 9 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_10,           // 并行第 10 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_11,           // 并行第 11 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_12,           // 并行第 12 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_13,           // 并行第 13 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_14,           // 并行第 14 路 32 bit 瞬时相位
    input  wire [31:0]  phase_r_15,           // 并行第 15 路 32 bit 瞬时相位

    input  wire         dds_red_en_sync,      // 幅度衰减使能，1 时样点乘衰减系数再右移 11 位
    input  wire signed [10:0] dds_signal_reduction_sync, // 11 bit 衰减系数，与 16 bit 样点相乘

    output reg  [255:0] dds_sin,              // 16 路 × 16 bit 打包后的 DAC 正弦总线
    output wire         dds_valid             // 输出样点有效标志

);


// BEGIN INLINE HEADER: da_lfm_common.vh (module内局部作用域)
// =============================================================================
// 文件: da_lfm_common.vh
// 功能: DA_LFM 子系统公共参数与工具函数
// 说明:
//   - 在需要的 module 内部 `include 本文件（函数作用域需在 module 内）
//   - 频率/相位/斜率控制字位宽: 32 bit
//   - 等效采样率约 4.8 GSPS（300 MHz fabric × 16 并行样点）
//   - freq_word = round(f_Hz * 2^32 / 4.8e9)
// =============================================================================

// 分段表最大段数
localparam integer DA_LFM_MAX_SEG = 8;

// 指标限幅（32-bit 频率字域）
// MAX_SLOPE ≈ ±40 MHz/us；MAX_BW ≈ 2000 MHz
localparam signed [31:0] DA_LFM_MAX_SLOPE_WORD = 32'sd4938;
localparam [31:0]        DA_LFM_MAX_BW_WORD    = 32'd1456069925;

// 配置状态机编码
// MUX → LIMIT0(差/方向) → LIMIT1(写 stop) → INIT
// 禁止段表 mux 与 32bit 限幅同拍
localparam [2:0] DA_LFM_CFG_IDLE   = 3'd0;
localparam [2:0] DA_LFM_CFG_LIMIT  = 3'd1; // 只算 |stop-start|、方向、斜率钳位
localparam [2:0] DA_LFM_CFG_INIT   = 3'd2;
localparam [2:0] DA_LFM_CFG_GEN    = 3'd3;
localparam [2:0] DA_LFM_CFG_COMMIT = 3'd4;
localparam [2:0] DA_LFM_CFG_STORE  = 3'd5;
localparam [2:0] DA_LFM_CFG_MUX    = 3'd6; // raw mux → prep_*
localparam [2:0] DA_LFM_CFG_LIMIT1 = 3'd7; // 只按 over 写 prep_freq_stop

// -----------------------------------------------------------------------------
// 斜率限幅：超过协议指标的正/负斜率钳位到允许范围
// -----------------------------------------------------------------------------
function signed [31:0] da_lfm_limit_slope;
    input signed [31:0] slope_in;
    begin
        if (slope_in > DA_LFM_MAX_SLOPE_WORD)
            da_lfm_limit_slope = DA_LFM_MAX_SLOPE_WORD;
        else if (slope_in < -DA_LFM_MAX_SLOPE_WORD)
            da_lfm_limit_slope = -DA_LFM_MAX_SLOPE_WORD;
        else
            da_lfm_limit_slope = slope_in;
    end
endfunction

// -----------------------------------------------------------------------------
// 终止频率限幅：start/stop 带宽不超过约 2 GHz 对应字差值
// -----------------------------------------------------------------------------
function [31:0] da_lfm_limit_stop_word;
    input [31:0] start_word;
    input [31:0] stop_word;
    reg [32:0] diff_word;
    reg [32:0] sum_word;
    begin
        if (stop_word >= start_word) begin
            diff_word = {1'b0, stop_word} - {1'b0, start_word};
            if (diff_word <= {1'b0, DA_LFM_MAX_BW_WORD}) begin
                da_lfm_limit_stop_word = stop_word;
            end else begin
                sum_word = {1'b0, start_word} + {1'b0, DA_LFM_MAX_BW_WORD};
                da_lfm_limit_stop_word = sum_word[32] ? 32'hFFFFFFFF : sum_word[31:0];
            end
        end else begin
            diff_word = {1'b0, start_word} - {1'b0, stop_word};
            if (diff_word <= {1'b0, DA_LFM_MAX_BW_WORD})
                da_lfm_limit_stop_word = stop_word;
            else
                da_lfm_limit_stop_word =
                    (start_word > DA_LFM_MAX_BW_WORD) ?
                    (start_word - DA_LFM_MAX_BW_WORD) : 32'd0;
        end
    end
endfunction

// -----------------------------------------------------------------------------
// 幅度缩放：red_en=1 时 sample * reduction >> 11；否则直通
// -----------------------------------------------------------------------------
function signed [15:0] da_lfm_scale_sample;
    input signed [15:0] sample_in;
    input               red_en_in;
    input signed [10:0] reduction_in;
    reg signed [26:0] mult;
    begin
        if (red_en_in) begin
            mult = sample_in * $signed({1'b0, reduction_in});
            da_lfm_scale_sample = mult >>> 11;
        end else begin
            da_lfm_scale_sample = sample_in;
        end
    end
endfunction

// END INLINE HEADER: da_lfm_common.vh



reg         phase_valid_r = 1'b0;             // 打一拍后的相位有效，送 LUT tvalid
reg         phase_user_r  = 1'b0;             // 打一拍后的脉冲门控，送 LUT tuser
reg  [11:0] phase_slice_r [0:15];             // 16 路正弦 LUT 地址，取相位[31:20]


// =============================================================================
// 相位地址打一拍：切断 16 路相位加法到 LUT 地址的组合路径
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin                       // 同步复位

        phase_valid_r <= 1'b0;                // 相位有效清零
        phase_user_r  <= 1'b0;                // 门控清零
        phase_slice_r[0]  <= 12'd0;           // 第 0 路正弦地址清零
        phase_slice_r[1]  <= 12'd0;           // 第 1 路正弦地址清零
        phase_slice_r[2]  <= 12'd0;           // 第 2 路正弦地址清零
        phase_slice_r[3]  <= 12'd0;           // 第 3 路正弦地址清零
        phase_slice_r[4]  <= 12'd0;           // 第 4 路正弦地址清零
        phase_slice_r[5]  <= 12'd0;           // 第 5 路正弦地址清零
        phase_slice_r[6]  <= 12'd0;           // 第 6 路正弦地址清零
        phase_slice_r[7]  <= 12'd0;           // 第 7 路正弦地址清零
        phase_slice_r[8]  <= 12'd0;           // 第 8 路正弦地址清零
        phase_slice_r[9]  <= 12'd0;           // 第 9 路正弦地址清零
        phase_slice_r[10] <= 12'd0;           // 第 10 路正弦地址清零
        phase_slice_r[11] <= 12'd0;           // 第 11 路正弦地址清零
        phase_slice_r[12] <= 12'd0;           // 第 12 路正弦地址清零
        phase_slice_r[13] <= 12'd0;           // 第 13 路正弦地址清零
        phase_slice_r[14] <= 12'd0;           // 第 14 路正弦地址清零
        phase_slice_r[15] <= 12'd0;           // 第 15 路正弦地址清零

    end else begin                            // 正常锁存相位地址

        phase_valid_r <= phase_axis_valid;    // 相位有效打一拍
        phase_user_r  <= phase_axis_user;     // 脉冲门控打一拍
        phase_slice_r[0]  <= phase_r_0[31:20];  // 第 0 路取相位高 12 bit 作为正弦地址
        phase_slice_r[1]  <= phase_r_1[31:20];  // 第 1 路取相位高 12 bit 作为正弦地址
        phase_slice_r[2]  <= phase_r_2[31:20];  // 第 2 路取相位高 12 bit 作为正弦地址
        phase_slice_r[3]  <= phase_r_3[31:20];  // 第 3 路取相位高 12 bit 作为正弦地址
        phase_slice_r[4]  <= phase_r_4[31:20];  // 第 4 路取相位高 12 bit 作为正弦地址
        phase_slice_r[5]  <= phase_r_5[31:20];  // 第 5 路取相位高 12 bit 作为正弦地址
        phase_slice_r[6]  <= phase_r_6[31:20];  // 第 6 路取相位高 12 bit 作为正弦地址
        phase_slice_r[7]  <= phase_r_7[31:20];  // 第 7 路取相位高 12 bit 作为正弦地址
        phase_slice_r[8]  <= phase_r_8[31:20];  // 第 8 路取相位高 12 bit 作为正弦地址
        phase_slice_r[9]  <= phase_r_9[31:20];  // 第 9 路取相位高 12 bit 作为正弦地址
        phase_slice_r[10] <= phase_r_10[31:20]; // 第 10 路取相位高 12 bit 作为正弦地址
        phase_slice_r[11] <= phase_r_11[31:20]; // 第 11 路取相位高 12 bit 作为正弦地址
        phase_slice_r[12] <= phase_r_12[31:20]; // 第 12 路取相位高 12 bit 作为正弦地址
        phase_slice_r[13] <= phase_r_13[31:20]; // 第 13 路取相位高 12 bit 作为正弦地址
        phase_slice_r[14] <= phase_r_14[31:20]; // 第 14 路取相位高 12 bit 作为正弦地址
        phase_slice_r[15] <= phase_r_15[31:20]; // 第 15 路取相位高 12 bit 作为正弦地址

    end

end


wire [255:0] dds_raw_bus;                     // 16 路 LUT 原始正弦样点打包总线
wire         dds_group_valid;                 // 主 LUT 给出的整组数据有效
wire         dds_group_user;                  // 主 LUT 给出的整组脉冲门控

reg [1:0] dds_reset_hold = 2'd3;              // LUT 复位保持计数，上电先保持 3 拍
wire dds_core_aresetn = (dds_reset_hold == 2'd0); // 计数减到 0 后 LUT 退出复位


// =============================================================================
// LUT 复位保持：stop / cfg_load / clear_phase / rst 拉低 aresetn 共 3 拍
// =============================================================================

always @(posedge clk) begin

    if (rst_sync || stop_edge || cfg_load_edge || clear_phase_edge) // 复位或换参事件
        dds_reset_hold <= 2'd3;               // 重新装 3，拉低 aresetn 排空 LUT
    else if (dds_reset_hold != 2'd0)          // 尚未减到 0
        dds_reset_hold <= dds_reset_hold - 2'd1; // 每拍减 1，共保持约 3 拍

end


genvar dds_i;                                 // 16 路正弦 LUT 循环变量

generate
    for (dds_i = 0; dds_i < 16; dds_i = dds_i + 1) begin : GEN_DDS_SIN_LUT // 例化 16 个并行正弦 LUT
        if (dds_i == 0) begin : GEN_DDS_MASTER // 第 0 路作为主 LUT，引出 group valid/user
            vu13p_sine_lut u_dds_sin_lut (

                .aclk                (clk),                      // 工作时钟
                .aresetn             (dds_core_aresetn),         // 低有效复位，换参时拉低排空流水
                .s_axis_phase_tvalid (phase_valid_r),            // 相位地址有效
                .s_axis_phase_tdata  ({4'b0000, phase_slice_r[dds_i]}), // 12 bit 地址拼成 16 bit AXIS
                .s_axis_phase_tuser  (phase_user_r),             // 脉冲门控随路到 LUT 输出
                .m_axis_data_tvalid  (dds_group_valid),          // 主 LUT 给出整组数据有效
                .m_axis_data_tdata   (dds_raw_bus[dds_i*16 +: 16]), // 第 i 路 16 bit 正弦样点
                .m_axis_data_tuser   (dds_group_user)            // 主 LUT 给出整组门控

            );
        end else begin : GEN_DDS_SLAVE        // 其余路为从 LUT，不引出 valid/user
            vu13p_sine_lut u_dds_sin_lut (

                .aclk                (clk),                      // 工作时钟
                .aresetn             (dds_core_aresetn),         // 低有效复位，与主 LUT 同步排空
                .s_axis_phase_tvalid (phase_valid_r),            // 相位地址有效
                .s_axis_phase_tdata  ({4'b0000, phase_slice_r[dds_i]}), // 12 bit 地址拼成 16 bit AXIS
                .s_axis_phase_tuser  (phase_user_r),             // 脉冲门控随路，本路不向外引出
                .m_axis_data_tvalid  (),                         // 从 LUT 不引出 valid
                .m_axis_data_tdata   (dds_raw_bus[dds_i*16 +: 16]), // 第 i 路 16 bit 正弦样点
                .m_axis_data_tuser   ()                          // 从 LUT 不引出 user

            );
        end
    end
endgenerate


wire signed [15:0] sample_raw_0  = dds_raw_bus[15:0];     // 第 0 路 LUT 有符号样点
wire signed [15:0] sample_raw_1  = dds_raw_bus[31:16];    // 第 1 路 LUT 有符号样点
wire signed [15:0] sample_raw_2  = dds_raw_bus[47:32];    // 第 2 路 LUT 有符号样点
wire signed [15:0] sample_raw_3  = dds_raw_bus[63:48];    // 第 3 路 LUT 有符号样点
wire signed [15:0] sample_raw_4  = dds_raw_bus[79:64];    // 第 4 路 LUT 有符号样点
wire signed [15:0] sample_raw_5  = dds_raw_bus[95:80];    // 第 5 路 LUT 有符号样点
wire signed [15:0] sample_raw_6  = dds_raw_bus[111:96];   // 第 6 路 LUT 有符号样点
wire signed [15:0] sample_raw_7  = dds_raw_bus[127:112];  // 第 7 路 LUT 有符号样点
wire signed [15:0] sample_raw_8  = dds_raw_bus[143:128];  // 第 8 路 LUT 有符号样点
wire signed [15:0] sample_raw_9  = dds_raw_bus[159:144];  // 第 9 路 LUT 有符号样点
wire signed [15:0] sample_raw_10 = dds_raw_bus[175:160];  // 第 10 路 LUT 有符号样点
wire signed [15:0] sample_raw_11 = dds_raw_bus[191:176];  // 第 11 路 LUT 有符号样点
wire signed [15:0] sample_raw_12 = dds_raw_bus[207:192];  // 第 12 路 LUT 有符号样点
wire signed [15:0] sample_raw_13 = dds_raw_bus[223:208];  // 第 13 路 LUT 有符号样点
wire signed [15:0] sample_raw_14 = dds_raw_bus[239:224];  // 第 14 路 LUT 有符号样点
wire signed [15:0] sample_raw_15 = dds_raw_bus[255:240];  // 第 15 路 LUT 有符号样点

reg signed [26:0] sample_prod_r_0  = 27'sd0;  // 第 0 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_1  = 27'sd0;  // 第 1 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_2  = 27'sd0;  // 第 2 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_3  = 27'sd0;  // 第 3 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_4  = 27'sd0;  // 第 4 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_5  = 27'sd0;  // 第 5 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_6  = 27'sd0;  // 第 6 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_7  = 27'sd0;  // 第 7 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_8  = 27'sd0;  // 第 8 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_9  = 27'sd0;  // 第 9 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_10 = 27'sd0;  // 第 10 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_11 = 27'sd0;  // 第 11 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_12 = 27'sd0;  // 第 12 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_13 = 27'sd0;  // 第 13 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_14 = 27'sd0;  // 第 14 路 16×11 幅度乘积
reg signed [26:0] sample_prod_r_15 = 27'sd0;  // 第 15 路 16×11 幅度乘积
reg signed [15:0] sample_raw_d_0  = 16'sd0;   // 第 0 路直通样点打一拍，与乘积对齐
reg signed [15:0] sample_raw_d_1  = 16'sd0;   // 第 1 路直通样点打一拍
reg signed [15:0] sample_raw_d_2  = 16'sd0;   // 第 2 路直通样点打一拍
reg signed [15:0] sample_raw_d_3  = 16'sd0;   // 第 3 路直通样点打一拍
reg signed [15:0] sample_raw_d_4  = 16'sd0;   // 第 4 路直通样点打一拍
reg signed [15:0] sample_raw_d_5  = 16'sd0;   // 第 5 路直通样点打一拍
reg signed [15:0] sample_raw_d_6  = 16'sd0;   // 第 6 路直通样点打一拍
reg signed [15:0] sample_raw_d_7  = 16'sd0;   // 第 7 路直通样点打一拍
reg signed [15:0] sample_raw_d_8  = 16'sd0;   // 第 8 路直通样点打一拍
reg signed [15:0] sample_raw_d_9  = 16'sd0;   // 第 9 路直通样点打一拍
reg signed [15:0] sample_raw_d_10 = 16'sd0;   // 第 10 路直通样点打一拍
reg signed [15:0] sample_raw_d_11 = 16'sd0;   // 第 11 路直通样点打一拍
reg signed [15:0] sample_raw_d_12 = 16'sd0;   // 第 12 路直通样点打一拍
reg signed [15:0] sample_raw_d_13 = 16'sd0;   // 第 13 路直通样点打一拍
reg signed [15:0] sample_raw_d_14 = 16'sd0;   // 第 14 路直通样点打一拍
reg signed [15:0] sample_raw_d_15 = 16'sd0;   // 第 15 路直通样点打一拍
reg signed [15:0] sample_scaled_r_0  = 16'sd0; // 第 0 路缩放后样点
reg signed [15:0] sample_scaled_r_1  = 16'sd0; // 第 1 路缩放后样点
reg signed [15:0] sample_scaled_r_2  = 16'sd0; // 第 2 路缩放后样点
reg signed [15:0] sample_scaled_r_3  = 16'sd0; // 第 3 路缩放后样点
reg signed [15:0] sample_scaled_r_4  = 16'sd0; // 第 4 路缩放后样点
reg signed [15:0] sample_scaled_r_5  = 16'sd0; // 第 5 路缩放后样点
reg signed [15:0] sample_scaled_r_6  = 16'sd0; // 第 6 路缩放后样点
reg signed [15:0] sample_scaled_r_7  = 16'sd0; // 第 7 路缩放后样点
reg signed [15:0] sample_scaled_r_8  = 16'sd0; // 第 8 路缩放后样点
reg signed [15:0] sample_scaled_r_9  = 16'sd0; // 第 9 路缩放后样点
reg signed [15:0] sample_scaled_r_10 = 16'sd0; // 第 10 路缩放后样点
reg signed [15:0] sample_scaled_r_11 = 16'sd0; // 第 11 路缩放后样点
reg signed [15:0] sample_scaled_r_12 = 16'sd0; // 第 12 路缩放后样点
reg signed [15:0] sample_scaled_r_13 = 16'sd0; // 第 13 路缩放后样点
reg signed [15:0] sample_scaled_r_14 = 16'sd0; // 第 14 路缩放后样点
reg signed [15:0] sample_scaled_r_15 = 16'sd0; // 第 15 路缩放后样点
reg red_d = 1'b0;                             // 衰减使能打一拍，与乘积/直通对齐
reg mul_valid = 1'b0;                         // LUT 输出 valid 打一拍
reg mul_user  = 1'b0;                         // LUT 输出 user 打一拍，且要求 valid=1

wire [255:0] sample_bus_r = {                 // 16 路缩放后样点打包，高位为第 15 路
    sample_scaled_r_15, sample_scaled_r_14, sample_scaled_r_13, sample_scaled_r_12,
    sample_scaled_r_11, sample_scaled_r_10, sample_scaled_r_9,  sample_scaled_r_8,
    sample_scaled_r_7,  sample_scaled_r_6,  sample_scaled_r_5,  sample_scaled_r_4,
    sample_scaled_r_3,  sample_scaled_r_2,  sample_scaled_r_1,  sample_scaled_r_0
};

reg scaled_valid_r = 1'b0;                    // 缩放后样点 valid，再打一拍送出
reg scaled_user_r  = 1'b0;                    // 缩放后样点门控，再打一拍送出
reg raw_valid      = 1'b0;                    // 最终输出 valid，对齐 dds_sin

assign dds_valid = raw_valid;                 // 模块输出 valid 直接等于 raw_valid


// =============================================================================
// 幅度缩放流水：乘一拍，>>>11 或直通一拍，再按 user 送出或静音
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin                       // 同步复位，清空缩放流水和输出

        sample_prod_r_0  <= 27'sd0;           // 第 0 路乘积清零
        sample_prod_r_1  <= 27'sd0;           // 第 1 路乘积清零
        sample_prod_r_2  <= 27'sd0;           // 第 2 路乘积清零
        sample_prod_r_3  <= 27'sd0;           // 第 3 路乘积清零
        sample_prod_r_4  <= 27'sd0;           // 第 4 路乘积清零
        sample_prod_r_5  <= 27'sd0;           // 第 5 路乘积清零
        sample_prod_r_6  <= 27'sd0;           // 第 6 路乘积清零
        sample_prod_r_7  <= 27'sd0;           // 第 7 路乘积清零
        sample_prod_r_8  <= 27'sd0;           // 第 8 路乘积清零
        sample_prod_r_9  <= 27'sd0;           // 第 9 路乘积清零
        sample_prod_r_10 <= 27'sd0;           // 第 10 路乘积清零
        sample_prod_r_11 <= 27'sd0;           // 第 11 路乘积清零
        sample_prod_r_12 <= 27'sd0;           // 第 12 路乘积清零
        sample_prod_r_13 <= 27'sd0;           // 第 13 路乘积清零
        sample_prod_r_14 <= 27'sd0;           // 第 14 路乘积清零
        sample_prod_r_15 <= 27'sd0;           // 第 15 路乘积清零
        sample_raw_d_0  <= 16'sd0;            // 第 0 路直通打拍清零
        sample_raw_d_1  <= 16'sd0;            // 第 1 路直通打拍清零
        sample_raw_d_2  <= 16'sd0;            // 第 2 路直通打拍清零
        sample_raw_d_3  <= 16'sd0;            // 第 3 路直通打拍清零
        sample_raw_d_4  <= 16'sd0;            // 第 4 路直通打拍清零
        sample_raw_d_5  <= 16'sd0;            // 第 5 路直通打拍清零
        sample_raw_d_6  <= 16'sd0;            // 第 6 路直通打拍清零
        sample_raw_d_7  <= 16'sd0;            // 第 7 路直通打拍清零
        sample_raw_d_8  <= 16'sd0;            // 第 8 路直通打拍清零
        sample_raw_d_9  <= 16'sd0;            // 第 9 路直通打拍清零
        sample_raw_d_10 <= 16'sd0;            // 第 10 路直通打拍清零
        sample_raw_d_11 <= 16'sd0;            // 第 11 路直通打拍清零
        sample_raw_d_12 <= 16'sd0;            // 第 12 路直通打拍清零
        sample_raw_d_13 <= 16'sd0;            // 第 13 路直通打拍清零
        sample_raw_d_14 <= 16'sd0;            // 第 14 路直通打拍清零
        sample_raw_d_15 <= 16'sd0;            // 第 15 路直通打拍清零
        sample_scaled_r_0  <= 16'sd0;         // 第 0 路缩放样点清零
        sample_scaled_r_1  <= 16'sd0;         // 第 1 路缩放样点清零
        sample_scaled_r_2  <= 16'sd0;         // 第 2 路缩放样点清零
        sample_scaled_r_3  <= 16'sd0;         // 第 3 路缩放样点清零
        sample_scaled_r_4  <= 16'sd0;         // 第 4 路缩放样点清零
        sample_scaled_r_5  <= 16'sd0;         // 第 5 路缩放样点清零
        sample_scaled_r_6  <= 16'sd0;         // 第 6 路缩放样点清零
        sample_scaled_r_7  <= 16'sd0;         // 第 7 路缩放样点清零
        sample_scaled_r_8  <= 16'sd0;         // 第 8 路缩放样点清零
        sample_scaled_r_9  <= 16'sd0;         // 第 9 路缩放样点清零
        sample_scaled_r_10 <= 16'sd0;         // 第 10 路缩放样点清零
        sample_scaled_r_11 <= 16'sd0;         // 第 11 路缩放样点清零
        sample_scaled_r_12 <= 16'sd0;         // 第 12 路缩放样点清零
        sample_scaled_r_13 <= 16'sd0;         // 第 13 路缩放样点清零
        sample_scaled_r_14 <= 16'sd0;         // 第 14 路缩放样点清零
        sample_scaled_r_15 <= 16'sd0;         // 第 15 路缩放样点清零
        red_d          <= 1'b0;               // 衰减使能打拍清零
        mul_valid      <= 1'b0;               // 乘积级 valid 清零
        mul_user       <= 1'b0;               // 乘积级 user 清零
        scaled_valid_r <= 1'b0;               // 缩放级 valid 清零
        scaled_user_r  <= 1'b0;               // 缩放级 user 清零
        raw_valid      <= 1'b0;               // 输出 valid 清零
        dds_sin        <= 256'd0;             // DAC 总线静音

    end else begin                            // 正常幅度缩放流水

        sample_prod_r_0  <= sample_raw_0  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 0 路 16×11 乘
        sample_prod_r_1  <= sample_raw_1  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 1 路 16×11 乘
        sample_prod_r_2  <= sample_raw_2  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 2 路 16×11 乘
        sample_prod_r_3  <= sample_raw_3  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 3 路 16×11 乘
        sample_prod_r_4  <= sample_raw_4  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 4 路 16×11 乘
        sample_prod_r_5  <= sample_raw_5  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 5 路 16×11 乘
        sample_prod_r_6  <= sample_raw_6  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 6 路 16×11 乘
        sample_prod_r_7  <= sample_raw_7  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 7 路 16×11 乘
        sample_prod_r_8  <= sample_raw_8  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 8 路 16×11 乘
        sample_prod_r_9  <= sample_raw_9  * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 9 路 16×11 乘
        sample_prod_r_10 <= sample_raw_10 * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 10 路 16×11 乘
        sample_prod_r_11 <= sample_raw_11 * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 11 路 16×11 乘
        sample_prod_r_12 <= sample_raw_12 * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 12 路 16×11 乘
        sample_prod_r_13 <= sample_raw_13 * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 13 路 16×11 乘
        sample_prod_r_14 <= sample_raw_14 * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 14 路 16×11 乘
        sample_prod_r_15 <= sample_raw_15 * $signed({1'b0, dds_signal_reduction_sync}); // 拍1：第 15 路 16×11 乘
        sample_raw_d_0  <= sample_raw_0;      // 第 0 路直通路径打一拍，与乘积对齐
        sample_raw_d_1  <= sample_raw_1;      // 第 1 路直通路径打一拍
        sample_raw_d_2  <= sample_raw_2;      // 第 2 路直通路径打一拍
        sample_raw_d_3  <= sample_raw_3;      // 第 3 路直通路径打一拍
        sample_raw_d_4  <= sample_raw_4;      // 第 4 路直通路径打一拍
        sample_raw_d_5  <= sample_raw_5;      // 第 5 路直通路径打一拍
        sample_raw_d_6  <= sample_raw_6;      // 第 6 路直通路径打一拍
        sample_raw_d_7  <= sample_raw_7;      // 第 7 路直通路径打一拍
        sample_raw_d_8  <= sample_raw_8;      // 第 8 路直通路径打一拍
        sample_raw_d_9  <= sample_raw_9;      // 第 9 路直通路径打一拍
        sample_raw_d_10 <= sample_raw_10;     // 第 10 路直通路径打一拍
        sample_raw_d_11 <= sample_raw_11;     // 第 11 路直通路径打一拍
        sample_raw_d_12 <= sample_raw_12;     // 第 12 路直通路径打一拍
        sample_raw_d_13 <= sample_raw_13;     // 第 13 路直通路径打一拍
        sample_raw_d_14 <= sample_raw_14;     // 第 14 路直通路径打一拍
        sample_raw_d_15 <= sample_raw_15;     // 第 15 路直通路径打一拍
        red_d     <= dds_red_en_sync;         // 衰减使能打一拍，与乘积/直通对齐
        mul_valid <= dds_group_valid;         // LUT 输出 valid 打一拍
        mul_user  <= dds_group_valid && dds_group_user; // 有效且在脉冲门内才认为有用户数据

        sample_scaled_r_0  <= red_d ? sample_prod_r_0[26:11]  : sample_raw_d_0;  // 拍2：衰减则>>>11，否则直通
        sample_scaled_r_1  <= red_d ? sample_prod_r_1[26:11]  : sample_raw_d_1;  // 拍2：第 1 路缩放或直通
        sample_scaled_r_2  <= red_d ? sample_prod_r_2[26:11]  : sample_raw_d_2;  // 拍2：第 2 路缩放或直通
        sample_scaled_r_3  <= red_d ? sample_prod_r_3[26:11]  : sample_raw_d_3;  // 拍2：第 3 路缩放或直通
        sample_scaled_r_4  <= red_d ? sample_prod_r_4[26:11]  : sample_raw_d_4;  // 拍2：第 4 路缩放或直通
        sample_scaled_r_5  <= red_d ? sample_prod_r_5[26:11]  : sample_raw_d_5;  // 拍2：第 5 路缩放或直通
        sample_scaled_r_6  <= red_d ? sample_prod_r_6[26:11]  : sample_raw_d_6;  // 拍2：第 6 路缩放或直通
        sample_scaled_r_7  <= red_d ? sample_prod_r_7[26:11]  : sample_raw_d_7;  // 拍2：第 7 路缩放或直通
        sample_scaled_r_8  <= red_d ? sample_prod_r_8[26:11]  : sample_raw_d_8;  // 拍2：第 8 路缩放或直通
        sample_scaled_r_9  <= red_d ? sample_prod_r_9[26:11]  : sample_raw_d_9;  // 拍2：第 9 路缩放或直通
        sample_scaled_r_10 <= red_d ? sample_prod_r_10[26:11] : sample_raw_d_10; // 拍2：第 10 路缩放或直通
        sample_scaled_r_11 <= red_d ? sample_prod_r_11[26:11] : sample_raw_d_11; // 拍2：第 11 路缩放或直通
        sample_scaled_r_12 <= red_d ? sample_prod_r_12[26:11] : sample_raw_d_12; // 拍2：第 12 路缩放或直通
        sample_scaled_r_13 <= red_d ? sample_prod_r_13[26:11] : sample_raw_d_13; // 拍2：第 13 路缩放或直通
        sample_scaled_r_14 <= red_d ? sample_prod_r_14[26:11] : sample_raw_d_14; // 拍2：第 14 路缩放或直通
        sample_scaled_r_15 <= red_d ? sample_prod_r_15[26:11] : sample_raw_d_15; // 拍2：第 15 路缩放或直通

        if (!dds_core_aresetn || stop_edge || cfg_load_edge || clear_phase_edge) begin // LUT 复位或换参

            scaled_valid_r <= 1'b0;           // 清缩放 valid，避免旧样点继续有效
            scaled_user_r  <= 1'b0;           // 清缩放门控
            raw_valid      <= 1'b0;           // 清输出 valid
            mul_valid      <= 1'b0;           // 清乘积级 valid
            mul_user       <= 1'b0;           // 清乘积级 user
            dds_sin        <= 256'd0;         // 输出静音

        end else begin                        // 正常向后递推 valid/user 并送出样点

            scaled_valid_r <= mul_valid;      // 乘积级 valid 再打一拍
            scaled_user_r  <= mul_user;       // 乘积级 user 再打一拍
            raw_valid      <= scaled_valid_r; // 输出 valid 再对齐一拍
            if (scaled_valid_r && scaled_user_r) // 有效且门控为 1 才送出 16 路样点
                dds_sin <= sample_bus_r;      // 打包后的缩放样点送 DAC 总线
            else
                dds_sin <= 256'd0;            // 否则输出静音 0

        end

    end

end

endmodule
