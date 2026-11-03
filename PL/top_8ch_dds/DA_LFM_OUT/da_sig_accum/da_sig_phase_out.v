// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_phase_out.v
// 功能：寄存基相位、调相码、线性项和二次项的和，同时对齐 valid 与脉冲门控。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_phase_out
// 功能: 完成 16 路最终相位加法：phase_base + phi_code + i·F + tri(i)。
// 时序: 16 路加法统一寄存，valid/user 同拍输出给正弦 LUT。
// =============================================================================

module da_sig_phase_out (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        start_edge,       // 启动沿；相位输出清零
    input  wire        valid_d,          // 上一拍的有效，本拍对齐到 phase
    input  wire        user_d,           // 上一拍的脉宽门
    input  wire [31:0] pb_phi,           // phase_base + phi_code
    input  wire [31:0] ft0,              // 第 0 路相对相位 i·F+tri
    input  wire [31:0] ft1,
    input  wire [31:0] ft2,
    input  wire [31:0] ft3,
    input  wire [31:0] ft4,
    input  wire [31:0] ft5,
    input  wire [31:0] ft6,
    input  wire [31:0] ft7,
    input  wire [31:0] ft8,
    input  wire [31:0] ft9,
    input  wire [31:0] ft10,
    input  wire [31:0] ft11,
    input  wire [31:0] ft12,
    input  wire [31:0] ft13,
    input  wire [31:0] ft14,
    input  wire [31:0] ft15,
    output reg         phase_valid,      // 16 路相位是否有效
    output reg         phase_user,       // 与相位对齐的脉宽门
    output reg  [31:0] phase_r_0,        // 样点 0 的 DDS 相位
    output reg  [31:0] phase_r_1,
    output reg  [31:0] phase_r_2,
    output reg  [31:0] phase_r_3,
    output reg  [31:0] phase_r_4,
    output reg  [31:0] phase_r_5,
    output reg  [31:0] phase_r_6,
    output reg  [31:0] phase_r_7,
    output reg  [31:0] phase_r_8,
    output reg  [31:0] phase_r_9,
    output reg  [31:0] phase_r_10,
    output reg  [31:0] phase_r_11,
    output reg  [31:0] phase_r_12,
    output reg  [31:0] phase_r_13,
    output reg  [31:0] phase_r_14,
    output reg  [31:0] phase_r_15
);

// phase_i = pb_phi + ft_i
//         = phase_base + phi_code + i·F + i(i-1)/2·S

always @(posedge clk) begin

    if (rst_sync || start_edge) begin

        phase_valid <= 1'b0;             // 启动/复位期间相位无效
        phase_user  <= 1'b0;             // 输出门关闭
        phase_r_0  <= 32'd0; phase_r_1  <= 32'd0;
        phase_r_2  <= 32'd0; phase_r_3  <= 32'd0;
        phase_r_4  <= 32'd0; phase_r_5  <= 32'd0;
        phase_r_6  <= 32'd0; phase_r_7  <= 32'd0;
        phase_r_8  <= 32'd0; phase_r_9  <= 32'd0;
        phase_r_10 <= 32'd0; phase_r_11 <= 32'd0;
        phase_r_12 <= 32'd0; phase_r_13 <= 32'd0;
        phase_r_14 <= 32'd0; phase_r_15 <= 32'd0;

    end else begin

        phase_valid <= valid_d;          // 有效再打一拍，与最终相位对齐
        phase_user  <= user_d;           // 脉宽门再打一拍

        phase_r_0  <= pb_phi + ft0;      // 第 0 路最终相位
        phase_r_1  <= pb_phi + ft1;
        phase_r_2  <= pb_phi + ft2;
        phase_r_3  <= pb_phi + ft3;
        phase_r_4  <= pb_phi + ft4;
        phase_r_5  <= pb_phi + ft5;
        phase_r_6  <= pb_phi + ft6;
        phase_r_7  <= pb_phi + ft7;
        phase_r_8  <= pb_phi + ft8;
        phase_r_9  <= pb_phi + ft9;
        phase_r_10 <= pb_phi + ft10;
        phase_r_11 <= pb_phi + ft11;
        phase_r_12 <= pb_phi + ft12;
        phase_r_13 <= pb_phi + ft13;
        phase_r_14 <= pb_phi + ft14;
        phase_r_15 <= pb_phi + ft15;

    end

end

endmodule
