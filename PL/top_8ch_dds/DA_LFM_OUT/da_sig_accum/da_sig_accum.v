// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_accum.v
// 功能：形成十六路连续 DDS 相位：基相位加 i×F 与 i(i−1)/2×S，并更新下一向量基相位。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// da_sig_accum：16 路并行 DDS 相位累加器
// 功能：φ_i = φ_base + i·F + i(i-1)/2·S
//       φ_base_next = φ_base + 16F + 120S
// i·F 在 da_sig_fx16；二次项在 da_sig_tri_pack；最终加在 da_sig_phase_out。
// 时序：16F 与 120S 先加进 inc_r，phase_base 只做 2 输入加。
// =============================================================================
module da_sig_accum (

    input  wire        clk,              // FPGA 工作时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        start_edge,       // 启动脉冲；相位装 phase_init，流水排空
    input  wire        clear_phase_edge, // 清相沿；只改相位初值，不停跑
    input  wire        running,          // 波形运行使能
    input  wire        wave_ready,       // PRI 时间轴已经正式启动
    input  wire        pulse_gate,       // 脉冲有效门控；脉间不推进相位基

    input  wire        need_tri,         // 二次项选通：LFM 或非正弦 NLFM 为 1
    input  wire signed [31:0] slope_word, // 每样点斜率 S；三角 NLFM 已在 wave 乘 2
    input  wire [31:0] phi_code,         // 调相 + hop 相位；非调相时 wave 送 0
    input  wire [31:0] phase_init,       // DDS 初相
    input  wire signed [31:0] f_next,    // 下一拍瞬时频率：carrier + F_mod
    input  wire signed [31:0] f_idle,    // 空闲频率；不跑时仍跟上，避免再启动跳频

    output wire        active,           // 活跃：running && wave_ready
    output wire        cfg_busy,         // 配置忙：本模块恒 0
    output wire        phase_valid,      // 16 路相位是否有效
    output wire        phase_user,       // 与相位对齐的脉宽门
    output reg signed [31:0] slope_x16,  // 16S，回给 LFM/NLFM 每拍加一次
    output wire [31:0] phase_r_0,        // 第 0 路 DDS 相位
    output wire [31:0] phase_r_1,        // 第 1 路 DDS 相位
    output wire [31:0] phase_r_2,        // 第 2 路 DDS 相位
    output wire [31:0] phase_r_3,        // 第 3 路 DDS 相位
    output wire [31:0] phase_r_4,        // 第 4 路 DDS 相位
    output wire [31:0] phase_r_5,        // 第 5 路 DDS 相位
    output wire [31:0] phase_r_6,        // 第 6 路 DDS 相位
    output wire [31:0] phase_r_7,        // 第 7 路 DDS 相位
    output wire [31:0] phase_r_8,        // 第 8 路 DDS 相位
    output wire [31:0] phase_r_9,        // 第 9 路 DDS 相位
    output wire [31:0] phase_r_10,       // 第 10 路 DDS 相位
    output wire [31:0] phase_r_11,       // 第 11 路 DDS 相位
    output wire [31:0] phase_r_12,       // 第 12 路 DDS 相位
    output wire [31:0] phase_r_13,       // 第 13 路 DDS 相位
    output wire [31:0] phase_r_14,       // 第 14 路 DDS 相位
    output wire [31:0] phase_r_15        // 第 15 路 DDS 相位
);


reg [31:0] phase_base;                  // 相位基：本拍第 0 样点的载波相位
reg signed [31:0] f_r;                  // 锁存后的瞬时 FTW，送给 fx16
reg signed [31:0] slope_x120;           // 120S：16 样点二次项之和
reg signed [31:0] slope_r;              // 斜率工作副本，切断类型译码组合链
reg               need_tri_r;           // 二次项选通打第 1 拍
reg               need_tri_rr;          // 二次项选通打第 2 拍
reg               need_tri_rrr;         // 二次项选通打第 3 拍，对齐 tri_pack

reg signed [31:0] lfm_tri0,  lfm_tri1,  lfm_tri2,  lfm_tri3;
                                        // 16 路二次项工作副本 i(i-1)/2·S
reg signed [31:0] lfm_tri4,  lfm_tri5,  lfm_tri6,  lfm_tri7;
reg signed [31:0] lfm_tri8,  lfm_tri9,  lfm_tri10, lfm_tri11;
reg signed [31:0] lfm_tri12, lfm_tri13, lfm_tri14, lfm_tri15;

reg [31:0] ft0,  ft1,  ft2,  ft3;       // i·F + 二次项，送给 phase_out
reg [31:0] ft4,  ft5,  ft6,  ft7;
reg [31:0] ft8,  ft9,  ft10, ft11;
reg [31:0] ft12, ft13, ft14, ft15;

reg [31:0] pb_phi;                      // phase_base + phi_code
reg [31:0] inc_r;                       // 锁存本拍增量 16F+120S
reg        tri_on;                      // 工作二次项使能，与 lfm_tri 同拍
reg        user_d;                      // 脉宽门打一拍，对齐相位输出
reg        valid_d;                     // 有效打一拍，对齐相位输出
reg        active_r;                    // 活跃寄存器


assign active   = active_r;             // 给 carrier / LFM / NLFM
assign cfg_busy = 1'b0;                 // 斜率已由 cfg_calc 算完，这里不忙


wire signed [31:0] p_tri0,  p_tri1,  p_tri2,  p_tri3;
                                        // tri_pack 算出的 16 路二次项
wire signed [31:0] p_tri4,  p_tri5,  p_tri6,  p_tri7;
wire signed [31:0] p_tri8,  p_tri9,  p_tri10, p_tri11;
wire signed [31:0] p_tri12, p_tri13, p_tri14, p_tri15;
wire signed [31:0] p_x16, p_x120;       // 打包模块给出的 16S / 120S
wire signed [31:0] f_emit;              // f_r 再打一拍，给对照
wire        [31:0] f16;                 // 16F = f_r<<4
wire        [31:0] inc_w;               // 16F+120S，与 fx 同拍
wire [31:0] fx0, fx1, fx2, fx3, fx4, fx5, fx6, fx7;
                                        // 16 路 i·F
wire [31:0] fx8, fx9, fx10, fx11, fx12, fx13, fx14, fx15;


// =============================================================================
// 二次项打包与 16 路 i·F
// =============================================================================

da_sig_tri_pack u_pack (

    .clk        (clk),                   // 工作时钟
    .rst_sync   (rst_sync),              // 同步复位
    .slope      (slope_r),               // 已打拍的每样点斜率 S
    .t0         (p_tri0),                // i=0：0
    .t1         (p_tri1),                // i=1：0
    .t2         (p_tri2),                // i=2：1S
    .t3         (p_tri3),                // i=3：3S
    .t4         (p_tri4),                // i=4：6S
    .t5         (p_tri5),                // i=5：10S
    .t6         (p_tri6),                // i=6：15S
    .t7         (p_tri7),                // i=7：21S
    .t8         (p_tri8),                // i=8：28S
    .t9         (p_tri9),                // i=9：36S
    .t10        (p_tri10),               // i=10：45S
    .t11        (p_tri11),               // i=11：55S
    .t12        (p_tri12),               // i=12：66S
    .t13        (p_tri13),               // i=13：78S
    .t14        (p_tri14),               // i=14：91S
    .t15        (p_tri15),               // i=15：105S
    .slope_x16  (p_x16),                 // 16S
    .slope_x120 (p_x120)                 // 120S = 0+1+...+15 的二次系数和

);


da_sig_fx16 u_fx (

    .clk        (clk),                   // 工作时钟
    .rst_sync   (rst_sync),              // 同步复位
    .start_edge (start_edge),            // 启动时 16 路乘积清零
    .f_in       (f_r),                   // 当前锁存的瞬时 FTW
    .s120       (tri_on ? slope_x120 : 32'sd0),
                                         // 无二次项时 120S 当 0
    .f_emit     (f_emit),                // f_in 再打一拍
    .f16        (f16),                   // 16F
    .inc        (inc_w),                 // 16F+120S，phase_base 只做 2 输入加
    .fx0        (fx0),                   // 0·F
    .fx1        (fx1),                   // 1·F
    .fx2        (fx2),                   // 2·F
    .fx3        (fx3),                   // 3·F
    .fx4        (fx4),                   // 4·F
    .fx5        (fx5),                   // 5·F
    .fx6        (fx6),                   // 6·F
    .fx7        (fx7),                   // 7·F
    .fx8        (fx8),                   // 8·F
    .fx9        (fx9),                   // 9·F
    .fx10       (fx10),                  // 10·F
    .fx11       (fx11),                  // 11·F
    .fx12       (fx12),                  // 12·F
    .fx13       (fx13),                  // 13·F
    .fx14       (fx14),                  // 14·F
    .fx15       (fx15)                   // 15·F

);


wire signed [31:0] t0  = tri_on ? lfm_tri0  : 32'sd0;
                                        // 二次项选通；tri_on=0 时全 0
wire signed [31:0] t1  = tri_on ? lfm_tri1  : 32'sd0;
wire signed [31:0] t2  = tri_on ? lfm_tri2  : 32'sd0;
wire signed [31:0] t3  = tri_on ? lfm_tri3  : 32'sd0;
wire signed [31:0] t4  = tri_on ? lfm_tri4  : 32'sd0;
wire signed [31:0] t5  = tri_on ? lfm_tri5  : 32'sd0;
wire signed [31:0] t6  = tri_on ? lfm_tri6  : 32'sd0;
wire signed [31:0] t7  = tri_on ? lfm_tri7  : 32'sd0;
wire signed [31:0] t8  = tri_on ? lfm_tri8  : 32'sd0;
wire signed [31:0] t9  = tri_on ? lfm_tri9  : 32'sd0;
wire signed [31:0] t10 = tri_on ? lfm_tri10 : 32'sd0;
wire signed [31:0] t11 = tri_on ? lfm_tri11 : 32'sd0;
wire signed [31:0] t12 = tri_on ? lfm_tri12 : 32'sd0;
wire signed [31:0] t13 = tri_on ? lfm_tri13 : 32'sd0;
wire signed [31:0] t14 = tri_on ? lfm_tri14 : 32'sd0;
wire signed [31:0] t15 = tri_on ? lfm_tri15 : 32'sd0;


// =============================================================================
// 斜率与二次项选通打拍
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        slope_r      <= 32'sd0;          // 斜率清零
        need_tri_r   <= 1'b0;            // 选通流水清零
        need_tri_rr  <= 1'b0;
        need_tri_rrr <= 1'b0;

    end else begin

        slope_r      <= slope_word;      // 拍 0：锁存 S，切断类型译码
        need_tri_r   <= need_tri;        // 拍 1
        need_tri_rr  <= need_tri_r;      // 拍 2，对齐 tri_pack 第一级
        need_tri_rrr <= need_tri_rr;     // 拍 3，对齐 tri_pack 输出

    end

end


// =============================================================================
// 装载 16 路二次项与 16S / 120S
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        tri_on <= 1'b0;                  // 默认不加二次项
        slope_x16  <= 32'sd0;            // 16S 清零
        slope_x120 <= 32'sd0;            // 120S 清零
        lfm_tri0  <= 32'sd0; lfm_tri1  <= 32'sd0;
        lfm_tri2  <= 32'sd0; lfm_tri3  <= 32'sd0;
        lfm_tri4  <= 32'sd0; lfm_tri5  <= 32'sd0;
        lfm_tri6  <= 32'sd0; lfm_tri7  <= 32'sd0;
        lfm_tri8  <= 32'sd0; lfm_tri9  <= 32'sd0;
        lfm_tri10 <= 32'sd0; lfm_tri11 <= 32'sd0;
        lfm_tri12 <= 32'sd0; lfm_tri13 <= 32'sd0;
        lfm_tri14 <= 32'sd0; lfm_tri15 <= 32'sd0;

    end else begin

        tri_on <= need_tri_rrr;          // 与二次项工作寄存器同拍

        if (need_tri_rrr) begin          // 需要二次项时装入打包结果

            lfm_tri0  <= p_tri0;  lfm_tri1  <= p_tri1;
            lfm_tri2  <= p_tri2;  lfm_tri3  <= p_tri3;
            lfm_tri4  <= p_tri4;  lfm_tri5  <= p_tri5;
            lfm_tri6  <= p_tri6;  lfm_tri7  <= p_tri7;
            lfm_tri8  <= p_tri8;  lfm_tri9  <= p_tri9;
            lfm_tri10 <= p_tri10; lfm_tri11 <= p_tri11;
            lfm_tri12 <= p_tri12; lfm_tri13 <= p_tri13;
            lfm_tri14 <= p_tri14; lfm_tri15 <= p_tri15;
            slope_x16  <= p_x16;         // 16S，回给 LFM/NLFM
            slope_x120 <= p_x120;        // 120S，加进相位基增量

        end else begin                   // CW / 调相 / 正弦 NLFM：二次项全 0

            lfm_tri0  <= 32'sd0; lfm_tri1  <= 32'sd0;
            lfm_tri2  <= 32'sd0; lfm_tri3  <= 32'sd0;
            lfm_tri4  <= 32'sd0; lfm_tri5  <= 32'sd0;
            lfm_tri6  <= 32'sd0; lfm_tri7  <= 32'sd0;
            lfm_tri8  <= 32'sd0; lfm_tri9  <= 32'sd0;
            lfm_tri10 <= 32'sd0; lfm_tri11 <= 32'sd0;
            lfm_tri12 <= 32'sd0; lfm_tri13 <= 32'sd0;
            lfm_tri14 <= 32'sd0; lfm_tri15 <= 32'sd0;
            slope_x16  <= 32'sd0;
            slope_x120 <= 32'sd0;

        end

    end

end


// =============================================================================
// 相位基推进：φ_base += 16F + 120S
// =============================================================================

always @(posedge clk) begin

    if (rst_sync) begin

        phase_base <= 32'd0;             // 相位基清零
        f_r        <= 32'sd0;            // 瞬时频率清零
        user_d     <= 1'b0;              // 输出门关闭
        valid_d    <= 1'b0;              // 相位无效
        active_r   <= 1'b0;              // 未运行
        pb_phi     <= 32'd0;             // 基相+调相清零
        inc_r      <= 32'd0;             // 增量清零
        ft0  <= 32'd0; ft1  <= 32'd0; ft2  <= 32'd0; ft3  <= 32'd0;
        ft4  <= 32'd0; ft5  <= 32'd0; ft6  <= 32'd0; ft7  <= 32'd0;
        ft8  <= 32'd0; ft9  <= 32'd0; ft10 <= 32'd0; ft11 <= 32'd0;
        ft12 <= 32'd0; ft13 <= 32'd0; ft14 <= 32'd0; ft15 <= 32'd0;

    end else if (start_edge) begin

        phase_base <= phase_init;        // 启动时装初相
        user_d   <= 1'b0;                // 启动本拍还不出波
        valid_d  <= 1'b0;                // 流水排空
        active_r <= 1'b0;                // 等 PRI 第一 tick
        pb_phi   <= phase_init;          // 启动时调相尚未叠上
        inc_r    <= 32'd0;               // 增量清零

    end else if (clear_phase_edge) begin

        phase_base <= phase_init;        // 清相：相位基回到初相
        pb_phi     <= phase_init + phi_code;
                                         // 清相后立即叠上当前调相字
        active_r   <= running && wave_ready;
                                         // 清相不停跑，保持活跃

    end else begin

        active_r <= running && wave_ready;
                                         // 运行且时间轴就绪才算活跃

        if (running && wave_ready) begin

            user_d  <= pulse_gate;       // 门控再打一拍，对齐相位
            valid_d <= 1'b1;             // 从这时起相位有效

            ft0  <= fx0  + t0;  ft1  <= fx1  + t1;
            ft2  <= fx2  + t2;  ft3  <= fx3  + t3;
            ft4  <= fx4  + t4;  ft5  <= fx5  + t5;
            ft6  <= fx6  + t6;  ft7  <= fx7  + t7;
            ft8  <= fx8  + t8;  ft9  <= fx9  + t9;
            ft10 <= fx10 + t10; ft11 <= fx11 + t11;
            ft12 <= fx12 + t12; ft13 <= fx13 + t13;
            ft14 <= fx14 + t14; ft15 <= fx15 + t15;
                                         // 各路相对相位 = i·F + i(i-1)/2·S

            pb_phi <= phase_base + phi_code;
                                         // 基相叠调相，下一拍与 ft_i 相加
            inc_r  <= inc_w;             // 锁存 16F+120S

            if (pulse_gate)              // 只在脉内推进相位基
                phase_base <= phase_base + inc_w;
                                         // φ_base += 16F + 120S
                                         // inc_w 已在 fx16 里把 16F 和 120S 加好

            f_r <= f_next;               // 锁存下一拍 F_inst

        end else begin

            valid_d <= 1'b0;             // 未运行时相位无效
            user_d  <= 1'b0;             // 输出门关闭
            f_r     <= f_idle;           // 空闲仍跟上频率，避免再启动跳变

        end

    end

end


// =============================================================================
// 16 路相位输出：φ_i = pb_phi + ft_i
// =============================================================================

da_sig_phase_out u_out (

    .clk         (clk),                  // 工作时钟
    .rst_sync    (rst_sync),             // 同步复位
    .start_edge  (start_edge),           // 启动时相位输出清零
    .valid_d     (valid_d),              // 上一拍有效，本拍对齐到 phase
    .user_d      (user_d),               // 上一拍脉宽门
    .pb_phi      (pb_phi),               // phase_base + phi_code
    .ft0         (ft0),                  // 第 0 路相对相位
    .ft1         (ft1),                  // 第 1 路相对相位
    .ft2         (ft2),
    .ft3         (ft3),
    .ft4         (ft4),
    .ft5         (ft5),
    .ft6         (ft6),
    .ft7         (ft7),
    .ft8         (ft8),
    .ft9         (ft9),
    .ft10        (ft10),
    .ft11        (ft11),
    .ft12        (ft12),
    .ft13        (ft13),
    .ft14        (ft14),
    .ft15        (ft15),                 // 第 15 路相对相位
    .phase_valid (phase_valid),          // 16 路相位是否有效
    .phase_user  (phase_user),           // 与相位对齐的脉宽门
    .phase_r_0   (phase_r_0),            // 样点 0 的 DDS 相位
    .phase_r_1   (phase_r_1),
    .phase_r_2   (phase_r_2),
    .phase_r_3   (phase_r_3),
    .phase_r_4   (phase_r_4),
    .phase_r_5   (phase_r_5),
    .phase_r_6   (phase_r_6),
    .phase_r_7   (phase_r_7),
    .phase_r_8   (phase_r_8),
    .phase_r_9   (phase_r_9),
    .phase_r_10  (phase_r_10),
    .phase_r_11  (phase_r_11),
    .phase_r_12  (phase_r_12),
    .phase_r_13  (phase_r_13),
    .phase_r_14  (phase_r_14),
    .phase_r_15  (phase_r_15)            // 样点 15 的 DDS 相位

);

endmodule
