// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_nlfm_tri.v
// 功能：产生三角形非线性调频轨迹，控制频偏步进、方向翻转和脉冲重新起始。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_nlfm_tri
// 功能: 产生三角折返型 NLFM 频偏，在上下限之间按固定斜率往返。
// 行为: pulse_rise 重装起点和方向，运行中按 16 个并行样点的跨度更新。
// =============================================================================

module da_sig_nlfm_tri (

    input  wire        clk,                  // FPGA 工作时钟
    input  wire        rst_sync,             // 同步复位，高有效
    input  wire        start_edge,           // 启动脉冲，重新装载 fm_start 并清样点计数

    input  wire        active,               // 累加器有效运行时，本模块才更新状态
    input  wire        pulse_rise,           // 新脉冲开始事件，每一发三角 NLFM 重新从 fm_start 开始
    input  wire        pulse_gate,           // 脉宽门控，只有脉冲有效区内才继续扫频
    input  wire        enable,               // 三角 NLFM 使能，上级选中 NLFM_TRI 时为 1

    input  wire signed [31:0] fm_start,      // 三角扫描起点频偏，正向通常为 -B/2，反向可为 +B/2
    input  wire signed [31:0] slope_x16,     // 每个 fabric 拍的频偏变化量 = 16×单样点斜率
                                             // 三角模式上级已把单样点斜率调整为 2S
    input  wire [31:0] n_sample,             // 一个完整三角调制周期的总 DAC 样点数

    output reg  signed [31:0] fm_mod,        // 当前三角 NLFM 频偏 F_mod
    output wire signed [31:0] fm_next        // 当前频偏预览，本版本直接等于 fm_mod

);


reg [31:0] n_cnt;                            // 当前三角周期已推进的样点数，每次 step 加 16

reg        do_rst;                           // 已寄存事件：下一拍执行新脉冲重新开始
reg        do_wrap;                          // 已寄存事件：下一拍执行周期结束并回起点
reg        do_step;                          // 已寄存事件：下一拍继续三角扫频
reg        down_r;                           // 已寄存方向：0=上坡加斜率，1=下坡减斜率


wire [31:0] half =                           // 半周期位置 = floor(N/2)
        {1'b0, n_sample[31:1]};              // 右移 1 位得到 N/2，并显式补 0 保持 32 bit

assign fm_next = fm_mod;                     // 当前版本不额外预测，直接输出已寄存的 fm_mod


// =============================================================================
// 三角 NLFM 主状态机
// 本拍根据 pulse_rise / n_cnt 预判 do_* 和 down_r，下一拍再真正更新 fm_mod 和 n_cnt
// =============================================================================

always @(posedge clk) begin

    if (rst_sync || start_edge) begin        // 复位或整机重新启动

        n_cnt <= 32'd0;                      // 三角样点计数从 0 重新开始

        fm_mod <= enable ?                   // 启动时装载起始频偏或清 0
                  fm_start :                  // 已使能三角 NLFM 则装 fm_start
                  32'sd0;                    // 未使能时频偏清 0

        do_rst  <= 1'b0;                     // 清除上一拍新脉冲事件
        do_wrap <= 1'b0;                     // 清除上一拍回绕事件
        do_step <= 1'b0;                     // 清除上一拍扫频步进事件

        down_r  <= 1'b0;                     // 初始方向为上坡：fm_mod 增加

    end


    else if (active) begin                   // 仅在 accum 有效运行时推进

        do_rst <= pulse_rise &&              // 预判：新一发开始则下一拍重装 fm_start
                  enable;                    // 必须使能三角 NLFM 才响应 pulse_rise


        do_wrap <= pulse_gate &&             // 预判：仍在脉冲内才可能回绕
                   enable &&                 // 必须使能三角 NLFM
                   !pulse_rise &&            // 新脉冲优先走 do_rst，本拍不 wrap
                   ((n_cnt + 32'd16) >= n_sample);
                                             // 下一组 16 样点已达/越过 n_sample


        do_step <= pulse_gate &&             // 预判：仍在脉冲内才继续扫频
                   enable &&                 // 必须使能三角 NLFM
                   !pulse_rise &&            // 新脉冲优先走 do_rst，本拍不 step
                   ((n_cnt + 32'd16) < n_sample);
                                             // 下一组 16 样点尚未到周期末


        down_r <= (n_cnt >= half) &&         // 预判：n_cnt 进入后半周期则改为下坡
                  (half != 32'd0);           // half=0 时禁止下坡，避免 N 过小误判


        if (do_rst || do_wrap) begin         // 执行上一拍预判的复位/回绕

            fm_mod <= fm_start;              // 新脉冲或周期结束：频偏回到扫描起点
            n_cnt  <= 32'd0;                 // 样点位置重新从 0 开始

        end

        else if (do_step) begin              // 执行上一拍预判的正常扫频

            fm_mod <= down_r ?               // 使用上一拍锁存的方向
                      (fm_mod - slope_x16) : // 下坡：每拍跨 16 样点，一次减去 16×2S
                      (fm_mod + slope_x16);  // 上坡：每拍跨 16 样点，一次增加 16×2S

            n_cnt <= n_cnt +                 // 当前已经继续推进 16 个 DAC 样点
                     32'd16;

        end                                  // 三个 do_* 都为 0 时保持 fm_mod/n_cnt 不变

    end

end

endmodule
