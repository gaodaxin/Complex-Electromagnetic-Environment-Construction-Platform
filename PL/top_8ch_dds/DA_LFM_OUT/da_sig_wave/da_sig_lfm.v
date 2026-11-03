// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_lfm.v
// 功能：生成逐拍推进的线性调频频偏，按脉冲或扫频周期回绕到起始频偏。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_lfm
// 功能: 产生线性调频支路的瞬时频偏 F_mod。
// 行为: 新脉冲从 fm_start 开始，每个 fabric 拍累加 slope_x16；扫频结束后
//       保持端点，下一次 pulse_rise 重新装载起点。
// 单位: 输入和输出均为 32 bit FTW 域，计数长度按并行样点拍计算。
// =============================================================================

module da_sig_lfm (

    input  wire        clk,                  // FPGA 工作时钟
    input  wire        rst_sync,             // 同步复位，高有效
    input  wire        start_edge,           // 启动脉冲，重新装载 fm_start 并清样点计数

    input  wire        active,               // 累加器有效运行时，本模块才更新状态
    input  wire        pulse_rise,           // 新脉冲开始事件，每一发 LFM 重新从 fm_start 开始
    input  wire        pulse_gate,           // 脉宽门控，只有脉冲有效区内才继续扫频
    input  wire        enable,               // LFM 使能，上级选中 LFM 时为 1

    input  wire signed [31:0] fm_start,      // LFM 起始频偏 FTW，例如 ±B/2
    input  wire signed [31:0] slope_x16,     // 每个 fabric 拍的频偏增量 = 16*S
    input  wire [31:0] n_sample,             // 一次完整 LFM 对应的总 DAC 样点数

    output reg  signed [31:0] fm_offset,     // 当前 LFM 频偏 F_mod
    output wire signed [31:0] fm_next        // 当前频偏预览，本版本直接等于 fm_offset

);


reg [31:0] n_cnt;                            // 当前 LFM 已推进的样点数，每次 step 加 16

reg        do_rst;                           // 已寄存事件：下一拍执行新脉冲重新开始
reg        do_wrap;                          // 已寄存事件：下一拍执行扫频周期结束并回起点
reg        do_step;                          // 已寄存事件：下一拍继续线性扫频


assign fm_next = fm_offset;                  // 当前版本不额外预测，直接输出已寄存的 fm_offset


// =============================================================================
// LFM 主状态机
// 本拍根据 pulse_rise / n_cnt 预判 do_*，下一拍再真正更新 fm_offset 和 n_cnt
// =============================================================================

always @(posedge clk) begin

    if (rst_sync || start_edge) begin        // 复位或整机重新启动

        n_cnt <= 32'd0;                      // LFM 样点计数从 0 重新开始

        fm_offset <= enable ?                // 启动时装载起始频偏或清 0
                     fm_start :               // 已使能 LFM 则装 fm_start
                     32'sd0;                 // 未使能时频偏清 0

        do_rst  <= 1'b0;                     // 清除上一拍新脉冲事件
        do_wrap <= 1'b0;                     // 清除上一拍回绕事件
        do_step <= 1'b0;                     // 清除上一拍扫频步进事件

    end


    else if (active) begin                   // 仅在 accum 有效运行时推进

        do_rst <= pulse_rise &&              // 预判：新一发开始则下一拍重装 fm_start
                  enable;                    // 必须使能 LFM 才响应 pulse_rise


        do_wrap <= pulse_gate &&             // 预判：仍在脉冲内才可能回绕
                   enable &&                 // 必须使能 LFM
                   !pulse_rise &&            // 新脉冲优先走 do_rst，本拍不 wrap
                   ((n_cnt + 32'd16) >= n_sample);
                                             // 下一组 16 样点已达/越过 n_sample


        do_step <= pulse_gate &&             // 预判：仍在脉冲内才继续扫频
                   enable &&                 // 必须使能 LFM
                   !pulse_rise &&            // 新脉冲优先走 do_rst，本拍不 step
                   ((n_cnt + 32'd16) < n_sample);
                                             // 下一组 16 样点尚未到周期末


        if (do_rst || do_wrap) begin         // 执行上一拍预判的复位/回绕

            fm_offset <= fm_start;           // 新脉冲或周期结束：频偏回到扫描起点
            n_cnt     <= 32'd0;              // 样点位置重新从 0 开始

        end

        else if (do_step) begin              // 执行上一拍预判的正常扫频

            fm_offset <= fm_offset +         // 每拍跨 16 个样点，一次增加 16*S
                         slope_x16;

            n_cnt <= n_cnt +                 // 当前已经继续推进 16 个 DAC 样点
                     32'd16;

        end                                  // 三个 do_* 都为 0 时保持 fm_offset/n_cnt 不变

    end

end

endmodule
