// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_run_ctrl.v
// 功能：管理加载、参数计算、辅助链就绪和挂起启动；换参先静音，停止事件取消运行及挂起动作。
// ============================================================================
`timescale 1ns/1ps
//=============================================================================
// da_sig_run_ctrl
//-----------------------------------------------------------------------------
// 功能：运行控制器，管理配置加载与计算启动/停止的时序。
//
// 主要职责：
//   1. 配置加载（cfg_load_edge / load_request）：
//        - 拉高 config_complete 之前，先静音 running 输出；
//        - 进入 loading 状态，等待 calc_done 与 aux_ready；
//        - 完成一次计算后，根据 calc_err 判断是否可以继续；
//   2. 计算启动（start_edge）：
//        - 若配置完成、未加载、辅助就绪、无错误、use_new 有效，
//          则启动计算（calc_go = 1，running = 1，start_n = 1）；
//        - 否则把 start_edge 挂起为 start_hold，等待条件满足；
//   3. 计算停止（stop_edge）：
//        - 无条件取消运行，清零 running、start_n、start_hold；
//   4. 计算结果（calc_done）：
//        - 记录 done_seen 与 error_seen；
//        - loading 结束条件：done_seen && aux_ready && !calc_busy。
//
// 关键设计：
//   - load_request 必须在 bundled-data 捕获之前静音；
//   - start 在 calc_go/busy 间隙期间不能复用上一次计算结果；
//   - stop 总是无条件取消正在进行的运行；
//   - 错误码 error_seen 非零时，start_hold 被清零，不允许自动续跑。
//
// 该模块全部为时序逻辑，对 clk 上升沿敏感。
//=============================================================================
module da_sig_run_ctrl (
    input  wire       clk,              // 系统时钟
    input  wire       rst_sync,         // 同步复位，高有效
    input  wire       use_new,          // 使用新配置标志，必须为 1 才允许启动
    input  wire       cfg_load_edge,    // 配置加载边沿，触发加载流程
    input  wire       start_edge,       // 启动边沿
    input  wire       stop_edge,        // 停止边沿
    input  wire       load_request,     // 加载请求，先于 bundled-data 捕获静音
    input  wire       aux_ready,        // 辅助模块就绪
    input  wire       calc_busy,        // 计算忙
    input  wire       calc_done,        // 计算完成
    input  wire [7:0] calc_err,         // 计算错误码

    output reg        calc_go      = 0, // 计算启动脉冲，单拍
    output reg        running      = 0, // 运行中标志
    output reg        start_n      = 0, // 启动脉冲，单拍
    output reg        config_complete = 1 // 配置完成标志
);

//=============================================================================
// 内部寄存器
//=============================================================================
reg       start_hold = 0;      // 挂起的启动请求
reg       loading    = 0;      // 加载中标志
reg       done_seen  = 0;      // 计算完成标志（锁存）
reg [7:0] error_seen = 0;      // 计算错误码（锁存）

//=============================================================================
// 时序逻辑：运行控制状态机
//-----------------------------------------------------------------------------
// 功能：
//   复位时清零所有输出和内部状态。
//   每个时钟沿默认清零 calc_go、start_n，形成单拍脉冲。
//
//   加载路径：
//     - load_request 有效：
//         running=0，config_complete=0，loading=1，
//         done_seen=0，start_hold=start_edge；
//     - cfg_load_edge 有效：
//         running=0，config_complete=0，loading=1，
//         done_seen=0，calc_go=1（发起一次计算）；
//     - calc_done 有效：
//         done_seen=1，error_seen=calc_err；
//     - loading && done_seen && aux_ready && !calc_busy：
//         config_complete=1，loading=0；
//         若 error_seen != 0，则清除 start_hold（有错不续跑）。
//
//   启动路径：
//     - start_edge && !stop_edge && !load_request：
//         若 config_complete && !loading && aux_ready
//            && error_seen == 0 && use_new：
//             running=1，start_n=1；
//         否则挂起 start_hold=1；
//     - start_hold 满足条件时（配置完成、未加载、辅助就绪、
//       无错误、use_new、无新请求）：
//         running=1，start_n=1，start_hold=0。
//
//   停止路径：
//     - stop_edge：running=0，start_n=0，start_hold=0。
//
// 该 always 块对 clk 上升沿敏感，属于时序逻辑。
//=============================================================================
always @(posedge clk) begin
    if (rst_sync) begin
        calc_go          <= 0;
        running          <= 0;
        start_n          <= 0;
        config_complete  <= 1;
        start_hold       <= 0;
        loading          <= 0;
        done_seen        <= 0;
        error_seen       <= 0;
    end
    else begin
        //---------------------------------------------------------------------
        // 默认输出单拍脉冲清零
        //---------------------------------------------------------------------
        calc_go <= 0;
        start_n <= 0;

        //---------------------------------------------------------------------
        // 加载路径 1：load_request 先于 bundled-data 捕获静音
        //---------------------------------------------------------------------
        if (load_request) begin
            running         <= 0;
            config_complete <= 0;
            loading         <= 1;
            done_seen       <= 0;
            start_hold      <= start_edge;
        end

        //---------------------------------------------------------------------
        // 加载路径 2：cfg_load_edge 发起一次计算
        //---------------------------------------------------------------------
        if (cfg_load_edge) begin
            running         <= 0;
            config_complete <= 0;
            loading         <= 1;
            done_seen       <= 0;
            calc_go         <= 1;
        end

        //---------------------------------------------------------------------
        // 计算完成：锁存 done_seen 与 error_seen
        //---------------------------------------------------------------------
        if (calc_done) begin
            done_seen  <= 1;
            error_seen <= calc_err;
        end

        //---------------------------------------------------------------------
        // 加载结束条件：done_seen && aux_ready && !calc_busy
        //   有错时清除 start_hold，不允许自动续跑。
        //---------------------------------------------------------------------
        if (loading && done_seen && aux_ready && !calc_busy) begin
            config_complete <= 1;
            loading         <= 0;
            if (error_seen != 0)
                start_hold <= 0;
        end

        //---------------------------------------------------------------------
        // 启动路径 1：start_edge 直接启动
        //---------------------------------------------------------------------
        if (start_edge && !stop_edge && !load_request) begin
            if (config_complete && !loading && aux_ready &&
                error_seen == 0 && use_new) begin
                running <= 1;
                start_n <= 1;
            end
            else
                start_hold <= 1;
        end

        //---------------------------------------------------------------------
        // 启动路径 2：start_hold 满足条件后启动
        //---------------------------------------------------------------------
        if (start_hold && config_complete && !loading && aux_ready &&
            error_seen == 0 && use_new && !load_request && !cfg_load_edge) begin
            running    <= 1;
            start_n    <= 1;
            start_hold <= 0;
        end

        //---------------------------------------------------------------------
        // 停止路径：无条件取消
        //---------------------------------------------------------------------
        if (stop_edge) begin
            running    <= 0;
            start_n    <= 0;
            start_hold <= 0;
        end
    end
end

endmodule
