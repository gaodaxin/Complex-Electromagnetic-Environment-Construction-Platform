// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_beam_cdc.v
// 功能：使用保持式请求/应答将方向图写入和范围配置原子传到波形域，并反传可接收状态。
// ============================================================================
`timescale 1ns/1ps
//=============================================================================
// da_sig_beam_cdc
//-----------------------------------------------------------------------------
// 功能：波束配置跨时钟域传输（保持式 bundled-data 请求/应答）。
//
// 主要机制：
//   1. 源时钟域（src_clk）：
//        - 监控 write_tgl / range_tgl 是否变化；
//        - 若 range_tgl 变化，捕获 {1'b1, waddr, wdata, a0, a1} 到 held；
//        - 否则若 write_tgl 变化，捕获 {1'b0, waddr, wdata, a0, a1} 到 held；
//        - 翻转 request 通知目的域；
//        - source_ready = (request == ack_sync[2]) &&
//                          write_seen == write_tgl &&
//                          range_seen == range_tgl。
//   2. 目的时钟域（clk）：
//        - 三级同步 request 到 request_sync；
//        - 检测到新的 request 且 destination_ready 有效时：
//            捕获 held 到 captured，翻转 pending；
//        - pending 有效且 destination_ready 有效时：
//            根据 captured[62] 区分 range_pulse / write_pulse，
//            置 ack = request_seen，清 pending。
//
// 关键设计：
//   - Kind（range/write）与 payload 同拍捕获，1 拍后本地解码，
//     不作为直接异步脉冲使用；
//   - held 与 captured 使用 DONT_TOUCH，防止被优化掉；
//   - ack_sync / request_sync 使用 ASYNC_REG，防止亚稳态；
//   - source_ready 保证源域在目的域未应答前不发起新传输。
//
// 说明：
//   本模块全部为时序逻辑，源域和目的域各有一个 always 块。
//=============================================================================
module da_sig_beam_cdc(
    input  wire        src_clk,           // 源时钟域时钟
    input  wire        src_rst,           // 源域复位，高有效
    input  wire        clk,               // 目的时钟域时钟
    input  wire        rst_sync,          // 目的域同步复位，高有效

    input  wire        write_tgl,         // 写请求 toggle（源域）
    input  wire        range_tgl,         // 范围请求 toggle（源域）
    input  wire [9:0]  waddr,             // 写地址
    input  wire [9:0]  a0,                // 范围起点
    input  wire [9:0]  a1,                // 范围终点
    input  wire [31:0] wdata,             // 写数据

    input  wire        destination_ready, // 目的域就绪

    output wire        source_ready,      // 源域可发起新传输

    output reg         write_pulse = 0,   // 目的域写脉冲，单拍
    output reg         range_pulse = 0,   // 目的域范围脉冲，单拍

    output wire [9:0]  dst_waddr,         // 目的域写地址
    output wire [9:0]  dst_a0,            // 目的域范围起点
    output wire [9:0]  dst_a1,            // 目的域范围终点
    output wire [31:0] dst_wdata          // 目的域写数据
);

//=============================================================================
// 源域内部寄存器
//=============================================================================
reg write_seen = 0;      // 已见过的 write_tgl
reg range_seen = 0;      // 已见过的 range_tgl
reg request    = 0;      // 请求 toggle
reg ack        = 0;      // 应答 toggle（从目的域同步回来）

// 保持寄存器：held 保存待发送数据，captured 保存已捕获数据
// 使用 DONT_TOUCH 防止综合工具优化
(* DONT_TOUCH="TRUE" *) reg [62:0] held     = 0;
(* DONT_TOUCH="TRUE" *) reg [62:0] captured = 0;

// 异步同步链
(* ASYNC_REG="TRUE" *) reg [2:0] ack_sync     = 0;   // ack 三级同步
(* ASYNC_REG="TRUE" *) reg [2:0] request_sync = 0;   // request 三级同步

//=============================================================================
// 目的域内部寄存器
//=============================================================================
reg request_seen = 0;    // 已见过的 request
reg pending      = 0;    // 有数据待处理

//=============================================================================
// 组合逻辑：输出映射与源域就绪
//-----------------------------------------------------------------------------
// 功能：
//   1. dst_waddr/dst_wdata/dst_a0/dst_a1 从 captured 中拆出：
//        captured[62]     —— kind（1=range，0=write）
//        captured[61:52]  —— waddr
//        captured[51:20]  —— wdata
//        captured[19:10]  —— a0
//        captured[9:0]    —— a1
//   2. source_ready = (request == ack_sync[2]) &&
//                     write_seen == write_tgl &&
//                     range_seen == range_tgl：
//        表示上次请求已被应答，且当前 toggle 与已发送一致。
//
// 该部分为纯组合逻辑。
//=============================================================================
assign {dst_waddr, dst_wdata, dst_a0, dst_a1} = captured[61:0];
assign source_ready = (request == ack_sync[2]) &&
                      write_seen == write_tgl &&
                      range_seen == range_tgl;

//=============================================================================
// 时序逻辑：源时钟域
//-----------------------------------------------------------------------------
// 功能：
//   1. src_rst 时清零所有源域寄存器；
//   2. 每拍将 ack 三级同步到 ack_sync；
//   3. 当 request == ack_sync[2]（上次请求已被应答）时：
//        - 若 range_tgl 变化，捕获 range 请求到 held，
//          翻转 request 通知目的域；
//        - 否则若 write_tgl 变化，捕获 write 请求到 held，
//          翻转 request 通知目的域。
//
// 说明：
//   held 的拼接格式：{kind, waddr, wdata, a0, a1}，共 63bit。
//
// 该 always 块对 src_clk 上升沿敏感，属于源域时序逻辑。
//=============================================================================
always @(posedge src_clk) begin
    if (src_rst) begin
        write_seen  <= 0;
        range_seen  <= 0;
        request     <= 0;
        ack_sync    <= 0;
        held        <= 0;
    end
    else begin
        // ack 三级同步
        ack_sync <= {ack_sync[1:0], ack};

        // 上次请求已被应答，可以发起新请求
        if (request == ack_sync[2]) begin
            if (range_seen != range_tgl) begin
                // 范围请求：kind = 1
                held       <= {1'b1, waddr, wdata, a0, a1};
                range_seen <= range_tgl;
                request    <= ~request;
            end
            else if (write_seen != write_tgl) begin
                // 写请求：kind = 0
                held       <= {1'b0, waddr, wdata, a0, a1};
                write_seen <= write_tgl;
                request    <= ~request;
            end
        end
    end
end

//=============================================================================
// 时序逻辑：目的时钟域
//-----------------------------------------------------------------------------
// 功能：
//   1. rst_sync 时清零所有目的域寄存器；
//   2. 每拍将 request 三级同步到 request_sync；
//   3. 默认清零 write_pulse、range_pulse（单拍脉冲）；
//   4. 若 pending 有效且 destination_ready 有效：
//        - range_pulse = captured[62]
//        - write_pulse = !captured[62]
//        - ack = request_seen
//        - pending = 0
//   5. 否则若 request_sync[2] != request_seen 且 destination_ready 有效：
//        - captured <= held
//        - request_seen <= request_sync[2]
//        - pending <= 1
//
// 说明：
//   kind（range/write）与 payload 同拍捕获，1 拍后本地解码，
//   不作为直接异步脉冲使用。
//
// 该 always 块对 clk 上升沿敏感，属于目的域时序逻辑。
//=============================================================================
always @(posedge clk) begin
    if (rst_sync) begin
        request_sync <= 0;
        request_seen <= 0;
        ack          <= 0;
        write_pulse  <= 0;
        range_pulse  <= 0;
        captured     <= 0;
        pending      <= 0;
    end
    else begin
        // request 三级同步
        request_sync <= {request_sync[1:0], request};

        // 默认单拍脉冲清零
        write_pulse <= 0;
        range_pulse <= 0;

        if (pending && destination_ready) begin
            // 处理已捕获数据，按 kind 区分脉冲类型
            range_pulse <= captured[62];
            write_pulse <= !captured[62];
            ack         <= request_seen;
            pending     <= 0;
        end
        else if (request_sync[2] != request_seen && destination_ready) begin
            // 检测到新请求，捕获 held 到 captured
            captured     <= held;
            request_seen <= request_sync[2];
            pending      <= 1;
        end
    end
end

endmodule
