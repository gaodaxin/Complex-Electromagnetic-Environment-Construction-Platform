// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_scan_beam.v
// 功能：维护方向图影子表和生效表，加载时原子复制；按相对目标角和非零范围读出增益。
// ============================================================================
`timescale 1ns/1ps
//=============================================================================
// da_sig_scan_beam
//-----------------------------------------------------------------------------
// 功能：波束扫描查表与目标相对索引生成。
//   本模块维护一张 256 项的波束增益表（gmem）和一份影子表（shadow），
//   支持"待写入方向数据不修改当前生效增益表"的双缓冲机制：
//     - 配置阶段：波束表写入 shadow，并置 shadow_valid 位；
//     - 加载阶段：cfg_load 触发后，每个时钟拷贝 4 个条目，共 64 拍完成；
//                  拷贝期间 config_busy 拉高，运行控制器静音输出；
//     - 运行阶段：根据 scan_idx 与 target_idx 的差，按 [a0,a1] 波束范围
//                  计算相对索引 relative_idx，查表输出增益 g。
//
// 主要数据流：
//   beam_a0/beam_a1  ──▶ shadow_a0/a1 ──▶ a0/a1（生效范围）
//   beam_tab_wdata   ──▶ shadow[0:255] ──▶ gmem[0:255]（生效增益表）
//   target_idx       ──▶ target_r
//   scan_idx         ──▶ relative_idx = scan_idx - target_r
//   relative_idx + a0/a1 ──▶ coff_r ──▶ g
//
// 说明：
//   - 波束范围支持回绕（wrap_rng = a1 < a0）：
//       非回绕：rel ∈ [a0, a1]
//       回绕：  rel ≥ a0 或 rel ≤ a1
//   - 每个表项有固定的数据源和本地写使能，避免可变索引读改写阵列 mux。
//   - load 拷贝期间运行控制器静音输出，保证增益表切换原子性。
//=============================================================================
module da_sig_scan_beam(
    input  wire        clk,              // 系统时钟
    input  wire        rst_sync,         // 同步复位，高有效
    input  wire        cfg_load,         // 配置加载脉冲，触发双缓冲拷贝

    input  wire [9:0]  target_idx,       // 目标索引，加载时锁存到 target_r
    input  wire [9:0]  scan_idx,         // 当前扫描索引，用于计算相对索引

    input  wire        beam_range_we,    // 波束范围写使能
    input  wire [9:0]  beam_a0,          // 波束范围起点，写入 shadow_a0
    input  wire [9:0]  beam_a1,          // 波束范围终点，写入 shadow_a1

    input  wire        beam_tab_we,      // 波束表写使能
    input  wire [9:0]  beam_tab_waddr,   // 波束表写地址（字节地址，每项 4 字节）
    input  wire [31:0] beam_tab_wdata,   // 波束表写数据，一次写入 4 个 8bit 增益

    output wire [9:0]  relative_idx,     // 相对索引 = scan_idx - target_r
    output wire [7:0]  g,                // 输出增益，未加载时输出 8'd255
    output wire        beam_valid,       // 波束表有效（table_loaded）
    output reg         config_busy = 0   // 配置拷贝忙标志，拷贝期间为 1
);

//=============================================================================
// 内部常量
//=============================================================================
localparam BEAM_N = 256;   // 波束表项数

//=============================================================================
// 内部寄存器
//=============================================================================
//---------------------------------------------------------------------------
// 双缓冲表
//---------------------------------------------------------------------------
reg [7:0]   shadow[0:255];      // 影子表：待写入的波束增益
reg [7:0]   gmem[0:255];        // 生效表：当前用于查表的波束增益
reg [255:0] shadow_valid = 0;   // 影子表每项有效标志

//---------------------------------------------------------------------------
// 波束范围（影子 + 生效）
//---------------------------------------------------------------------------
reg [9:0] shadow_a0 = 0, shadow_a1 = 0;   // 影子范围，配置阶段写入
reg [9:0] a0 = 0, a1 = 0;                 // 生效范围，cfg_load 时锁存

//---------------------------------------------------------------------------
// 运行状态
//---------------------------------------------------------------------------
reg [9:0] target_r = 0;         // 锁存的目标索引
reg       wrote_any = 0;        // 是否有任何配置写入
reg       table_loaded = 0;     // 波束表已加载
reg [5:0] copy_word = 0;        // 拷贝节拍计数，0..63，每拍拷贝 4 项
reg       hit_r = 0;            // 命中标志，延迟 1 拍
reg [7:0] coff_r = 0;           // 查表偏移，延迟 1 拍
reg [7:0] g_q = 255;            // 输出增益寄存器

//=============================================================================
// 组合逻辑：相对索引与命中判定
//-----------------------------------------------------------------------------
// 功能：
//   1. rel = scan_idx - target_r，计算相对索引；
//   2. wrap_rng = (a1 < a0)，判断波束范围是否回绕；
//   3. hit 判定：
//        - 非回绕：rel ∈ [a0, a1]
//        - 回绕：  rel ≥ a0 或 rel ≤ a1
//   4. off_raw = 命中时的相对 a0 偏移，用于查表：
//        - 非回绕：rel - a0
//        - 回绕且 rel < a0：1024 - a0 + rel（补偿回绕）
//   5. relative_idx 直连输出 rel；
//   6. beam_valid 直连 table_loaded；
//   7. g = table_loaded ? g_q : 8'd255（未加载时输出满增益 255）。
//
// 该部分为纯组合逻辑，依赖 scan_idx、target_r、a0、a1、table_loaded、g_q。
//=============================================================================
wire [9:0]  rel      = scan_idx - target_r;
wire        wrap_rng = a1 < a0;
wire        hit      = wrap_rng ? ((rel >= a0) || (rel <= a1))
                                : ((rel >= a0) && (rel <= a1));
wire [10:0] off_raw  = wrap_rng && rel < a0
                       ? 11'd1024 - {1'b0, a0} + {1'b0, rel}
                       : {1'b0, rel} - {1'b0, a0};

assign relative_idx = rel;
assign beam_valid   = table_loaded;
assign g            = table_loaded ? g_q : 8'd255;

//=============================================================================
// generate：256 个表项的固定数据源与本地写使能
//-----------------------------------------------------------------------------
// 功能：
//   为每个表项 entry 生成一个独立的时序块：
//     1. 若 beam_range_we 且非 config_busy：
//          清空该表项的 shadow_valid（范围更新时使表项失效）；
//     2. 若 beam_tab_we 且非 config_busy：
//          根据 beam_tab_waddr 低 8bit 与 entry 的偏移关系，
//          从 beam_tab_wdata 的 4 个字节中选择对应字节写入 shadow[entry]，
//          并置 shadow_valid[entry] = 1；
//     3. 若 cfg_load 完成且 config_busy 期间 copy_word == entry/4：
//          把 shadow[entry] 拷贝到 gmem[entry]，
//          若 shadow_valid[entry] 无效则写 0。
//
// 说明：
//   使用 generate 展开 256 份结构相同的逻辑，避免可变索引读改写阵列 mux；
//   每个表项有固定数据源和本地写使能，时序友好。
//=============================================================================
genvar entry;
generate
for (entry = 0; entry < 256; entry = entry + 1) begin : table_entry

    localparam [8:0] ENTRY = entry;
    wire [8:0] addr = {1'b0, beam_tab_waddr[7:0]};

    always @(posedge clk) begin
        //---------------------------------------------------------------------
        // 复位：清空该表项的有效标志
        //---------------------------------------------------------------------
        if (rst_sync) begin
            shadow_valid[entry] <= 0;
        end
        else begin
            //-----------------------------------------------------------------
            // 范围更新：清空该表项的有效标志
            //-----------------------------------------------------------------
            if (beam_range_we && !config_busy)
                shadow_valid[entry] <= 0;

            //-----------------------------------------------------------------
            // 表写入：从 32bit 数据中按字节偏移选取对应字节
            //   addr      对应 entry 时，写高字节 [31:24]
            //   addr+1    对应 entry 时，写 [23:16]
            //   addr+2    对应 entry 时，写 [15:8]
            //   addr+3    对应 entry 时，写 [7:0]
            //-----------------------------------------------------------------
            if (beam_tab_we && !config_busy) begin
                if (addr == ENTRY)
                    begin shadow[entry] <= beam_tab_wdata[31:24]; shadow_valid[entry] <= 1; end
                else if (addr + 9'd1 == ENTRY)
                    begin shadow[entry] <= beam_tab_wdata[23:16]; shadow_valid[entry] <= 1; end
                else if (addr + 9'd2 == ENTRY)
                    begin shadow[entry] <= beam_tab_wdata[15:8];  shadow_valid[entry] <= 1; end
                else if (addr + 9'd3 == ENTRY)
                    begin shadow[entry] <= beam_tab_wdata[7:0];   shadow_valid[entry] <= 1; end
            end
        end

        //---------------------------------------------------------------------
        // 拷贝：config_busy 期间，copy_word == entry/4 时拷贝该表项
        //   每拍拷贝 4 项，共 64 拍完成；
        //   shadow_valid 无效时写 0。
        //---------------------------------------------------------------------
        if (!rst_sync && !cfg_load && config_busy && copy_word == (entry / 4))
            gmem[entry] <= shadow_valid[entry] ? shadow[entry] : 8'd0;
    end

end
endgenerate

//=============================================================================
// 时序逻辑：配置/加载/查表主状态机
//-----------------------------------------------------------------------------
// 功能：
//   1. 复位：清零所有寄存器；
//   2. beam_range_we 有效时，锁存 beam_a0/beam_a1 到 shadow_a0/a1，
//      并置 wrote_any = 1；
//   3. beam_tab_we 有效时，置 wrote_any = 1；
//   4. cfg_load 触发时，进入 config_busy 状态，开始 64 拍拷贝：
//        - 锁存 target_idx 到 target_r；
//        - 把 shadow_a0/a1 拷贝到 a0/a1（范围原子切换）；
//        - copy_word 递增，到 63 时结束 config_busy，table_loaded = wrote_any；
//   5. 查表流水：
//        - hit_r / coff_r：延迟 1 拍寄存命中与偏移；
//        - g_q：根据 hit_r 决定输出 gmem[coff_r] 或 0；
//        - hit 判定要求 off_raw < 256，防止越界。
//
// 说明：
//   config_busy 期间运行控制器静音输出，保证增益表切换原子性；
//   拷贝期间 shadow 可以被继续写入，但不会影响正在拷贝的 gmem。
//
// 该 always 块对 clk 上升沿敏感，属于时序逻辑。
//=============================================================================
always @(posedge clk) begin
    if (rst_sync) begin
        shadow_a0      <= 0;
        shadow_a1      <= 0;
        a0             <= 0;
        a1             <= 0;
        target_r       <= 0;
        wrote_any      <= 0;
        table_loaded   <= 0;
        config_busy    <= 0;
        copy_word      <= 0;
        hit_r          <= 0;
        coff_r         <= 0;
        g_q            <= 255;
    end
    else begin
        //---------------------------------------------------------------------
        // 配置阶段：写入影子范围与影子表
        //---------------------------------------------------------------------
        if (beam_range_we && !config_busy) begin
            shadow_a0 <= beam_a0;
            shadow_a1 <= beam_a1;
            wrote_any <= 1;
        end
        if (beam_tab_we && !config_busy) begin
            wrote_any <= 1;
        end

        //---------------------------------------------------------------------
        // 加载阶段：cfg_load 触发 64 拍拷贝
        //---------------------------------------------------------------------
        if (cfg_load) begin
            config_busy <= 1;
            copy_word   <= 0;
            target_r    <= target_idx;
            a0          <= shadow_a0;   // 范围原子切换
            a1          <= shadow_a1;
        end
        else if (config_busy) begin
            if (copy_word == 63) begin
                config_busy  <= 0;
                table_loaded <= wrote_any;
            end
            else
                copy_word <= copy_word + 1'b1;
        end

        //---------------------------------------------------------------------
        // 查表流水：1 拍寄存
        //---------------------------------------------------------------------
        hit_r  <= hit && off_raw < 256;
        coff_r <= off_raw[7:0];
        g_q    <= hit_r ? gmem[coff_r] : 8'd0;
    end
end

endmodule
