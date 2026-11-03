// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_phase_tab.v
// 功能：按跳频下标同步读取 32 项附加相位表，关闭时输出零相位。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_phase_tab
// 功能: 按 hop_idx 从 32×32 bit 相位表读取附加相位。
// 时序: 地址和使能先寄存以切断 32:1 宽选择路径；en=0 时输出零相位。
// =============================================================================

module da_sig_phase_tab (

    input  wire          clk,            // FPGA 工作时钟
    input  wire          rst_sync,       // 同步复位，高有效
    input  wire          en,             // 1=使用相位表，0=输出 0
    input  wire [4:0]    hop_idx,        // 当前 hop 下标，与频表对齐
    input  wire [1023:0] phase_tab_bus,  // 32 个 × 32 bit 相位字
    output reg  [31:0]   phi_hop         // 查到的附加相位；en=0 时为 0
);

genvar gi;

wire [31:0] ptab_w [0:31];               // 1024 bit 总线拆成 32 个相位字

generate
    for (gi = 0; gi < 32; gi = gi + 1) begin : g_ptab
        assign ptab_w[gi] = phase_tab_bus[gi*32 +: 32];
    end
endgenerate


reg        en_r;                         // 使能打一拍，与数据对齐
reg [4:0]  idx_r;                        // 地址打一拍，切断 32:1 mux


always @(posedge clk) begin

    if (rst_sync) begin

        en_r    <= 1'b0;                 // 复位后不叠加相位表
        idx_r   <= 5'd0;
        phi_hop <= 32'd0;                // 附加相位清零

    end else begin

        en_r    <= en;                   // 锁存使能
        idx_r   <= hop_idx;              // 锁存 hop 下标
        phi_hop <= en_r ?
                   ptab_w[idx_r] :       // 使能时输出 phase_tab[hop]
                   32'd0;                // 关闭时附加相位为 0

    end

end

endmodule
