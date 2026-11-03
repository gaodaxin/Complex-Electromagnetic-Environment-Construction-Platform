// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_hop_rand.v
// 功能：将伪随机数映射到有效跳频点范围，流水乘法后仅在跳频事件更新下标。
// ============================================================================
`timescale 1ns / 1ps

// =============================================================================
// 模块: da_sig_hop_rand
// 功能: 将 16 bit LFSR 均匀映射为 [0,N-1] 的随机捷变频点下标。
// 时序: LFSR×N 通过 DSP 流水，输出只在 hop 事件到来时更新。
// 约束: N 在内部钳位到 1..32；N=1 时固定选择表项 0。
// =============================================================================

module da_sig_hop_rand (

    input  wire       clk,               // FPGA 工作时钟
    input  wire       rst_sync,          // 同步复位，高有效
    input  wire       start_edge,        // 启动沿；LFSR 重新装种子
    input  wire [5:0] hop_n,             // 捷变频点数 N，合法范围 1~32
    output reg  [4:0] hop_rand_n         // 预取好的下一随机 hop，范围 0..N-1
);

reg [15:0] lfsr;                         // 16 bit LFSR
reg [7:0]  u1, n1, n2, n4;               // 拍1：样值 u、N、2N、4N
reg        acc1;                         // 拍1：rejection 是否通过
reg [7:0]  x2, n1_d;                     // 拍2：减 4N/2N 后的余数，以及延迟的 N
reg        acc2;                         // 拍2：把 accept 再打一拍

wire [7:0] x2_m4 =                       // 若 u>=4N 则减 4N
    (u1 >= n4) ? (u1 - n4) : u1;
wire [7:0] x2_m2 =                       // 再若 >=2N 则减 2N
    (x2_m4 >= n2) ? (x2_m4 - n2) : x2_m4;


function [5:0] hop_accept_lim;           // N·⌊32/N⌋，5 bit 抽样的 rejection 上限
    input [5:0] n;
    begin
        case (n)
            6'd0, 6'd1, 6'd2, 6'd4, 6'd8, 6'd16:
                hop_accept_lim = 6'd32;
            6'd3, 6'd5, 6'd6, 6'd10, 6'd14, 6'd15:
                hop_accept_lim = 6'd30;
            6'd7:  hop_accept_lim = 6'd28;
            6'd9:  hop_accept_lim = 6'd27;
            6'd11: hop_accept_lim = 6'd22;
            6'd12: hop_accept_lim = 6'd24;
            6'd13: hop_accept_lim = 6'd26;
            default: hop_accept_lim = n; // 17..32：⌊32/N⌋=1
        endcase
    end
endfunction


wire [5:0] n_w =                         // N 钳到 1~32
    (hop_n < 6'd1)  ? 6'd1  :
    (hop_n > 6'd32) ? 6'd32 :
                       hop_n;


always @(posedge clk) begin

    if (rst_sync || start_edge) begin

        lfsr       <= 16'hACE1;          // 固定种子，方便仿真复现
        u1         <= 8'd0;
        n1         <= 8'd1;
        n2         <= 8'd2;
        n4         <= 8'd4;
        acc1       <= 1'b0;
        x2         <= 8'd0;
        n1_d       <= 8'd1;
        acc2       <= 1'b0;
        hop_rand_n <= 5'd0;              // 默认下一 hop=0

    end else begin

        lfsr <= {lfsr[14:0],
                 lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
                                         // 与 PRI LFSR 同一多项式


        // 拍1：锁样值、N、4N/2N/N、是否接受
        u1   <= {3'b000, lfsr[4:0]};     // 5 bit 无符号样值扩成 8 bit
        n1   <= {2'b00, n_w};            // N
        n2   <= {1'b0, n_w, 1'b0};       // 2N
        n4   <= {n_w, 2'b00};            // 4N
        acc1 <= ({1'b0, lfsr[4:0]} < hop_accept_lim(n_w));
                                         // rejection：u 必须 < N·⌊32/N⌋


        // 拍2：减 4N 再减 2N（8 bit 组合，来自已寄存的 u1/n4/n2）
        if (n1 <= 8'd1)
            x2 <= 8'd0;                  // N<=1 时只有 hop=0
        else
            x2 <= x2_m2;
        n1_d <= n1;                      // N 再打一拍，给拍3 减 N
        acc2 <= acc1;


        // 拍3：减 N；只有 accept 才更新输出
        if (acc2) begin
            if (n1_d <= 8'd1)
                hop_rand_n <= 5'd0;
            else if (x2 >= n1_d)
                hop_rand_n <= x2[4:0] - n1_d[4:0];
            else
                hop_rand_n <= x2[4:0];   // 已经落在 0..N-1
        end
                                         // reject 时保持上一拍 hop_rand_n

    end

end

endmodule
