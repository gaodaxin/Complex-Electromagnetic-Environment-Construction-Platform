// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/new/da_lfm/da_sig_lfm_q32.v
// 功能：16 样点并行 LFM 相位引擎，以 64 bit 算术保留频率和相位的小数部分，每拍接受一个完整向量。
// ============================================================================
`timescale 1ns/1ps
// 16-sample LFM vector engine. All frequency and phase arithmetic is modulo
// 2^64: upper 32 bits are the original FTW/phase, lower 32 bits are retained
// fractions. Each pipeline stage accepts one complete vector per clock.
module da_sig_lfm_q32 (
    input wire clk, rst_sync, start_edge, clear_phase_edge,
    input wire running, wave_ready, pulse_gate, pulse_rise,
    input wire [31:0] carrier, phase_init, n_sample,
    input wire signed [63:0] slope_q32, fm_start_q32,
    output wire [511:0] phase_bus,
    output wire phase_valid, phase_user
);
    reg [63:0] fm_frequency=0;
    reg [31:0] sample_count=0;
    reg was_gate=0;
    reg [63:0] f0=0, slope0=0;
    reg gate0=0, gate1=0, gate2=0, gate3=0;
    reg valid0=0, valid1=0, valid2=0, valid3=0;
    reg [63:0] increment1=0, phase_base=0, base2=0;
    wire gate_now=running && wave_ready && pulse_gate;
    wire restart=pulse_rise || !was_gate || (sample_count==0);
    wire [63:0] mod_now=restart ? fm_start_q32 : fm_frequency;
    wire [63:0] delta16=slope_q32<<4;
    wire [63:0] delta120=(slope0<<7)-(slope0<<3);

    always @(posedge clk) begin
        if (rst_sync || start_edge) begin
            fm_frequency<=fm_start_q32; sample_count<=0; was_gate<=0;
            f0<=0; slope0<=0; gate0<=0; valid0<=0;
        end else begin
            was_gate<=gate_now;
            f0<={carrier,32'd0}+mod_now;
            slope0<=slope_q32;
            gate0<=gate_now; valid0<=running && wave_ready;
            if (gate_now) begin
                if (restart) begin
                    fm_frequency<=fm_start_q32+delta16;
                    sample_count<=16;
                end else if ((sample_count+32'd16)>=n_sample) begin
                    fm_frequency<=fm_start_q32; sample_count<=0;
                end else begin
                    fm_frequency<=fm_frequency+delta16;
                    sample_count<=sample_count+32'd16;
                end
            end
            if (clear_phase_edge) begin gate0<=0; valid0<=0; end
        end
    end

    // Frequency products and the vector phase increment are computed before
    // the phase feedback adder. Data, gate and valid share the same latency.
    always @(posedge clk) begin
        if (rst_sync || start_edge || clear_phase_edge) begin
            increment1<=0; phase_base<={phase_init,32'd0}; base2<=0;
            gate1<=0; gate2<=0; gate3<=0;
            valid1<=0; valid2<=0; valid3<=0;
        end else begin
            increment1<=(f0<<4)+delta120;
            gate1<=gate0; gate2<=gate1; gate3<=gate2;
            valid1<=valid0; valid2<=valid1; valid3<=valid2;
            base2<=phase_base;
            if (gate1) phase_base<=phase_base+increment1;
        end
    end
    genvar lane;
    generate for (lane=0;lane<16;lane=lane+1) begin:g_lane
        localparam integer TRI=lane*(lane-1)/2;
        reg [63:0] linear1=0, quadratic1=0, offset2=0, phase3=0;
        always @(posedge clk) begin
            if (rst_sync || start_edge || clear_phase_edge) begin
                linear1<=0;quadratic1<=0;offset2<=0;phase3<=0;
            end else begin
                linear1<=f0*lane; quadratic1<=slope0*TRI;
                offset2<=linear1+quadratic1;
                phase3<=base2+offset2;
            end
        end
        assign phase_bus[lane*32+:32]=phase3[63:32];
    end endgenerate
    assign phase_valid=valid3;
    assign phase_user=gate3 && valid3;
endmodule
