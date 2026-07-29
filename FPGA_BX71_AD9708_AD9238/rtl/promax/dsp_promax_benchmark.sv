`timescale 1ns/1ps

// =============================================================================
// DSP ProMax 并行能力综合基准顶层
// =============================================================================
// 这个顶层不是“把 STM32 的几个函数搬进 FPGA”，而是把适合空间并行的重负载
// 同时展开成硬件。一个输入样本到来时，三类任务并行推进：
//
//   1. 8 路独立频点 DDC：16 个 I/Q 乘法器；
//   2. 4 x 32 抽头匹配滤波：128 个模板乘法器；
//   3. 8 路复基带功率统计：I/Q 共 16 个平方乘法器。
//
// 默认合计约 160 个并行乘法运算单元（实际 DSP48E1 数量以 Vivado 报告为准）。
// 所有单元都能在流水线填满后持续接收 1 sample/clock，而不是排队调用同一个
// 算法函数。它用于 out-of-context 综合、时序和资源报告，也是后续竞赛算法挑选
// 性能档位的依据；实际接 ADC/AXI DMA 时，可在它外面再包流接口与跨时钟 FIFO。
//
// 注意：默认资源组合针对 XC7Z020-2CLG400I。若还要同时加入大型 FFT/PFB，
// 应按场景选择 profile，而不是把所有最大配置永远固化进同一个 bitstream。
module dsp_promax_benchmark #(
    parameter integer DDC_CHANNELS    = 8,
    parameter integer MF_BANKS        = 4,
    parameter integer MF_TAPS         = 32,
    parameter integer PHASE_W         = 32,
    parameter integer DATA_W          = 18,
    parameter integer LO_W            = 18,
    parameter integer DDC_OUT_W       = 18,
    parameter integer MF_COEFF_W      = 18,
    parameter integer MF_ACC_W        = 48,
    parameter integer MF_OUT_W        = 24,
    parameter integer MF_OUT_SHIFT    = 16,
    parameter integer NCO_ITER         = 16,
    parameter integer CIC_RATE         = 64,
    parameter integer CIC_STAGES       = 3,
    parameter integer CIC_ACC_W        = 48,
    parameter integer CIC_GAIN_SHIFT   = 18,
    parameter integer POWER_BLOCK_LOG2 = 10,
    parameter integer PEAK_INDEX_W     = 32,
    parameter integer MF_BANK_INDEX_W = (MF_BANKS <= 1) ? 1 : $clog2(MF_BANKS),
    parameter integer MF_TAP_INDEX_W  = (MF_TAPS  <= 1) ? 1 : $clog2(MF_TAPS)
) (
    input  wire                                      clk,
    input  wire                                      reset_n,
    input  wire                                      state_clear,

    input  wire                                      s_valid,
    input  wire signed [DATA_W-1:0]                  s_data,

    input  wire [DDC_CHANNELS*PHASE_W-1:0]           ddc_phase_inc_flat,
    input  wire [DDC_CHANNELS*PHASE_W-1:0]           ddc_phase_offset_flat,

    input  wire                                      mf_cfg_valid,
    input  wire [MF_BANK_INDEX_W-1:0]                mf_cfg_bank,
    input  wire [MF_TAP_INDEX_W-1:0]                 mf_cfg_index,
    input  wire signed [MF_COEFF_W-1:0]              mf_cfg_data,

    output wire                                      ddc_valid,
    output wire [DDC_CHANNELS*DDC_OUT_W-1:0]         ddc_i_flat,
    output wire [DDC_CHANNELS*DDC_OUT_W-1:0]         ddc_q_flat,

    output wire                                      ddc_filtered_valid,
    output wire [DDC_CHANNELS*DDC_OUT_W-1:0]         ddc_filtered_i_flat,
    output wire [DDC_CHANNELS*DDC_OUT_W-1:0]         ddc_filtered_q_flat,

    output wire                                      mf_valid,
    output wire [MF_BANKS*MF_OUT_W-1:0]              mf_score_flat,
    output wire                                      mf_peak_valid,
    output wire [MF_BANKS*MF_OUT_W-1:0]              mf_peak_abs_flat,
    output wire [MF_BANKS*MF_OUT_W-1:0]              mf_peak_score_flat,
    output wire [MF_BANKS*PEAK_INDEX_W-1:0]          mf_peak_index_flat,

    output wire                                      band_power_valid,
    output wire [DDC_CHANNELS*(2*DDC_OUT_W+1)-1:0]   band_power_flat
);
    localparam integer POWER_W = 2*DDC_OUT_W + 1;

    // keep_hierarchy 让综合报告中仍可清楚看到三个并行层次，便于教学和核查资源。
    (* keep_hierarchy = "yes" *)
    dsp_ddc_bank #(
        .CHANNELS (DDC_CHANNELS),
        .PHASE_W  (PHASE_W),
        .IN_W     (DATA_W),
        .LO_W     (LO_W),
        .OUT_W    (DDC_OUT_W),
        .ITER     (NCO_ITER)
    ) u_parallel_ddc (
        .clk               (clk),
        .reset_n           (reset_n),
        .state_clear       (state_clear),
        .s_valid           (s_valid),
        .s_data            (s_data),
        .phase_inc_flat    (ddc_phase_inc_flat),
        .phase_offset_flat (ddc_phase_offset_flat),
        .m_valid           (ddc_valid),
        .m_i_flat          (ddc_i_flat),
        .m_q_flat          (ddc_q_flat)
    );

    (* keep_hierarchy = "yes" *)
    dsp_parallel_matched_filter #(
        .BANKS        (MF_BANKS),
        .TAPS         (MF_TAPS),
        .IN_W         (DATA_W),
        .COEFF_W      (MF_COEFF_W),
        .ACC_W        (MF_ACC_W),
        .OUT_W        (MF_OUT_W),
        .OUT_SHIFT    (MF_OUT_SHIFT),
        .BANK_INDEX_W (MF_BANK_INDEX_W),
        .TAP_INDEX_W  (MF_TAP_INDEX_W)
    ) u_parallel_matched_filter (
        .clk         (clk),
        .reset_n     (reset_n),
        .state_clear (state_clear),
        .cfg_valid   (mf_cfg_valid),
        .cfg_bank    (mf_cfg_bank),
        .cfg_index   (mf_cfg_index),
        .cfg_data    (mf_cfg_data),
        .s_valid     (s_valid),
        .s_data      (s_data),
        .m_valid     (mf_valid),
        .m_data_flat (mf_score_flat)
    );

    // A one-clock correlation peak is much narrower than an SPI/UART read.
    // Keep magnitude, polarity and sample index until the next state_clear.
    dsp_matched_peak_capture #(
        .BANKS   (MF_BANKS),
        .SCORE_W (MF_OUT_W),
        .INDEX_W (PEAK_INDEX_W)
    ) u_matched_peak_capture (
        .clk               (clk),
        .reset_n           (reset_n),
        .state_clear       (state_clear),
        .s_valid           (mf_valid),
        .score_flat        (mf_score_flat),
        .peak_update_valid (mf_peak_valid),
        .peak_abs_flat     (mf_peak_abs_flat),
        .peak_score_flat   (mf_peak_score_flat),
        .peak_index_flat   (mf_peak_index_flat)
    );

    wire [DDC_CHANNELS-1:0] cic_i_valid_vector;
    wire [DDC_CHANNELS-1:0] cic_q_valid_vector;
    wire [DDC_CHANNELS-1:0] power_valid_vector;

    genvar power_channel;
    generate
        for (power_channel = 0; power_channel < DDC_CHANNELS;
             power_channel = power_channel + 1) begin : gen_band_power
            wire i_power_valid;
            wire q_power_valid;
            wire [2*DDC_OUT_W-1:0] i_mean_square;
            wire [2*DDC_OUT_W-1:0] q_mean_square;
            wire [POWER_W-1:0] complex_mean_power;
            wire signed [DDC_OUT_W-1:0] filtered_i;
            wire signed [DDC_OUT_W-1:0] filtered_q;
            reg signed [DDC_OUT_W-1:0] power_i_registered;
            reg signed [DDC_OUT_W-1:0] power_q_registered;
            reg power_sample_valid;

            // Mixing alone does not select a band: for an ideal real tone,
            // I^2+Q^2 is almost independent of the programmed LO.  Each I/Q
            // pair therefore passes through a real low-pass/decimator before
            // squaring.  RATE=64/STAGES=3 gives a narrow, multiplier-free CIC
            // channel filter and reduces detector work by 64x.
            dsp_cic_decimator #(
                .IN_W       (DDC_OUT_W),
                .OUT_W      (DDC_OUT_W),
                .RATE       (CIC_RATE),
                .STAGES     (CIC_STAGES),
                .ACC_W      (CIC_ACC_W),
                .GAIN_SHIFT (CIC_GAIN_SHIFT)
            ) u_i_channel_filter (
                .clk         (clk),
                .reset_n     (reset_n),
                .state_clear (state_clear),
                .s_valid     (ddc_valid),
                .s_data      (ddc_i_flat[
                                 power_channel*DDC_OUT_W +: DDC_OUT_W]),
                .m_valid     (cic_i_valid_vector[power_channel]),
                .m_data      (filtered_i)
            );

            dsp_cic_decimator #(
                .IN_W       (DDC_OUT_W),
                .OUT_W      (DDC_OUT_W),
                .RATE       (CIC_RATE),
                .STAGES     (CIC_STAGES),
                .ACC_W      (CIC_ACC_W),
                .GAIN_SHIFT (CIC_GAIN_SHIFT)
            ) u_q_channel_filter (
                .clk         (clk),
                .reset_n     (reset_n),
                .state_clear (state_clear),
                .s_valid     (ddc_valid),
                .s_data      (ddc_q_flat[
                                 power_channel*DDC_OUT_W +: DDC_OUT_W]),
                .m_valid     (cic_q_valid_vector[power_channel]),
                .m_data      (filtered_q)
            );

            assign ddc_filtered_i_flat[
                       power_channel*DDC_OUT_W +: DDC_OUT_W] = filtered_i;
            assign ddc_filtered_q_flat[
                       power_channel*DDC_OUT_W +: DDC_OUT_W] = filtered_q;

            // Register the rounded CIC output before the square/accumulate
            // detector.  Besides clean module timing, this cuts the former
            // 14.4 ns path "CIC round/saturate -> square -> power sum" into
            // two stages without changing throughput or numeric results.
            always @(posedge clk or negedge reset_n) begin
                if (!reset_n) begin
                    power_i_registered <= {DDC_OUT_W{1'b0}};
                    power_q_registered <= {DDC_OUT_W{1'b0}};
                    power_sample_valid <= 1'b0;
                end else if (state_clear) begin
                    power_i_registered <= {DDC_OUT_W{1'b0}};
                    power_q_registered <= {DDC_OUT_W{1'b0}};
                    power_sample_valid <= 1'b0;
                end else begin
                    power_sample_valid <= cic_i_valid_vector[power_channel] &
                                          cic_q_valid_vector[power_channel];
                    if (cic_i_valid_vector[power_channel] &
                        cic_q_valid_vector[power_channel]) begin
                        power_i_registered <= filtered_i;
                        power_q_registered <= filtered_q;
                    end
                end
            end

            // I、Q 平方也各自并行，不阻塞 DDC 和相关器流水线。
            dsp_power_meter #(
                .IN_W       (DDC_OUT_W),
                .BLOCK_LOG2 (POWER_BLOCK_LOG2)
            ) u_i_power (
                .clk         (clk),
                .reset_n     (reset_n),
                .state_clear (state_clear),
                .s_valid     (power_sample_valid),
                .s_data      (power_i_registered),
                .m_valid     (i_power_valid),
                .mean_square (i_mean_square)
            );

            dsp_power_meter #(
                .IN_W       (DDC_OUT_W),
                .BLOCK_LOG2 (POWER_BLOCK_LOG2)
            ) u_q_power (
                .clk         (clk),
                .reset_n     (reset_n),
                .state_clear (state_clear),
                .s_valid     (power_sample_valid),
                .s_data      (power_q_registered),
                .m_valid     (q_power_valid),
                .mean_square (q_mean_square)
            );

            // 复信号功率 = E[I^2] + E[Q^2]，多保留 1 位防止相加溢出。
            assign complex_mean_power = {1'b0, i_mean_square} +
                                        {1'b0, q_mean_square};
            assign power_valid_vector[power_channel] =
                                        i_power_valid & q_power_valid;
            assign band_power_flat[power_channel*POWER_W +: POWER_W] =
                                        complex_mean_power;
        end
    endgenerate

    assign band_power_valid = &power_valid_vector;
    assign ddc_filtered_valid = &(cic_i_valid_vector & cic_q_valid_vector);
endmodule
