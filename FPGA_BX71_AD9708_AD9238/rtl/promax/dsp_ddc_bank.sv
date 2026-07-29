`timescale 1ns/1ps

// =============================================================================
// 多通道并行数字下变频器（DDC Bank）
// =============================================================================
// 设计目的：
//   单片机通常只能按顺序处理“某一个频点”；本模块把同一个 ADC 样本同时送入
//   CHANNELS 套独立 NCO + I/Q 混频流水线。流水线填满后，每来一个有效样本，
//   所有通道都在同一拍给出结果，因此可以同时盯住多个候选载波。
//
// 资源特征（默认 8 路）：
//   * 每路 I、Q 各一个 18x18 乘法，通常映射为 2 个 DSP48E1；
//   * 8 路共 16 个并行乘法，而不是复用一个乘法器跑 16 次；
//   * CORDIC NCO 本身为逐级流水线，填满后吞吐率同样为 1 sample/clock/channel。
//
// 扁平向量约定：
//   通道 k 总是位于 [k*WIDTH +: WIDTH]。这样既兼容 Vivado 2018.3，
//   又避免把 unpacked array 暴露到顶层端口给初学者带来连接困难。
//
// 重要限制：底层 dsp_nco_cordic 的 BAM 常数按 32 位相位设计，因此 PHASE_W
// 必须保持 32。phase_inc = round(f_out / f_sample * 2^32)。
module dsp_ddc_bank #(
    parameter integer CHANNELS = 8,
    parameter integer PHASE_W  = 32,
    parameter integer IN_W     = 18,
    parameter integer LO_W     = 18,
    parameter integer OUT_W    = 18,
    parameter integer ITER     = 16
) (
    input  wire                              clk,
    input  wire                              reset_n,
    input  wire                              state_clear,

    input  wire                              s_valid,
    input  wire signed [IN_W-1:0]            s_data,

    input  wire [CHANNELS*PHASE_W-1:0]       phase_inc_flat,
    input  wire [CHANNELS*PHASE_W-1:0]       phase_offset_flat,

    output wire                              m_valid,
    output wire [CHANNELS*OUT_W-1:0]         m_i_flat,
    output wire [CHANNELS*OUT_W-1:0]         m_q_flat
);
    // NCO 的正弦/余弦要经过 ITER 级 CORDIC。ADC 数据必须延迟同样的拍数，
    // 否则会把“当前样本”乘上“过去样本对应的本振相位”。
    reg signed [IN_W-1:0] data_delay [0:ITER];

    integer delay_index;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            for (delay_index = 0; delay_index <= ITER;
                 delay_index = delay_index + 1)
                data_delay[delay_index] <= {IN_W{1'b0}};
        end else if (state_clear) begin
            for (delay_index = 0; delay_index <= ITER;
                 delay_index = delay_index + 1)
                data_delay[delay_index] <= {IN_W{1'b0}};
        end else begin
            // 无效拍保持第 0 级即可；后级仍逐拍移动，和 NCO 的 valid 流水一致。
            if (s_valid)
                data_delay[0] <= s_data;

            for (delay_index = 0; delay_index < ITER;
                 delay_index = delay_index + 1)
                data_delay[delay_index+1] <= data_delay[delay_index];
        end
    end

    wire [CHANNELS-1:0] nco_valid_vector;
    wire [CHANNELS-1:0] mixer_valid_vector;

    genvar channel_index;
    generate
        for (channel_index = 0; channel_index < CHANNELS;
             channel_index = channel_index + 1) begin : gen_ddc_channel
            wire signed [LO_W-1:0] local_sin;
            wire signed [LO_W-1:0] local_cos;
            wire signed [OUT_W-1:0] local_i;
            wire signed [OUT_W-1:0] local_q;

            // 每个通道拥有独立相位累加器，所以频率、初相均可独立配置。
            dsp_nco_cordic #(
                .PHASE_W (PHASE_W),
                .AMP_W   (LO_W),
                .ITER    (ITER)
            ) u_nco (
                .clk          (clk),
                .reset_n      (reset_n),
                .state_clear  (state_clear),
                .s_valid      (s_valid),
                .phase_inc    (phase_inc_flat[
                                   channel_index*PHASE_W +: PHASE_W]),
                .phase_offset (phase_offset_flat[
                                   channel_index*PHASE_W +: PHASE_W]),
                .m_valid      (nco_valid_vector[channel_index]),
                .sin_out      (local_sin),
                .cos_out      (local_cos)
            );

            // I = x*cos，Q = -x*sin。两个乘法器同时工作，不做时分复用。
            dsp_iq_mixer #(
                .IN_W  (IN_W),
                .LO_W  (LO_W),
                .OUT_W (OUT_W)
            ) u_iq_mixer (
                .clk      (clk),
                .reset_n  (reset_n),
                .state_clear (state_clear),
                .s_valid  (nco_valid_vector[channel_index]),
                .s_data   (data_delay[ITER]),
                .lo_sin   (local_sin),
                .lo_cos   (local_cos),
                .m_valid  (mixer_valid_vector[channel_index]),
                .m_i      (local_i),
                .m_q      (local_q)
            );

            assign m_i_flat[channel_index*OUT_W +: OUT_W] = local_i;
            assign m_q_flat[channel_index*OUT_W +: OUT_W] = local_q;
        end
    endgenerate

    // 所有通道使用同一个 s_valid，故正常情况下 valid 必然逐拍对齐。
    // 使用归约与还能在仿真时避免误把某个尚未有效的通道当成整组有效。
    assign m_valid = &mixer_valid_vector;
endmodule
