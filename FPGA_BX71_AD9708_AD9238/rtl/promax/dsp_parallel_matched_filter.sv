`timescale 1ns/1ps

// =============================================================================
// 多模板全并行匹配滤波 / 相关器组
// =============================================================================
// 每个 bank 是一套 TAPS 阶转置 FIR。若把系数写成待检测模板的“时间反转”，
// FIR 输出就是该模板和输入滑动窗口的互相关值。
//
// 默认配置在每个有效样本上同时完成：
//       4 个模板 x 32 个抽头 = 128 次乘加
// 这 128 个乘法器是真正同时存在于 FPGA 中的硬件，流水线填满后每拍产生 4 个
// 新相关分数。它适合竞赛中的同步字搜索、脉冲压缩、FSK/PSK 前导检测、
// 多种已知波形并行判决；这正是 STM32H7 很难以同样吞吐率完成的工作负载。
//
// 配置说明：
//   cfg_bank 选择模板，cfg_index 选择该模板中的抽头；可以逐项写入。
//   cfg_valid 与 s_valid 同拍出现时，当前样本仍使用旧系数，新系数从下一样本生效。
//   建议在开始采样前装完全部模板，或在明确允许热切换时再在线写系数。
module dsp_parallel_matched_filter #(
    parameter integer BANKS        = 4,
    parameter integer TAPS         = 32,
    parameter integer IN_W         = 18,
    parameter integer COEFF_W      = 18,
    parameter integer ACC_W        = 48,
    parameter integer OUT_W        = 24,
    parameter integer OUT_SHIFT    = 16,
    // 显式参数让 BANKS=1 或 TAPS=1 时端口宽度仍至少为 1 位。
    parameter integer BANK_INDEX_W = (BANKS <= 1) ? 1 : $clog2(BANKS),
    parameter integer TAP_INDEX_W  = (TAPS  <= 1) ? 1 : $clog2(TAPS)
) (
    input  wire                              clk,
    input  wire                              reset_n,
    input  wire                              state_clear,

    input  wire                              cfg_valid,
    input  wire [BANK_INDEX_W-1:0]           cfg_bank,
    input  wire [TAP_INDEX_W-1:0]            cfg_index,
    input  wire signed [COEFF_W-1:0]         cfg_data,

    input  wire                              s_valid,
    input  wire signed [IN_W-1:0]            s_data,

    output wire                              m_valid,
    output wire [BANKS*OUT_W-1:0]            m_data_flat
);
    wire [BANKS-1:0] bank_valid_vector;

    genvar bank_index;
    generate
        for (bank_index = 0; bank_index < BANKS;
             bank_index = bank_index + 1) begin : gen_matched_filter_bank
            wire signed [OUT_W-1:0] local_score;
            wire local_cfg_valid;

            // cfg_bank 若超出范围，不会选中任何 bank，不会破坏已有模板。
            assign local_cfg_valid = cfg_valid &&
                                     (cfg_bank == bank_index[BANK_INDEX_W-1:0]);

            dsp_fir_transposed #(
                .TAPS      (TAPS),
                .IN_W      (IN_W),
                .COEFF_W   (COEFF_W),
                .ACC_W     (ACC_W),
                .OUT_W     (OUT_W),
                .OUT_SHIFT (OUT_SHIFT)
            ) u_matched_fir (
                .clk       (clk),
                .reset_n   (reset_n),
                .state_clear (state_clear),
                .cfg_valid (local_cfg_valid),
                .cfg_index (cfg_index),
                .cfg_data  (cfg_data),
                .s_valid   (s_valid),
                .s_data    (s_data),
                .m_valid   (bank_valid_vector[bank_index]),
                .m_data    (local_score)
            );

            assign m_data_flat[bank_index*OUT_W +: OUT_W] = local_score;
        end
    endgenerate

    assign m_valid = &bank_valid_vector;
endmodule
