`timescale 1ns/1ps

// =============================================================================
// 分组匹配滤波峰值保持：保存最大绝对分数、带符号分数和首次峰值位置。
// Per-bank matched-filter peak capture
// =============================================================================
// A matched-filter maximum may exist for only one FPGA clock.  SPI/UART is far
// too slow to poll that pulse directly, so this block keeps a durable summary:
//   * the largest unsigned |score| seen since state_clear;
//   * the corresponding signed score (polarity is not lost);
//   * the sample index at which that score appeared.
//
// The first valid score is always recorded, including an all-zero score.  Later
// scores replace it only when their absolute magnitude is strictly greater;
// equal peaks therefore keep the earliest arrival index.
module dsp_matched_peak_capture #(
    parameter integer BANKS       = 4,
    parameter integer SCORE_W     = 24,
    parameter integer INDEX_W     = 32
) (
    input  wire                              clk,
    input  wire                              reset_n,
    input  wire                              state_clear,
    input  wire                              s_valid,
    input  wire [BANKS*SCORE_W-1:0]          score_flat,

    output reg                               peak_update_valid,
    output wire [BANKS*SCORE_W-1:0]          peak_abs_flat,
    output wire [BANKS*SCORE_W-1:0]          peak_score_flat,
    output wire [BANKS*INDEX_W-1:0]          peak_index_flat
);
    reg [INDEX_W-1:0] sample_index;
    reg input_valid_d;
    reg [BANKS*SCORE_W-1:0] score_flat_d;
    reg [BANKS-1:0] peak_seen;
    reg [SCORE_W-1:0] peak_abs [0:BANKS-1];
    reg signed [SCORE_W-1:0] peak_score [0:BANKS-1];
    reg [INDEX_W-1:0] peak_index [0:BANKS-1];

    genvar output_bank;
    generate
        for (output_bank = 0; output_bank < BANKS;
             output_bank = output_bank + 1) begin : gen_peak_outputs
            assign peak_abs_flat[output_bank*SCORE_W +: SCORE_W] =
                       peak_abs[output_bank];
            assign peak_score_flat[output_bank*SCORE_W +: SCORE_W] =
                       peak_score[output_bank];
            assign peak_index_flat[output_bank*INDEX_W +: INDEX_W] =
                       peak_index[output_bank];
        end
    endgenerate

    integer bank_index;
    reg signed [SCORE_W-1:0] current_score;
    reg [SCORE_W-1:0] current_abs;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_index      <= {INDEX_W{1'b0}};
            input_valid_d     <= 1'b0;
            score_flat_d      <= {(BANKS*SCORE_W){1'b0}};
            peak_seen         <= {BANKS{1'b0}};
            peak_update_valid <= 1'b0;
            for (bank_index = 0; bank_index < BANKS;
                 bank_index = bank_index + 1) begin
                peak_abs[bank_index]   <= {SCORE_W{1'b0}};
                peak_score[bank_index] <= {SCORE_W{1'b0}};
                peak_index[bank_index] <= {INDEX_W{1'b0}};
            end
        end else if (state_clear) begin
            sample_index      <= {INDEX_W{1'b0}};
            input_valid_d     <= 1'b0;
            score_flat_d      <= {(BANKS*SCORE_W){1'b0}};
            peak_seen         <= {BANKS{1'b0}};
            peak_update_valid <= 1'b0;
            for (bank_index = 0; bank_index < BANKS;
                 bank_index = bank_index + 1) begin
                peak_abs[bank_index]   <= {SCORE_W{1'b0}};
                peak_score[bank_index] <= {SCORE_W{1'b0}};
                peak_index[bank_index] <= {INDEX_W{1'b0}};
            end
        end else begin
            peak_update_valid <= 1'b0;
            // 先把相关器的舍入/饱和输出打一拍，再做绝对值和峰值比较。
            // 这切断“48位FIR缩位 -> abs -> compare -> update”长组合路径，
            // 不降低每拍一组分数的吞吐，只让峰值摘要固定多一拍延迟。
            input_valid_d <= s_valid;
            if (s_valid)
                score_flat_d <= score_flat;

            if (input_valid_d) begin
                sample_index <= sample_index + 1'b1;
                for (bank_index = 0; bank_index < BANKS;
                     bank_index = bank_index + 1) begin
                    current_score = $signed(score_flat_d[
                                      bank_index*SCORE_W +: SCORE_W]);
                    // Two's-complement magnitude remains SCORE_W bits so the
                    // most-negative value (1000...0) is represented exactly.
                    current_abs = current_score[SCORE_W-1] ?
                                  (~current_score + 1'b1) : current_score;
                    if (!peak_seen[bank_index] ||
                        (current_abs > peak_abs[bank_index])) begin
                        peak_seen[bank_index]  <= 1'b1;
                        peak_abs[bank_index]   <= current_abs;
                        peak_score[bank_index] <= current_score;
                        peak_index[bank_index] <= sample_index;
                        peak_update_valid      <= 1'b1;
                    end
                end
            end
        end
    end
endmodule
