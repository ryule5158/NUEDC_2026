`timescale 1ns / 1ps

/*
 * 连续波形统计器。
 *
 * FPGA在ADC满速数据流上完成每个2^N点数据块的最值、和、平方和、
 * 高电平计数与上升沿周期累计。STM32只需读取一次快照，就能换算
 * DC、RMS、AC RMS、Vpp、频率和占空比，不必通过SPI搬运整块采样。
 *
 * 门限使用上一块的(min+max)/2，因此第一块用于自动学习门限，
 * 从第二块开始threshold_settled=1。统计结果受hold冻结，但内部
 * 仍继续处理数据，解除冻结后会发布最新完整数据块。
 */
module dsp_stream_stats #(
    parameter integer MIN_BLOCK_LOG2 = 6,
    parameter integer MAX_BLOCK_LOG2 = 25
)(
    input  wire               clk,
    input  wire               reset_n,
    input  wire               clear,
    input  wire               hold,
    input  wire               s_valid,
    input  wire signed [7:0]  s_data,
    input  wire [4:0]         block_log2,

    output reg                result_valid,
    output reg                threshold_settled,
    output reg [7:0]          generation,
    output reg signed [7:0]   result_min,
    output reg signed [7:0]   result_max,
    output reg signed [7:0]   result_threshold,
    output reg signed [39:0]  result_sum,
    output reg [47:0]         result_sum_square,
    output reg [31:0]         result_high_count,
    output reg [31:0]         result_rising_count,
    output reg [31:0]         result_period_sum,
    output reg [31:0]         result_sample_count
);

    reg [31:0] sample_index;
    reg signed [7:0] min_acc;
    reg signed [7:0] max_acc;
    reg signed [39:0] sum_acc;
    reg [47:0] sum_square_acc;
    reg [31:0] high_count_acc;
    reg [31:0] rising_count_acc;
    reg [31:0] period_sum_acc;
    reg [31:0] last_rising_index;
    reg signed [7:0] active_threshold;
    reg have_previous;
    reg previous_above;
    reg have_rising;
    reg threshold_ready;

    localparam [4:0] MIN_BLOCK_LOG2_VALUE = MIN_BLOCK_LOG2;
    localparam [4:0] MAX_BLOCK_LOG2_VALUE = MAX_BLOCK_LOG2;
    wire [4:0] safe_block_log2 =
        (block_log2 < MIN_BLOCK_LOG2_VALUE) ? MIN_BLOCK_LOG2_VALUE :
        (block_log2 > MAX_BLOCK_LOG2_VALUE) ? MAX_BLOCK_LOG2_VALUE :
                                        block_log2;
    wire [31:0] block_size = 32'd1 << safe_block_log2;
    wire first_sample = (sample_index == 32'd0);
    wire block_end = (sample_index == (block_size - 1'b1));
    /* 等于门限时保持上一判决，避免恰落在中点的样本使占空比产生固定偏差。 */
    wire sample_above =
        ($signed(s_data) > $signed(active_threshold)) ? 1'b1 :
        ($signed(s_data) < $signed(active_threshold)) ? 1'b0 :
        (have_previous ? previous_above : 1'b0);
    wire rising_now = have_previous && !previous_above && sample_above;

    wire signed [7:0] min_next =
        first_sample ? s_data :
        (($signed(s_data) < $signed(min_acc)) ? s_data : min_acc);
    wire signed [7:0] max_next =
        first_sample ? s_data :
        (($signed(s_data) > $signed(max_acc)) ? s_data : max_acc);
    wire signed [39:0] sample_extended = {{32{s_data[7]}}, s_data};
    wire signed [39:0] sum_next =
        first_sample ? sample_extended : (sum_acc + sample_extended);
    wire signed [15:0] sample_product = $signed(s_data) * $signed(s_data);
    wire [47:0] square_extended = {32'd0, sample_product[15:0]};
    wire [47:0] sum_square_next =
        first_sample ? square_extended : (sum_square_acc + square_extended);
    wire [31:0] high_count_next =
        (first_sample ? 32'd0 : high_count_acc) +
        (sample_above ? 1'b1 : 1'b0);
    wire [31:0] rising_count_next =
        (first_sample ? 32'd0 : rising_count_acc) +
        (rising_now ? 1'b1 : 1'b0);
    wire [31:0] period_sum_next =
        (first_sample ? 32'd0 : period_sum_acc) +
        ((rising_now && have_rising) ?
         (sample_index - last_rising_index) : 32'd0);
    wire signed [8:0] midpoint_sum =
        $signed({min_next[7], min_next}) +
        $signed({max_next[7], max_next});
    wire signed [7:0] next_threshold = midpoint_sum >>> 1;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_index <= 32'd0;
            min_acc <= 8'sd0;
            max_acc <= 8'sd0;
            sum_acc <= 40'sd0;
            sum_square_acc <= 48'd0;
            high_count_acc <= 32'd0;
            rising_count_acc <= 32'd0;
            period_sum_acc <= 32'd0;
            last_rising_index <= 32'd0;
            active_threshold <= 8'sd0;
            have_previous <= 1'b0;
            previous_above <= 1'b0;
            have_rising <= 1'b0;
            threshold_ready <= 1'b0;
            result_valid <= 1'b0;
            threshold_settled <= 1'b0;
            generation <= 8'd0;
            result_min <= 8'sd0;
            result_max <= 8'sd0;
            result_threshold <= 8'sd0;
            result_sum <= 40'sd0;
            result_sum_square <= 48'd0;
            result_high_count <= 32'd0;
            result_rising_count <= 32'd0;
            result_period_sum <= 32'd0;
            result_sample_count <= 32'd0;
        end else if (clear) begin
            sample_index <= 32'd0;
            min_acc <= 8'sd0;
            max_acc <= 8'sd0;
            sum_acc <= 40'sd0;
            sum_square_acc <= 48'd0;
            high_count_acc <= 32'd0;
            rising_count_acc <= 32'd0;
            period_sum_acc <= 32'd0;
            last_rising_index <= 32'd0;
            active_threshold <= 8'sd0;
            have_previous <= 1'b0;
            previous_above <= 1'b0;
            have_rising <= 1'b0;
            threshold_ready <= 1'b0;
            result_valid <= 1'b0;
            threshold_settled <= 1'b0;
            generation <= 8'd0;
        end else if (s_valid) begin
            min_acc <= min_next;
            max_acc <= max_next;
            sum_acc <= sum_next;
            sum_square_acc <= sum_square_next;
            high_count_acc <= high_count_next;
            rising_count_acc <= rising_count_next;
            period_sum_acc <= period_sum_next;
            have_previous <= 1'b1;
            previous_above <= sample_above;

            if (rising_now) begin
                last_rising_index <= sample_index;
                have_rising <= 1'b1;
            end

            if (block_end) begin
                if (!hold) begin
                    result_valid <= 1'b1;
                    threshold_settled <= threshold_ready;
                    generation <= generation + 1'b1;
                    result_min <= min_next;
                    result_max <= max_next;
                    result_threshold <= active_threshold;
                    result_sum <= sum_next;
                    result_sum_square <= sum_square_next;
                    result_high_count <= high_count_next;
                    result_rising_count <= rising_count_next;
                    result_period_sum <= period_sum_next;
                    result_sample_count <= block_size;
                end

                /* 新门限从下一完整块开始生效。 */
                active_threshold <= next_threshold;
                threshold_ready <= 1'b1;
                sample_index <= 32'd0;
                min_acc <= 8'sd0;
                max_acc <= 8'sd0;
                sum_acc <= 40'sd0;
                sum_square_acc <= 48'd0;
                high_count_acc <= 32'd0;
                rising_count_acc <= 32'd0;
                period_sum_acc <= 32'd0;
                last_rising_index <= 32'd0;
                have_previous <= 1'b0;
                previous_above <= 1'b0;
                have_rising <= 1'b0;
            end else begin
                sample_index <= sample_index + 1'b1;
            end
        end
    end

endmodule
