`timescale 1ns / 1ps

/*
 * Stand-alone 307-tap anti-alias FIR for the AD9238 sample stream.
 *
 * This module is intentionally not connected to ad9238_capture_core.  It is a
 * full-rate reference implementation for implementation/resource evaluation.
 *
 * Input:
 *   - 50 MHz, 12-bit offset-binary AD9238 samples.
 *   - input_valid may contain gaps; the delay line advances only on valid data.
 *
 * Filter:
 *   - 307 symmetric Q16 coefficients (154 unique values).
 *   - 153 pre-add/multiply paths plus one centre-tap multiply.
 *   - Eight registered, balanced adder-tree levels.
 *   - Linear-phase group delay: 153 valid input samples.
 *
 * Output:
 *   - Signed sample-code units in Q4 (one ADC code = 16 output counts).
 *   - Round-to-nearest, ties away from zero, with signed-16 saturation.
 *   - The first result represents the 307th valid input.  It appears nine
 *     clocks later because of the multiplier/tree/output pipeline.
 */
module fir307_antialias_q4 (
    input  wire               clk,
    input  wire               reset_n,
    input  wire               clear,
    input  wire               input_valid,
    input  wire [11:0]        input_code,
    output reg                output_valid,
    output reg signed [15:0]  output_sample_q4
);
    localparam integer TAP_COUNT = 307;
    localparam integer UNIQUE_COEFFICIENT_COUNT = 154;
    localparam integer HISTORY_SAMPLE_COUNT = TAP_COUNT - 1;
    localparam integer PIPELINE_TREE_LEVELS = 8;

    /*
     * Before a valid edge, delay_line[0] is x[n-1] and delay_line[305]
     * is x[n-306].  The current x[n] is used directly in symmetric pair 0.
     */
    reg signed [11:0] delay_line [0:HISTORY_SAMPLE_COUNT-1];
    reg [8:0] valid_sample_count;

    /*
     * Forty bits comfortably contain the worst-case coefficient L1 sum while
     * keeping every tree level at one width.  Registered products make the
     * structure capable of accepting one sample on every 50 MHz clock.
     */
    (* use_dsp = "yes" *)
    reg signed [39:0] product_pipe [0:UNIQUE_COEFFICIENT_COUNT-1];
    reg signed [39:0] sum_level_1 [0:76];
    reg signed [39:0] sum_level_2 [0:38];
    reg signed [39:0] sum_level_3 [0:19];
    reg signed [39:0] sum_level_4 [0:9];
    reg signed [39:0] sum_level_5 [0:4];
    reg signed [39:0] sum_level_6 [0:2];
    reg signed [39:0] sum_level_7 [0:1];
    reg signed [39:0] sum_level_8;

    /*
     * valid_pipeline[0] accompanies product_pipe; [1]..[8] accompany the
     * corresponding adder-tree levels.  output_valid is the final register.
     */
    reg [PIPELINE_TREE_LEVELS:0] valid_pipeline;

    integer reset_index;
    integer tree_index;

    wire signed [12:0] centered_input =
        $signed({1'b0, input_code}) - 13'sd2048;
    wire history_full = (valid_sample_count >= 9'd306);

    function signed [17:0] FIR307_Coefficient;
        input [7:0] coefficient_index;
        begin
            case (coefficient_index)
`include "fir307_antialias_coeffs.vh"
            endcase
        end
    endfunction

    /*
     * Convert Q16 accumulator data to Q4.  Keeping this as a function makes
     * the rounding and saturation boundary directly testable in simulation.
     */
    function signed [15:0] FIR307_ToQ4;
        input signed [39:0] accumulator_q16;
        reg signed [40:0] magnitude;
        reg signed [40:0] rounded_q4;
        begin
            if (accumulator_q16 >= 0) begin
                rounded_q4 =
                    ($signed({1'b0, accumulator_q16}) + 41'sd2048) >>> 12;
            end else begin
                magnitude = -$signed({accumulator_q16[39],
                                      accumulator_q16});
                rounded_q4 = -((magnitude + 41'sd2048) >>> 12);
            end

            if (rounded_q4 > 41'sd32767) begin
                FIR307_ToQ4 = 16'sh7FFF;
            end else if (rounded_q4 < -41'sd32768) begin
                FIR307_ToQ4 = -16'sd32768;
            end else begin
                FIR307_ToQ4 = rounded_q4[15:0];
            end
        end
    endfunction

    /*
     * 153 symmetric pre-add/multiply paths.  Each generate iteration has
     * constant tap indices and a constant coefficient, so synthesis unrolls
     * this into 153 independent full-rate paths.
     */
    genvar symmetric_index;
    generate
        for (symmetric_index = 0;
             symmetric_index < 153;
             symmetric_index = symmetric_index + 1) begin : g_symmetric_product
            wire signed [12:0] left_sample;
            wire signed [12:0] right_sample;
            wire signed [12:0] symmetric_sum;
            wire signed [17:0] coefficient;
            (* use_dsp = "yes" *)
            wire signed [30:0] product_value;

            if (symmetric_index == 0) begin : g_current_sample
                assign left_sample = centered_input;
            end else begin : g_delayed_left_sample
                assign left_sample =
                    {{1{delay_line[symmetric_index-1][11]}},
                     delay_line[symmetric_index-1]};
            end
            assign right_sample =
                {{1{delay_line[HISTORY_SAMPLE_COUNT-1-symmetric_index][11]}},
                 delay_line[HISTORY_SAMPLE_COUNT-1-symmetric_index]};
            assign symmetric_sum = left_sample + right_sample;
            assign coefficient = FIR307_Coefficient(symmetric_index);
            assign product_value = symmetric_sum * coefficient;

            always @(posedge clk or negedge reset_n) begin
                if (!reset_n) begin
                    product_pipe[symmetric_index] <= 40'sd0;
                end else if (clear) begin
                    product_pipe[symmetric_index] <= 40'sd0;
                end else if (input_valid) begin
                    product_pipe[symmetric_index] <=
                        {{9{product_value[30]}}, product_value};
                end else begin
                    product_pipe[symmetric_index] <= 40'sd0;
                end
            end
        end
    endgenerate

    /*
     * Centre tap x[n-153] * h[153].  It is the 154th independent multiply.
     */
    wire signed [12:0] center_sample =
        {{1{delay_line[152][11]}}, delay_line[152]};
    wire signed [17:0] center_coefficient = FIR307_Coefficient(8'd153);
    (* use_dsp = "yes" *)
    wire signed [30:0] center_product = center_sample * center_coefficient;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            product_pipe[153] <= 40'sd0;
        end else if (clear) begin
            product_pipe[153] <= 40'sd0;
        end else if (input_valid) begin
            product_pipe[153] <=
                {{9{center_product[30]}}, center_product};
        end else begin
            product_pipe[153] <= 40'sd0;
        end
    end

    /*
     * Valid-sample history.  clear removes both signal history and the
     * fill-state, so no stale sample can become valid after reconfiguration.
     */
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            valid_sample_count <= 9'd0;
            for (reset_index = 0;
                 reset_index < HISTORY_SAMPLE_COUNT;
                 reset_index = reset_index + 1) begin
                delay_line[reset_index] <= 12'sd0;
            end
        end else if (clear) begin
            valid_sample_count <= 9'd0;
            for (reset_index = 0;
                 reset_index < HISTORY_SAMPLE_COUNT;
                 reset_index = reset_index + 1) begin
                delay_line[reset_index] <= 12'sd0;
            end
        end else if (input_valid) begin
            if (valid_sample_count < 9'd307) begin
                valid_sample_count <= valid_sample_count + 1'b1;
            end
            for (reset_index = HISTORY_SAMPLE_COUNT-1;
                 reset_index > 0;
                 reset_index = reset_index - 1) begin
                delay_line[reset_index] <= delay_line[reset_index-1];
            end
            delay_line[0] <= centered_input[11:0];
        end
    end

    /*
     * Balanced registered tree:
     *   154 -> 77 -> 39 -> 20 -> 10 -> 5 -> 3 -> 2 -> 1.
     * Odd entries are forwarded through a register at their current level.
     */
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            for (tree_index = 0; tree_index < 77;
                 tree_index = tree_index + 1)
                sum_level_1[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 39;
                 tree_index = tree_index + 1)
                sum_level_2[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 20;
                 tree_index = tree_index + 1)
                sum_level_3[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 10;
                 tree_index = tree_index + 1)
                sum_level_4[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 5;
                 tree_index = tree_index + 1)
                sum_level_5[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 3;
                 tree_index = tree_index + 1)
                sum_level_6[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 2;
                 tree_index = tree_index + 1)
                sum_level_7[tree_index] <= 40'sd0;
            sum_level_8 <= 40'sd0;
        end else if (clear) begin
            for (tree_index = 0; tree_index < 77;
                 tree_index = tree_index + 1)
                sum_level_1[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 39;
                 tree_index = tree_index + 1)
                sum_level_2[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 20;
                 tree_index = tree_index + 1)
                sum_level_3[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 10;
                 tree_index = tree_index + 1)
                sum_level_4[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 5;
                 tree_index = tree_index + 1)
                sum_level_5[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 3;
                 tree_index = tree_index + 1)
                sum_level_6[tree_index] <= 40'sd0;
            for (tree_index = 0; tree_index < 2;
                 tree_index = tree_index + 1)
                sum_level_7[tree_index] <= 40'sd0;
            sum_level_8 <= 40'sd0;
        end else begin
            for (tree_index = 0; tree_index < 77;
                 tree_index = tree_index + 1)
                sum_level_1[tree_index] <=
                    product_pipe[2*tree_index] +
                    product_pipe[2*tree_index+1];

            for (tree_index = 0; tree_index < 38;
                 tree_index = tree_index + 1)
                sum_level_2[tree_index] <=
                    sum_level_1[2*tree_index] +
                    sum_level_1[2*tree_index+1];
            sum_level_2[38] <= sum_level_1[76];

            for (tree_index = 0; tree_index < 19;
                 tree_index = tree_index + 1)
                sum_level_3[tree_index] <=
                    sum_level_2[2*tree_index] +
                    sum_level_2[2*tree_index+1];
            sum_level_3[19] <= sum_level_2[38];

            for (tree_index = 0; tree_index < 10;
                 tree_index = tree_index + 1)
                sum_level_4[tree_index] <=
                    sum_level_3[2*tree_index] +
                    sum_level_3[2*tree_index+1];

            for (tree_index = 0; tree_index < 5;
                 tree_index = tree_index + 1)
                sum_level_5[tree_index] <=
                    sum_level_4[2*tree_index] +
                    sum_level_4[2*tree_index+1];

            sum_level_6[0] <= sum_level_5[0] + sum_level_5[1];
            sum_level_6[1] <= sum_level_5[2] + sum_level_5[3];
            sum_level_6[2] <= sum_level_5[4];

            sum_level_7[0] <= sum_level_6[0] + sum_level_6[1];
            sum_level_7[1] <= sum_level_6[2];

            sum_level_8 <= sum_level_7[0] + sum_level_7[1];
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            valid_pipeline <= {(PIPELINE_TREE_LEVELS+1){1'b0}};
            output_valid <= 1'b0;
            output_sample_q4 <= 16'sd0;
        end else if (clear) begin
            valid_pipeline <= {(PIPELINE_TREE_LEVELS+1){1'b0}};
            output_valid <= 1'b0;
            output_sample_q4 <= 16'sd0;
        end else begin
            valid_pipeline[0] <= input_valid && history_full;
            valid_pipeline[PIPELINE_TREE_LEVELS:1] <=
                valid_pipeline[PIPELINE_TREE_LEVELS-1:0];
            output_valid <= valid_pipeline[PIPELINE_TREE_LEVELS];
            if (valid_pipeline[PIPELINE_TREE_LEVELS]) begin
                output_sample_q4 <= FIR307_ToQ4(sum_level_8);
            end else begin
                output_sample_q4 <= 16'sd0;
            end
        end
    end
endmodule
