`timescale 1ns / 1ps

/*
 * LTC2208 anti-interference decimator.
 *
 *   130 MSPS signed samples
 *       -> five-stage CIC, R=65
 *       -> 81-tap linear-phase inverse-CIC low-pass
 *       -> 2 MSPS signed-20 Q3 samples
 *
 * The FIR was designed against the exact CIC5/R65 response.  Its combined
 * response is within +0.021/-0.000 dB from 10 kHz through 500 kHz.  The
 * transition starts above the required band and is stopped by 650 kHz.
 * Across a dense 1 MHz..65 MHz sweep the worst input-tone leakage after
 * decimation is below -42 dB (the limiting tone is near 1.43 MHz).
 *
 * Coefficients include both the FIR Q47 scale and the CIC DC normalization
 * 1/(65^5).  This avoids rounding the 56-bit CIC result before the equalizer.
 * Only one symmetric FIR product is issued per 130 MHz clock; 41 products
 * finish well before the next 2 MHz CIC result arrives.  Three fractional
 * raw-code bits are retained so filtering does not requantize the 16-bit ADC
 * back to integer codes.
 */
module ltc2208_cic5_fir81_decimator(
    input  wire               clk,
    input  wire               reset_n,
    input  wire               clear,
    input  wire               input_valid,
    input  wire signed [15:0] input_sample,
    output reg                output_valid,
    output reg signed [19:0]  output_sample
);
    localparam [6:0] DECIMATION = 7'd65;
    localparam [2:0] CIC_WARMUP_OUTPUTS = 3'd5;
    localparam integer FIR_TAPS = 81;
    localparam integer FIR_UNIQUE_TAPS = 41;

    reg signed [55:0] integrator1;
    reg signed [55:0] integrator2;
    reg signed [55:0] integrator3;
    reg signed [55:0] integrator4;
    reg signed [55:0] integrator5;
    reg signed [55:0] comb_delay1;
    reg signed [55:0] comb_delay2;
    reg signed [55:0] comb_delay3;
    reg signed [55:0] comb_delay4;
    reg signed [55:0] comb_delay5;
    reg signed [55:0] comb1_stage;
    reg signed [55:0] comb2_stage;
    reg signed [55:0] comb3_stage;
    reg signed [55:0] comb4_stage;
    reg signed [55:0] comb5_stage;
    reg comb1_valid;
    reg comb2_valid;
    reg comb3_valid;
    reg comb4_valid;
    reg comb5_valid;
    reg [6:0] cic_phase;

    wire signed [55:0] input_extended =
        {{40{input_sample[15]}}, input_sample};
    wire signed [55:0] integrator1_next =
        integrator1 + input_extended;
    wire signed [55:0] integrator2_next =
        integrator2 + integrator1;
    wire signed [55:0] integrator3_next =
        integrator3 + integrator2;
    wire signed [55:0] integrator4_next =
        integrator4 + integrator3;
    wire signed [55:0] integrator5_next =
        integrator5 + integrator4;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            integrator1 <= 56'sd0;
            integrator2 <= 56'sd0;
            integrator3 <= 56'sd0;
            integrator4 <= 56'sd0;
            integrator5 <= 56'sd0;
            comb_delay1 <= 56'sd0;
            comb_delay2 <= 56'sd0;
            comb_delay3 <= 56'sd0;
            comb_delay4 <= 56'sd0;
            comb_delay5 <= 56'sd0;
            comb1_stage <= 56'sd0;
            comb2_stage <= 56'sd0;
            comb3_stage <= 56'sd0;
            comb4_stage <= 56'sd0;
            comb5_stage <= 56'sd0;
            comb1_valid <= 1'b0;
            comb2_valid <= 1'b0;
            comb3_valid <= 1'b0;
            comb4_valid <= 1'b0;
            comb5_valid <= 1'b0;
            cic_phase <= 7'd0;
        end else if (clear) begin
            integrator1 <= 56'sd0;
            integrator2 <= 56'sd0;
            integrator3 <= 56'sd0;
            integrator4 <= 56'sd0;
            integrator5 <= 56'sd0;
            comb_delay1 <= 56'sd0;
            comb_delay2 <= 56'sd0;
            comb_delay3 <= 56'sd0;
            comb_delay4 <= 56'sd0;
            comb_delay5 <= 56'sd0;
            comb1_stage <= 56'sd0;
            comb2_stage <= 56'sd0;
            comb3_stage <= 56'sd0;
            comb4_stage <= 56'sd0;
            comb5_stage <= 56'sd0;
            comb1_valid <= 1'b0;
            comb2_valid <= 1'b0;
            comb3_valid <= 1'b0;
            comb4_valid <= 1'b0;
            comb5_valid <= 1'b0;
            cic_phase <= 7'd0;
        end else begin
            comb1_valid <= 1'b0;

            if (input_valid) begin
                integrator1 <= integrator1_next;
                integrator2 <= integrator2_next;
                integrator3 <= integrator3_next;
                integrator4 <= integrator4_next;
                integrator5 <= integrator5_next;

                if (cic_phase == (DECIMATION - 1'b1)) begin
                    cic_phase <= 7'd0;
                    comb1_stage <= integrator5_next - comb_delay1;
                    comb_delay1 <= integrator5_next;
                    comb1_valid <= 1'b1;
                end else begin
                    cic_phase <= cic_phase + 1'b1;
                end
            end

            comb2_valid <= comb1_valid;
            if (comb1_valid) begin
                comb2_stage <= comb1_stage - comb_delay2;
                comb_delay2 <= comb1_stage;
            end

            comb3_valid <= comb2_valid;
            if (comb2_valid) begin
                comb3_stage <= comb2_stage - comb_delay3;
                comb_delay3 <= comb2_stage;
            end

            comb4_valid <= comb3_valid;
            if (comb3_valid) begin
                comb4_stage <= comb3_stage - comb_delay4;
                comb_delay4 <= comb3_stage;
            end

            comb5_valid <= comb4_valid;
            if (comb4_valid) begin
                comb5_stage <= comb4_stage - comb_delay5;
                comb_delay5 <= comb4_stage;
            end
        end
    end

    reg signed [55:0] fir_history [0:FIR_TAPS-1];
    reg [6:0] fir_history_count;
    reg [2:0] cic_warmup_count;
    reg fir_busy;
    reg [5:0] issue_index;
    reg [5:0] accumulated_count;
    reg pair_valid;
    reg partial_valid;
    reg combine_valid;
    reg product_valid;
    reg signed [56:0] pair_pipe;
    reg signed [17:0] coefficient_pipe;
    (* use_dsp = "yes" *) reg signed [35:0] partial_low_pipe;
    (* use_dsp = "yes" *) reg signed [35:0] partial_mid_pipe;
    (* use_dsp = "yes" *) reg signed [40:0] partial_high_pipe;
    reg signed [74:0] low_mid_pipe;
    reg signed [74:0] high_shift_pipe;
    reg signed [74:0] product_pipe;
    reg signed [81:0] accumulator;
    reg signed [81:0] result_accumulator;
    reg signed [81:0] rounded_accumulator;
    reg result_pending;
    reg round_pending;
    integer shift_index;

    function signed [17:0] FIR81_Coefficient;
        input [5:0] index;
        begin
            case (index)
                6'd0:  FIR81_Coefficient = 18'sd0;
                6'd1:  FIR81_Coefficient = 18'sd0;
                6'd2:  FIR81_Coefficient = 18'sd0;
                6'd3:  FIR81_Coefficient = -18'sd1;
                6'd4:  FIR81_Coefficient = 18'sd3;
                6'd5:  FIR81_Coefficient = -18'sd1;
                6'd6:  FIR81_Coefficient = -18'sd8;
                6'd7:  FIR81_Coefficient = 18'sd9;
                6'd8:  FIR81_Coefficient = 18'sd11;
                6'd9:  FIR81_Coefficient = -18'sd27;
                6'd10: FIR81_Coefficient = 18'sd0;
                6'd11: FIR81_Coefficient = 18'sd49;
                6'd12: FIR81_Coefficient = -18'sd36;
                6'd13: FIR81_Coefficient = -18'sd50;
                6'd14: FIR81_Coefficient = 18'sd84;
                6'd15: FIR81_Coefficient = 18'sd11;
                6'd16: FIR81_Coefficient = -18'sd106;
                6'd17: FIR81_Coefficient = 18'sd42;
                6'd18: FIR81_Coefficient = 18'sd75;
                6'd19: FIR81_Coefficient = -18'sd39;
                6'd20: FIR81_Coefficient = -18'sd47;
                6'd21: FIR81_Coefficient = -18'sd68;
                6'd22: FIR81_Coefficient = 18'sd180;
                6'd23: FIR81_Coefficient = 18'sd166;
                6'd24: FIR81_Coefficient = -18'sd613;
                6'd25: FIR81_Coefficient = 18'sd87;
                6'd26: FIR81_Coefficient = 18'sd1209;
                6'd27: FIR81_Coefficient = -18'sd1107;
                6'd28: FIR81_Coefficient = -18'sd1355;
                6'd29: FIR81_Coefficient = 18'sd2965;
                6'd30: FIR81_Coefficient = 18'sd58;
                6'd31: FIR81_Coefficient = -18'sd4935;
                6'd32: FIR81_Coefficient = 18'sd3555;
                6'd33: FIR81_Coefficient = 18'sd5329;
                6'd34: FIR81_Coefficient = -18'sd9610;
                6'd35: FIR81_Coefficient = -18'sd1732;
                6'd36: FIR81_Coefficient = 18'sd17417;
                6'd37: FIR81_Coefficient = -18'sd9271;
                6'd38: FIR81_Coefficient = -18'sd28148;
                6'd39: FIR81_Coefficient = 18'sd38927;
                6'd40: FIR81_Coefficient = 18'sd95372;
                default: FIR81_Coefficient = 18'sd0;
            endcase
        end
    endfunction

    wire signed [56:0] selected_pair =
        (issue_index == 6'd40) ?
        {{1{fir_history[40][55]}}, fir_history[40]} :
        ($signed({fir_history[issue_index][55],
                  fir_history[issue_index]}) +
         $signed({fir_history[80-issue_index][55],
                  fir_history[80-issue_index]}));
    /*
     * Split the 57x18 multiply into three single-DSP products.  Registering
     * each partial product and the two following adder levels closes 130 MHz
     * timing without changing the exact two's-complement result.
     */
    wire signed [17:0] pair_low_chunk =
        $signed({1'b0, pair_pipe[16:0]});
    wire signed [17:0] pair_mid_chunk =
        $signed({1'b0, pair_pipe[33:17]});
    wire signed [22:0] pair_high_chunk =
        $signed(pair_pipe[56:34]);
    wire signed [35:0] partial_low_result =
        pair_low_chunk * coefficient_pipe;
    wire signed [35:0] partial_mid_result =
        pair_mid_chunk * coefficient_pipe;
    wire signed [40:0] partial_high_result =
        pair_high_chunk * coefficient_pipe;
    wire signed [74:0] partial_low_extended =
        {{39{partial_low_pipe[35]}}, partial_low_pipe};
    wire signed [74:0] partial_mid_shifted =
        $signed({{39{partial_mid_pipe[35]}}, partial_mid_pipe}) <<< 17;
    wire signed [74:0] partial_high_shifted =
        $signed({{34{partial_high_pipe[40]}}, partial_high_pipe}) <<< 34;
    wire signed [74:0] low_mid_result =
        partial_low_extended + partial_mid_shifted;
    wire signed [74:0] product_combine_result =
        low_mid_pipe + high_shift_pipe;
    wire signed [81:0] product_extended =
        {{7{product_pipe[74]}}, product_pipe};
    wire signed [81:0] accumulator_next =
        accumulator + product_extended;
    /*
     * Coefficients carry a Q47 normalization.  Shifting by 44 rather than 47
     * retains three fractional raw-code bits.  The guard width permits a
     * signed-20 saturation decision before capture RAM stores the result.
     */
    wire signed [37:0] rounded_q3_sample = rounded_accumulator[81:44];

    /*
     * History contents need no reset: fir_history_count is cleared and all 81
     * entries are overwritten before the first FIR transaction.  Keeping the
     * shift bank outside the asynchronous-reset block avoids set/reset logic
     * on every history bit.
     */
    always @(posedge clk) begin
        if (reset_n && !clear && comb5_valid &&
            (cic_warmup_count >= CIC_WARMUP_OUTPUTS)) begin
            for (shift_index = FIR_TAPS-1;
                 shift_index > 0;
                 shift_index = shift_index - 1) begin
                fir_history[shift_index] <= fir_history[shift_index-1];
            end
            fir_history[0] <= comb5_stage;
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            fir_history_count <= 7'd0;
            cic_warmup_count <= 3'd0;
            fir_busy <= 1'b0;
            issue_index <= 6'd0;
            accumulated_count <= 6'd0;
            pair_valid <= 1'b0;
            partial_valid <= 1'b0;
            combine_valid <= 1'b0;
            product_valid <= 1'b0;
            pair_pipe <= 57'sd0;
            coefficient_pipe <= 18'sd0;
            partial_low_pipe <= 36'sd0;
            partial_mid_pipe <= 36'sd0;
            partial_high_pipe <= 41'sd0;
            low_mid_pipe <= 75'sd0;
            high_shift_pipe <= 75'sd0;
            product_pipe <= 75'sd0;
            accumulator <= 82'sd0;
            result_accumulator <= 82'sd0;
            rounded_accumulator <= 82'sd0;
            result_pending <= 1'b0;
            round_pending <= 1'b0;
            output_valid <= 1'b0;
            output_sample <= 20'sd0;
        end else if (clear) begin
            fir_history_count <= 7'd0;
            cic_warmup_count <= 3'd0;
            fir_busy <= 1'b0;
            issue_index <= 6'd0;
            accumulated_count <= 6'd0;
            pair_valid <= 1'b0;
            partial_valid <= 1'b0;
            combine_valid <= 1'b0;
            product_valid <= 1'b0;
            pair_pipe <= 57'sd0;
            coefficient_pipe <= 18'sd0;
            partial_low_pipe <= 36'sd0;
            partial_mid_pipe <= 36'sd0;
            partial_high_pipe <= 41'sd0;
            low_mid_pipe <= 75'sd0;
            high_shift_pipe <= 75'sd0;
            product_pipe <= 75'sd0;
            accumulator <= 82'sd0;
            result_accumulator <= 82'sd0;
            rounded_accumulator <= 82'sd0;
            result_pending <= 1'b0;
            round_pending <= 1'b0;
            output_valid <= 1'b0;
            output_sample <= 20'sd0;
        end else begin
            output_valid <= 1'b0;

            if (round_pending) begin
                if (rounded_q3_sample > 38'sd524287)
                    output_sample <= 20'sh7FFFF;
                else if (rounded_q3_sample < -38'sd524288)
                    output_sample <= 20'sh80000;
                else
                    output_sample <= rounded_q3_sample[19:0];
                output_valid <= 1'b1;
                round_pending <= 1'b0;
            end

            if (result_pending) begin
                /*
                 * Q47 -> Q3: round to nearest at bit 44, with half-LSB ties
                 * away from zero.  2^43 is half of one signed-20 Q3 LSB.
                 */
                rounded_accumulator <= result_accumulator +
                    (result_accumulator[81] ?
                     82'sd8796093022207 : 82'sd8796093022208);
                result_pending <= 1'b0;
                round_pending <= 1'b1;
            end

            if (comb5_valid) begin
                if (cic_warmup_count < CIC_WARMUP_OUTPUTS) begin
                    cic_warmup_count <= cic_warmup_count + 1'b1;
                end else begin
                    if (fir_history_count < FIR_TAPS)
                        fir_history_count <= fir_history_count + 1'b1;

                    if ((fir_history_count >= FIR_TAPS-1) && !fir_busy) begin
                        fir_busy <= 1'b1;
                        issue_index <= 6'd0;
                        accumulated_count <= 6'd0;
                        pair_valid <= 1'b0;
                        partial_valid <= 1'b0;
                        combine_valid <= 1'b0;
                        product_valid <= 1'b0;
                        accumulator <= 82'sd0;
                    end
                end
            end

            if (fir_busy) begin
                if (issue_index < FIR_UNIQUE_TAPS) begin
                    pair_pipe <= selected_pair;
                    coefficient_pipe <= FIR81_Coefficient(issue_index);
                    pair_valid <= 1'b1;
                    issue_index <= issue_index + 1'b1;
                end else begin
                    pair_valid <= 1'b0;
                end

                partial_valid <= pair_valid;
                if (pair_valid) begin
                    partial_low_pipe <= partial_low_result;
                    partial_mid_pipe <= partial_mid_result;
                    partial_high_pipe <= partial_high_result;
                end

                combine_valid <= partial_valid;
                if (partial_valid) begin
                    low_mid_pipe <= low_mid_result;
                    high_shift_pipe <= partial_high_shifted;
                end

                product_valid <= combine_valid;
                if (combine_valid)
                    product_pipe <= product_combine_result;

                if (product_valid) begin
                    if (accumulated_count == FIR_UNIQUE_TAPS-1) begin
                        result_accumulator <= accumulator_next;
                        result_pending <= 1'b1;
                        fir_busy <= 1'b0;
                        pair_valid <= 1'b0;
                        partial_valid <= 1'b0;
                        combine_valid <= 1'b0;
                        product_valid <= 1'b0;
                    end else begin
                        accumulator <= accumulator_next;
                    end
                    accumulated_count <= accumulated_count + 1'b1;
                end
            end
        end
    end
endmodule
