`timescale 1ns / 1ps

/*
 * Three-stage CIC decimator for ACM9238 channel 1.
 * Input:  12-bit unsigned offset-binary AD9238 code at 50 MSPS.
 * Output: signed two's-complement Q8.8 sample at 2 MSPS (R=25).
 *
 * The R=25 CIC gain is 25^3=15625.  The output transport is normalized to
 * the 16-bit Q8.8 convention used by the analyzer, so one native ADC code is
 * 16 transport counts:
 *
 *     q8_8 = round_to_nearest(comb_value * 16 / 15625)
 *
 * This division normalizes DC gain only.  The deterministic CIC passband
 * magnitude/phase inverse is applied in the paired STM32 analyzer; residual
 * analog-front-end/PCB/cable error remains an empirical calibration layer.
 *
 * Integrator, comb, and scaling stages are registered separately.  This is
 * important at 50 MHz: the ADC input must never see a single combinational
 * path through the complete CIC and the constant divider.
 */
module cic3_decimator_25(
    input  wire              clk,
    input  wire              reset_n,
    input  wire              clear,
    input  wire              input_valid,
    input  wire [11:0]       input_code,
    output reg               output_valid,
    output reg signed [15:0] output_sample_q8_8
);

    localparam [4:0] DECIMATION = 5'd25;
    localparam [2:0] WARMUP_OUTPUTS = 3'd3;
    localparam [22:0] RECIPROCAL_Q32 = 23'd4398047;

    reg signed [31:0] integrator1;
    reg signed [31:0] integrator2;
    reg signed [31:0] integrator3;
    reg signed [31:0] comb_delay1;
    reg signed [31:0] comb_delay2;
    reg signed [31:0] comb_delay3;
    reg signed [31:0] comb1_stage;
    reg signed [31:0] comb2_stage;
    reg signed [31:0] comb3_stage;
    reg [54:0] reciprocal_product;
    reg product_negative;
    reg comb1_valid;
    reg comb2_valid;
    reg comb3_valid;
    reg product_valid;
    reg comb1_emit;
    reg comb2_emit;
    reg comb3_emit;
    reg product_emit;
    reg [4:0] phase;
    reg [2:0] warmup_count;

    wire signed [12:0] centered =
        $signed({1'b0, input_code}) - 13'sd2048;
    wire signed [31:0] centered_ext =
        {{19{centered[12]}}, centered};
    wire signed [31:0] integrator1_next = integrator1 + centered_ext;
    wire signed [31:0] integrator2_next = integrator2 + integrator1;
    wire signed [31:0] integrator3_next = integrator3 + integrator2;

    wire [31:0] comb3_magnitude =
        comb3_stage[31] ? (~comb3_stage + 32'd1) : comb3_stage;
    wire [55:0] rounded_product =
        {1'b0, reciprocal_product} + 56'd2147483648;
    wire [23:0] rounded_magnitude = rounded_product[55:32];

    /*
     * Exact reference quantizer used by the testbench to prove rounding and
     * saturation boundaries.  Hardware uses the registered reciprocal path.
     */
    function signed [15:0] CIC_ToQ8_8;
        input signed [31:0] value;
        reg [32:0] magnitude;
        reg [36:0] numerator;
        reg [36:0] quotient;
        reg [13:0] remainder;
        reg signed [37:0] scaled;
        begin
            if (value[31])
                magnitude = {1'b0, ~value} + 33'd1;
            else
                magnitude = {1'b0, value};

            numerator = magnitude << 4;
            quotient = numerator / 14'd15625;
            remainder = numerator % 14'd15625;
            if (remainder >= 14'd7813)
                quotient = quotient + 1'b1;

            if (value[31])
                scaled = -$signed({1'b0, quotient});
            else
                scaled = $signed({1'b0, quotient});

            if (scaled > 38'sd32767)
                CIC_ToQ8_8 = 16'sh7FFF;
            else if (scaled < -38'sd32768)
                CIC_ToQ8_8 = 16'sh8000;
            else
                CIC_ToQ8_8 = scaled[15:0];
        end
    endfunction

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            integrator1 <= 32'sd0;
            integrator2 <= 32'sd0;
            integrator3 <= 32'sd0;
            comb_delay1 <= 32'sd0;
            comb_delay2 <= 32'sd0;
            comb_delay3 <= 32'sd0;
            comb1_stage <= 32'sd0;
            comb2_stage <= 32'sd0;
            comb3_stage <= 32'sd0;
            reciprocal_product <= 55'd0;
            product_negative <= 1'b0;
            comb1_valid <= 1'b0;
            comb2_valid <= 1'b0;
            comb3_valid <= 1'b0;
            product_valid <= 1'b0;
            comb1_emit <= 1'b0;
            comb2_emit <= 1'b0;
            comb3_emit <= 1'b0;
            product_emit <= 1'b0;
            phase <= 5'd0;
            warmup_count <= 3'd0;
            output_valid <= 1'b0;
            output_sample_q8_8 <= 16'sd0;
        end else begin
            output_valid <= 1'b0;
            comb1_valid <= 1'b0;

            if (clear) begin
                integrator1 <= 32'sd0;
                integrator2 <= 32'sd0;
                integrator3 <= 32'sd0;
                comb_delay1 <= 32'sd0;
                comb_delay2 <= 32'sd0;
                comb_delay3 <= 32'sd0;
                comb1_stage <= 32'sd0;
                comb2_stage <= 32'sd0;
                comb3_stage <= 32'sd0;
                reciprocal_product <= 55'd0;
                product_negative <= 1'b0;
                comb1_valid <= 1'b0;
                comb2_valid <= 1'b0;
                comb3_valid <= 1'b0;
                product_valid <= 1'b0;
                comb1_emit <= 1'b0;
                comb2_emit <= 1'b0;
                comb3_emit <= 1'b0;
                product_emit <= 1'b0;
                phase <= 5'd0;
                warmup_count <= 3'd0;
                output_sample_q8_8 <= 16'sd0;
            end else begin
                if (input_valid) begin
                    integrator1 <= integrator1_next;
                    integrator2 <= integrator2_next;
                    integrator3 <= integrator3_next;

                    if (phase == (DECIMATION - 1'b1)) begin
                        phase <= 5'd0;
                        comb1_stage <= integrator3_next - comb_delay1;
                        comb_delay1 <= integrator3_next;
                        comb1_valid <= 1'b1;
                        comb1_emit <= (warmup_count >= WARMUP_OUTPUTS);

                        if (warmup_count < WARMUP_OUTPUTS)
                            warmup_count <= warmup_count + 1'b1;
                    end else begin
                        phase <= phase + 1'b1;
                    end
                end

                comb2_valid <= comb1_valid;
                comb2_emit <= comb1_emit;
                if (comb1_valid) begin
                    comb2_stage <= comb1_stage - comb_delay2;
                    comb_delay2 <= comb1_stage;
                end

                comb3_valid <= comb2_valid;
                comb3_emit <= comb2_emit;
                if (comb2_valid) begin
                    comb3_stage <= comb2_stage - comb_delay3;
                    comb_delay3 <= comb2_stage;
                end

                product_valid <= comb3_valid;
                product_emit <= comb3_emit;
                if (comb3_valid) begin
                    reciprocal_product <=
                        comb3_magnitude * RECIPROCAL_Q32;
                    product_negative <= comb3_stage[31];
                end

                if (product_valid && product_emit) begin
                    if (product_negative) begin
                        if (rounded_magnitude >= 24'd32768)
                            output_sample_q8_8 <= 16'sh8000;
                        else
                            output_sample_q8_8 <=
                                -$signed({1'b0, rounded_magnitude[14:0]});
                    end else begin
                        if (rounded_magnitude > 24'd32767)
                            output_sample_q8_8 <= 16'sh7FFF;
                        else
                            output_sample_q8_8 <= rounded_magnitude[15:0];
                    end
                    output_valid <= 1'b1;
                end
            end
        end
    end

endmodule