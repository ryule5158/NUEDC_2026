`timescale 1ns / 1ps

module tb_cic3_decimator_16;
    localparam real PI = 3.14159265358979323846;
    localparam integer INPUT_FS_HZ = 32000000;
    localparam integer OUTPUT_FS_HZ = 2000000;
    /*
     * 4000 output samples contain an integer number of periods for every
     * frequency exercised below.  That makes the unwindowed sine fit exact
     * instead of confusing spectral leakage with CIC gain error.
     */
    localparam integer OUTPUT_COUNT = 4000;
    localparam integer AMPLITUDE_CODES = 64;
    localparam real CIC_GROUP_DELAY_INPUT_SAMPLES = 22.5;
    localparam real FIRST_REPORTED_INPUT_INDEX = 63.0;

    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg clear = 1'b0;
    reg input_valid = 1'b0;
    reg [7:0] input_code = 8'd128;
    wire output_valid;
    wire signed [15:0] output_sample_q8_8;

    integer input_index;
    integer output_index;
    integer code_integer;
    integer fractional_count;
    real input_angle;
    real output_angle;
    real sine_sum;
    real cosine_sum;
    real source_sine_sum;
    real source_cosine_sum;
    real source_amplitude_q8_8;
    real measured_q8_8;
    real cic_gain;
    real compensated_q8_8;
    real error_q8_8;
    real phase_rotation;
    real expected_sine_sum;
    real expected_cosine_sum;
    real phase_sine_signed;
    real phase_sine_error;

    always #15.625 clk = ~clk;

    cic3_decimator_16 dut (
        .clk(clk),
        .reset_n(reset_n),
        .clear(clear),
        .input_valid(input_valid),
        .input_code(input_code),
        .output_valid(output_valid),
        .output_sample_q8_8(output_sample_q8_8)
    );

    task ResetFilter;
        begin
            @(negedge clk);
            input_valid = 1'b0;
            input_code = 8'd128;
            clear = 1'b1;
            @(negedge clk);
            clear = 1'b0;
        end
    endtask

    task CheckQuantizer;
        input signed [23:0] value;
        input signed [15:0] expected;
        reg signed [15:0] actual;
        begin
            actual = dut.CIC_ToQ8_8(value);
            if (actual !== expected) begin
                $fatal(1,
                    "Q8.8 quantizer mismatch input=%0d actual=%0d expected=%0d",
                    value, actual, expected);
            end
        end
    endtask

    task CheckDcCode;
        input [7:0] code;
        input signed [15:0] expected;
        begin
            ResetFilter();
            input_valid = 1'b1;
            input_code = code;
            output_index = 0;
            while (output_index < 8) begin
                @(posedge clk);
                #1;
                if (output_valid) begin
                    if ($signed(output_sample_q8_8) !== expected) begin
                        $fatal(1,
                            "CIC DC mismatch code=%0d actual=%0d expected=%0d",
                            code, $signed(output_sample_q8_8), expected);
                    end
                    output_index = output_index + 1;
                end
            end
        end
    endtask

    task CheckTone;
        input integer frequency_hz;
        begin
            ResetFilter();
            input_valid = 1'b1;
            input_index = 0;
            output_index = 0;
            sine_sum = 0.0;
            cosine_sum = 0.0;
            source_sine_sum = 0.0;
            source_cosine_sum = 0.0;
            fractional_count = 0;

            while (output_index < OUTPUT_COUNT) begin
                input_angle =
                    2.0 * PI * frequency_hz * input_index / INPUT_FS_HZ;
                code_integer = $rtoi(
                    128.5 + AMPLITUDE_CODES * $sin(input_angle));
                if (code_integer < 0)
                    code_integer = 0;
                else if (code_integer > 255)
                    code_integer = 255;
                input_code = code_integer[7:0];

                /*
                 * Three discarded R=16 warm-up outputs consume 48 samples.
                 * The remaining 64000 input samples form a coherent record
                 * matching the 4000 measured output samples.
                 */
                if (input_index >= 48) begin
                    source_sine_sum = source_sine_sum +
                        (code_integer - 128) * $sin(input_angle);
                    source_cosine_sum = source_cosine_sum +
                        (code_integer - 128) * $cos(input_angle);
                end
                input_index = input_index + 1;

                @(posedge clk);
                #1;
                if (output_valid) begin
                    if (output_sample_q8_8[7:0] != 8'h00)
                        fractional_count = fractional_count + 1;
                    output_angle =
                        2.0 * PI * frequency_hz *
                        output_index / OUTPUT_FS_HZ;
                    sine_sum = sine_sum +
                        $signed(output_sample_q8_8) * $sin(output_angle);
                    cosine_sum = cosine_sum +
                        $signed(output_sample_q8_8) * $cos(output_angle);
                    output_index = output_index + 1;
                end
                if (output_index < OUTPUT_COUNT)
                    @(negedge clk);
            end

            measured_q8_8 =
                2.0 * $sqrt(sine_sum * sine_sum +
                            cosine_sum * cosine_sum) / OUTPUT_COUNT;
            source_amplitude_q8_8 =
                2.0 * 256.0 *
                $sqrt(source_sine_sum * source_sine_sum +
                      source_cosine_sum * source_cosine_sum) /
                (OUTPUT_COUNT * 16.0);
            cic_gain =
                $sin(PI * frequency_hz * 16.0 / INPUT_FS_HZ) /
                (16.0 * $sin(PI * frequency_hz / INPUT_FS_HZ));
            cic_gain = cic_gain * cic_gain * cic_gain;
            compensated_q8_8 = measured_q8_8 / cic_gain;
            error_q8_8 = compensated_q8_8 - source_amplitude_q8_8;
            if (error_q8_8 < 0.0) error_q8_8 = -error_q8_8;

            /*
             * The first reported sample is produced at physical input index
             * 63.  Relative to that sample time, CIC3/R16 contributes the
             * exact linear phase -omega*3*(R-1)/2 = -omega*22.5.
             */
            phase_rotation =
                2.0 * PI * frequency_hz *
                (FIRST_REPORTED_INPUT_INDEX -
                 CIC_GROUP_DELAY_INPUT_SAMPLES) / INPUT_FS_HZ;
            expected_sine_sum =
                source_sine_sum * $cos(phase_rotation) -
                source_cosine_sum * $sin(phase_rotation);
            expected_cosine_sum =
                source_cosine_sum * $cos(phase_rotation) +
                source_sine_sum * $sin(phase_rotation);
            phase_sine_signed =
                (cosine_sum * expected_sine_sum -
                 sine_sum * expected_cosine_sum) /
                ($sqrt(sine_sum * sine_sum +
                       cosine_sum * cosine_sum) *
                 $sqrt(expected_sine_sum * expected_sine_sum +
                       expected_cosine_sum * expected_cosine_sum));
            phase_sine_error = phase_sine_signed;
            if (phase_sine_error < 0.0)
                phase_sine_error = -phase_sine_error;

            /*
             * The source is first quantized to the physical 8-bit ADC code.
             * Sixteen Q8.8 LSB are 1/16 code; this bound is below the ADC's
             * quantization uncertainty while strongly detecting any wrong
             * CIC order, rate, normalization, or host correction formula.
             */
            if (error_q8_8 > 2.0) begin
                $fatal(1,
                    "CIC compensation mismatch f=%0d raw=%f gain=%f corrected=%f error=%f",
                    frequency_hz, measured_q8_8, cic_gain,
                    compensated_q8_8, error_q8_8);
            end
            if (phase_sine_error > 0.0001) begin
                $fatal(1,
                    "CIC linear-phase mismatch f=%0d sine_error=%f signed=%f",
                    frequency_hz, phase_sine_error, phase_sine_signed);
            end
            if (fractional_count < (OUTPUT_COUNT / 2)) begin
                $fatal(1,
                    "CIC fractional precision missing f=%0d fractional=%0d/%0d",
                    frequency_hz, fractional_count, OUTPUT_COUNT);
            end
            $display(
                "CIC_COMP_PASS f=%0d raw_gain=%f source_q8_8=%f corrected_q8_8=%f error=%f phase_sine_error=%f",
                frequency_hz,
                measured_q8_8 / source_amplitude_q8_8,
                source_amplitude_q8_8,
                compensated_q8_8,
                error_q8_8,
                phase_sine_error);
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        reset_n = 1'b1;

        /*
         * Preserve the low-level numerical regressions: symmetric
         * round-to-nearest/ties-to-even and signed saturation at both rails.
         */
        CheckQuantizer(24'sd7, 16'sd0);
        CheckQuantizer(24'sd8, 16'sd0);
        CheckQuantizer(24'sd9, 16'sd1);
        CheckQuantizer(24'sd23, 16'sd1);
        CheckQuantizer(24'sd24, 16'sd2);
        CheckQuantizer(24'sd40, 16'sd2);
        CheckQuantizer(-24'sd7, 16'sd0);
        CheckQuantizer(-24'sd8, 16'sd0);
        CheckQuantizer(-24'sd9, -16'sd1);
        CheckQuantizer(-24'sd23, -16'sd1);
        CheckQuantizer(-24'sd24, -16'sd2);
        CheckQuantizer(-24'sd40, -16'sd2);
        CheckQuantizer(24'sd600000, 16'sh7FFF);
        CheckQuantizer(-24'sd600000, 16'sh8000);

        /* Positive, negative, and both full-scale DC endpoints. */
        CheckDcCode(8'd160, 16'sh2000);
        CheckDcCode(8'd96, -16'sh2000);
        CheckDcCode(8'd255, 16'sh7F00);
        CheckDcCode(8'd0, 16'sh8000);

        /* A 2 MHz square wave is at the CIC's first exact null. */
        ResetFilter();
        input_valid = 1'b1;
        output_index = 0;
        input_index = 0;
        while (output_index < 8) begin
            input_code = ((input_index % 16) < 8) ? 8'd192 : 8'd64;
            input_index = input_index + 1;
            @(posedge clk);
            #1;
            if (output_valid) begin
                if (($signed(output_sample_q8_8) < -16'sd1) ||
                    ($signed(output_sample_q8_8) > 16'sd1)) begin
                    $fatal(1, "CIC first-null mismatch: %0d",
                           $signed(output_sample_q8_8));
                end
                output_index = output_index + 1;
            end
            if (output_index < 8)
                @(negedge clk);
        end

        CheckTone(10000);
        CheckTone(50000);
        CheckTone(100000);
        CheckTone(200000);
        CheckTone(300000);
        CheckTone(400000);
        CheckTone(500000);

        $display("G_CIC_RESPONSE_COMPENSATION_TEST_PASS");
        $finish;
    end
endmodule
