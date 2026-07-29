`timescale 1ns / 1ps

module tb_cic3_decimator_16;
    localparam real PI = 3.14159265358979323846;
    localparam integer INPUT_SAMPLE_RATE_HZ = 32000000;
    localparam integer OUTPUT_SAMPLE_RATE_HZ = 2000000;
    localparam integer TONE_OUTPUT_COUNT = 4000;
    localparam integer TONE_AMPLITUDE_CODES = 64;

    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg clear = 1'b0;
    reg input_valid = 1'b0;
    reg [7:0] input_code = 8'd128;
    wire output_valid;
    wire signed [15:0] output_sample_q8_8;
    integer input_index;
    integer output_count;

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

    task ClearFilter;
        begin
            @(negedge clk);
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
            ClearFilter();
            input_valid = 1'b1;
            input_code = code;
            output_count = 0;
            while (output_count < 8) begin
                @(posedge clk);
                #1;
                if (output_valid) begin
                    if ($signed(output_sample_q8_8) !== expected) begin
                        $fatal(1,
                               "CIC DC mismatch code=%0d actual=%0d expected=%0d",
                               code, $signed(output_sample_q8_8), expected);
                    end
                    output_count = output_count + 1;
                end
            end
        end
    endtask

    task RunToneTest;
        input integer frequency_hz;
        integer code_integer;
        integer sample_integer;
        integer fractional_count;
        real input_angle;
        real output_angle;
        real sine_sum;
        real cosine_sum;
        real measured_amplitude_q8_8;
        real response_ratio;
        real expected_amplitude_q8_8;
        real amplitude_error_q8_8;
        begin
            ClearFilter();
            input_valid = 1'b1;
            input_index = 0;
            output_count = 0;
            sine_sum = 0.0;
            cosine_sum = 0.0;
            fractional_count = 0;

            while (output_count < TONE_OUTPUT_COUNT) begin
                @(negedge clk);
                input_angle =
                    2.0 * PI * frequency_hz * input_index /
                    INPUT_SAMPLE_RATE_HZ;
                code_integer = $rtoi(
                    128.5 +
                    TONE_AMPLITUDE_CODES * $sin(input_angle));
                if (code_integer < 0)
                    code_integer = 0;
                else if (code_integer > 255)
                    code_integer = 255;
                input_code = code_integer[7:0];
                input_index = input_index + 1;

                @(posedge clk);
                #1;
                if (output_valid) begin
                    output_angle =
                        2.0 * PI * frequency_hz * output_count /
                        OUTPUT_SAMPLE_RATE_HZ;
                    sample_integer = $signed(output_sample_q8_8);
                    if (output_sample_q8_8[7:0] != 8'h00)
                        fractional_count = fractional_count + 1;
                    sine_sum = sine_sum +
                               sample_integer * $sin(output_angle);
                    cosine_sum = cosine_sum +
                                 sample_integer * $cos(output_angle);
                    output_count = output_count + 1;
                end
            end

            measured_amplitude_q8_8 =
                2.0 * $sqrt(sine_sum * sine_sum +
                            cosine_sum * cosine_sum) /
                TONE_OUTPUT_COUNT;
            response_ratio =
                $sin(PI * frequency_hz * 16.0 /
                     INPUT_SAMPLE_RATE_HZ) /
                (16.0 * $sin(PI * frequency_hz /
                             INPUT_SAMPLE_RATE_HZ));
            response_ratio =
                response_ratio * response_ratio * response_ratio;
            expected_amplitude_q8_8 =
                TONE_AMPLITUDE_CODES * 256.0 * response_ratio;
            amplitude_error_q8_8 =
                measured_amplitude_q8_8 - expected_amplitude_q8_8;
            if (amplitude_error_q8_8 < 0.0)
                amplitude_error_q8_8 = -amplitude_error_q8_8;

            /*
             * The source itself is first quantized to the AD9280's eight-bit
             * codes.  Eight Q8.8 counts are 1/32 of an ADC code.  The
             * fractional-output assertion independently prevents a regression
             * to integer eight-bit samples, even if a particular tone happens
             * to have a small fitted gain error after requantization.
             */
            if (amplitude_error_q8_8 > 8.0) begin
                $fatal(1,
                       "CIC tone gain mismatch f=%0d measured=%f expected=%f error=%f",
                       frequency_hz, measured_amplitude_q8_8,
                       expected_amplitude_q8_8, amplitude_error_q8_8);
            end
            if (fractional_count < (TONE_OUTPUT_COUNT / 2)) begin
                $fatal(1,
                       "CIC fractional precision missing f=%0d fractional=%0d/%0d",
                       frequency_hz, fractional_count, TONE_OUTPUT_COUNT);
            end
            $display("CIC_TONE_PASS f=%0d measured_q8_8=%f expected_q8_8=%f",
                     frequency_hz, measured_amplitude_q8_8,
                     expected_amplitude_q8_8);
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        reset_n = 1'b1;

        /*
         * Directly verify symmetric round-to-nearest, ties-to-even.  The
         * function is also exercised beyond both Q8.8 rails to verify signed
         * saturation rather than wraparound.
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

        /* A 2 MHz square wave is at the CIC's first null. */
        ClearFilter();
        input_valid = 1'b1;
        output_count = 0;
        input_index = 0;
        while (output_count < 8) begin
            @(negedge clk);
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
                output_count = output_count + 1;
            end
        end

        RunToneTest(10000);
        RunToneTest(20000);
        RunToneTest(30000);

        $display("G_CIC3_DECIMATOR_TEST_PASS");
        $finish;
    end
endmodule
