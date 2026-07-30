`timescale 1ns / 1ps

module tb_cic3_decimator_25;
    localparam real PI = 3.14159265358979323846;
    localparam integer INPUT_SAMPLE_RATE_HZ = 50000000;
    localparam integer OUTPUT_SAMPLE_RATE_HZ = 2000000;
    localparam integer TONE_OUTPUT_COUNT = 4000;
    localparam real RTL_GAIN_SCALE = 1.0000003385357559;
    localparam integer FIRST_STORED_INPUT_INDEX = 99;
    localparam integer CIC_TOTAL_DELAY_INPUT_SAMPLES = 38;

    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg clear = 1'b0;
    reg input_valid = 1'b0;
    reg signed [15:0] input_sample_q4 = 16'sd0;
    wire output_valid;
    wire signed [15:0] output_sample_q8_8;
    integer input_index;
    integer output_count;

    always #10 clk = ~clk;

    cic3_decimator_25 dut (
        .clk(clk),
        .reset_n(reset_n),
        .clear(clear),
        .input_valid(input_valid),
        .input_sample_q4(input_sample_q4),
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
        input signed [31:0] value;
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
        input [11:0] code;
        input signed [15:0] expected;
        begin
            ClearFilter();
            input_valid = 1'b1;
            input_sample_q4 = ($signed({1'b0, code}) - 13'sd2048) <<< 4;
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
        input integer amplitude_codes;
        input real max_amplitude_error_q8_8;
        input real max_phase_error_rad;
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
        real measured_phase_rad;
        real measured_filter_phase_rad;
        real expected_filter_phase_rad;
        real phase_error_rad;
        begin
            input_valid = 1'b0;
            ClearFilter();
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
                    2048.5 +
                    amplitude_codes * $sin(input_angle));
                if (code_integer < 0)
                    code_integer = 0;
                else if (code_integer > 4095)
                    code_integer = 4095;
                input_sample_q4 = (code_integer - 2048) <<< 4;
                input_valid = 1'b1;
                input_index = input_index + 1;

                @(posedge clk);
                #1;
                if (output_valid) begin
                    output_angle =
                        2.0 * PI * frequency_hz * output_count /
                        OUTPUT_SAMPLE_RATE_HZ;
                    sample_integer = $signed(output_sample_q8_8);
                    if (output_sample_q8_8[3:0] != 4'h0)
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
                $sin(PI * frequency_hz * 25.0 /
                     INPUT_SAMPLE_RATE_HZ) /
                (25.0 * $sin(PI * frequency_hz /
                             INPUT_SAMPLE_RATE_HZ));
            response_ratio =
                response_ratio * response_ratio * response_ratio;
            expected_amplitude_q8_8 =
                amplitude_codes * 16.0 * response_ratio * RTL_GAIN_SCALE;
            amplitude_error_q8_8 =
                measured_amplitude_q8_8 - expected_amplitude_q8_8;
            if (amplitude_error_q8_8 < 0.0)
                amplitude_error_q8_8 = -amplitude_error_q8_8;

            if (amplitude_error_q8_8 > max_amplitude_error_q8_8) begin
                $fatal(1,
                       "CIC tone gain mismatch f=%0d measured=%f expected=%f error=%f",
                       frequency_hz, measured_amplitude_q8_8,
                       expected_amplitude_q8_8, amplitude_error_q8_8);
            end

            /*
             * The first stored value is the fourth R=25 output (input index
             * 99).  Reference phase to that physical sample time: the ideal
             * CIC delay is 36 clocks and the registered integrators add two.
             */
            measured_phase_rad = $atan2(cosine_sum, sine_sum);
            measured_filter_phase_rad = measured_phase_rad -
                2.0 * PI * frequency_hz * FIRST_STORED_INPUT_INDEX /
                INPUT_SAMPLE_RATE_HZ;
            expected_filter_phase_rad =
                -2.0 * PI * frequency_hz *
                CIC_TOTAL_DELAY_INPUT_SAMPLES / INPUT_SAMPLE_RATE_HZ;
            while (measured_filter_phase_rad > PI)
                measured_filter_phase_rad = measured_filter_phase_rad -
                                            2.0 * PI;
            while (measured_filter_phase_rad <= -PI)
                measured_filter_phase_rad = measured_filter_phase_rad +
                                            2.0 * PI;
            while (expected_filter_phase_rad > PI)
                expected_filter_phase_rad = expected_filter_phase_rad -
                                            2.0 * PI;
            while (expected_filter_phase_rad <= -PI)
                expected_filter_phase_rad = expected_filter_phase_rad +
                                            2.0 * PI;
            phase_error_rad = measured_filter_phase_rad -
                              expected_filter_phase_rad;
            while (phase_error_rad > PI)
                phase_error_rad = phase_error_rad - 2.0 * PI;
            while (phase_error_rad <= -PI)
                phase_error_rad = phase_error_rad + 2.0 * PI;
            if (phase_error_rad < 0.0)
                phase_error_rad = -phase_error_rad;
            if (phase_error_rad > max_phase_error_rad) begin
                $fatal(1,
                       "CIC tone phase mismatch f=%0d measured=%f expected=%f error=%f",
                       frequency_hz, measured_filter_phase_rad,
                       expected_filter_phase_rad, phase_error_rad);
            end

            if (fractional_count < (TONE_OUTPUT_COUNT / 2)) begin
                $fatal(1,
                       "CIC sub-code precision missing f=%0d fractional=%0d/%0d",
                       frequency_hz, fractional_count, TONE_OUTPUT_COUNT);
            end
            $display("CIC_TONE_PASS f=%0d amplitude_codes=%0d measured_q8_8=%f expected_q8_8=%f phase=%f",
                     frequency_hz, amplitude_codes,
                     measured_amplitude_q8_8,
                     expected_amplitude_q8_8,
                     measured_filter_phase_rad);
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        reset_n = 1'b1;

        /* Exact rounding thresholds for signed-Q4 value/15625. */
        CheckQuantizer(32'sd7812, 16'sd0);
        CheckQuantizer(32'sd7813, 16'sd1);
        CheckQuantizer(32'sd23437, 16'sd1);
        CheckQuantizer(32'sd23438, 16'sd2);
        CheckQuantizer(-32'sd7812, 16'sd0);
        CheckQuantizer(-32'sd7813, -16'sd1);
        CheckQuantizer(-32'sd23437, -16'sd1);
        CheckQuantizer(-32'sd23438, -16'sd2);
        CheckQuantizer(32'sd600000000, 16'sh7FFF);
        CheckQuantizer(-32'sd600000000, 16'sh8000);

        /* Positive, negative, and both full-scale 12-bit DC endpoints. */
        CheckDcCode(12'd2560, 16'sh2000);
        CheckDcCode(12'd1536, -16'sh2000);
        CheckDcCode(12'd4095, 16'sh7FF0);
        CheckDcCode(12'd0, 16'sh8000);

        /* A zero-mean period-25 sequence lies at the first 2 MHz CIC null. */
        ClearFilter();
        input_valid = 1'b1;
        output_count = 0;
        input_index = 0;
        while (output_count < 8) begin
            @(negedge clk);
            input_sample_q4 = (-12 + (input_index % 25)) <<< 4;
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

        RunToneTest(10000, 1024, 12.0, 0.015);
        RunToneTest(20000, 1024, 12.0, 0.015);
        RunToneTest(30000, 1024, 12.0, 0.015);
        RunToneTest(100000, 1024, 12.0, 0.015);
        RunToneTest(200000, 1024, 12.0, 0.015);
        RunToneTest(300000, 1024, 12.0, 0.015);
        RunToneTest(400000, 1024, 12.0, 0.015);
        RunToneTest(500000, 1024, 12.0, 0.015);

        /* 140 mVpp is about 29 AD9238 codes peak at the module input. */
        RunToneTest(100000, 29, 3.0, 0.035);
        RunToneTest(200000, 29, 3.0, 0.035);
        RunToneTest(300000, 29, 3.0, 0.035);
        RunToneTest(400000, 29, 3.0, 0.035);
        RunToneTest(500000, 29, 3.0, 0.035);

        /* Field levels: H1~5 codes, H2~3 codes, H4~29/39 codes peak. */
        RunToneTest(100000, 5, 2.0, 0.080);
        RunToneTest(200000, 3, 2.0, 0.100);
        RunToneTest(400000, 3, 2.0, 0.100);
        RunToneTest(400000, 5, 2.0, 0.080);
        RunToneTest(400000, 39, 3.0, 0.035);
        RunToneTest(500000, 39, 3.0, 0.035);
        $display("G_CIC3_DECIMATOR_R25_FIELD_LEVEL_PASS");
        $display("G_CIC3_DECIMATOR_R25_LOW_AMPLITUDE_PASS");

        $display("G_CIC3_DECIMATOR_R25_500KHZ_COVERAGE_PASS");
        $display("G_CIC3_DECIMATOR_R25_TEST_PASS");
        $finish;
    end
endmodule