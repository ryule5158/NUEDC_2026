`timescale 1ns / 1ps

module tb_fir307_antialias_q4;
    localparam real PI = 3.14159265358979323846;
    localparam integer SAMPLE_RATE_HZ = 50000000;
    localparam integer GROUP_DELAY_SAMPLES = 153;
    localparam integer PIPELINE_LATENCY_CLOCKS = 9;
    localparam integer TONE_DISCARD_COUNT = 500;
    localparam integer TONE_MEASURE_COUNT = 5000;
    localparam real ADC_CODE_LSB_MV = 10000.0 / 4096.0;

    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg clear = 1'b0;
    reg input_valid = 1'b0;
    reg [11:0] input_code = 12'd2048;
    wire output_valid;
    wire signed [15:0] output_sample_q4;

    integer impulse_response [0:306];
    real measured_amplitude_q4;
    real measured_gain;
    real measured_phase_rad;
    real measured_vpp_mv;

    always #10 clk = ~clk;

    fir307_antialias_q4 dut (
        .clk(clk),
        .reset_n(reset_n),
        .clear(clear),
        .input_valid(input_valid),
        .input_code(input_code),
        .output_valid(output_valid),
        .output_sample_q4(output_sample_q4)
    );

    function real AbsoluteReal;
        input real value;
        begin
            AbsoluteReal = (value < 0.0) ? -value : value;
        end
    endfunction

    function real WrapPhase;
        input real phase;
        begin
            WrapPhase = phase;
            while (WrapPhase > PI) WrapPhase = WrapPhase - 2.0 * PI;
            while (WrapPhase < -PI) WrapPhase = WrapPhase + 2.0 * PI;
        end
    endfunction

    function [11:0] ToneCode;
        input integer frequency_hz;
        input integer amplitude_codes;
        input integer sample_index;
        real angle;
        real sample_real;
        integer sample_integer;
        integer code_integer;
        begin
            angle = 2.0 * PI * frequency_hz * sample_index /
                    SAMPLE_RATE_HZ;
            sample_real = amplitude_codes * $sin(angle);
            if (sample_real >= 0.0)
                sample_integer = $rtoi(sample_real + 0.5);
            else
                sample_integer = $rtoi(sample_real - 0.5);
            code_integer = 2048 + sample_integer;
            if (code_integer < 0) code_integer = 0;
            if (code_integer > 4095) code_integer = 4095;
            ToneCode = code_integer[11:0];
        end
    endfunction

    task ClearFilter;
        begin
            @(negedge clk);
            input_valid = 1'b0;
            input_code = 12'd2048;
            clear = 1'b1;
            @(negedge clk);
            clear = 1'b0;
        end
    endtask

    task SendSample;
        input [11:0] code;
        begin
            @(negedge clk);
            input_valid = 1'b1;
            input_code = code;
            @(posedge clk);
            #1;
        end
    endtask

    task SendInvalidCycle;
        begin
            @(negedge clk);
            input_valid = 1'b0;
            input_code = 12'd2048;
            @(posedge clk);
            #1;
        end
    endtask

    task CheckCoefficientAndQuantizer;
        integer coefficient_sum;
        integer index;
        reg signed [15:0] quantized;
        begin
            coefficient_sum = dut.FIR307_Coefficient(8'd153);
            for (index = 0; index < 153; index = index + 1)
                coefficient_sum = coefficient_sum +
                                  2 * dut.FIR307_Coefficient(index);
            if (coefficient_sum != 65536)
                $fatal(1, "FIR coefficient sum mismatch: %0d",
                       coefficient_sum);

            quantized = dut.FIR307_ToQ4(40'sd0);
            if (quantized !== 16'sd0)
                $fatal(1, "Q4 zero mismatch: %0d", quantized);
            quantized = dut.FIR307_ToQ4(40'sd2047);
            if (quantized !== 16'sd0)
                $fatal(1, "Q4 positive below-half mismatch: %0d", quantized);
            quantized = dut.FIR307_ToQ4(40'sd2048);
            if (quantized !== 16'sd1)
                $fatal(1, "Q4 positive half mismatch: %0d", quantized);
            quantized = dut.FIR307_ToQ4(-40'sd2047);
            if (quantized !== 16'sd0)
                $fatal(1, "Q4 negative below-half mismatch: %0d", quantized);
            quantized = dut.FIR307_ToQ4(-40'sd2048);
            if (quantized !== -16'sd1)
                $fatal(1, "Q4 negative half mismatch: %0d", quantized);
            quantized = dut.FIR307_ToQ4(40'sd134217728);
            if (quantized !== 16'sh7FFF)
                $fatal(1, "Q4 positive saturation mismatch: %0d", quantized);
            quantized = dut.FIR307_ToQ4(-40'sd134221824);
            if (quantized !== -16'sd32768)
                $fatal(1, "Q4 negative saturation mismatch: %0d", quantized);

            $display("PASS coefficient_count=154 symmetric_sum=%0d Q4",
                     coefficient_sum);
        end
    endtask

    task CheckFillClearAndFullRate;
        integer valid_count;
        integer latency_count;
        integer output_count;
        begin
            ClearFilter();
            valid_count = 0;
            while (valid_count < 306) begin
                if ((valid_count % 47) == 0) begin
                    SendInvalidCycle();
                    if (output_valid)
                        $fatal(1,
                               "output_valid before 307 valid inputs during gap");
                end
                SendSample(12'd2049);
                valid_count = valid_count + 1;
                if (output_valid)
                    $fatal(1,
                           "output_valid after only %0d valid inputs",
                           valid_count);
            end

            SendSample(12'd2049);
            valid_count = valid_count + 1;
            if (output_valid)
                $fatal(1, "pipeline latency missing on valid input 307");

            latency_count = 0;
            while (!output_valid) begin
                SendInvalidCycle();
                latency_count = latency_count + 1;
                if (latency_count > 20)
                    $fatal(1, "first FIR result did not leave pipeline");
            end
            if (latency_count != PIPELINE_LATENCY_CLOCKS)
                $fatal(1,
                       "pipeline latency mismatch actual=%0d expected=%0d",
                       latency_count, PIPELINE_LATENCY_CLOCKS);
            if ($signed(output_sample_q4) !== 16'sd16)
                $fatal(1,
                       "first complete DC convolution mismatch: %0d",
                       $signed(output_sample_q4));

            /*
             * Refill the pipeline continuously and require one result on every
             * clock once the initial bubble has passed.
             */
            output_count = 0;
            /* Consume the isolated first result, then wait for the refill. */
            SendSample(12'd2049);
            while (!output_valid)
                SendSample(12'd2049);
            while (output_count < 64) begin
                SendSample(12'd2049);
                if (!output_valid)
                    $fatal(1,
                           "full-rate stream inserted an unexpected valid gap");
                if ($signed(output_sample_q4) !== 16'sd16)
                    $fatal(1, "full-rate DC mismatch: %0d",
                           $signed(output_sample_q4));
                output_count = output_count + 1;
            end

            ClearFilter();
            repeat (16) begin
                SendInvalidCycle();
                if (output_valid || ($signed(output_sample_q4) !== 16'sd0))
                    $fatal(1, "clear failed to flush FIR pipeline");
            end
            $display("PASS fill=307 valid_samples pipeline=%0d clocks full_rate",
                     PIPELINE_LATENCY_CLOCKS);
        end
    endtask

    task CheckImpulseSymmetryAndDelay;
        integer input_index;
        integer response_count;
        integer index;
        real response_sum;
        real weighted_sum;
        real measured_delay;
        begin
            ClearFilter();

            /* Fill x[0]..x[305] with zero, then put an impulse at x[306]. */
            for (input_index = 0; input_index < 306;
                 input_index = input_index + 1)
                SendSample(12'd2048);
            SendSample(12'd3072);

            response_count = 0;
            while (response_count < 307) begin
                SendSample(12'd2048);
                if (output_valid) begin
                    impulse_response[response_count] =
                        $signed(output_sample_q4);
                    response_count = response_count + 1;
                end
            end

            if (impulse_response[0] != 14)
                $fatal(1,
                       "first valid result is not h[0]*x[306]: %0d",
                       impulse_response[0]);
            for (index = 0; index < 307; index = index + 1) begin
                if (impulse_response[index] !=
                    impulse_response[306-index])
                    $fatal(1,
                           "impulse symmetry mismatch k=%0d left=%0d right=%0d",
                           index, impulse_response[index],
                           impulse_response[306-index]);
            end

            response_sum = 0.0;
            weighted_sum = 0.0;
            for (index = 0; index < 307; index = index + 1) begin
                response_sum = response_sum + impulse_response[index];
                weighted_sum = weighted_sum +
                               index * impulse_response[index];
            end
            measured_delay = weighted_sum / response_sum;
            if (AbsoluteReal(measured_delay - GROUP_DELAY_SAMPLES) > 1.0e-9)
                $fatal(1,
                       "linear-phase group delay mismatch actual=%.9f",
                       measured_delay);
            $display("PASS impulse symmetry=307 group_delay=%.3f samples",
                     measured_delay);
        end
    endtask

    task MeasureTone;
        input integer frequency_hz;
        input integer amplitude_codes;
        integer input_index;
        integer output_index;
        integer output_seen;
        integer measured_count;
        real angle;
        real sine_sum;
        real cosine_sum;
        real output_value;
        begin
            ClearFilter();
            input_index = 0;
            output_index = 306;
            output_seen = 0;
            measured_count = 0;
            sine_sum = 0.0;
            cosine_sum = 0.0;

            while (measured_count < TONE_MEASURE_COUNT) begin
                SendSample(ToneCode(frequency_hz,
                                    amplitude_codes,
                                    input_index));
                input_index = input_index + 1;
                if (output_valid) begin
                    if (output_seen >= TONE_DISCARD_COUNT) begin
                        angle = 2.0 * PI * frequency_hz * output_index /
                                SAMPLE_RATE_HZ;
                        output_value = $signed(output_sample_q4);
                        sine_sum = sine_sum + output_value * $sin(angle);
                        cosine_sum = cosine_sum + output_value * $cos(angle);
                        measured_count = measured_count + 1;
                    end
                    output_seen = output_seen + 1;
                    output_index = output_index + 1;
                end
            end

            measured_amplitude_q4 =
                2.0 * $sqrt(sine_sum * sine_sum +
                            cosine_sum * cosine_sum) /
                TONE_MEASURE_COUNT;
            measured_gain =
                measured_amplitude_q4 / (16.0 * amplitude_codes);
            measured_phase_rad = $atan2(cosine_sum, sine_sum);
            measured_vpp_mv =
                2.0 * (measured_amplitude_q4 / 16.0) * ADC_CODE_LSB_MV;
        end
    endtask

    task CheckPassTone;
        input integer frequency_hz;
        input real expected_gain;
        real expected_phase;
        real gain_error;
        real phase_error;
        begin
            MeasureTone(frequency_hz, 1000);
            expected_phase = WrapPhase(
                -2.0 * PI * frequency_hz * GROUP_DELAY_SAMPLES /
                SAMPLE_RATE_HZ);
            gain_error = AbsoluteReal(measured_gain - expected_gain);
            phase_error = AbsoluteReal(
                WrapPhase(measured_phase_rad - expected_phase));
            if (gain_error > 0.0020)
                $fatal(1,
                       "passband gain mismatch f=%0d actual=%.8f expected=%.8f",
                       frequency_hz, measured_gain, expected_gain);
            if (phase_error > 0.010)
                $fatal(1,
                       "linear phase mismatch f=%0d actual=%.8f expected=%.8f",
                       frequency_hz, measured_phase_rad, expected_phase);
            $display("PASS passband f=%0dHz gain=%.8f phase=%.6frad",
                     frequency_hz, measured_gain, measured_phase_rad);
        end
    endtask

    task CheckStopTone;
        input integer frequency_hz;
        input real expected_gain;
        real gain_error;
        begin
            MeasureTone(frequency_hz, 1000);
            gain_error = AbsoluteReal(measured_gain - expected_gain);
            if (measured_gain > 0.0030)
                $fatal(1,
                       "stopband attenuation insufficient f=%0d gain=%.8f",
                       frequency_hz, measured_gain);
            if (gain_error > 0.00020)
                $fatal(1,
                       "stopband response mismatch f=%0d actual=%.8f expected=%.8f",
                       frequency_hz, measured_gain, expected_gain);
            $display("PASS stopband f=%0dHz gain=%.8f attenuation=%.2fdB",
                     frequency_hz, measured_gain,
                     20.0 * $log10(measured_gain));
        end
    endtask

    task CheckInterferenceResidual;
        input integer frequency_hz;
        begin
            /*
             * 41 ADC codes peak is 200.195 mVpp on the nominal 10 V span.
             * The independent FIR must leave less than 0.6 mVpp.
             */
            MeasureTone(frequency_hz, 41);
            if (measured_vpp_mv > 0.6)
                $fatal(1,
                       "200mVpp interference residual too high f=%0d residual=%.6fmVpp",
                       frequency_hz, measured_vpp_mv);
            $display("PASS 200mVpp interference f=%0dHz residual=%.6fmVpp",
                     frequency_hz, measured_vpp_mv);
        end
    endtask

    initial begin
        repeat (6) @(posedge clk);
        @(negedge clk);
        reset_n = 1'b1;

        CheckCoefficientAndQuantizer();
        CheckFillClearAndFullRate();
        CheckImpulseSymmetryAndDelay();

        CheckPassTone(100000, 0.998576182561);
        CheckPassTone(200000, 0.999435740632);
        CheckPassTone(400000, 0.998629648639);
        CheckPassTone(500000, 0.998434741952);

        CheckStopTone(1000000, 0.001082054512);
        CheckStopTone(1600000, 0.001122052010);
        CheckStopTone(5000000, 0.000720965936);
        CheckStopTone(10000000, 0.001897794119);

        CheckInterferenceResidual(1000000);
        CheckInterferenceResidual(1600000);
        CheckInterferenceResidual(5000000);
        CheckInterferenceResidual(10000000);

        $display("FIR307_ANTIALIAS_Q4_TEST_PASS");
        $finish;
    end
endmodule
