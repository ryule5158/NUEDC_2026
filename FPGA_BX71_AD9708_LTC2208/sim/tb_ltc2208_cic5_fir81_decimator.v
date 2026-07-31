`timescale 1ns / 1ps

module tb_ltc2208_cic5_fir81_decimator;
    localparam integer Q3_SCALE = 8;
    real PI = 3.14159265358979323846;
    real drive_frequency_hz;
    real phase_rad;
    real ratio;
    real alias_frequency_hz;
    real sum_sine;
    real sum_cosine;
    real fitted_amplitude;
    integer drive_amplitude;
    integer generated_sample;
    integer output_count;
    integer cycle_count;
    integer output_minimum;
    integer output_maximum;
    integer output_absolute_maximum;
    integer fractional_output_seen;
    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg clear = 1'b0;
    reg input_valid = 1'b0;
    reg drive_enable = 1'b0;
    reg signed [15:0] input_sample = 16'sd0;
    wire output_valid;
    wire signed [19:0] output_sample;

    always #3.846 clk = ~clk;

    ltc2208_cic5_fir81_decimator dut (
        .clk(clk),
        .reset_n(reset_n),
        .clear(clear),
        .input_valid(input_valid),
        .input_sample(input_sample),
        .output_valid(output_valid),
        .output_sample(output_sample)
    );

    always @(negedge clk) begin
        if (drive_enable) begin
            generated_sample = $rtoi(
                drive_amplitude * $sin(phase_rad));
            input_sample = generated_sample;
            phase_rad = phase_rad +
                2.0 * PI * drive_frequency_hz / 130000000.0;
            if (phase_rad >= 2.0 * PI)
                phase_rad = phase_rad - 2.0 * PI;
        end else begin
            input_sample = 16'sd0;
        end
    end

    task run_tone;
        input real frequency_hz;
        input integer amplitude;
        input real minimum_ratio;
        input real maximum_ratio;
        begin
            drive_enable = 1'b0;
            input_valid = 1'b0;
            clear = 1'b1;
            repeat (4) @(posedge clk);
            clear = 1'b0;
            drive_frequency_hz = frequency_hz;
            drive_amplitude = amplitude;
            phase_rad = 0.0;
            output_count = 0;
            cycle_count = 0;
            output_minimum = 524287;
            output_maximum = -524288;
            output_absolute_maximum = 0;
            sum_sine = 0.0;
            sum_cosine = 0.0;
            alias_frequency_hz = frequency_hz;
            while (alias_frequency_hz >= 2000000.0)
                alias_frequency_hz = alias_frequency_hz - 2000000.0;
            if (alias_frequency_hz > 1000000.0)
                alias_frequency_hz = 2000000.0 - alias_frequency_hz;
            drive_enable = 1'b1;
            input_valid = 1'b1;

            while ((output_count < 600) && (cycle_count < 100000)) begin
                @(posedge clk);
                cycle_count = cycle_count + 1;
                if (output_valid) begin
                    if (output_sample[2:0] != 3'b000)
                        fractional_output_seen = 1;
                    if ($signed(output_sample) < output_minimum)
                        output_minimum = $signed(output_sample);
                    if ($signed(output_sample) > output_maximum)
                        output_maximum = $signed(output_sample);
                    if ($signed(output_sample) < 0) begin
                        if (-$signed(output_sample) >
                            output_absolute_maximum)
                            output_absolute_maximum =
                                -$signed(output_sample);
                    end else if ($signed(output_sample) >
                                 output_absolute_maximum) begin
                        output_absolute_maximum = $signed(output_sample);
                    end
                    sum_cosine = sum_cosine + $signed(output_sample) *
                        $cos(2.0 * PI * alias_frequency_hz *
                             output_count / 2000000.0);
                    sum_sine = sum_sine + $signed(output_sample) *
                        $sin(2.0 * PI * alias_frequency_hz *
                             output_count / 2000000.0);
                    output_count = output_count + 1;
                end
            end
            drive_enable = 1'b0;
            input_valid = 1'b0;

            if (output_count != 600)
                $fatal(1, "tone %.0f Hz timed out with %0d outputs",
                       frequency_hz, output_count);

            if (maximum_ratio <= 0.010) begin
                ratio = output_absolute_maximum /
                        (1.0 * amplitude * Q3_SCALE);
            end else begin
                fitted_amplitude = (2.0 / output_count) *
                    $sqrt(sum_cosine * sum_cosine +
                          sum_sine * sum_sine);
                ratio = fitted_amplitude / (amplitude * Q3_SCALE);
            end
            $display("LTC2208_TONE frequency=%.0fHz ratio=%.8f min_q3=%0d max_q3=%0d",
                     frequency_hz, ratio,
                     output_minimum, output_maximum);
            if ((ratio < minimum_ratio) || (ratio > maximum_ratio))
                $fatal(1,
                       "tone %.0f Hz ratio %.8f outside %.8f..%.8f",
                       frequency_hz, ratio,
                       minimum_ratio, maximum_ratio);
        end
    endtask

    initial begin
        fractional_output_seen = 0;
        repeat (8) @(posedge clk);
        reset_n = 1'b1;

        /* Required-band flatness. */
        run_tone(10000.0, 12000, 0.985, 1.015);
        run_tone(100000.0, 12000, 0.985, 1.015);
        run_tone(200000.0, 12000, 0.985, 1.015);
        run_tone(300000.0, 12000, 0.985, 1.015);
        run_tone(400000.0, 12000, 0.985, 1.015);
        /*
         * Official minimum component is 5 mV peak. At 2.25 Vpp full scale it
         * occupies 145.6 native LTC2208 codes; use 146 to prove that the CIC
         * and FIR retain the small-signal advantage instead of requantizing it.
         */
        run_tone(10000.0, 146, 0.970, 1.030);
        run_tone(100000.0, 146, 0.970, 1.030);
        run_tone(500000.0, 146, 0.970, 1.030);
        run_tone(500000.0, 12000, 0.985, 1.015);

        /* Interference points, including the analytically worst region. */
        run_tone(1000000.0, 12000, 0.0, 0.010);
        run_tone(1250000.0, 12000, 0.0, 0.010);
        run_tone(1400000.0, 12000, 0.0, 0.010);
        run_tone(1430000.0, 12000, 0.0, 0.010);
        run_tone(1500000.0, 12000, 0.0, 0.010);
        run_tone(1600000.0, 12000, 0.0, 0.010);
        run_tone(2500000.0, 12000, 0.0, 0.010);

        /* Exact 200 mVpp interference amplitude at the nominal ADC scale. */
        run_tone(1000000.0, 2913, 0.0, 0.010);
        run_tone(1430000.0, 2913, 0.0, 0.010);
        if (!fractional_output_seen)
            $fatal(1, "signed-20 Q3 output never retained a fractional code");
        $display("LTC2208_SIGNED20_Q3_FRACTION_PASS");
        $display("LTC2208_CIC5_FIR81_TEST_PASS");
        $finish;
    end
endmodule
