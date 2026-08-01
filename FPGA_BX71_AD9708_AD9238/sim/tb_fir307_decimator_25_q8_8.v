`timescale 1ns / 1ps

module tb_fir307_decimator_25_q8_8;
    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg clear = 1'b0;
    reg input_valid = 1'b0;
    reg [11:0] input_code = 12'd2048;
    wire output_valid;
    wire signed [15:0] output_sample_q8_8;

    integer reference_history [0:511];
    reg signed [15:0] expected_samples [0:127];
    integer reference_write_pointer;
    integer reference_valid_count;
    integer reference_phase;
    integer expected_write_count;
    integer expected_read_count;
    integer sample_index;
    integer tap_index;
    integer sample_address;
    integer centered_sample;
    reg signed [63:0] convolution_accumulator;
    reg signed [17:0] coefficient;

    always #10 clk = ~clk;

    fir307_decimator_25_q8_8 dut (
        .clk(clk),
        .reset_n(reset_n),
        .clear(clear),
        .input_valid(input_valid),
        .input_code(input_code),
        .output_valid(output_valid),
        .output_sample_q8_8(output_sample_q8_8)
    );

    function [11:0] TestCode;
        input integer index;
        integer centered;
        integer code;
        begin
            centered = ((index * 73 + index * index * 11) % 1801) - 900;
            code = 2048 + centered;
            TestCode = code[11:0];
        end
    endfunction

    function signed [17:0] FullCoefficient;
        input integer index;
        begin
            if (index <= 153)
                FullCoefficient = dut.FIR307_Coefficient(index[7:0]);
            else
                FullCoefficient =
                    dut.FIR307_Coefficient((306 - index) & 8'hFF);
        end
    endfunction

    always @(posedge clk) begin
        if (!reset_n || clear) begin
            reference_write_pointer = 0;
            reference_valid_count = 0;
            reference_phase = 0;
            expected_write_count = 0;
            expected_read_count = 0;
            for (sample_address = 0; sample_address < 512;
                 sample_address = sample_address + 1)
                reference_history[sample_address] = 0;
        end else if (input_valid) begin
            centered_sample = $signed({1'b0, input_code}) - 2048;
            if ((reference_valid_count >= 306) &&
                (reference_phase == 24)) begin
                convolution_accumulator =
                    centered_sample * FullCoefficient(0);
                for (tap_index = 1; tap_index <= 306;
                     tap_index = tap_index + 1) begin
                    sample_address =
                        (reference_write_pointer - tap_index) & 511;
                    coefficient = FullCoefficient(tap_index);
                    convolution_accumulator = convolution_accumulator +
                        reference_history[sample_address] * coefficient;
                end
                expected_samples[expected_write_count] =
                    dut.FIR307_ToQ8_8(convolution_accumulator[39:0]);
                expected_write_count = expected_write_count + 1;
            end

            reference_history[reference_write_pointer] = centered_sample;
            reference_write_pointer = (reference_write_pointer + 1) & 511;
            if (reference_valid_count < 307)
                reference_valid_count = reference_valid_count + 1;
            if (reference_phase == 24)
                reference_phase = 0;
            else
                reference_phase = reference_phase + 1;
        end
    end

    always @(negedge clk) begin
        if (reset_n && output_valid) begin
            if (expected_read_count >= expected_write_count)
                $fatal(1, "Unexpected decimator output: %0d",
                       output_sample_q8_8);
            if ($signed(output_sample_q8_8) !==
                $signed(expected_samples[expected_read_count]))
                $fatal(1,
                       "Output %0d mismatch: got %0d expected %0d",
                       expected_read_count, output_sample_q8_8,
                       expected_samples[expected_read_count]);
            expected_read_count = expected_read_count + 1;
        end
    end

    initial begin
        repeat (5) @(posedge clk);
        reset_n = 1'b1;
        @(negedge clk);
        clear = 1'b1;
        @(negedge clk);
        clear = 1'b0;

        for (sample_index = 0; sample_index < 1800;
             sample_index = sample_index + 1) begin
            input_valid = 1'b1;
            input_code = TestCode(sample_index);
            @(negedge clk);
        end
        input_valid = 1'b0;
        input_code = 12'd2048;
        repeat (40) @(negedge clk);

        if (expected_write_count < 50)
            $fatal(1, "Too few reference outputs: %0d",
                   expected_write_count);
        if (expected_read_count != expected_write_count)
            $fatal(1, "Output count mismatch: got %0d expected %0d",
                   expected_read_count, expected_write_count);
        $display("FIR307_DECIMATOR_25_Q8_8_TEST_PASS outputs=%0d",
                 expected_read_count);
        $finish;
    end
endmodule
