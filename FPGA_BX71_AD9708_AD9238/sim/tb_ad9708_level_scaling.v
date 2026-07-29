`timescale 1ns / 1ps

module tb_ad9708_level_scaling;

    reg cfg_clk = 1'b0;
    reg dac_clk = 1'b0;
    reg reset_n = 1'b0;
    reg cfg_commit_toggle = 1'b0;
    reg enable_cfg = 1'b0;
    reg [2:0] mode_cfg = 3'd2;
    reg [31:0] freq_word_cfg = 32'h01000000;
    reg [31:0] phase_offset_cfg = 32'd0;
    reg [7:0] constant_code_cfg = 8'd128;
    reg [10:0] ram_points_cfg = 11'd1024;
    reg [7:0] amplitude_cfg = 8'd12;
    reg [7:0] amplitude_fraction_cfg = 8'd0;
    reg [7:0] offset_cfg = 8'd142;
    reg [7:0] offset_fraction_cfg = 8'd0;
    reg phase_reset_toggle = 1'b0;
    reg [1:0] mod_mode_cfg = 2'd0;
    reg [31:0] mod_freq_word_cfg = 32'd0;
    reg [15:0] am_carrier_gain_cfg = 16'd32767;
    reg [15:0] am_sideband_gain_cfg = 16'd0;
    reg [31:0] fm_deviation_word_cfg = 32'd0;
    reg [31:0] mod_phase_offset_cfg = 32'd0;
    reg sweep_enable_cfg = 1'b0;
    reg [1:0] sweep_mode_cfg = 2'd0;
    reg sweep_hold_cfg = 1'b0;
    reg sweep_direction_cfg = 1'b1;
    reg [31:0] sweep_low_cfg = 32'd0;
    reg [31:0] sweep_high_cfg = 32'd0;
    reg [31:0] sweep_step_cfg = 32'd0;
    reg [31:0] sweep_dwell_cfg = 32'd1;
    reg sweep_start_toggle = 1'b0;
    reg ram_wr_en = 1'b0;
    reg [9:0] ram_wr_addr = 10'd0;
    reg [7:0] ram_wr_data = 8'd0;

    wire [7:0] dac_data;
    wire [31:0] phase_debug;
    wire [31:0] current_freq_word;
    wire sweep_running;
    wire sweep_done;
    wire sweep_direction_up;

    integer index;
    integer minimum_code;
    integer maximum_code;
    integer transition_count;
    integer previous_code;
    integer code_sum;
    integer centre_code_count;
    reg [31:0] previous_phase;
    reg [31:0] phase_step;
    reg [31:0] minimum_phase_step;
    reg [31:0] maximum_phase_step;

    always #10 cfg_clk = ~cfg_clk;
    always #4 dac_clk = ~dac_clk;

    ad9708_wave_core dut (
        .cfg_clk(cfg_clk),
        .reset_n(reset_n),
        .dac_clk(dac_clk),
        .cfg_commit_toggle(cfg_commit_toggle),
        .enable_cfg(enable_cfg),
        .mode_cfg(mode_cfg),
        .freq_word_cfg(freq_word_cfg),
        .phase_offset_cfg(phase_offset_cfg),
        .constant_code_cfg(constant_code_cfg),
        .ram_points_cfg(ram_points_cfg),
        .amplitude_cfg(amplitude_cfg),
        .amplitude_fraction_cfg(amplitude_fraction_cfg),
        .offset_cfg(offset_cfg),
        .offset_fraction_cfg(offset_fraction_cfg),
        .phase_reset_toggle(phase_reset_toggle),
        .mod_mode_cfg(mod_mode_cfg),
        .mod_freq_word_cfg(mod_freq_word_cfg),
        .am_carrier_gain_cfg(am_carrier_gain_cfg),
        .am_sideband_gain_cfg(am_sideband_gain_cfg),
        .fm_deviation_word_cfg(fm_deviation_word_cfg),
        .mod_phase_offset_cfg(mod_phase_offset_cfg),
        .sweep_enable_cfg(sweep_enable_cfg),
        .sweep_mode_cfg(sweep_mode_cfg),
        .sweep_hold_cfg(sweep_hold_cfg),
        .sweep_direction_cfg(sweep_direction_cfg),
        .sweep_low_cfg(sweep_low_cfg),
        .sweep_high_cfg(sweep_high_cfg),
        .sweep_step_cfg(sweep_step_cfg),
        .sweep_dwell_cfg(sweep_dwell_cfg),
        .sweep_start_toggle(sweep_start_toggle),
        .ram_wr_en(ram_wr_en),
        .ram_wr_addr(ram_wr_addr),
        .ram_wr_data(ram_wr_data),
        .dac_data(dac_data),
        .phase_debug(phase_debug),
        .current_freq_word(current_freq_word),
        .sweep_running(sweep_running),
        .sweep_done(sweep_done),
        .sweep_direction_up(sweep_direction_up)
    );

    /* 提交一组配置并等待其跨入125MHz DAC时钟域。 */
    task commit_configuration;
    begin
        @(posedge cfg_clk);
        cfg_commit_toggle = ~cfg_commit_toggle;
        repeat (12) @(posedge dac_clk);
    end
    endtask

    /* 统计连续512个DAC码的范围和变化次数。 */
    task measure_output;
    begin
        minimum_code = 255;
        maximum_code = 0;
        transition_count = 0;
        code_sum = 0;
        centre_code_count = 0;
        previous_code = dac_data;
        for (index = 0; index < 512; index = index + 1) begin
            @(posedge dac_clk);
            if (dac_data < minimum_code)
                minimum_code = dac_data;
            if (dac_data > maximum_code)
                maximum_code = dac_data;
            if (dac_data != previous_code)
                transition_count = transition_count + 1;
            if ((dac_data >= 8'd126) && (dac_data <= 8'd129))
                centre_code_count = centre_code_count + 1;
            code_sum = code_sum + dac_data;
            previous_code = dac_data;
        end
    end
    endtask

    /* Measure the instantaneous DDS phase step used by hardware FM. */
    task measure_phase_steps;
    begin
        minimum_phase_step = 32'hFFFFFFFF;
        maximum_phase_step = 32'd0;
        previous_phase = phase_debug;
        for (index = 0; index < 128; index = index + 1) begin
            @(posedge dac_clk);
            phase_step = phase_debug - previous_phase;
            if (phase_step < minimum_phase_step)
                minimum_phase_step = phase_step;
            if (phase_step > maximum_phase_step)
                maximum_phase_step = phase_step;
            previous_phase = phase_debug;
        end
    end
    endtask

    initial begin
        repeat (5) @(posedge dac_clk);
        reset_n = 1'b1;

        commit_configuration();
        enable_cfg = 1'b1;
        commit_configuration();
        repeat (16) @(posedge dac_clk);
        measure_output();

        if ((minimum_code != 130) || (maximum_code != 153)) begin
            $display("TEST_FAIL: low-level range %0d..%0d", minimum_code,
                     maximum_code);
            $finish;
        end
        if (transition_count < 200) begin
            $display("TEST_FAIL: error feedback inactive, transitions=%0d",
                     transition_count);
            $finish;
        end

        enable_cfg = 1'b0;
        commit_configuration();
        amplitude_cfg = 8'd128;
        offset_cfg = 8'd128;
        commit_configuration();
        enable_cfg = 1'b1;
        commit_configuration();
        repeat (16) @(posedge dac_clk);
        measure_output();

        if ((minimum_code != 0) || (maximum_code != 255)) begin
            $display("TEST_FAIL: full-scale range %0d..%0d", minimum_code,
                     maximum_code);
            $finish;
        end

        enable_cfg = 1'b0;
        commit_configuration();
        mode_cfg = 3'd4;
        freq_word_cfg = 32'h40000000;
        amplitude_cfg = 8'd43;
        offset_cfg = 8'd136;
        offset_fraction_cfg = 8'd128;
        commit_configuration();
        enable_cfg = 1'b1;
        commit_configuration();
        repeat (16) @(posedge dac_clk);
        measure_output();

        if ((minimum_code != 93) || (maximum_code != 179)) begin
            $display("TEST_FAIL: fractional offset range %0d..%0d",
                     minimum_code, maximum_code);
            $finish;
        end
        if (code_sum != (136 * 512)) begin
            $display("TEST_FAIL: fractional offset mean sum=%0d", code_sum);
            $finish;
        end

        enable_cfg = 1'b0;
        commit_configuration();
        amplitude_cfg = 8'd42;
        amplitude_fraction_cfg = 8'd64;
        commit_configuration();
        enable_cfg = 1'b1;
        commit_configuration();
        repeat (16) @(posedge dac_clk);
        measure_output();

        if ((minimum_code != 94) || (maximum_code != 178)) begin
            $display("TEST_FAIL: fractional amplitude range %0d..%0d",
                     minimum_code, maximum_code);
            $finish;
        end
        if (code_sum != (136 * 512)) begin
            $display("TEST_FAIL: fractional amplitude mean sum=%0d", code_sum);
            $finish;
        end

        /*
         * 100% AM uses carrier_gain=sideband_gain=0.5 in Q1.15. The
         * instantaneous envelope must range from almost zero to full scale,
         * while the existing Q8.8 output stage still owns final DAC clipping.
         */
        enable_cfg = 1'b0;
        commit_configuration();
        mode_cfg = 3'd4;
        freq_word_cfg = 32'h40000000;
        amplitude_cfg = 8'd128;
        amplitude_fraction_cfg = 8'd0;
        offset_cfg = 8'd128;
        offset_fraction_cfg = 8'd0;
        mod_freq_word_cfg = 32'h04000000;
        am_carrier_gain_cfg = 16'd16384;
        am_sideband_gain_cfg = 16'd16383;
        mod_phase_offset_cfg = 32'd0;
        mod_mode_cfg = 2'd1;
        commit_configuration();
        enable_cfg = 1'b1;
        commit_configuration();
        repeat (24) @(posedge dac_clk);
        measure_output();

        if ((minimum_code > 1) || (maximum_code < 253)) begin
            $display("TEST_FAIL: AM envelope range %0d..%0d",
                     minimum_code, maximum_code);
            $finish;
        end
        if (centre_code_count < 8) begin
            $display("TEST_FAIL: AM envelope never approaches zero, count=%0d",
                     centre_code_count);
            $finish;
        end

        /*
         * FM keeps the programmed carrier word unchanged and varies only the
         * per-cycle phase increment by +/- the programmed deviation word.
         */
        enable_cfg = 1'b0;
        commit_configuration();
        freq_word_cfg = 32'h10000000;
        mod_freq_word_cfg = 32'h04000000;
        fm_deviation_word_cfg = 32'h08000000;
        mod_phase_offset_cfg = 32'd0;
        mod_mode_cfg = 2'd2;
        commit_configuration();
        enable_cfg = 1'b1;
        commit_configuration();
        repeat (24) @(posedge dac_clk);
        measure_phase_steps();

        if ((minimum_phase_step > 32'h08100000) ||
            (maximum_phase_step < 32'h17F00000)) begin
            $display("TEST_FAIL: FM step range 0x%08X..0x%08X",
                     minimum_phase_step, maximum_phase_step);
            $finish;
        end
        if (current_freq_word != 32'h10000000) begin
            $display("TEST_FAIL: FM changed carrier word to 0x%08X",
                     current_freq_word);
            $finish;
        end

        $display("TEST_PASS: Q8.8 scaling plus hardware AM/FM");
        $finish;
    end

endmodule
