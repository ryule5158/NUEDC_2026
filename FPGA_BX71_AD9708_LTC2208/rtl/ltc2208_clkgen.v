`timescale 1ns / 1ps

/*
 * LTC2208 sample-clock generator for the ClearDream dual-converter module.
 *
 * The module reference design drives the converter at 130 MHz and captures
 * the parallel CMOS bus with a second 130 MHz clock.  A 99-degree capture
 * phase captures the preceding converter word with one explicit clock-period
 * latency after the clock-output, converter tD and FPGA input delays.  Keeping
 * both clocks in the same MMCM makes that relationship explicit and
 * repeatable.
 */
module ltc2208_clkgen(
    input  wire clk_50m,
    input  wire reset,
    output wire clk_adc_drive,
    output wire clk_adc_capture,
    output wire locked
);
    wire clkfb;
    wire clkfb_buf;
    wire clk130_drive_unbuffered;
    wire clk130_capture_unbuffered;

    /* 50 MHz * 13 / 5 = 130 MHz; VCO = 650 MHz. */
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKFBOUT_MULT_F(13.0),
        .CLKFBOUT_PHASE(0.0),
        .CLKIN1_PERIOD(20.0),
        .CLKOUT0_DIVIDE_F(5.0),
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DIVIDE(5),
        .CLKOUT1_DUTY_CYCLE(0.5),
        .CLKOUT1_PHASE(99.0),
        .DIVCLK_DIVIDE(1),
        .REF_JITTER1(0.01),
        .STARTUP_WAIT("FALSE")
    ) mmcm_i (
        .CLKIN1(clk_50m),
        .CLKFBIN(clkfb_buf),
        .RST(reset),
        .PWRDWN(1'b0),
        .CLKFBOUT(clkfb),
        .CLKFBOUTB(),
        .CLKOUT0(clk130_drive_unbuffered),
        .CLKOUT0B(),
        .CLKOUT1(clk130_capture_unbuffered),
        .CLKOUT1B(),
        .CLKOUT2(),
        .CLKOUT2B(),
        .CLKOUT3(),
        .CLKOUT3B(),
        .CLKOUT4(),
        .CLKOUT5(),
        .CLKOUT6(),
        .LOCKED(locked)
    );

    BUFG clkfb_buf_i (
        .I(clkfb),
        .O(clkfb_buf)
    );

    BUFG clk_drive_buf_i (
        .I(clk130_drive_unbuffered),
        .O(clk_adc_drive)
    );

    BUFG clk_capture_buf_i (
        .I(clk130_capture_unbuffered),
        .O(clk_adc_capture)
    );
endmodule
