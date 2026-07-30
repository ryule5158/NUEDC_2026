`timescale 1ns / 1ps

/*
 * ACM9238 sample-clock generator.
 * The module is clocked at 50 MSPS and PL decimation produces the 2 MSPS
 * analyzer stream; lowering the converter clock is intentionally avoided.
 */
module ad9238_clkgen(
    input  wire clk_50m,
    input  wire reset,
    output wire clk_adc,
    output wire locked
);

    /* MMCM反馈与32 MHz采样时钟网络。 */
    wire clkfb;
    wire clkfb_buf;
    wire clk50;

    /* 将板载50 MHz时钟精确转换为AD9238最高额定采样时钟32 MHz。 */
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKFBOUT_MULT_F(20.0),
        .CLKFBOUT_PHASE(0.0),
        .CLKIN1_PERIOD(20.0),
        .CLKOUT0_DIVIDE_F(20.0),
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0.0),
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
        .CLKOUT0(clk50),
        .CLKOUT0B(),
        .CLKOUT1(),
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

    /* 全局反馈缓冲器用于闭合MMCM反馈环。 */
    BUFG clkfb_buf_i (
        .I(clkfb),
        .O(clkfb_buf)
    );

    /* 全局时钟缓冲器向采集逻辑分发32 MHz时钟。 */
    BUFG clk50_buf_i (
        .I(clk50),
        .O(clk_adc)
    );

endmodule
