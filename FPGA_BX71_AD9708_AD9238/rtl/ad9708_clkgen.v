`timescale 1ns / 1ps

/* AD9708时钟模块：将板载50 MHz时钟倍频为125 MHz DAC时钟。 */
module ad9708_clkgen(
    input  wire clk_50m,
    input  wire reset,
    output wire clk_dac,
    output wire locked
);

    /* PLL反馈与125 MHz生成时钟网络。 */
    wire clkfb;
    wire clkfb_buf;
    wire clk125;

    /* 20倍频后8分频，保持AD9708更新时钟由FPGA内部PLL产生。 */
    PLLE2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKFBOUT_MULT(20),
        .CLKFBOUT_PHASE(0.0),
        .CLKIN1_PERIOD(20.0),
        .CLKOUT0_DIVIDE(8),
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0.0),
        .DIVCLK_DIVIDE(1),
        .REF_JITTER1(0.01),
        .STARTUP_WAIT("FALSE")
    ) pll_i (
        .CLKIN1(clk_50m),
        .CLKFBIN(clkfb_buf),
        .RST(reset),
        .PWRDWN(1'b0),
        .CLKFBOUT(clkfb),
        .CLKOUT0(clk125),
        .CLKOUT1(),
        .CLKOUT2(),
        .CLKOUT3(),
        .CLKOUT4(),
        .CLKOUT5(),
        .LOCKED(locked)
    );

    /* 全局反馈缓冲器用于闭合PLL反馈环。 */
    BUFG clkfb_buf_i (
        .I(clkfb),
        .O(clkfb_buf)
    );

    /* 全局时钟缓冲器向DAC逻辑分发125 MHz时钟。 */
    BUFG clk125_buf_i (
        .I(clk125),
        .O(clk_dac)
    );

endmodule
