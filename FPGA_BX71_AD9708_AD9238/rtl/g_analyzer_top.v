`timescale 1ns / 1ps

/*
 * Timing-clean top level for the 2026 G signal analyzer.
 * AD9238 channel 1 is the calibrated measurement input. Channel 2 receives
 * the unamplified signal through the external 15 MHz low-pass filter and is
 * used only as an anti-alias reference. Both ADCs use the same 50 MHz clock.
 */
module g_analyzer_top(
    input  wire        clk_50m,
    input  wire        reset_n,
    input  wire        stm32_spi_sck,
    input  wire        stm32_spi_mosi,
    output wire        stm32_spi_miso,
    input  wire        stm32_spi_cs_n,
    output wire        stm32_irq,
    output wire        ad9708_clk_o,
    output wire [7:0]  ad9708_data_o,
    output wire        ad9238_ch1_clk_o,
    output wire        ad9238_ch2_clk_o,
    input  wire [11:0] ad9238_ch1_data_i,
    input  wire        ad9238_ch1_otr_i,
    input  wire [11:0] ad9238_ch2_data_i,
    input  wire        ad9238_ch2_otr_i,
    output wire        led
);

    localparam [6:0] REG_READ_CHANNEL = 7'h30;

    wire [6:0] spi_addr;
    wire [31:0] spi_wdata;
    wire spi_write;
    wire spi_read_commit;
    reg [31:0] spi_rdata;

    wire adc_ch1_bus_select;
    wire adc_ch2_bus_select;
    wire [31:0] adc_ch1_bus_rdata;
    wire [31:0] adc_ch2_bus_rdata;
    reg read_channel_cfg;
    wire clk_adc;
    wire adc_pll_locked;
    wire adc_reset_n = reset_n & adc_pll_locked;
    (* IOB = "TRUE" *) reg [11:0] ad9238_ch1_data_iob;
    (* IOB = "TRUE" *) reg ad9238_ch1_otr_iob;
    (* IOB = "TRUE" *) reg [11:0] ad9238_ch2_data_iob;
    (* IOB = "TRUE" *) reg ad9238_ch2_otr_iob;

    wire clk_dac;
    wire dac_pll_locked;
    wire dac_reset_n = reset_n & dac_pll_locked;
    (* IOB = "TRUE" *) reg [7:0] dac_idle_data;

    assign ad9708_data_o = dac_idle_data;
    assign stm32_irq = adc_pll_locked;
    assign led = adc_pll_locked;

    stm32_spi_reg_slave spi_if (
        .clk(clk_50m),
        .reset_n(reset_n),
        .spi_sck(stm32_spi_sck),
        .spi_mosi(stm32_spi_mosi),
        .spi_miso(stm32_spi_miso),
        .spi_cs_n(stm32_spi_cs_n),
        .reg_addr(spi_addr),
        .reg_wdata(spi_wdata),
        .reg_write(spi_write),
        .reg_read_commit(spi_read_commit),
        .reg_rdata(spi_rdata)
    );

    ad9238_clkgen adc_clkgen (
        .clk_50m(clk_50m),
        .reset(~reset_n),
        .clk_adc(clk_adc),
        .locked(adc_pll_locked)
    );

    always @(posedge clk_adc or negedge adc_reset_n) begin
        if (!adc_reset_n) begin
            ad9238_ch1_data_iob <= 12'd2048;
            ad9238_ch1_otr_iob <= 1'b0;
            ad9238_ch2_data_iob <= 12'd2048;
            ad9238_ch2_otr_iob <= 1'b0;
        end else begin
            ad9238_ch1_data_iob <= ad9238_ch1_data_i;
            ad9238_ch1_otr_iob <= ad9238_ch1_otr_i;
            ad9238_ch2_data_iob <= ad9238_ch2_data_i;
            ad9238_ch2_otr_iob <= ad9238_ch2_otr_i;
        end
    end

    ODDR #(
        .DDR_CLK_EDGE("SAME_EDGE"),
        .INIT(1'b0),
        .SRTYPE("SYNC")
    ) adc_ch1_clk_oddr (
        .Q(ad9238_ch1_clk_o),
        .C(clk_adc),
        .CE(1'b1),
        .D1(1'b1),
        .D2(1'b0),
        .R(~adc_reset_n),
        .S(1'b0)
    );

    ODDR #(
        .DDR_CLK_EDGE("SAME_EDGE"),
        .INIT(1'b0),
        .SRTYPE("SYNC")
    ) adc_ch2_clk_oddr (
        .Q(ad9238_ch2_clk_o),
        .C(clk_adc),
        .CE(1'b1),
        .D1(1'b1),
        .D2(1'b0),
        .R(~adc_reset_n),
        .S(1'b0)
    );
    /*
     * Configuration and start/abort writes are broadcast to both capture
     * cores in the same cfg_clk cycle. Channel 1 keeps its existing full-rate
     * FIR+CIC path. Channel 2 uses a small multiplier-shared FIR/R25 path, so
     * out-of-band energy is removed before either stream is down-sampled.
     */
    ad9238_capture_core #(
        .DSP_ENABLED(1),
        .REFERENCE_FILTER_ENABLED(0)
    ) adc_capture (
        .cfg_clk(clk_50m),
        .adc_clk(clk_adc),
        .reset_n(reset_n),
        .adc_ready(adc_pll_locked),
        .adc_data(ad9238_ch1_data_iob),
        .adc_otr(ad9238_ch1_otr_iob),
        .bus_addr(spi_addr),
        .bus_wdata(spi_wdata),
        .bus_write(spi_write),
        .bus_read_commit(spi_read_commit && !read_channel_cfg),
        .bus_select(adc_ch1_bus_select),
        .bus_rdata(adc_ch1_bus_rdata)
    );

    ad9238_capture_core #(
        .DSP_ENABLED(0),
        .REFERENCE_FILTER_ENABLED(1)
    ) adc_capture_ch2 (
        .cfg_clk(clk_50m),
        .adc_clk(clk_adc),
        .reset_n(reset_n),
        .adc_ready(adc_pll_locked),
        .adc_data(ad9238_ch2_data_iob),
        .adc_otr(ad9238_ch2_otr_iob),
        .bus_addr(spi_addr),
        .bus_wdata(spi_wdata),
        .bus_write(spi_write),
        .bus_read_commit(spi_read_commit && read_channel_cfg),
        .bus_select(adc_ch2_bus_select),
        .bus_rdata(adc_ch2_bus_rdata)
    );

    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin
            read_channel_cfg <= 1'b0;
        end else if (spi_write && (spi_addr == REG_READ_CHANNEL)) begin
            read_channel_cfg <= spi_wdata[0];
        end
    end

    always @(*) begin
        if (spi_addr == REG_READ_CHANNEL)
            spi_rdata = {31'd0, read_channel_cfg};
        else if (read_channel_cfg && adc_ch2_bus_select)
            spi_rdata = adc_ch2_bus_rdata;
        else if (adc_ch1_bus_select)
            spi_rdata = adc_ch1_bus_rdata;
        else
            spi_rdata = 32'd0;
    end

    /* Keep the unused DAC at a quiet midscale code with a legal clock. */
    ad9708_clkgen dac_clkgen (
        .clk_50m(clk_50m),
        .reset(~reset_n),
        .clk_dac(clk_dac),
        .locked(dac_pll_locked)
    );

    always @(posedge clk_dac or negedge dac_reset_n) begin
        if (!dac_reset_n) dac_idle_data <= 8'd128;
        else dac_idle_data <= 8'd128;
    end

    ODDR #(
        .DDR_CLK_EDGE("SAME_EDGE"),
        .INIT(1'b0),
        .SRTYPE("SYNC")
    ) dac_clk_oddr (
        .Q(ad9708_clk_o),
        .C(clk_dac),
        .CE(1'b1),
        .D1(1'b0),
        .D2(1'b1),
        .R(~dac_reset_n),
        .S(1'b0)
    );

endmodule
