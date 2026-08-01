`timescale 1ns / 1ps

/*
 * Persistent-boot top level for the 2026 G signal analyzer.
 * PS7 is retained so FSBL + bitstream can boot through Zynq PCAP from QSPI.
 * The measurement data path remains entirely in PL.
 */
module g_analyzer_zynq_top(
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
    output wire        led,

    inout  wire [14:0] DDR_addr,
    inout  wire [2:0]  DDR_ba,
    inout  wire        DDR_cas_n,
    inout  wire        DDR_ck_n,
    inout  wire        DDR_ck_p,
    inout  wire        DDR_cke,
    inout  wire        DDR_cs_n,
    inout  wire [3:0]  DDR_dm,
    inout  wire [31:0] DDR_dq,
    inout  wire [3:0]  DDR_dqs_n,
    inout  wire [3:0]  DDR_dqs_p,
    inout  wire        DDR_odt,
    inout  wire        DDR_ras_n,
    inout  wire        DDR_reset_n,
    inout  wire        DDR_we_n,
    inout  wire        FIXED_IO_ddr_vrn,
    inout  wire        FIXED_IO_ddr_vrp,
    inout  wire [53:0] FIXED_IO_mio,
    inout  wire        FIXED_IO_ps_clk,
    inout  wire        FIXED_IO_ps_porb,
    inout  wire        FIXED_IO_ps_srstb
);

    sys ps7_board_config (
        .DDR_addr(DDR_addr),
        .DDR_ba(DDR_ba),
        .DDR_cas_n(DDR_cas_n),
        .DDR_ck_n(DDR_ck_n),
        .DDR_ck_p(DDR_ck_p),
        .DDR_cke(DDR_cke),
        .DDR_cs_n(DDR_cs_n),
        .DDR_dm(DDR_dm),
        .DDR_dq(DDR_dq),
        .DDR_dqs_n(DDR_dqs_n),
        .DDR_dqs_p(DDR_dqs_p),
        .DDR_odt(DDR_odt),
        .DDR_ras_n(DDR_ras_n),
        .DDR_reset_n(DDR_reset_n),
        .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
        .FIXED_IO_mio(FIXED_IO_mio),
        .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb)
    );

    g_analyzer_top analyzer_logic (
        .clk_50m(clk_50m),
        .reset_n(reset_n),
        .stm32_spi_sck(stm32_spi_sck),
        .stm32_spi_mosi(stm32_spi_mosi),
        .stm32_spi_miso(stm32_spi_miso),
        .stm32_spi_cs_n(stm32_spi_cs_n),
        .stm32_irq(stm32_irq),
        .ad9708_clk_o(ad9708_clk_o),
        .ad9708_data_o(ad9708_data_o),
        .ad9238_ch1_clk_o(ad9238_ch1_clk_o),
        .ad9238_ch2_clk_o(ad9238_ch2_clk_o),
        .ad9238_ch1_data_i(ad9238_ch1_data_i),
        .ad9238_ch1_otr_i(ad9238_ch1_otr_i),
        .ad9238_ch2_data_i(ad9238_ch2_data_i),
        .ad9238_ch2_otr_i(ad9238_ch2_otr_i),
        .led(led)
    );

endmodule
