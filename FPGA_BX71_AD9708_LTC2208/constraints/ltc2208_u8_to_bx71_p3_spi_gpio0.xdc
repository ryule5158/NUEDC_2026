# BX71 board clock, reset and status LED.
set_property IOSTANDARD LVCMOS33 [get_ports clk_50m]
set_property PACKAGE_PIN U18 [get_ports clk_50m]
create_clock -period 20.000 -name clk_50m [get_ports clk_50m]

set_property IOSTANDARD LVCMOS33 [get_ports reset_n]
set_property PACKAGE_PIN T19 [get_ports reset_n]

set_property IOSTANDARD LVCMOS33 [get_ports led]
set_property PACKAGE_PIN T10 [get_ports led]

# IMPORTANT: the ClearDream U8 header is NOT pin-compatible with BX71 P3.
# Use a crossed adapter/ribbon and continuity-check every wire before power.
#
# ClearDream U8       BX71 P3   XC7Z020       Signal
#  17 A_D0              1       P15           data[0]
#  18 A_D1              4       P16           data[1]
#  15 A_D2              3       N17           data[2]
#  16 A_D3              6       P18           data[3]
#  13 A_D4              5       R18           data[4]
#  14 A_D5              8       T17           data[5]
#  11 A_D6              7       U17           data[6]
#  12 A_D7             10       W19           data[7]
#   9 A_D8              9       Y19           data[8]
#  10 A_D9             14       R16           data[9]
#   7 A_D10            13       V18           data[10]
#   8 A_D11            16       T16           data[11]
#   5 A_D12            15       W16           data[12]
#   6 A_D13            19       T14           data[13]
#   3 A_D14            20       U14           data[14]
#   4 A_D15            21       W18           data[15]
#  20 AD_CLKA           2       P14           130 MHz clock
#   2 +5V              11       power only     +5 V
#   1 GND              12       power only     GND
# U8-39/U8-40 are module 3V3 outputs and MUST remain unconnected.

set_property IOSTANDARD LVCMOS33 [get_ports {ltc2208_ch1_data_i[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports ltc2208_ch1_clk_o]
set_property SLEW FAST [get_ports ltc2208_ch1_clk_o]
set_property DRIVE 8 [get_ports ltc2208_ch1_clk_o]

set_property PACKAGE_PIN P15 [get_ports {ltc2208_ch1_data_i[0]}]
set_property PACKAGE_PIN P16 [get_ports {ltc2208_ch1_data_i[1]}]
set_property PACKAGE_PIN N17 [get_ports {ltc2208_ch1_data_i[2]}]
set_property PACKAGE_PIN P18 [get_ports {ltc2208_ch1_data_i[3]}]
set_property PACKAGE_PIN R18 [get_ports {ltc2208_ch1_data_i[4]}]
set_property PACKAGE_PIN T17 [get_ports {ltc2208_ch1_data_i[5]}]
set_property PACKAGE_PIN U17 [get_ports {ltc2208_ch1_data_i[6]}]
set_property PACKAGE_PIN W19 [get_ports {ltc2208_ch1_data_i[7]}]
set_property PACKAGE_PIN Y19 [get_ports {ltc2208_ch1_data_i[8]}]
set_property PACKAGE_PIN R16 [get_ports {ltc2208_ch1_data_i[9]}]
set_property PACKAGE_PIN V18 [get_ports {ltc2208_ch1_data_i[10]}]
set_property PACKAGE_PIN T16 [get_ports {ltc2208_ch1_data_i[11]}]
set_property PACKAGE_PIN W16 [get_ports {ltc2208_ch1_data_i[12]}]
set_property PACKAGE_PIN T14 [get_ports {ltc2208_ch1_data_i[13]}]
set_property PACKAGE_PIN U14 [get_ports {ltc2208_ch1_data_i[14]}]
set_property PACKAGE_PIN W18 [get_ports {ltc2208_ch1_data_i[15]}]
set_property PACKAGE_PIN P14 [get_ports ltc2208_ch1_clk_o]

# LTC2208 CMOS data changes 1.3..4.0 ns after the rising ENC edge.  The IOB
# registers use the related 99-degree capture clock and intentionally capture
# the preceding converter word.  The setup MCP2 assigns that word one complete
# drive-clock period of latency.  Do not add the usual MCP2/hold1 relaxation:
# unlike a clock-enabled multicycle register path, this ADC bus changes every
# drive cycle.  The hold relationship derived from setup MCP2 must therefore
# remain active so the next converter word is checked against the active
# capture edge.
create_generated_clock -name ltc2208_ch1_clk \
    -source [get_pins -hier -filter {NAME =~ *adc_ch1_clk_oddr/C}] \
    -divide_by 1 [get_ports ltc2208_ch1_clk_o]
set_input_delay -clock [get_clocks ltc2208_ch1_clk] -max 4.000 \
    [get_ports {ltc2208_ch1_data_i[*]}]
set_input_delay -clock [get_clocks ltc2208_ch1_clk] -min 1.300 \
    [get_ports {ltc2208_ch1_data_i[*]}]
set_multicycle_path 2 -setup \
    -from [get_ports {ltc2208_ch1_data_i[*]}] \
    -to [get_cells -hier -filter {NAME =~ *ltc2208_ch1_data_iob_reg*}]

# P3 is occupied by the LTC2208 adapter. Keep the otherwise unchanged AD9708
# idle output on P2 for optional flying-wire use.
set_property IOSTANDARD LVCMOS33 [get_ports ad9708_clk_o]
set_property IOSTANDARD LVCMOS33 [get_ports {ad9708_data_o[*]}]
set_property SLEW FAST [get_ports ad9708_clk_o]
set_property SLEW FAST [get_ports {ad9708_data_o[*]}]
set_property DRIVE 8 [get_ports ad9708_clk_o]
set_property DRIVE 8 [get_ports {ad9708_data_o[*]}]
set_property PACKAGE_PIN L14 [get_ports {ad9708_data_o[0]}]
set_property PACKAGE_PIN L15 [get_ports {ad9708_data_o[1]}]
set_property PACKAGE_PIN K14 [get_ports {ad9708_data_o[2]}]
set_property PACKAGE_PIN J14 [get_ports {ad9708_data_o[3]}]
set_property PACKAGE_PIN H15 [get_ports {ad9708_data_o[4]}]
set_property PACKAGE_PIN G15 [get_ports {ad9708_data_o[5]}]
set_property PACKAGE_PIN F16 [get_ports {ad9708_data_o[6]}]
set_property PACKAGE_PIN F17 [get_ports {ad9708_data_o[7]}]
set_property PACKAGE_PIN E17 [get_ports ad9708_clk_o]

create_generated_clock -name ad9708_dclock \
    -source [get_pins -hier -filter {NAME =~ *dac_clk_oddr/C}] \
    -divide_by 1 -invert [get_ports ad9708_clk_o]
set_output_delay -clock [get_clocks ad9708_dclock] -max 2.000 \
    [get_ports {ad9708_data_o[*]}]
set_output_delay -clock [get_clocks ad9708_dclock] -min -1.500 \
    [get_ports {ad9708_data_o[*]}]

# Configuration and statistics cross between cfg_clk and adc_clk through
# explicit two-stage synchronizers.
set_false_path -to [get_pins -hier -regexp \
    {.*adc_capture/(sample_count_meta_reg|decimation_meta_reg|trigger_mode_meta_reg|trigger_threshold_meta_reg|filter_enable_meta_reg|start_toggle_sync_reg\[0\]|abort_toggle_sync_reg\[0\])(\[[0-9]+\])?/D}]
set_false_path -to [get_pins -hier -regexp \
    {.*adc_capture/(busy_sync_reg\[0\]|triggered_sync_reg\[0\]|overrange_sync_reg\[0\]|completion_toggle_sync_reg\[0\]|captured_count_meta_reg|overrange_count_meta_reg|sample_sum_meta_reg|sample_min_meta_reg|sample_max_meta_reg|latest_sample_meta_reg)(\[[0-9]+\])?/D}]

# STM32 SPI link on BX71 P2.
set_property IOSTANDARD LVCMOS33 [get_ports stm32_spi_sck]
set_property IOSTANDARD LVCMOS33 [get_ports stm32_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports stm32_spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports stm32_spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports stm32_irq]
set_property PULLDOWN true [get_ports {stm32_spi_sck stm32_spi_mosi}]
set_property PULLUP true [get_ports stm32_spi_cs_n]
set_property PACKAGE_PIN K19 [get_ports stm32_spi_sck]
set_property PACKAGE_PIN L19 [get_ports stm32_spi_mosi]
set_property PACKAGE_PIN K17 [get_ports stm32_spi_miso]
set_property PACKAGE_PIN K18 [get_ports stm32_spi_cs_n]
set_property PACKAGE_PIN L16 [get_ports stm32_irq]

set_false_path -from [get_ports {stm32_spi_sck stm32_spi_mosi stm32_spi_cs_n}]
set_false_path -from [get_ports reset_n]
set_false_path -to [get_ports {stm32_spi_miso stm32_irq led}]
