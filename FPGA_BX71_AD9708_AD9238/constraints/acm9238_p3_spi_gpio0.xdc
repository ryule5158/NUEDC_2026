# BX71 board clock, reset button and status LED.
set_property IOSTANDARD LVCMOS33 [get_ports clk_50m]
set_property PACKAGE_PIN U18 [get_ports clk_50m]
create_clock -period 20.000 -name clk_50m [get_ports clk_50m]

set_property IOSTANDARD LVCMOS33 [get_ports reset_n]
set_property PACKAGE_PIN T19 [get_ports reset_n]

set_property IOSTANDARD LVCMOS33 [get_ports led]
set_property PACKAGE_PIN T10 [get_ports led]

# ACM9238 P1 plugs directly into BX71 P3 with matching pin numbers.
# The module itself consumes P3-11=+5 V and P3-12=GND.
#
# P1/P3 signal     FPGA package pin
#  1 CH1_D0        P15
#  2 CH1_CLK       P14
#  3 CH1_D2        N17
#  4 CH1_D1        P16
#  5 CH1_D4        R18
#  6 CH1_D3        P18
#  7 CH1_D6        U17
#  8 CH1_D5        T17
#  9 CH1_D8        Y19
# 10 CH1_D7        W19
# 13 CH1_D10       V18
# 14 CH1_D9        R16
# 15 CH1_OTR       W16
# 16 CH1_D11       T16
# 32 CH2_CLK       Y16
set_property IOSTANDARD LVCMOS33 [get_ports {ad9238_ch1_data_i[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports ad9238_ch1_otr_i]
set_property IOSTANDARD LVCMOS33 [get_ports {ad9238_ch1_clk_o ad9238_ch2_clk_o}]
set_property SLEW FAST [get_ports {ad9238_ch1_clk_o ad9238_ch2_clk_o}]
set_property DRIVE 8 [get_ports {ad9238_ch1_clk_o ad9238_ch2_clk_o}]

set_property PACKAGE_PIN P15 [get_ports {ad9238_ch1_data_i[0]}]
set_property PACKAGE_PIN P16 [get_ports {ad9238_ch1_data_i[1]}]
set_property PACKAGE_PIN N17 [get_ports {ad9238_ch1_data_i[2]}]
set_property PACKAGE_PIN P18 [get_ports {ad9238_ch1_data_i[3]}]
set_property PACKAGE_PIN R18 [get_ports {ad9238_ch1_data_i[4]}]
set_property PACKAGE_PIN T17 [get_ports {ad9238_ch1_data_i[5]}]
set_property PACKAGE_PIN U17 [get_ports {ad9238_ch1_data_i[6]}]
set_property PACKAGE_PIN W19 [get_ports {ad9238_ch1_data_i[7]}]
set_property PACKAGE_PIN Y19 [get_ports {ad9238_ch1_data_i[8]}]
set_property PACKAGE_PIN R16 [get_ports {ad9238_ch1_data_i[9]}]
set_property PACKAGE_PIN V18 [get_ports {ad9238_ch1_data_i[10]}]
set_property PACKAGE_PIN T16 [get_ports {ad9238_ch1_data_i[11]}]
set_property PACKAGE_PIN W16 [get_ports ad9238_ch1_otr_i]
set_property PACKAGE_PIN P14 [get_ports ad9238_ch1_clk_o]
set_property PACKAGE_PIN Y16 [get_ports ad9238_ch2_clk_o]

# AD9238 changes D0..D11 and OTR 2..6 ns after each rising clock edge.
# The FPGA input IOB registers capture the previous stable word on the next edge.
create_generated_clock -name ad9238_ch1_clk \
    -source [get_pins -hier -filter {NAME =~ *adc_ch1_clk_oddr/C}] \
    -divide_by 1 [get_ports ad9238_ch1_clk_o]
create_generated_clock -name ad9238_ch2_clk \
    -source [get_pins -hier -filter {NAME =~ *adc_ch2_clk_oddr/C}] \
    -divide_by 1 [get_ports ad9238_ch2_clk_o]
set_input_delay -clock [get_clocks ad9238_ch1_clk] -max 6.000 \
    [get_ports {ad9238_ch1_data_i[*] ad9238_ch1_otr_i}]
set_input_delay -clock [get_clocks ad9238_ch1_clk] -min 2.000 \
    [get_ports {ad9238_ch1_data_i[*] ad9238_ch1_otr_i}]

# P3 is occupied by ACM9238.  Keep the otherwise unchanged AD9708 idle output
# available on the unused P2 GPIO0_0..8 pins for optional flying-wire use.
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

# Configuration and statistics cross between cfg_clk and adc_clk through the
# explicit two-stage synchronizers in ad9238_capture_core.
set_false_path -to [get_pins -hier -regexp \
    {.*adc_capture/(sample_count_meta_reg|decimation_meta_reg|trigger_mode_meta_reg|trigger_threshold_meta_reg|filter_enable_meta_reg|start_toggle_sync_reg\[0\]|abort_toggle_sync_reg\[0\])(\[[0-9]+\])?/D}]
set_false_path -to [get_pins -hier -regexp \
    {.*adc_capture/(busy_sync_reg\[0\]|triggered_sync_reg\[0\]|overrange_sync_reg\[0\]|completion_toggle_sync_reg\[0\]|captured_count_meta_reg|overrange_count_meta_reg|sample_sum_meta_reg|sample_min_meta_reg|sample_max_meta_reg|latest_sample_meta_reg)(\[[0-9]+\])?/D}]

# STM32 SPI link on BX71 P2.
# P2 GPIO0_26/K19=SCK, GPIO0_32/L19=MOSI, GPIO0_28/K17=MISO,
# GPIO0_29/K18=CS, GPIO0_30/L16=IRQ.
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

# SPI/reset are asynchronous at their external pins and enter synchronizers.
set_false_path -from [get_ports {stm32_spi_sck stm32_spi_mosi stm32_spi_cs_n}]
set_false_path -from [get_ports reset_n]
set_false_path -to [get_ports {stm32_spi_miso stm32_irq led}]