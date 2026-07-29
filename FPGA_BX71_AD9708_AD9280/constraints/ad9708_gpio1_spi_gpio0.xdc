set_property IOSTANDARD LVCMOS33 [get_ports clk_50m]
set_property PACKAGE_PIN U18 [get_ports clk_50m]
create_clock -period 20.000 -name clk_50m [get_ports clk_50m]

set_property IOSTANDARD LVCMOS33 [get_ports reset_n]
set_property PACKAGE_PIN T19 [get_ports reset_n]

set_property IOSTANDARD LVCMOS33 [get_ports led]
set_property PACKAGE_PIN T10 [get_ports led]

# Lingzhi module U3 is aligned tail-to-tail with the BX71-V4 P3 connector.
# Module P1-5  DB1    -> P3 pin 39 / GPIO1_0  -> data[1]
# Module P1-6  DB0    -> P3 pin 40 / GPIO1_2  -> data[0]
# Module P1-7  DB3    -> P3 pin 37 / GPIO1_5  -> data[3]
# Module P1-8  DB2    -> P3 pin 38 / GPIO1_4  -> data[2]
# Module P1-9  DB5    -> P3 pin 35 / GPIO1_6  -> data[5]
# Module P1-10 DB4    -> P3 pin 36 / GPIO1_13 -> data[4]
# Module P1-11 DB7    -> P3 pin 33 / GPIO1_14 -> data[7]
# Module P1-12 DB6    -> P3 pin 34 / GPIO1_18 -> data[6]
# Module P1-14 DCLOCK -> P3 pin 32 / GPIO1_11 -> DAC clock
set_property IOSTANDARD LVCMOS33 [get_ports ad9708_clk_o]
set_property IOSTANDARD LVCMOS33 [get_ports {ad9708_data_o[*]}]
set_property SLEW FAST [get_ports ad9708_clk_o]
set_property SLEW FAST [get_ports {ad9708_data_o[*]}]
set_property DRIVE 8 [get_ports ad9708_clk_o]
set_property DRIVE 8 [get_ports {ad9708_data_o[*]}]

set_property PACKAGE_PIN Y16 [get_ports ad9708_clk_o]
set_property PACKAGE_PIN Y14 [get_ports {ad9708_data_o[7]}]
set_property PACKAGE_PIN W15 [get_ports {ad9708_data_o[6]}]
set_property PACKAGE_PIN W13 [get_ports {ad9708_data_o[5]}]
set_property PACKAGE_PIN W14 [get_ports {ad9708_data_o[4]}]
set_property PACKAGE_PIN V12 [get_ports {ad9708_data_o[3]}]
set_property PACKAGE_PIN V13 [get_ports {ad9708_data_o[2]}]
set_property PACKAGE_PIN T11 [get_ports {ad9708_data_o[1]}]
set_property PACKAGE_PIN U12 [get_ports {ad9708_data_o[0]}]

# AD9708 captures data on the rising edge of the forwarded, inverted clock.
# The output delays model the datasheet 2.0 ns setup and 1.5 ns hold times.
create_generated_clock -name ad9708_dclock \
    -source [get_pins -hier -filter {NAME =~ *dac_clk_oddr/C}] -divide_by 1 -invert \
    [get_ports ad9708_clk_o]
set_output_delay -clock [get_clocks ad9708_dclock] -max 2.000 \
    [get_ports {ad9708_data_o[*]}]
set_output_delay -clock [get_clocks ad9708_dclock] -min -1.500 \
    [get_ports {ad9708_data_o[*]}]

# Lingzhi module ADC pins use the same U3 tail-aligned adapter as the DAC.
# Module P1-23 AB1    -> adapter GPIO17 -> P3 pin 19 / GPIO1_7.
# Module P1-24 AB0    -> adapter GPIO18 -> P3 pin 20 / GPIO1_19.
# Module P1-21 AB3    -> adapter GPIO19 -> P3 pin 21 / GPIO1_30.
# Module P1-22 AB2    -> adapter GPIO20 -> P3 pin 22 / GPIO1_1.
# Module P1-19 AB5    -> adapter GPIO21 -> P3 pin 23 / GPIO1_10.
# Module P1-20 AB4    -> adapter GPIO22 -> P3 pin 24 / GPIO1_3.
# Module P1-17 AB7    -> adapter GPIO23 -> P3 pin 25 / GPIO1_28.
# Module P1-18 AB6    -> adapter GPIO24 -> P3 pin 26 / GPIO1_20.
# Module P1-15 OTR    -> adapter GPIO25 -> P3 pin 27 / GPIO1_22.
# Module P1-16 ACLOCK -> adapter GPIO26 -> P3 pin 28 / GPIO1_12.
set_property IOSTANDARD LVCMOS33 [get_ports ad9280_clk_o]
set_property IOSTANDARD LVCMOS33 [get_ports {ad9280_data_i[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports ad9280_otr_i]
set_property SLEW FAST [get_ports ad9280_clk_o]
set_property DRIVE 8 [get_ports ad9280_clk_o]

set_property PACKAGE_PIN U14 [get_ports {ad9280_data_i[0]}]
set_property PACKAGE_PIN T14 [get_ports {ad9280_data_i[1]}]
set_property PACKAGE_PIN T12 [get_ports {ad9280_data_i[2]}]
set_property PACKAGE_PIN W18 [get_ports {ad9280_data_i[3]}]
set_property PACKAGE_PIN U13 [get_ports {ad9280_data_i[4]}]
set_property PACKAGE_PIN R14 [get_ports {ad9280_data_i[5]}]
set_property PACKAGE_PIN Y18 [get_ports {ad9280_data_i[6]}]
set_property PACKAGE_PIN V17 [get_ports {ad9280_data_i[7]}]
set_property PACKAGE_PIN V16 [get_ports ad9280_otr_i]
set_property PACKAGE_PIN Y17 [get_ports ad9280_clk_o]

# AD9280 updates D0..D7 and OTR about 25 ns after each rising edge.
# The FPGA input registers capture that stable word on the following edge.
create_generated_clock -name ad9280_aclock \
    -source [get_pins -hier -filter {NAME =~ *adc_clk_oddr/C}] -divide_by 1 \
    [get_ports ad9280_clk_o]
set_input_delay -clock [get_clocks ad9280_aclock] -max 25.000 \
    [get_ports {ad9280_data_i[*] ad9280_otr_i}]
set_input_delay -clock [get_clocks ad9280_aclock] -min 0.000 \
    [get_ports {ad9280_data_i[*] ad9280_otr_i}]

# 配置量与采集统计量使用两级同步；仅忽略跨时钟域第一级的时序。
set_false_path -to [get_pins -hier -regexp \
    {.*adc_capture/(sample_count_meta_reg|trigger_mode_meta_reg|trigger_threshold_meta_reg|start_toggle_sync_reg\[0\]|abort_toggle_sync_reg\[0\])(\[[0-9]+\])?/D}]
set_false_path -to [get_pins -hier -regexp \
    {.*adc_capture/(busy_sync_reg\[0\]|triggered_sync_reg\[0\]|overrange_sync_reg\[0\]|completion_toggle_sync_reg\[0\]|captured_count_meta_reg|overrange_count_meta_reg|sample_sum_meta_reg|sample_min_meta_reg|sample_max_meta_reg|latest_sample_meta_reg)(\[[0-9]+\])?/D}]

# STM32 SPI link on BX71-V4 P2; MOSI uses spare L19 because J19 did not follow PI3 in boundary-scan tests.
# P2-31/K19=SCK, P2-37/L19=MOSI, P2-33/K17=MISO,
# P2-34/K18=CS, and optional P2-35/L16=IRQ.
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

# SPI and reset inputs are asynchronous to clk_50m and enter synchronizers.
set_false_path -from [get_ports {stm32_spi_sck stm32_spi_mosi stm32_spi_cs_n}]
set_false_path -from [get_ports reset_n]
set_false_path -to [get_ports {stm32_spi_miso stm32_irq led}]
