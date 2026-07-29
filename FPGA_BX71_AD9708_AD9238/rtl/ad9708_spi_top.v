`timescale 1ns / 1ps

/* 高速AD/DA顶层：管理STM32寄存器总线、AD9708波形输出和AD9238采集。 */
module ad9708_spi_top(
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

    output wire        led
);

    /* AD9708寄存器地址，与STM32底层驱动保持一致。 */
    localparam REG_CONTROL          = 7'h00; /* 输出使能和相位复位。 */
    localparam REG_MODE             = 7'h01; /* 波形模式。 */
    localparam REG_FREQ_WORD        = 7'h02; /* DDS频率控制字。 */
    localparam REG_CONSTANT         = 7'h03; /* 恒定输出码。 */
    localparam REG_STATUS           = 7'h04; /* DAC和扫频状态。 */
    localparam REG_PHASE_OFFSET     = 7'h05; /* DDS起始相位。 */
    localparam REG_RAM_ADDR         = 7'h06; /* 波形RAM写地址。 */
    localparam REG_RAM_DATA         = 7'h07; /* 波形RAM写数据。 */
    localparam REG_PHASE_NOW        = 7'h08; /* 当前相位快照。 */
    localparam REG_DAC_CLK_HZ       = 7'h09; /* DAC时钟频率。 */
    localparam REG_DEVICE_ID        = 7'h0A; /* FPGA设备标识。 */
    localparam REG_RAM_POINTS       = 7'h0B; /* 有效波形点数。 */
    localparam REG_LEVEL            = 7'h0C; /* Q8.8幅度和偏置。 */
    localparam REG_SWEEP_LOW        = 7'h0D; /* 扫频下限控制字。 */
    localparam REG_SWEEP_HIGH       = 7'h0E; /* 扫频上限控制字。 */
    localparam REG_SWEEP_STEP       = 7'h0F; /* 扫频步进控制字。 */
    localparam REG_SWEEP_DWELL      = 7'h10; /* 单步驻留周期数。 */
    localparam REG_SWEEP_CONTROL    = 7'h11; /* 扫频控制。 */
    localparam REG_SWEEP_STATUS     = 7'h12; /* 扫频状态。 */
    localparam REG_CURRENT_FREQ     = 7'h13; /* 当前频率控制字。 */
    localparam REG_FIRMWARE_VERSION = 7'h14; /* 固件协议版本。 */
    localparam REG_MOD_CONTROL      = 7'h15; /* 0=关闭，1=AM，2=FM。 */
    localparam REG_MOD_FREQ_WORD    = 7'h16; /* 调制DDS频率字。 */
    localparam REG_AM_GAINS         = 7'h17; /* {边带增益,载波增益} Q1.15。 */
    localparam REG_FM_DEVIATION     = 7'h18; /* FM峰值频偏字。 */
    localparam REG_MOD_PHASE_OFFSET = 7'h19; /* 调制DDS相位偏移。 */

    localparam [31:0] DEVICE_ID = 32'hAD970802; /* AD9708逻辑标识。 */
    localparam [31:0] FIRMWARE_VERSION = 32'h00020004; /* 增加硬件AM/FM。 */

    /* STM32可读写的DAC基础配置寄存器。 */
    reg enable_reg;
    reg [2:0] mode_reg;
    reg [31:0] freq_word_reg;
    reg [31:0] phase_offset_reg;
    reg [7:0] constant_code_reg;
    reg [10:0] ram_points_reg;
    reg [7:0] amplitude_reg;
    reg [7:0] amplitude_fraction_reg;
    reg [7:0] offset_reg;
    reg [7:0] offset_fraction_reg;
    reg [1:0] mod_mode_reg;
    reg [31:0] mod_freq_word_reg;
    reg [15:0] am_carrier_gain_reg;
    reg [15:0] am_sideband_gain_reg;
    reg [31:0] fm_deviation_word_reg;
    reg [31:0] mod_phase_offset_reg;

    /* STM32可读写的硬件扫频配置寄存器。 */
    reg sweep_enable_reg;
    reg [1:0] sweep_mode_reg;
    reg sweep_hold_reg;
    reg sweep_direction_reg;
    reg [31:0] sweep_low_reg;
    reg [31:0] sweep_high_reg;
    reg [31:0] sweep_step_reg;
    reg [31:0] sweep_dwell_reg;

    /* 跨时钟域控制使用的翻转量。 */
    reg cfg_commit_toggle;
    reg phase_reset_toggle;
    reg sweep_start_toggle;

    /* 波形RAM写地址和最近写入数据。 */
    reg [9:0] ram_addr_reg;
    reg [9:0] ram_wr_addr;
    reg [7:0] ram_wr_data;
    reg ram_wr_en;

    /* SPI寄存器总线信号。 */
    wire [6:0] spi_addr;
    wire [31:0] spi_wdata;
    wire spi_write;
    wire spi_read_commit;
    reg [31:0] spi_rdata;

    /* AD9238寄存器总线和采样时钟信号。 */
    wire adc_bus_select;
    wire [31:0] adc_bus_rdata;
    wire dsp_bus_select;
    wire [31:0] dsp_bus_rdata;
    wire clk_adc;
    wire adc_pll_locked;
    wire adc_reset_n = reset_n & adc_pll_locked;
    (* IOB = "TRUE" *) reg [11:0] ad9238_ch1_data_iob;
    (* IOB = "TRUE" *) reg ad9238_ch1_otr_iob;

    /* DAC时钟域输出及调试状态。 */
    wire clk_dac;
    wire pll_locked;
    wire dac_reset_n = reset_n & pll_locked;
    wire [7:0] dac_data;
    wire [31:0] phase_debug_dac;
    wire [31:0] current_freq_dac;
    wire sweep_running_dac;
    wire sweep_done_dac;
    wire sweep_direction_dac;

    /* 返回STM32前使用两级同步的DAC域状态。 */
    (* ASYNC_REG = "TRUE" *) reg [1:0] sweep_running_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sweep_done_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sweep_direction_sync;
    reg [31:0] phase_debug_meta;
    reg [31:0] phase_debug_sync;
    reg [31:0] current_freq_meta;
    reg [31:0] current_freq_sync;

    assign ad9708_data_o = dac_data;
    assign stm32_irq = pll_locked;
    assign led = pll_locked & ~enable_reg;

    /* STM32使用6字节SPI模式0帧访问FPGA寄存器。 */
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

    /* 将板载50MHz时钟倍频为AD9708标称最高125MHz更新时钟。 */
    ad9708_clkgen clkgen (
        .clk_50m(clk_50m),
        .reset(~reset_n),
        .clk_dac(clk_dac),
        .locked(pll_locked)
    );

    /* 生成AD9238额定最高32 MHz采样时钟。 */
    ad9238_clkgen adc_clkgen (
        .clk_50m(clk_50m),
        .reset(~reset_n),
        .clk_adc(clk_adc),
        .locked(adc_pll_locked)
    );

    /* 将返回数据先锁存在输入寄存器，缩短32 MHz源同步输入路径。 */
    always @(posedge clk_adc or negedge adc_reset_n) begin
        if (!adc_reset_n) begin
            ad9238_ch1_data_iob <= 12'd2048;
            ad9238_ch1_otr_iob <= 1'b0;
        end else begin
            ad9238_ch1_data_iob <= ad9238_ch1_data_i;
            ad9238_ch1_otr_iob <= ad9238_ch1_otr_i;
        end
    end

    /* 输出与FPGA内部同相的32 MHz采样时钟给AD9238。 */
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

    /* FPGA完成高速采集、触发、抽取和缓存，STM32读取采集后的快照。 */
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
    ad9238_capture_core adc_capture (
        .cfg_clk(clk_50m),
        .adc_clk(clk_adc),
        .reset_n(reset_n),
        .adc_ready(adc_pll_locked),
        .adc_data(ad9238_ch1_data_iob),
        .adc_otr(ad9238_ch1_otr_iob),
        .bus_addr(spi_addr),
        .bus_wdata(spi_wdata),
        .bus_write(spi_write),
        .bus_read_commit(spi_read_commit),
        .bus_select(adc_bus_select),
        .bus_rdata(adc_bus_rdata)
    );

    /* 原始50 MSPS采样同时送入ProMax，采集触发和抽取不会影响DSP数据流。 */
    ad9238_promax_core dsp_core (
        .cfg_clk(clk_50m),
        .adc_clk(clk_adc),
        .reset_n(reset_n),
        .adc_ready(adc_pll_locked),
        .adc_data(ad9238_ch1_data_iob),
        .bus_addr(spi_addr),
        .bus_wdata(spi_wdata),
        .bus_write(spi_write),
        .bus_select(dsp_bus_select),
        .bus_rdata(dsp_bus_rdata)
    );

    /* 生成恒定值、任意波、锯齿波、三角波、方波及硬件扫频。 */
    ad9708_wave_core wave_core (
        .cfg_clk(clk_50m),
        .reset_n(dac_reset_n),
        .dac_clk(clk_dac),
        .cfg_commit_toggle(cfg_commit_toggle),
        .enable_cfg(enable_reg),
        .mode_cfg(mode_reg),
        .freq_word_cfg(freq_word_reg),
        .phase_offset_cfg(phase_offset_reg),
        .constant_code_cfg(constant_code_reg),
        .ram_points_cfg(ram_points_reg),
        .amplitude_cfg(amplitude_reg),
        .amplitude_fraction_cfg(amplitude_fraction_reg),
        .offset_cfg(offset_reg),
        .offset_fraction_cfg(offset_fraction_reg),
        .phase_reset_toggle(phase_reset_toggle),
        .mod_mode_cfg(mod_mode_reg),
        .mod_freq_word_cfg(mod_freq_word_reg),
        .am_carrier_gain_cfg(am_carrier_gain_reg),
        .am_sideband_gain_cfg(am_sideband_gain_reg),
        .fm_deviation_word_cfg(fm_deviation_word_reg),
        .mod_phase_offset_cfg(mod_phase_offset_reg),
        .sweep_enable_cfg(sweep_enable_reg),
        .sweep_mode_cfg(sweep_mode_reg),
        .sweep_hold_cfg(sweep_hold_reg),
        .sweep_direction_cfg(sweep_direction_reg),
        .sweep_low_cfg(sweep_low_reg),
        .sweep_high_cfg(sweep_high_reg),
        .sweep_step_cfg(sweep_step_reg),
        .sweep_dwell_cfg(sweep_dwell_reg),
        .sweep_start_toggle(sweep_start_toggle),
        .ram_wr_en(ram_wr_en),
        .ram_wr_addr(ram_wr_addr),
        .ram_wr_data(ram_wr_data),
        .dac_data(dac_data),
        .phase_debug(phase_debug_dac),
        .current_freq_word(current_freq_dac),
        .sweep_running(sweep_running_dac),
        .sweep_done(sweep_done_dac),
        .sweep_direction_up(sweep_direction_dac)
    );

    /* 输出反相转发时钟，使数据在AD9708上升沿前稳定半个周期。 */
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

    /* 同步扫频状态，并提供只读调试频率和相位快照。 */
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin
            sweep_running_sync <= 2'b00;
            sweep_done_sync <= 2'b00;
            sweep_direction_sync <= 2'b11;
            phase_debug_meta <= 32'd0;
            phase_debug_sync <= 32'd0;
            current_freq_meta <= 32'd0;
            current_freq_sync <= 32'd0;
        end else begin
            sweep_running_sync <= {sweep_running_sync[0], sweep_running_dac};
            sweep_done_sync <= {sweep_done_sync[0], sweep_done_dac};
            sweep_direction_sync <= {sweep_direction_sync[0], sweep_direction_dac};
            phase_debug_meta <= phase_debug_dac;
            phase_debug_sync <= phase_debug_meta;
            current_freq_meta <= current_freq_dac;
            current_freq_sync <= current_freq_meta;
        end
    end

    /* STM32寄存器读回多路选择。 */
    always @(*) begin
        if (adc_bus_select) begin
            spi_rdata = adc_bus_rdata;
        end else if (dsp_bus_select) begin
            spi_rdata = dsp_bus_rdata;
        end else begin
        case (spi_addr)
            REG_CONTROL: spi_rdata = {31'd0, enable_reg};
            REG_MODE: spi_rdata = {29'd0, mode_reg};
            REG_FREQ_WORD: spi_rdata = freq_word_reg;
            REG_CONSTANT: spi_rdata = {24'd0, constant_code_reg};
            REG_STATUS: spi_rdata = {26'd0,
                                     sweep_enable_reg,
                                     sweep_done_sync[1],
                                     sweep_running_sync[1],
                                     pll_locked,
                                     enable_reg,
                                     dac_reset_n};
            REG_PHASE_OFFSET: spi_rdata = phase_offset_reg;
            REG_RAM_ADDR: spi_rdata = {22'd0, ram_addr_reg};
            REG_RAM_DATA: spi_rdata = {24'd0, ram_wr_data};
            REG_PHASE_NOW: spi_rdata = phase_debug_sync;
            REG_DAC_CLK_HZ: spi_rdata = 32'd125000000;
            REG_DEVICE_ID: spi_rdata = DEVICE_ID;
            REG_RAM_POINTS: spi_rdata = {21'd0, ram_points_reg};
            REG_LEVEL: spi_rdata = {amplitude_fraction_reg,
                                    offset_fraction_reg,
                                    amplitude_reg, offset_reg};
            REG_SWEEP_LOW: spi_rdata = sweep_low_reg;
            REG_SWEEP_HIGH: spi_rdata = sweep_high_reg;
            REG_SWEEP_STEP: spi_rdata = sweep_step_reg;
            REG_SWEEP_DWELL: spi_rdata = sweep_dwell_reg;
            REG_SWEEP_CONTROL: spi_rdata = {26'd0,
                                            sweep_direction_reg,
                                            1'b0,
                                            sweep_hold_reg,
                                            sweep_mode_reg,
                                            sweep_enable_reg};
            REG_SWEEP_STATUS: spi_rdata = {28'd0,
                                           sweep_direction_sync[1],
                                           sweep_done_sync[1],
                                           sweep_hold_reg,
                                           sweep_running_sync[1]};
            REG_CURRENT_FREQ: spi_rdata = current_freq_sync;
            REG_FIRMWARE_VERSION: spi_rdata = FIRMWARE_VERSION;
            REG_MOD_CONTROL: spi_rdata = {30'd0, mod_mode_reg};
            REG_MOD_FREQ_WORD: spi_rdata = mod_freq_word_reg;
            REG_AM_GAINS: spi_rdata = {am_sideband_gain_reg,
                                       am_carrier_gain_reg};
            REG_FM_DEVIATION: spi_rdata = fm_deviation_word_reg;
            REG_MOD_PHASE_OFFSET: spi_rdata = mod_phase_offset_reg;
            default: spi_rdata = 32'd0;
        endcase
        end
    end

    /* 处理STM32寄存器写入，并为多位配置产生原子提交事件。 */
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin
            enable_reg <= 1'b0;
            mode_reg <= 3'd0;
            freq_word_reg <= 32'd0;
            phase_offset_reg <= 32'd0;
            constant_code_reg <= 8'd128;
            ram_points_reg <= 11'd1024;
            amplitude_reg <= 8'd128;
            amplitude_fraction_reg <= 8'd0;
            offset_reg <= 8'd128;
            offset_fraction_reg <= 8'd0;
            mod_mode_reg <= 2'd0;
            mod_freq_word_reg <= 32'd0;
            am_carrier_gain_reg <= 16'd32767;
            am_sideband_gain_reg <= 16'd0;
            fm_deviation_word_reg <= 32'd0;
            mod_phase_offset_reg <= 32'd0;
            sweep_enable_reg <= 1'b0;
            sweep_mode_reg <= 2'd0;
            sweep_hold_reg <= 1'b0;
            sweep_direction_reg <= 1'b1;
            sweep_low_reg <= 32'd4294967;
            sweep_high_reg <= 32'd42949673;
            sweep_step_reg <= 32'd429497;
            sweep_dwell_reg <= 32'd100000;
            cfg_commit_toggle <= 1'b0;
            phase_reset_toggle <= 1'b0;
            sweep_start_toggle <= 1'b0;
            ram_addr_reg <= 10'd0;
            ram_wr_addr <= 10'd0;
            ram_wr_data <= 8'd128;
            ram_wr_en <= 1'b0;
        end else begin
            ram_wr_en <= 1'b0;

            if (spi_write) begin
                case (spi_addr)
                    REG_CONTROL: begin
                        enable_reg <= spi_wdata[0];
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                        if (spi_wdata[1]) begin
                            phase_reset_toggle <= ~phase_reset_toggle;
                        end
                    end

                    REG_MODE: begin
                        mode_reg <= spi_wdata[2:0];
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_FREQ_WORD: begin
                        freq_word_reg <= spi_wdata;
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_CONSTANT: begin
                        constant_code_reg <= spi_wdata[7:0];
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_PHASE_OFFSET: begin
                        phase_offset_reg <= spi_wdata;
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_RAM_ADDR: begin
                        ram_addr_reg <= spi_wdata[9:0];
                    end

                    REG_RAM_DATA: begin
                        ram_wr_addr <= ram_addr_reg;
                        ram_wr_data <= spi_wdata[7:0];
                        ram_wr_en <= 1'b1;
                        ram_addr_reg <= ram_addr_reg + 1'b1;
                    end

                    REG_RAM_POINTS: begin
                        ram_points_reg <= spi_wdata[10:0];
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_LEVEL: begin
                        amplitude_reg <= spi_wdata[15:8];
                        amplitude_fraction_reg <= spi_wdata[31:24];
                        offset_reg <= spi_wdata[7:0];
                        offset_fraction_reg <= spi_wdata[23:16];
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    /*
                     * 参数寄存器先暂存；最后写MOD_CONTROL时才一次性提交，
                     * 避免DAC域看到新旧AM/FM参数混合的中间状态。
                     */
                    REG_MOD_FREQ_WORD: begin
                        mod_freq_word_reg <= spi_wdata;
                    end

                    REG_AM_GAINS: begin
                        am_carrier_gain_reg <= spi_wdata[15:0];
                        am_sideband_gain_reg <= spi_wdata[31:16];
                    end

                    REG_FM_DEVIATION: begin
                        fm_deviation_word_reg <= spi_wdata;
                    end

                    REG_MOD_PHASE_OFFSET: begin
                        mod_phase_offset_reg <= spi_wdata;
                    end

                    REG_MOD_CONTROL: begin
                        mod_mode_reg <= spi_wdata[1:0];
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_SWEEP_LOW: begin
                        sweep_low_reg <= spi_wdata;
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_SWEEP_HIGH: begin
                        sweep_high_reg <= spi_wdata;
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_SWEEP_STEP: begin
                        sweep_step_reg <= spi_wdata;
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_SWEEP_DWELL: begin
                        sweep_dwell_reg <= spi_wdata;
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                    end

                    REG_SWEEP_CONTROL: begin
                        sweep_enable_reg <= spi_wdata[0];
                        sweep_mode_reg <= spi_wdata[2:1];
                        sweep_hold_reg <= spi_wdata[3];
                        sweep_direction_reg <= spi_wdata[5];
                        cfg_commit_toggle <= ~cfg_commit_toggle;
                        if (spi_wdata[4]) begin
                            sweep_start_toggle <= ~sweep_start_toggle;
                        end
                    end

                    default: begin
                    end
                endcase
            end
        end
    end

endmodule
