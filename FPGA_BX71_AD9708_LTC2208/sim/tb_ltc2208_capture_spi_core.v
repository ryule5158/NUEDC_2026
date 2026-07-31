`timescale 1ns / 1ps

/*
 * End-to-end regression for the LTC2208 capture path behind the STM32 SPI
 * register slave.  Every configuration and observation below crosses the
 * same six-byte SPI frame used by the firmware; no direct testbench access to
 * the register bus is used.
 */
module tb_ltc2208_capture_spi_core;
    localparam REG_CONTROL          = 7'h20;
    localparam REG_STATUS           = 7'h21;
    localparam REG_SAMPLE_COUNT     = 7'h22;
    localparam REG_DECIMATION       = 7'h23;
    localparam REG_TRIGGER          = 7'h24;
    localparam REG_READ_ADDR        = 7'h25;
    localparam REG_READ_DATA        = 7'h26;
    localparam REG_CAPTURED_COUNT   = 7'h27;
    localparam REG_SAMPLE_CLK_HZ    = 7'h29;
    localparam REG_DEVICE_ID        = 7'h2A;
    localparam REG_FIRMWARE_VERSION = 7'h2B;
    localparam REG_MIN_MAX          = 7'h2C;
    localparam REG_SUM              = 7'h2D;
    localparam REG_LATEST           = 7'h2E;
    localparam REG_FILTER           = 7'h2F;

    localparam [31:0] EXPECTED_DEVICE_ID = 32'h22080001;
    localparam [31:0] EXPECTED_FIRMWARE  = 32'h00030002;
    localparam [31:0] EXPECTED_ADC_HZ    = 32'd130000000;

    reg cfg_clk = 1'b0;
    reg adc_clk = 1'b0;
    reg reset_n = 1'b0;
    reg adc_ready = 1'b1;
    reg [15:0] adc_data = 16'h8000;
    reg spi_sck = 1'b0;
    reg spi_mosi = 1'b0;
    reg spi_cs_n = 1'b1;
    wire spi_miso;

    wire [6:0] bus_addr;
    wire [31:0] bus_wdata;
    wire bus_write;
    wire bus_read_commit;
    wire bus_select;
    wire [31:0] bus_rdata;

    reg [7:0] spi_rx [0:5];
    reg [31:0] value;
    reg [31:0] sample_word;
    integer byte_index;
    integer bit_index;
    integer sample_index;
    integer poll_count;
    integer signed filtered_sample_q3;
    integer signed filtered_stats_sample;
    integer signed filtered_stats_min;
    integer signed filtered_stats_max;
    integer signed filtered_stats_sum;
    integer filtered_fraction_seen;

    always #10.000 cfg_clk = ~cfg_clk;  // 50 MHz configuration clock
    always #3.846 adc_clk = ~adc_clk;   // nominal 130 MHz ADC clock

    stm32_spi_reg_slave spi_slave (
        .clk(cfg_clk),
        .reset_n(reset_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .reg_addr(bus_addr),
        .reg_wdata(bus_wdata),
        .reg_write(bus_write),
        .reg_read_commit(bus_read_commit),
        .reg_rdata(bus_rdata)
    );

    ltc2208_capture_core capture_core (
        .cfg_clk(cfg_clk),
        .adc_clk(adc_clk),
        .reset_n(reset_n),
        .adc_ready(adc_ready),
        .adc_data(adc_data),
        .bus_addr(bus_addr),
        .bus_wdata(bus_wdata),
        .bus_write(bus_write),
        .bus_read_commit(bus_read_commit),
        .bus_select(bus_select),
        .bus_rdata(bus_rdata)
    );

    task SpiClockByte;
        input [7:0] tx_byte;
        output [7:0] rx_byte;
        begin
            rx_byte = 8'd0;
            for (bit_index = 7; bit_index >= 0;
                 bit_index = bit_index - 1) begin
                spi_mosi = tx_byte[bit_index];
                #100;
                /* Mode-0 master samples the value already present at SCK. */
                rx_byte[bit_index] = spi_miso;
                spi_sck = 1'b1;
                #100;
                spi_sck = 1'b0;
            end
        end
    endtask

    task SpiFrame;
        input [7:0] command;
        input [31:0] payload;
        begin
            spi_cs_n = 1'b0;
            spi_sck = 1'b0;
            #300;
            SpiClockByte(command,       spi_rx[0]);
            SpiClockByte(payload[31:24], spi_rx[1]);
            SpiClockByte(payload[23:16], spi_rx[2]);
            SpiClockByte(payload[15:8],  spi_rx[3]);
            SpiClockByte(payload[7:0],   spi_rx[4]);
            SpiClockByte(8'h00,          spi_rx[5]);
            #100;
            spi_cs_n = 1'b1;
            spi_mosi = 1'b0;
            #400;

            if ((spi_rx[0] !== 8'hA5) || (spi_rx[1] !== 8'h5A))
                $fatal(1,
                       "SPI response header mismatch: %02h %02h",
                       spi_rx[0], spi_rx[1]);
        end
    endtask

    task WriteReg;
        input [6:0] addr;
        input [31:0] write_value;
        begin
            SpiFrame({1'b1, addr}, write_value);
        end
    endtask

    task ReadReg;
        input [6:0] addr;
        output [31:0] read_value;
        begin
            SpiFrame({1'b0, addr}, 32'd0);
            read_value = {spi_rx[2], spi_rx[3], spi_rx[4], spi_rx[5]};
        end
    endtask

    task WaitDone;
        begin
            value = 32'd0;
            poll_count = 0;
            while (((value & 32'h00000004) == 0) &&
                   (poll_count < 200)) begin
                ReadReg(REG_STATUS, value);
                poll_count = poll_count + 1;
            end
            if (poll_count >= 200)
                $fatal(1, "LTC2208 capture timeout status=%08h", value);
            if ((value & 32'h00000001) == 0)
                $fatal(1, "ADC READY was lost during capture");
        end
    endtask

    task StartImmediateCapture;
        input [12:0] count;
        input [15:0] decimation;
        input filter_enable;
        begin
            WriteReg(REG_FILTER, filter_enable ? 32'd1 : 32'd0);
            WriteReg(REG_SAMPLE_COUNT, {19'd0, count});
            WriteReg(REG_DECIMATION, {16'd0, decimation});
            WriteReg(REG_TRIGGER, 32'd0);
            WriteReg(REG_CONTROL, 32'd1);
        end
    endtask

    initial begin
        repeat (8) @(posedge cfg_clk);
        reset_n = 1'b1;
        repeat (12) @(posedge cfg_clk);

        /* Identity and protocol are read through the real SPI frame. */
        ReadReg(REG_DEVICE_ID, value);
        if (value !== EXPECTED_DEVICE_ID)
            $fatal(1, "device ID mismatch actual=%08h", value);
        ReadReg(REG_FIRMWARE_VERSION, value);
        if (value !== EXPECTED_FIRMWARE)
            $fatal(1, "firmware version mismatch actual=%08h", value);
        ReadReg(REG_SAMPLE_CLK_HZ, value);
        if (value !== EXPECTED_ADC_HZ)
            $fatal(1, "ADC clock identity mismatch actual=%0d", value);
        $display("LTC2208_SPI_ID_FW_PASS id=%08h fw=%08h adc_hz=%0d",
                 EXPECTED_DEVICE_ID, EXPECTED_FIRMWARE, EXPECTED_ADC_HZ);

        /*
         * 0x9000 offset-binary is +4096 centered codes.  Four samples prove
         * conversion, capture, signed-20 Q3 BRAM packing and SPI read-address
         * auto-increment. Statistics deliberately remain raw S16.
         */
        adc_data = 16'h9000;
        StartImmediateCapture(13'd4, 16'd1, 1'b0);
        WaitDone();
        ReadReg(REG_CAPTURED_COUNT, value);
        if (value[12:0] !== 13'd4)
            $fatal(1, "raw captured count mismatch actual=%0d", value[12:0]);
        WriteReg(REG_READ_ADDR, 32'd0);
        for (sample_index = 0; sample_index < 4;
             sample_index = sample_index + 1) begin
            ReadReg(REG_READ_DATA, sample_word);
            if (sample_word[31:20] !== sample_index[11:0])
                $fatal(1,
                       "RAM address tag mismatch actual=%0d expected=%0d",
                       sample_word[31:20], sample_index);
            if ($signed(sample_word[19:0]) !== 20'sd32768)
                $fatal(1,
                       "offset-binary/Q3 RAM sample mismatch actual=%0d",
                       $signed(sample_word[19:0]));
        end
        ReadReg(REG_MIN_MAX, value);
        if (($signed(value[15:0]) !== 16'sd4096) ||
            ($signed(value[31:16]) !== 16'sd4096))
            $fatal(1, "raw MIN/MAX mismatch actual=%08h", value);
        ReadReg(REG_SUM, value);
        if ($signed(value) !== 32'sd16384)
            $fatal(1, "raw SUM mismatch actual=%0d", $signed(value));
        ReadReg(REG_LATEST, value);
        if ($signed(value) !== 32'sd4096)
            $fatal(1, "raw LATEST mismatch actual=%0d", $signed(value));
        $display("LTC2208_SPI_RAM_OFFSET_BINARY_Q3_PASS");

        /* Signed transport must preserve a negative raw code through SPI. */
        adc_data = 16'h7000;
        StartImmediateCapture(13'd1, 16'd1, 1'b0);
        WaitDone();
        WriteReg(REG_READ_ADDR, 32'd0);
        ReadReg(REG_READ_DATA, sample_word);
        if ((sample_word[31:20] !== 12'd0) ||
            ($signed(sample_word[19:0]) !== -20'sd32768))
            $fatal(1, "negative signed-20 Q3 mismatch word=%08h", sample_word);
        ReadReg(REG_MIN_MAX, value);
        if (($signed(value[15:0]) !== -16'sd4096) ||
            ($signed(value[31:16]) !== -16'sd4096))
            $fatal(1, "negative raw MIN/MAX mismatch actual=%08h", value);
        ReadReg(REG_SUM, value);
        if ($signed(value) !== -32'sd4096)
            $fatal(1, "negative raw SUM mismatch actual=%0d", $signed(value));
        $display("LTC2208_SIGNED20_Q3_NEGATIVE_PASS");

        /*
         * The anti-interference path is only effective for R=65.  Constant
         * input should remain constant after CIC5 normalization and FIR81;
         * the tolerance covers fixed-point coefficient quantization.
         */
        adc_data = 16'h9000;
        StartImmediateCapture(13'd4, 16'd65, 1'b1);
        repeat (12) @(posedge adc_clk);
        if (capture_core.filter_enable_latched !== 1'b1)
            $fatal(1, "filter did not latch for R=65");
        WaitDone();
        ReadReg(REG_DECIMATION, value);
        if (value[15:0] !== 16'd65)
            $fatal(1, "R65 readback mismatch actual=%0d", value[15:0]);
        ReadReg(REG_CAPTURED_COUNT, value);
        if (value[12:0] !== 13'd4)
            $fatal(1,
                   "filtered captured count mismatch actual=%0d",
                   value[12:0]);
        WriteReg(REG_READ_ADDR, 32'd0);
        filtered_stats_min = 32767;
        filtered_stats_max = -32768;
        filtered_stats_sum = 0;
        filtered_fraction_seen = 0;
        for (sample_index = 0; sample_index < 4;
             sample_index = sample_index + 1) begin
            ReadReg(REG_READ_DATA, sample_word);
            if (sample_word[31:20] !== sample_index[11:0])
                $fatal(1, "filtered RAM address tag mismatch word=%08h",
                       sample_word);
            filtered_sample_q3 = $signed(sample_word[19:0]);
            if ((filtered_sample_q3 < 32320) ||
                (filtered_sample_q3 > 33280))
                $fatal(1,
                       "CIC5/R65/FIR81 Q3 DC mismatch sample=%0d value=%0d",
                       sample_index, filtered_sample_q3);
            if ((filtered_sample_q3 & 7) != 0)
                filtered_fraction_seen = 1;
            filtered_stats_sample = (filtered_sample_q3 < 0) ?
                ((filtered_sample_q3 + 3) >>> 3) :
                ((filtered_sample_q3 + 4) >>> 3);
            if (filtered_stats_sample < filtered_stats_min)
                filtered_stats_min = filtered_stats_sample;
            if (filtered_stats_sample > filtered_stats_max)
                filtered_stats_max = filtered_stats_sample;
            filtered_stats_sum = filtered_stats_sum + filtered_stats_sample;
        end
        if (!filtered_fraction_seen)
            $fatal(1, "filtered READ_DATA lost all Q3 fractional bits");
        ReadReg(REG_MIN_MAX, value);
        if (($signed(value[15:0]) !== filtered_stats_min) ||
            ($signed(value[31:16]) !== filtered_stats_max))
            $fatal(1,
                   "filtered raw-S16 MIN/MAX mismatch actual=%08h expected=%0d/%0d",
                   value, filtered_stats_min, filtered_stats_max);
        ReadReg(REG_SUM, value);
        if ($signed(value) !== filtered_stats_sum)
            $fatal(1,
                   "filtered raw-S16 SUM mismatch actual=%0d expected=%0d",
                   $signed(value), filtered_stats_sum);
        ReadReg(REG_LATEST, value);
        if ($signed(value) !== 32'sd4096)
            $fatal(1, "filtered LATEST lost raw-S16 meaning actual=%0d",
                   $signed(value));
        $display("LTC2208_SPI_SIGNED20_Q3_STATS_S16_PASS");
        $display("LTC2208_CAPTURE_CIC5_R65_FIR81_PASS");
        $display("LTC2208_CAPTURE_SPI_REGRESSION_PASS");
        $finish;
    end
endmodule
