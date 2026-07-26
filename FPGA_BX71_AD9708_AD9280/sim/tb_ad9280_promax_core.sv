`timescale 1ns / 1ps

/* ProMax集成自检：覆盖寄存器、模板、ADC码制、异步FIFO、快照和频带功率。 */
module tb_ad9280_promax_core;

    localparam [6:0] REG_ID            = 7'h30;
    localparam [6:0] REG_VERSION       = 7'h31;
    localparam [6:0] REG_CAPABILITY    = 7'h32;
    localparam [6:0] REG_CONTROL       = 7'h33;
    localparam [6:0] REG_STATUS        = 7'h34;
    localparam [6:0] REG_SNAPSHOT      = 7'h35;
    localparam [6:0] REG_SAMPLE_RATE   = 7'h36;
    localparam [6:0] REG_FIFO_STATUS   = 7'h37;
    localparam [6:0] REG_PHASE_INC0    = 7'h38;
    localparam [6:0] REG_PHASE_OFFSET0 = 7'h40;
    localparam [6:0] REG_MF_SELECTOR   = 7'h48;
    localparam [6:0] REG_MF_COEFF      = 7'h49;
    localparam [6:0] REG_POWER_LO0     = 7'h50;
    localparam [6:0] REG_POWER_HI0     = 7'h58;
    localparam [6:0] REG_WAVE_CONTROL  = 7'h68;
    localparam [6:0] REG_WAVE_STATUS   = 7'h69;
    localparam [6:0] REG_WAVE_MIN_MAX  = 7'h6A;
    localparam [6:0] REG_WAVE_SUM_LO   = 7'h6B;
    localparam [6:0] REG_WAVE_SUM_HI   = 7'h6C;
    localparam [6:0] REG_WAVE_SQ_LO    = 7'h6D;
    localparam [6:0] REG_WAVE_SQ_HI    = 7'h6E;
    localparam [6:0] REG_WAVE_HIGH     = 7'h6F;
    localparam [6:0] REG_WAVE_RISING   = 7'h70;
    localparam [6:0] REG_WAVE_PERIOD   = 7'h71;
    localparam [6:0] REG_WAVE_SAMPLES  = 7'h72;

    reg cfg_clk = 1'b0;
    reg adc_clk = 1'b0;
    reg reset_n = 1'b0;
    reg adc_ready = 1'b0;
    reg [7:0] adc_data = 8'd128;
    reg [6:0] bus_addr = 7'd0;
    reg [31:0] bus_wdata = 32'd0;
    reg bus_write = 1'b0;
    wire bus_select;
    wire [31:0] bus_rdata;

    reg fifo_check_enable = 1'b0;
    reg sine_enable = 1'b0;
    reg [2:0] sine_index = 3'd0;
    reg [17:0] expected_fifo [0:127];
    integer expected_write_count = 0;
    integer expected_read_count = 0;
    integer index;
    integer timeout_count;
    reg [31:0] read_low;
    reg [31:0] read_high;
    reg [31:0] generation_before;
    reg [63:0] power_channel0;
    reg [63:0] power_channel1;
    reg [31:0] wave_status;
    reg [31:0] wave_min_max;
    reg [31:0] wave_sum_low;
    reg [31:0] wave_sum_high;
    reg [31:0] wave_square_low;
    reg [31:0] wave_square_high;
    reg [31:0] wave_count;
    reg [8*37-1:0] forced_power;

    always #10 cfg_clk = ~cfg_clk;
    always #15.625 adc_clk = ~adc_clk;

    /* 产生fs/8余弦，中心码128、峰值100码。 */
    always @(negedge adc_clk) begin
        if (sine_enable) begin
            case (sine_index)
                3'd0: adc_data <= 8'd228;
                3'd1: adc_data <= 8'd199;
                3'd2: adc_data <= 8'd128;
                3'd3: adc_data <= 8'd57;
                3'd4: adc_data <= 8'd28;
                3'd5: adc_data <= 8'd57;
                3'd6: adc_data <= 8'd128;
                default: adc_data <= 8'd199;
            endcase
            sine_index <= sine_index + 1'b1;
        end
    end

    /* FIFO连续性记分板在两个时钟的下降沿读取稳定值。 */
    always @(posedge adc_clk) begin
        if (fifo_check_enable && adc_ready && !dut.fifo_write_busy) begin
            expected_fifo[expected_write_count] = dut.fifo_write_data;
            expected_write_count = expected_write_count + 1;
        end
    end

    always @(negedge cfg_clk) begin
        if (fifo_check_enable && dut.fifo_data_valid) begin
            if (dut.fifo_read_data !== expected_fifo[expected_read_count])
                $fatal(1, "FIFO sample sequence mismatch at %0d",
                       expected_read_count);
            expected_read_count = expected_read_count + 1;
        end
    end

    ad9280_promax_core #(
        .POWER_BLOCK_LOG2(4)
    ) dut (
        .cfg_clk(cfg_clk),
        .adc_clk(adc_clk),
        .reset_n(reset_n),
        .adc_ready(adc_ready),
        .adc_data(adc_data),
        .bus_addr(bus_addr),
        .bus_wdata(bus_wdata),
        .bus_write(bus_write),
        .bus_select(bus_select),
        .bus_rdata(bus_rdata)
    );

    /* 模拟6字节SPI从机解码后的单拍寄存器写。 */
    task WriteReg;
        input [6:0] address;
        input [31:0] value;
        begin
            @(negedge cfg_clk);
            bus_addr = address;
            bus_wdata = value;
            bus_write = 1'b1;
            @(negedge cfg_clk);
            bus_write = 1'b0;
        end
    endtask

    /* 读取组合寄存器并返回稳定快照。 */
    task ReadReg;
        input [6:0] address;
        output [31:0] value;
        begin
            @(negedge cfg_clk);
            bus_addr = address;
            #1 value = bus_rdata;
        end
    endtask

    /* 注入一拍功率结果，单独验证快照冻结语义。 */
    task InjectPower;
        begin
            force dut.band_power_flat = forced_power;
            force dut.band_power_valid = 1'b1;
            @(posedge cfg_clk);
            #1;
            release dut.band_power_valid;
            release dut.band_power_flat;
            @(posedge cfg_clk);
        end
    endtask

    initial begin
        repeat (12) @(posedge cfg_clk);
        reset_n = 1'b1;
        wait (!dut.fifo_write_busy && !dut.fifo_read_busy);
        repeat (4) @(posedge cfg_clk);

        /* 核对固定标识、能力、地址选择和32 MSPS边界。 */
        ReadReg(REG_ID, read_low);
        if (read_low != 32'h4258504D) $fatal(1, "DSP ID mismatch");
        ReadReg(REG_VERSION, read_low);
        if (read_low != 32'h00010100) $fatal(1, "DSP version mismatch");
        ReadReg(REG_CAPABILITY, read_low);
        if (read_low != 32'h29200408) $fatal(1, "DSP capability mismatch");
        ReadReg(REG_SAMPLE_RATE, read_low);
        if (read_low != 32'd32000000) $fatal(1, "DSP sample rate mismatch");
        bus_addr = 7'h2F;
        #1;
        if (bus_select) $fatal(1, "DSP address overlaps legacy registers");
        bus_addr = REG_WAVE_SAMPLES;
        #1;
        if (!bus_select) $fatal(1, "DSP wave-stat address not selected");

        /* 偏移二进制转换必须覆盖18位满量程且零点准确。 */
        adc_data = 8'd0;
        #1;
        if (dut.fifo_write_data != 18'h20000) $fatal(1, "ADC code 0 mismatch");
        adc_data = 8'd128;
        #1;
        if (dut.fifo_write_data != 18'h00000) $fatal(1, "ADC zero mismatch");
        adc_data = 8'd255;
        #1;
        if (dut.fifo_write_data != 18'h1FC00) $fatal(1, "ADC code 255 mismatch");

        /* 写入64个递增样本，确认32/50 MHz异步FIFO不丢样、不乱序。 */
        expected_write_count = 0;
        expected_read_count = 0;
        fifo_check_enable = 1'b1;
        @(negedge adc_clk);
        adc_ready = 1'b1;
        for (index = 0; index < 64; index = index + 1) begin
            adc_data = 8'd64 + index;
            @(negedge adc_clk);
        end
        adc_ready = 1'b0;
        timeout_count = 0;
        while ((expected_read_count < expected_write_count) &&
               (timeout_count < 1000)) begin
            @(posedge cfg_clk);
            timeout_count = timeout_count + 1;
        end
        fifo_check_enable = 1'b0;
        if ((expected_write_count != 64) ||
            (expected_read_count != expected_write_count))
            $fatal(1, "FIFO sample count mismatch: write=%0d read=%0d",
                   expected_write_count, expected_read_count);
        ReadReg(REG_FIFO_STATUS, read_low);
        if (read_low[2]) $fatal(1, "FIFO overflow during nominal clocks");

        /* 配置寄存器必须可读回，模板写必须真正到达指定bank/tap。 */
        WriteReg(REG_PHASE_INC0 + 3, 32'h12345678);
        ReadReg(REG_PHASE_INC0 + 3, read_low);
        if (read_low != 32'h12345678) $fatal(1, "Phase increment mismatch");
        WriteReg(REG_PHASE_OFFSET0 + 5, 32'h89ABCDEF);
        ReadReg(REG_PHASE_OFFSET0 + 5, read_low);
        if (read_low != 32'h89ABCDEF) $fatal(1, "Phase offset mismatch");
        WriteReg(REG_MF_SELECTOR, 32'h00000205);
        WriteReg(REG_MF_COEFF, 32'h0003FFFE);
        repeat (3) @(posedge cfg_clk);
        ReadReg(REG_MF_COEFF, read_low);
        if (read_low != 32'hFFFFFFFE) $fatal(1, "MF coefficient readback mismatch");
        if ($signed(dut.realtime_dsp.u_parallel_matched_filter
                    .gen_matched_filter_bank[2].u_matched_fir.coeff_mem[5])
            != -18'sd2)
            $fatal(1, "MF coefficient did not reach selected hardware tap");

        /* 冻结期间结果和generation必须保持，解冻后才接收新结果。 */
        forced_power = {(8*37){1'b0}};
        forced_power[36:0] = 37'h012345678;
        InjectPower();
        ReadReg(REG_SNAPSHOT, generation_before);
        ReadReg(REG_POWER_LO0, read_low);
        if (read_low != 32'h12345678) $fatal(1, "Initial power snapshot mismatch");
        WriteReg(REG_SNAPSHOT, 32'd1);
        forced_power[36:0] = 37'h001ABCDEF;
        InjectPower();
        ReadReg(REG_SNAPSHOT, read_high);
        if (read_high != (generation_before | 32'd1))
            $fatal(1, "Frozen generation changed");
        ReadReg(REG_POWER_LO0, read_low);
        if (read_low != 32'h12345678) $fatal(1, "Frozen snapshot changed");
        WriteReg(REG_SNAPSHOT, 32'd0);
        InjectPower();
        ReadReg(REG_POWER_LO0, read_low);
        if (read_low != 32'h01ABCDEF) $fatal(1, "Unfrozen snapshot did not update");

        /* 实际ADC fs/8余弦应集中到配置为fs/8的通道，而非fs/4通道。 */
        WriteReg(REG_PHASE_INC0 + 0, 32'h20000000);
        WriteReg(REG_PHASE_INC0 + 1, 32'h40000000);
        WriteReg(REG_PHASE_OFFSET0 + 0, 32'd0);
        WriteReg(REG_PHASE_OFFSET0 + 1, 32'd0);
        WriteReg(REG_WAVE_CONTROL, 32'h00000106);
        WriteReg(REG_CONTROL, 32'd3);
        sine_index = 3'd0;
        sine_enable = 1'b1;
        adc_ready = 1'b1;
        bus_addr = REG_STATUS;
        timeout_count = 0;
        while (!bus_rdata[1] && (timeout_count < 20000)) begin
            @(posedge cfg_clk);
            timeout_count = timeout_count + 1;
        end
        if (timeout_count >= 20000) $fatal(1, "DSP power result timeout");

        /* 第二个64点块应使用自动学习后的门限并给出完整波形统计。 */
        timeout_count = 0;
        ReadReg(REG_WAVE_STATUS, wave_status);
        while (((wave_status & 32'h00000003) != 32'h00000003) &&
               (timeout_count < 20000)) begin
            @(posedge cfg_clk);
            ReadReg(REG_WAVE_STATUS, wave_status);
            timeout_count = timeout_count + 1;
        end
        if (timeout_count >= 20000) $fatal(1, "Wave-stat result timeout");
        ReadReg(REG_WAVE_MIN_MAX, wave_min_max);
        ReadReg(REG_WAVE_SUM_LO, wave_sum_low);
        ReadReg(REG_WAVE_SUM_HI, wave_sum_high);
        ReadReg(REG_WAVE_SQ_LO, wave_square_low);
        ReadReg(REG_WAVE_SQ_HI, wave_square_high);
        ReadReg(REG_WAVE_HIGH, wave_count);
        if ($signed(wave_min_max[7:0]) != -8'sd100 ||
            $signed(wave_min_max[15:8]) != 8'sd100 ||
            $signed(wave_min_max[23:16]) != 8'sd0)
            $fatal(1, "Wave-stat min/max/threshold mismatch: %h",
                   wave_min_max);
        if ({wave_sum_high[7:0], wave_sum_low} != 40'd0)
            $fatal(1, "Wave-stat sum mismatch");
        if ({wave_square_high[15:0], wave_square_low} != 48'd321312)
            $fatal(1, "Wave-stat square sum mismatch: %h_%h",
                   wave_square_high, wave_square_low);
        if (wave_count != 32'd32)
            $fatal(1, "Wave-stat high count mismatch: %0d", wave_count);
        /* 窗口外的前一采样不可见，所以64点内可能只有7个块内上升沿。 */
        ReadReg(REG_WAVE_RISING, wave_count);
        if ((wave_count < 32'd7) || (wave_count > 32'd8))
            $fatal(1, "Wave-stat rising count mismatch: %0d", wave_count);
        ReadReg(REG_WAVE_PERIOD, read_low);
        if (read_low != ((wave_count - 1'b1) << 3))
            $fatal(1, "Wave-stat period mismatch: edges=%0d sum=%0d",
                   wave_count, read_low);
        ReadReg(REG_WAVE_SAMPLES, wave_count);
        if (wave_count != 32'd64)
            $fatal(1, "Wave-stat sample count mismatch: %0d", wave_count);

        WriteReg(REG_SNAPSHOT, 32'd1);
        ReadReg(REG_POWER_LO0 + 0, read_low);
        ReadReg(REG_POWER_HI0 + 0, read_high);
        power_channel0 = {27'd0, read_high[4:0], read_low};
        ReadReg(REG_POWER_LO0 + 1, read_low);
        ReadReg(REG_POWER_HI0 + 1, read_high);
        power_channel1 = {27'd0, read_high[4:0], read_low};
        if ((power_channel0 == 0) ||
            (power_channel0 <= (power_channel1 << 4)))
            $fatal(1, "DDC selectivity mismatch: target=%0d other=%0d",
                   power_channel0, power_channel1);
        ReadReg(REG_STATUS, read_low);
        if (read_low[3]) $fatal(1, "FIFO overflow during DSP run");

        sine_enable = 1'b0;
        adc_ready = 1'b0;
        WriteReg(REG_CONTROL, 32'd0);
        $display("PROMAX_INTEGRATION_TEST_PASS target=%0d other=%0d",
                 power_channel0, power_channel1);
        $finish;
    end

endmodule
