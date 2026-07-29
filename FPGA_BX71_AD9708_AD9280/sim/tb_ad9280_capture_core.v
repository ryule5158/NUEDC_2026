`timescale 1ns / 1ps

module tb_ad9280_capture_core;

    localparam REG_CONTROL        = 7'h20; /* 采集控制寄存器地址。 */
    localparam REG_STATUS         = 7'h21; /* 采集状态寄存器地址。 */
    localparam REG_SAMPLE_COUNT   = 7'h22; /* 保存点数寄存器地址。 */
    localparam REG_DECIMATION     = 7'h23; /* 抽取倍数寄存器地址。 */
    localparam REG_TRIGGER        = 7'h24; /* 触发配置寄存器地址。 */
    localparam REG_READ_ADDR      = 7'h25; /* 缓存读地址寄存器地址。 */
    localparam REG_READ_DATA      = 7'h26; /* 缓存读数据寄存器地址。 */
    localparam REG_CAPTURED_COUNT = 7'h27; /* 实际保存点数寄存器地址。 */
    localparam REG_OVERRANGE_COUNT = 7'h28; /* 越界采样时钟计数地址。 */
    localparam REG_MIN_MAX        = 7'h2C; /* 样本最小值和最大值地址。 */
    localparam REG_SUM            = 7'h2D; /* 样本累加和地址。 */
    localparam REG_FIRMWARE_VERSION = 7'h2B;
    localparam REG_FILTER         = 7'h2F;

    /* 仿真时钟、复位、ADC数据和寄存器总线信号。 */
    reg cfg_clk = 1'b0;
    reg adc_clk = 1'b0;
    reg reset_n = 1'b0;
    reg adc_ready = 1'b1;
    reg [7:0] adc_data = 8'd20;
    reg adc_otr = 1'b0;
    reg ramp_enable = 1'b0;
    reg [6:0] bus_addr = 7'd0;
    reg [31:0] bus_wdata = 32'd0;
    reg bus_write = 1'b0;
    reg bus_read_commit = 1'b0;
    wire bus_select;
    wire [31:0] bus_rdata;

    /* 仿真检查使用的采样值和循环变量。 */
    reg [7:0] sample_now;
    reg [7:0] sample_previous;
    reg [7:0] sample_min;
    reg [7:0] sample_max;
    reg [31:0] sample_sum;
    integer index;
    integer timeout_count;

    always #10 cfg_clk = ~cfg_clk;
    always #15.625 adc_clk = ~adc_clk;

    /* 在ADC下降沿更新输入，保证上升沿采样数据稳定。 */
    always @(negedge adc_clk) begin
        if (ramp_enable) begin
            adc_data <= adc_data + 1'b1;
        end
    end

    ad9280_capture_core dut (
        .cfg_clk(cfg_clk),
        .adc_clk(adc_clk),
        .reset_n(reset_n),
        .adc_ready(adc_ready),
        .adc_data(adc_data),
        .adc_otr(adc_otr),
        .bus_addr(bus_addr),
        .bus_wdata(bus_wdata),
        .bus_write(bus_write),
        .bus_read_commit(bus_read_commit),
        .bus_select(bus_select),
        .bus_rdata(bus_rdata)
    );

    /* 模拟STM32写一个FPGA寄存器。 */
    task WriteReg;
        input [6:0] addr;
        input [31:0] value;
        begin
            @(negedge cfg_clk);
            bus_addr = addr;
            bus_wdata = value;
            bus_write = 1'b1;
            @(negedge cfg_clk);
            bus_write = 1'b0;
        end
    endtask

    /* 等待FPGA完成当前采集并限制仿真超时。 */
    task WaitDone;
        begin
            bus_addr = REG_STATUS;
            timeout_count = 0;
            while ((bus_rdata[2] == 1'b0) && (timeout_count < 10000)) begin
                @(posedge cfg_clk);
                timeout_count = timeout_count + 1;
            end
            if (timeout_count >= 10000) begin
                $fatal(1, "AD9280 capture timeout");
            end
        end
    endtask

    /* 读取一个缓存样本并提交自动地址递增。 */
    task ReadSample;
        input [11:0] expected_addr;
        output [7:0] value;
        begin
            bus_addr = REG_READ_DATA;
            repeat (2) @(posedge cfg_clk);
            if (bus_rdata[19:8] != expected_addr) begin
                $fatal(1, "Read address tag mismatch");
            end
            value = bus_rdata[7:0];
            @(negedge cfg_clk);
            bus_read_commit = 1'b1;
            @(negedge cfg_clk);
            bus_read_commit = 1'b0;
            repeat (2) @(posedge cfg_clk);
        end
    endtask

    initial begin
        repeat (5) @(posedge cfg_clk);
        reset_n = 1'b1;
        repeat (5) @(posedge cfg_clk);

        /* 用8点、2倍抽取验证缓存点数和抽取间隔。 */
        ramp_enable = 1'b1;
        WriteReg(REG_SAMPLE_COUNT, 32'd8);
        WriteReg(REG_DECIMATION, 32'd2);
        WriteReg(REG_TRIGGER, 32'd0);
        WriteReg(REG_CONTROL, 32'd1);
        WaitDone();
        bus_addr = REG_CAPTURED_COUNT;
        #1;
        if (bus_rdata[12:0] != 13'd8) begin
            $fatal(1, "Immediate capture count mismatch");
        end

        WriteReg(REG_READ_ADDR, 32'd0);
        ReadSample(12'd0, sample_previous);
        sample_min = sample_previous;
        sample_max = sample_previous;
        sample_sum = sample_previous;
        for (index = 1; index < 8; index = index + 1) begin
            ReadSample(index, sample_now);
            if ((sample_now - sample_previous) != 8'd2) begin
                $fatal(1, "Decimation interval mismatch");
            end
            if (sample_now < sample_min) sample_min = sample_now;
            if (sample_now > sample_max) sample_max = sample_now;
            sample_sum = sample_sum + sample_now;
            sample_previous = sample_now;
        end
        bus_addr = REG_MIN_MAX;
        repeat (2) @(posedge cfg_clk);
        if ((bus_rdata[7:0] != sample_min) ||
            (bus_rdata[15:8] != sample_max)) begin
            $fatal(1, "Min/max statistics mismatch");
        end
        bus_addr = REG_SUM;
        #1;
        if (bus_rdata != sample_sum) begin
            $fatal(1, "Sample sum mismatch");
        end

        /* 保持90码，确认上升阈值触发前不会写入缓存。 */
        ramp_enable = 1'b0;
        adc_data = 8'd90;
        WriteReg(REG_SAMPLE_COUNT, 32'd4);
        WriteReg(REG_DECIMATION, 32'd1);
        WriteReg(REG_TRIGGER, {22'd0, 2'd1, 8'd100});
        WriteReg(REG_CONTROL, 32'd1);
        bus_addr = REG_STATUS;
        repeat (20) @(posedge cfg_clk);
        if ((bus_rdata[5] == 1'b0) || (bus_rdata[2] != 1'b0)) begin
            $fatal(1, "Rising trigger wait state mismatch");
        end

        ramp_enable = 1'b1;
        WaitDone();
        WriteReg(REG_READ_ADDR, 32'd0);
        ReadSample(12'd0, sample_now);
        if (sample_now != 8'd100) begin
            $fatal(1, "Rising trigger first sample mismatch");
        end

        /* 验证下降沿触发能把跨阈值样本作为首个缓存点。 */
        ramp_enable = 1'b0;
        adc_data = 8'd110;
        WriteReg(REG_SAMPLE_COUNT, 32'd4);
        WriteReg(REG_TRIGGER, {22'd0, 2'd2, 8'd100});
        WriteReg(REG_CONTROL, 32'd1);
        repeat (12) @(posedge adc_clk);
        @(negedge adc_clk);
        adc_data = 8'd90;
        WaitDone();
        WriteReg(REG_READ_ADDR, 32'd0);
        ReadSample(12'd0, sample_now);
        if (sample_now != 8'd90) begin
            $fatal(1, "Falling trigger first sample mismatch");
        end

        /* 验证OTR在完整采集窗口内按物理采样时钟计数。 */
        adc_otr = 1'b1;
        WriteReg(REG_TRIGGER, 32'd0);
        WriteReg(REG_CONTROL, 32'd1);
        WaitDone();
        adc_otr = 1'b0;
        bus_addr = REG_STATUS;
        repeat (2) @(posedge cfg_clk);
        if (bus_rdata[4] != 1'b1) begin
            $fatal(1, "Overrange status mismatch");
        end
        bus_addr = REG_OVERRANGE_COUNT;
        #1;
        if (bus_rdata != 32'd4) begin
            $fatal(1, "Overrange count mismatch");
        end

        /* 验证等待触发期间可以主动终止，且不会留下伪完成状态。 */
        adc_data = 8'd80;
        WriteReg(REG_TRIGGER, {22'd0, 2'd1, 8'd200});
        WriteReg(REG_CONTROL, 32'd1);
        repeat (20) @(posedge cfg_clk);
        WriteReg(REG_CONTROL, 32'd2);
        repeat (20) @(posedge cfg_clk);
        bus_addr = REG_STATUS;
        #1;
        if ((bus_rdata[1] != 1'b0) || (bus_rdata[2] != 1'b0)) begin
            $fatal(1, "Abort state mismatch");
        end

        /* G-mode integration: CIC3/R16 keeps a constant offset-binary code. */
        ramp_enable = 1'b0;
        adc_data = 8'd128;
        bus_addr = REG_FIRMWARE_VERSION;
        #1;
        if (bus_rdata != 32'h00010002) begin
            $fatal(1, "G capture firmware version mismatch");
        end
        WriteReg(REG_FILTER, 32'd1);
        bus_addr = REG_FILTER;
        #1;
        if (bus_rdata[0] != 1'b1) begin
            $fatal(1, "G CIC enable readback mismatch");
        end
        WriteReg(REG_SAMPLE_COUNT, 32'd4);
        WriteReg(REG_DECIMATION, 32'd16);
        WriteReg(REG_TRIGGER, 32'd0);
        WriteReg(REG_CONTROL, 32'd1);
        WaitDone();
        WriteReg(REG_READ_ADDR, 32'd0);
        for (index = 0; index < 4; index = index + 1) begin
            ReadSample(index, sample_now);
            if (sample_now != 8'd128) begin
                $fatal(1, "G CIC constant-code mismatch index=%0d value=%0d", index, sample_now);
            end
        end

        $display("AD9280_CAPTURE_TEST_PASS");
        $finish;
    end

endmodule
