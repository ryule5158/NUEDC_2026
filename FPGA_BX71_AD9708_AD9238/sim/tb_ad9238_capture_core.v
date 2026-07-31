`timescale 1ns / 1ps

module tb_ad9238_capture_core;

    localparam REG_CONTROL          = 7'h20;
    localparam REG_STATUS           = 7'h21;
    localparam REG_SAMPLE_COUNT     = 7'h22;
    localparam REG_DECIMATION       = 7'h23;
    localparam REG_TRIGGER          = 7'h24;
    localparam REG_READ_ADDR        = 7'h25;
    localparam REG_READ_DATA        = 7'h26;
    localparam REG_CAPTURED_COUNT   = 7'h27;
    localparam REG_OVERRANGE_COUNT  = 7'h28;
    localparam REG_SAMPLE_CLK_HZ    = 7'h29;
    localparam REG_DEVICE_ID        = 7'h2A;
    localparam REG_FIRMWARE_VERSION = 7'h2B;
    localparam REG_MIN_MAX          = 7'h2C;
    localparam REG_SUM              = 7'h2D;
    localparam REG_LATEST           = 7'h2E;
    localparam REG_FILTER           = 7'h2F;

    reg cfg_clk = 1'b0;
    reg adc_clk = 1'b0;
    reg reset_n = 1'b0;
    reg adc_ready = 1'b1;
    reg [11:0] adc_data = 12'd2048;
    reg adc_otr = 1'b0;
    reg ramp_enable = 1'b0;
    reg [6:0] bus_addr = 7'd0;
    reg [31:0] bus_wdata = 32'd0;
    reg bus_write = 1'b0;
    reg bus_read_commit = 1'b0;
    wire bus_select;
    wire [31:0] bus_rdata;

    reg signed [15:0] sample_now;
    reg signed [15:0] sample_previous;
    reg signed [15:0] sample_min;
    reg signed [15:0] sample_max;
    reg signed [31:0] sample_sum;
    integer index;
    integer timeout_count;

    always #10 cfg_clk = ~cfg_clk;
    always #10 adc_clk = ~adc_clk;

    always @(negedge adc_clk) begin
        if (ramp_enable)
            adc_data <= adc_data + 1'b1;
    end

    ad9238_capture_core dut (
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

    function signed [15:0] CodeToQ8_8;
        input [11:0] code;
        reg signed [12:0] centered;
        begin
            centered = $signed({1'b0, code}) - 13'sd2048;
            CodeToQ8_8 = {centered[11:0], 4'h0};
        end
    endfunction

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

    task WaitDone;
        begin
            bus_addr = REG_STATUS;
            timeout_count = 0;
            while ((bus_rdata[2] == 1'b0) &&
                   (timeout_count < 10000)) begin
                @(posedge cfg_clk);
                timeout_count = timeout_count + 1;
            end
            if (timeout_count >= 10000)
                $fatal(1, "AD9238 capture timeout");
        end
    endtask

    /*
     * Protocol v2.2:
     *   bits 31:28 = zero
     *   bits 27:16 = 12-bit address tag
     *   bits 15:0  = signed two's-complement Q8.8 sample
     */
    task ReadSample;
        input [11:0] expected_addr;
        output signed [15:0] value;
        begin
            bus_addr = REG_READ_DATA;
            repeat (2) @(posedge cfg_clk);
            if (bus_rdata[31:28] != 4'd0)
                $fatal(1, "READ_DATA reserved bits are nonzero");
            if (bus_rdata[27:16] != expected_addr)
                $fatal(1, "Read address tag mismatch");
            value = $signed(bus_rdata[15:0]);
            @(negedge cfg_clk);
            bus_read_commit = 1'b1;
            @(negedge cfg_clk);
            bus_read_commit = 1'b0;
            repeat (2) @(posedge cfg_clk);
        end
    endtask

    task CheckRawConstant;
        input [11:0] code;
        input signed [15:0] expected_q8_8;
        begin
            ramp_enable = 1'b0;
            adc_data = code;
            WriteReg(REG_FILTER, 32'd0);
            WriteReg(REG_SAMPLE_COUNT, 32'd4);
            WriteReg(REG_DECIMATION, 32'd1);
            WriteReg(REG_TRIGGER, 32'd0);
            WriteReg(REG_CONTROL, 32'd1);
            WaitDone();
            WriteReg(REG_READ_ADDR, 32'd0);
            for (index = 0; index < 4; index = index + 1) begin
                ReadSample(index[11:0], sample_now);
                if ($signed(sample_now) !== expected_q8_8) begin
                    $fatal(1,
                           "Raw Q8.8 mismatch code=%0d actual=%0d expected=%0d",
                           code, $signed(sample_now), expected_q8_8);
                end
            end

            bus_addr = REG_MIN_MAX;
            repeat (2) @(posedge cfg_clk);
            if (($signed(bus_rdata[15:0]) !== expected_q8_8) ||
                ($signed(bus_rdata[31:16]) !== expected_q8_8))
                $fatal(1, "Raw signed MIN/MAX mismatch");

            bus_addr = REG_SUM;
            #1;
            if ($signed(bus_rdata) !==
                (4 * $signed(expected_q8_8)))
                $fatal(1, "Raw signed SUM mismatch");

            bus_addr = REG_LATEST;
            repeat (2) @(posedge cfg_clk);
            if ($signed(bus_rdata[15:0]) !== expected_q8_8)
                $fatal(1, "Raw signed LATEST mismatch");
        end
    endtask

    initial begin
        repeat (5) @(posedge cfg_clk);
        reset_n = 1'b1;
        repeat (5) @(posedge cfg_clk);

        bus_addr = REG_FIRMWARE_VERSION;
        #1;
        if (bus_rdata != 32'h00020002)
            $fatal(1, "G capture protocol version mismatch");
        bus_addr = REG_DEVICE_ID;
        #1;
        if (bus_rdata != 32'hAD923801)
            $fatal(1, "AD9238 device ID mismatch");
        bus_addr = REG_SAMPLE_CLK_HZ;
        #1;
        if (bus_rdata != 32'd50000000)
            $fatal(1, "AD9238 sample clock mismatch");

        /*
         * Raw bypass uses the same centered signed Q8.8 format.  These checks
         * also prove the READ_DATA address shift and high-byte/low-byte order.
         */
        CheckRawConstant(12'd2560, 16'sh2000);
        CheckRawConstant(12'd1536, -16'sh2000);

        /* Eight samples with R=2 must advance by two ADC codes = 0x0200. */
        adc_data = 12'd2000;
        ramp_enable = 1'b1;
        WriteReg(REG_FILTER, 32'd0);
        WriteReg(REG_SAMPLE_COUNT, 32'd8);
        WriteReg(REG_DECIMATION, 32'd2);
        WriteReg(REG_TRIGGER, 32'd0);
        WriteReg(REG_CONTROL, 32'd1);
        WaitDone();
        bus_addr = REG_CAPTURED_COUNT;
        #1;
        if (bus_rdata[12:0] != 13'd8)
            $fatal(1, "Immediate capture count mismatch");

        WriteReg(REG_READ_ADDR, 32'd0);
        ReadSample(12'd0, sample_previous);
        sample_min = sample_previous;
        sample_max = sample_previous;
        sample_sum = sample_previous;
        for (index = 1; index < 8; index = index + 1) begin
            ReadSample(index[11:0], sample_now);
            if (($signed(sample_now) -
                 $signed(sample_previous)) != 16'sd32)
                $fatal(1, "Raw Q8.8 decimation interval mismatch");
            if ($signed(sample_now) < $signed(sample_min))
                sample_min = sample_now;
            if ($signed(sample_now) > $signed(sample_max))
                sample_max = sample_now;
            sample_sum = sample_sum + sample_now;
            sample_previous = sample_now;
        end

        bus_addr = REG_MIN_MAX;
        repeat (2) @(posedge cfg_clk);
        if (($signed(bus_rdata[15:0]) !== $signed(sample_min)) ||
            ($signed(bus_rdata[31:16]) !== $signed(sample_max)))
            $fatal(1, "Signed Q8.8 min/max statistics mismatch");
        bus_addr = REG_SUM;
        #1;
        if ($signed(bus_rdata) !== $signed(sample_sum))
            $fatal(1, "Signed Q8.8 sum statistics mismatch");

        /* A rising trigger stores the threshold-crossing raw sample in Q8.8. */
        ramp_enable = 1'b0;
        adc_data = 12'd2000;
        WriteReg(REG_SAMPLE_COUNT, 32'd4);
        WriteReg(REG_DECIMATION, 32'd1);
        WriteReg(REG_TRIGGER, {14'd0, 2'd1, 4'd0, 12'd2100});
        WriteReg(REG_CONTROL, 32'd1);
        bus_addr = REG_STATUS;
        repeat (20) @(posedge cfg_clk);
        if ((bus_rdata[5] == 1'b0) || (bus_rdata[2] != 1'b0))
            $fatal(1, "Rising trigger wait state mismatch");

        ramp_enable = 1'b1;
        WaitDone();
        WriteReg(REG_READ_ADDR, 32'd0);
        ReadSample(12'd0, sample_now);
        if ($signed(sample_now) !==
            $signed(CodeToQ8_8(12'd2100)))
            $fatal(1, "Rising trigger first sample mismatch");

        /* Falling trigger stores the crossing sample in the same format. */
        ramp_enable = 1'b0;
        adc_data = 12'd2200;
        WriteReg(REG_SAMPLE_COUNT, 32'd4);
        WriteReg(REG_TRIGGER, {14'd0, 2'd2, 4'd0, 12'd2100});
        WriteReg(REG_CONTROL, 32'd1);
        repeat (12) @(posedge adc_clk);
        @(negedge adc_clk);
        adc_data = 12'd2000;
        WaitDone();
        WriteReg(REG_READ_ADDR, 32'd0);
        ReadSample(12'd0, sample_now);
        if ($signed(sample_now) !==
            $signed(CodeToQ8_8(12'd2000)))
            $fatal(1, "Falling trigger first sample mismatch");

        /* OTR is still counted on every physical ADC sample. */
        adc_otr = 1'b1;
        WriteReg(REG_TRIGGER, 32'd0);
        WriteReg(REG_CONTROL, 32'd1);
        WaitDone();
        adc_otr = 1'b0;
        bus_addr = REG_STATUS;
        repeat (2) @(posedge cfg_clk);
        if (bus_rdata[4] != 1'b1)
            $fatal(1, "Overrange status mismatch");
        bus_addr = REG_OVERRANGE_COUNT;
        #1;
        if (bus_rdata != 32'd4)
            $fatal(1, "Overrange count mismatch");

        /* Waiting for a trigger may be aborted without a stale done token. */
        adc_data = 12'd1000;
        WriteReg(REG_TRIGGER, {14'd0, 2'd1, 4'd0, 12'd3000});
        WriteReg(REG_CONTROL, 32'd1);
        repeat (20) @(posedge cfg_clk);
        WriteReg(REG_CONTROL, 32'd2);
        repeat (20) @(posedge cfg_clk);
        bus_addr = REG_STATUS;
        #1;
        if ((bus_rdata[1] != 1'b0) || (bus_rdata[2] != 1'b0))
            $fatal(1, "Abort state mismatch");

        /* CIC mode retains a non-zero DC level as signed Q8.8. */
        ramp_enable = 1'b0;
        adc_data = 12'd2560;
        WriteReg(REG_FILTER, 32'd1);
        bus_addr = REG_FILTER;
        #1;
        if (bus_rdata[0] != 1'b1)
            $fatal(1, "G CIC enable readback mismatch");
        WriteReg(REG_SAMPLE_COUNT, 32'd4);
        WriteReg(REG_DECIMATION, 32'd25);
        WriteReg(REG_TRIGGER, 32'd0);
        WriteReg(REG_CONTROL, 32'd1);
        WaitDone();
        WriteReg(REG_READ_ADDR, 32'd0);
        for (index = 0; index < 4; index = index + 1) begin
            ReadSample(index[11:0], sample_now);
            if ($signed(sample_now) !== 16'sh2000) begin
                $fatal(1,
                       "G CIC Q8.8 DC mismatch index=%0d value=%0d",
                       index, $signed(sample_now));
            end
        end

        $display("AD9238_CAPTURE_R25_TEST_PASS");
        $finish;
    end
endmodule
