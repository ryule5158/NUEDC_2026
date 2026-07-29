`timescale 1ns / 1ps

module tb_ad9280_capture_core;

    localparam REG_CONTROL          = 7'h20;
    localparam REG_STATUS           = 7'h21;
    localparam REG_SAMPLE_COUNT     = 7'h22;
    localparam REG_DECIMATION       = 7'h23;
    localparam REG_TRIGGER          = 7'h24;
    localparam REG_READ_ADDR        = 7'h25;
    localparam REG_READ_DATA        = 7'h26;
    localparam REG_CAPTURED_COUNT   = 7'h27;
    localparam REG_OVERRANGE_COUNT  = 7'h28;
    localparam REG_FIRMWARE_VERSION = 7'h2B;
    localparam REG_MIN_MAX          = 7'h2C;
    localparam REG_SUM              = 7'h2D;
    localparam REG_LATEST           = 7'h2E;
    localparam REG_RESPONSE_PROFILE = 7'h2F;

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

    reg signed [15:0] sample_now;
    integer index;
    integer timeout_count;

    always #10 cfg_clk = ~cfg_clk;
    always #15.625 adc_clk = ~adc_clk;

    always @(negedge adc_clk) begin
        if (ramp_enable)
            adc_data <= adc_data + 1'b1;
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
                $fatal(1, "AD9280 capture timeout");
        end
    endtask

    /*
     * Protocol v1.4:
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

    task CheckCicConstant;
        input [7:0] code;
        input signed [15:0] expected_q8_8;
        begin
            ramp_enable = 1'b0;
            adc_data = code;
            WriteReg(REG_SAMPLE_COUNT, 32'd4);
            WriteReg(REG_DECIMATION, 32'd16);
            WriteReg(REG_TRIGGER, 32'd0);
            WriteReg(REG_CONTROL, 32'd1);
            WaitDone();
            WriteReg(REG_READ_ADDR, 32'd0);
            for (index = 0; index < 4; index = index + 1) begin
                ReadSample(index[11:0], sample_now);
                if ($signed(sample_now) !== expected_q8_8) begin
                    $fatal(1,
                           "CIC Q8.8 mismatch code=%0d actual=%0d expected=%0d",
                           code, $signed(sample_now), expected_q8_8);
                end
            end

            bus_addr = REG_MIN_MAX;
            repeat (2) @(posedge cfg_clk);
            if (($signed(bus_rdata[15:0]) !== expected_q8_8) ||
                ($signed(bus_rdata[31:16]) !== expected_q8_8))
                $fatal(1, "CIC signed MIN/MAX mismatch");

            bus_addr = REG_SUM;
            #1;
            if ($signed(bus_rdata) !==
                (4 * $signed(expected_q8_8)))
                $fatal(1, "CIC signed SUM mismatch");

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
        if (bus_rdata != 32'h00010004)
            $fatal(1, "G capture protocol version mismatch");

        /*
         * Fixed CIC3/R16 uses centered signed Q8.8. Constant codes have unity
         * DC gain and also prove READ_DATA address/byte ordering.
         */
        CheckCicConstant(8'd160, 16'sh2000);
        CheckCicConstant(8'd96, -16'sh2000);

        /* Any requested decimation is forced to the reviewed R=16 profile. */
        WriteReg(REG_DECIMATION, 32'd2);
        bus_addr = REG_DECIMATION;
        #1;
        if (bus_rdata != 32'd16)
            $fatal(1, "Fixed R16 decimation readback mismatch");

        /*
         * A rising trigger starts the fixed CIC capture at the threshold
         * crossing; the buffered frame begins after the CIC warm-up.
         */
        ramp_enable = 1'b0;
        adc_data = 8'd90;
        WriteReg(REG_SAMPLE_COUNT, 32'd4);
        WriteReg(REG_DECIMATION, 32'd16);
        WriteReg(REG_TRIGGER, {22'd0, 2'd1, 8'd100});
        WriteReg(REG_CONTROL, 32'd1);
        bus_addr = REG_STATUS;
        repeat (20) @(posedge cfg_clk);
        if ((bus_rdata[5] == 1'b0) || (bus_rdata[2] != 1'b0))
            $fatal(1, "Rising trigger wait state mismatch");

        ramp_enable = 1'b1;
        WaitDone();
        bus_addr = REG_CAPTURED_COUNT;
        #1;
        if (bus_rdata[12:0] != 13'd4)
            $fatal(1, "Rising trigger capture count mismatch");

        /* Falling trigger starts the same fixed CIC capture path. */
        ramp_enable = 1'b0;
        adc_data = 8'd110;
        WriteReg(REG_SAMPLE_COUNT, 32'd4);
        WriteReg(REG_TRIGGER, {22'd0, 2'd2, 8'd100});
        WriteReg(REG_CONTROL, 32'd1);
        repeat (12) @(posedge adc_clk);
        @(negedge adc_clk);
        adc_data = 8'd90;
        WaitDone();
        bus_addr = REG_CAPTURED_COUNT;
        #1;
        if (bus_rdata[12:0] != 13'd4)
            $fatal(1, "Falling trigger capture count mismatch");

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
        if (bus_rdata < 32'd64)
            $fatal(1, "Physical-clock overrange count mismatch");

        /* Waiting for a trigger may be aborted without a stale done token. */
        adc_data = 8'd80;
        WriteReg(REG_TRIGGER, {22'd0, 2'd1, 8'd200});
        WriteReg(REG_CONTROL, 32'd1);
        repeat (20) @(posedge cfg_clk);
        WriteReg(REG_CONTROL, 32'd2);
        repeat (20) @(posedge cfg_clk);
        bus_addr = REG_STATUS;
        #1;
        if ((bus_rdata[1] != 1'b0) || (bus_rdata[2] != 1'b0))
            $fatal(1, "Abort state mismatch");

        /*
         * The response profile is immutable. The MCU uses this ID to apply
         * the exact CIC3/R16 inverse magnitude response.
         */
        ramp_enable = 1'b0;
        adc_data = 8'd160;
        WriteReg(REG_RESPONSE_PROFILE, 32'hFFFFFFFF);
        bus_addr = REG_RESPONSE_PROFILE;
        #1;
        if (bus_rdata != 32'h00030010)
            $fatal(1, "CIC3/R16 response profile mismatch");
        WriteReg(REG_SAMPLE_COUNT, 32'd4);
        WriteReg(REG_DECIMATION, 32'd16);
        WriteReg(REG_TRIGGER, 32'd0);
        WriteReg(REG_CONTROL, 32'd1);
        WaitDone();
        WriteReg(REG_READ_ADDR, 32'd0);
        for (index = 0; index < 4; index = index + 1) begin
            ReadSample(index[11:0], sample_now);
            if ($signed(sample_now) !== 16'sh2000) begin
                $fatal(1,
                       "CIC3/R16 Q8.8 DC mismatch index=%0d value=%0d",
                       index, $signed(sample_now));
            end
        end

        $display("AD9280_CAPTURE_TEST_PASS");
        $finish;
    end
endmodule
