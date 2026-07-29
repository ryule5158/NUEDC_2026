`timescale 1ns / 1ps

module tb_stm32_spi_reg_slave;
    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg spi_sck = 1'b0;
    reg spi_mosi = 1'b0;
    reg spi_cs_n = 1'b1;
    wire spi_miso;
    wire [6:0] reg_addr;
    wire [31:0] reg_wdata;
    wire reg_write;
    wire reg_read_commit;
    reg [31:0] reg_rdata = 32'h89ABCDEF;

    reg write_seen = 1'b0;
    reg read_commit_seen = 1'b0;
    integer index;
    reg [47:0] readback;

    always #10 clk = ~clk; /* 50 MHz */

    stm32_spi_reg_slave dut (
        .clk(clk),
        .reset_n(reset_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .reg_addr(reg_addr),
        .reg_wdata(reg_wdata),
        .reg_write(reg_write),
        .reg_read_commit(reg_read_commit),
        .reg_rdata(reg_rdata)
    );

    always @(posedge clk) begin
        if (reg_write) write_seen <= 1'b1;
        if (reg_read_commit) read_commit_seen <= 1'b1;
    end

    task SpiBit;
        input tx_bit;
        output rx_bit;
        begin
            spi_mosi = tx_bit;
            #500;
            rx_bit = spi_miso;
            spi_sck = 1'b1;
            #500;
            spi_sck = 1'b0;
        end
    endtask

    task SpiFrame;
        input [47:0] tx_frame;
        output [47:0] rx_frame;
        reg rx_bit;
        begin
            rx_frame = 48'd0;
            spi_cs_n = 1'b0;
            #2000;
            for (index = 47; index >= 0; index = index - 1) begin
                SpiBit(tx_frame[index], rx_bit);
                rx_frame[index] = rx_bit;
            end
            #2000;
            spi_cs_n = 1'b1;
            #2000;
        end
    endtask

    initial begin
        #200;
        reset_n = 1'b1;
        #2000;

        SpiFrame({8'hA2, 32'h12345678, 8'h00}, readback);
        if (!write_seen || (reg_addr != 7'h22) ||
            (reg_wdata != 32'h12345678)) begin
            $fatal(1,
                   "SPI write failed seen=%0d addr=%02h data=%08h",
                   write_seen, reg_addr, reg_wdata);
        end

        read_commit_seen = 1'b0;
        SpiFrame({8'h22, 32'h00000000, 8'h00}, readback);
        if (readback != {8'hA5, 8'h5A, 32'h89ABCDEF}) begin
            $fatal(1, "SPI read payload mismatch: %012h", readback);
        end
        if (!read_commit_seen || (reg_addr != 7'h22)) begin
            $fatal(1,
                   "SPI read commit failed seen=%0d addr=%02h",
                   read_commit_seen, reg_addr);
        end

        $display("STM32_SPI_REG_SLAVE_TEST_PASS");
        $finish;
    end
endmodule
