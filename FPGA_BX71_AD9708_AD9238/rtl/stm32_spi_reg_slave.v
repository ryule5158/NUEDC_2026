`timescale 1ns / 1ps

/* STM32 SPI寄存器从机：解析固定6字节、模式0的读写帧。 */
module stm32_spi_reg_slave(
    input  wire        clk,
    input  wire        reset_n,

    input  wire        spi_sck,
    input  wire        spi_mosi,
    output wire        spi_miso,
    input  wire        spi_cs_n,

    output reg  [6:0]  reg_addr,
    output reg  [31:0] reg_wdata,
    output reg         reg_write,
    output reg         reg_read_commit,
    input  wire [31:0] reg_rdata
);

    /* 用50 MHz控制时钟同步异步SPI输入。 */
    (* ASYNC_REG = "TRUE" *)
    reg [2:0] sck_sync;
    (* ASYNC_REG = "TRUE" *)
    reg [2:0] cs_sync;
    (* ASYNC_REG = "TRUE" *)
    reg [1:0] mosi_sync;
    reg sck_level;
    reg sck_level_d;
    reg cs_level;
    reg cs_level_d;

    /* SPI帧解析状态、接收移位寄存器和读数据快照。 */
    reg [2:0] bit_cnt;
    reg [2:0] byte_idx;
    reg [7:0] rx_shift;
    reg [7:0] miso_byte;
    reg [31:0] read_snapshot;
    reg [31:0] write_shift;
    reg is_write;
    reg frame_ready;

    /* 经滤波后的片选和时钟边沿事件。 */
    wire cs_active = ~cs_level;
    wire cs_fall = cs_level_d && ~cs_level;
    wire cs_rise = ~cs_level_d && cs_level;
    wire sck_rise = cs_active && ~sck_level_d && sck_level;

    /* 由当前字节和位索引直接选择MISO，避免异步边沿引起移位。 */
    always @(*) begin
        case (byte_idx)
            3'd0: miso_byte = 8'hA5;
            3'd1: miso_byte = 8'h5A;
            3'd2: miso_byte = read_snapshot[31:24];
            3'd3: miso_byte = read_snapshot[23:16];
            3'd4: miso_byte = read_snapshot[15:8];
            3'd5: miso_byte = read_snapshot[7:0];
            default: miso_byte = 8'h00;
        endcase
    end

    assign spi_miso = cs_active ? miso_byte[3'd7 - bit_cnt] : 1'bz;

    /* 过采样并滤除短毛刺；SPI SCK应低于控制时钟的四分之一。 */
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sck_sync <= 3'b000;
            cs_sync <= 3'b111;
            mosi_sync <= 2'b00;
            sck_level <= 1'b0;
            sck_level_d <= 1'b0;
            cs_level <= 1'b1;
            cs_level_d <= 1'b1;
        end else begin
            sck_sync <= {sck_sync[1:0], spi_sck};
            cs_sync <= {cs_sync[1:0], spi_cs_n};
            mosi_sync <= {mosi_sync[0], spi_mosi};
            sck_level_d <= sck_level;
            cs_level_d <= cs_level;

            /* 连续三个采样一致才更新电平，滤除跳线上的短毛刺。 */
            if (&sck_sync) begin
                sck_level <= 1'b1;
            end else if (~|sck_sync) begin
                sck_level <= 1'b0;
            end

            if (&cs_sync) begin
                cs_level <= 1'b1;
            end else if (~|cs_sync) begin
                cs_level <= 1'b0;
            end
        end
    end

    /* 解析SPI帧：字节0为{写标志, 地址[6:0]}，字节1至4承载数据。 */
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            bit_cnt <= 3'd0;
            byte_idx <= 3'd0;
            rx_shift <= 8'd0;
            read_snapshot <= 32'd0;
            write_shift <= 32'd0;
            reg_addr <= 7'd0;
            reg_wdata <= 32'd0;
            reg_write <= 1'b0;
            reg_read_commit <= 1'b0;
            is_write <= 1'b0;
            frame_ready <= 1'b0;
        end else begin
            reg_write <= 1'b0;
            reg_read_commit <= 1'b0;

            if (cs_fall) begin
                bit_cnt <= 3'd0;
                byte_idx <= 3'd0;
                rx_shift <= 8'd0;
                write_shift <= 32'd0;
                is_write <= 1'b0;
                frame_ready <= 1'b0;
            end

            if (sck_rise) begin
                rx_shift <= {rx_shift[6:0], mosi_sync[1]};

                if (bit_cnt == 3'd7) begin
                    bit_cnt <= 3'd0;
                    byte_idx <= byte_idx + 1'b1;

                    case (byte_idx)
                        3'd0: begin
                            is_write <= rx_shift[6];
                            reg_addr <= {rx_shift[5:0], mosi_sync[1]};
                        end

                        3'd1: begin
                            read_snapshot <= reg_rdata;
                            write_shift[31:24] <= {rx_shift[6:0], mosi_sync[1]};
                        end

                        3'd2: begin
                            write_shift[23:16] <= {rx_shift[6:0], mosi_sync[1]};
                        end

                        3'd3: begin
                            write_shift[15:8] <= {rx_shift[6:0], mosi_sync[1]};
                        end

                        3'd4: begin
                            write_shift[7:0] <= {rx_shift[6:0], mosi_sync[1]};
                        end

                        3'd5: begin
                            reg_wdata <= write_shift;
                            if (is_write) begin
                                reg_write <= 1'b1;
                            end
                            frame_ready <= 1'b1;
                        end

                        default: begin end
                    endcase
                end else begin
                    bit_cnt <= bit_cnt + 1'b1;
                end
            end

            if (cs_rise && frame_ready && !is_write) begin
                reg_read_commit <= 1'b1;
            end
        end
    end

endmodule
