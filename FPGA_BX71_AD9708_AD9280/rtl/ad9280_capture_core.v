`timescale 1ns / 1ps

/* AD9280采集核心：完成触发、抽取、缓存、统计和寄存器读写。 */
module ad9280_capture_core(
    input  wire        cfg_clk,
    input  wire        adc_clk,
    input  wire        reset_n,
    input  wire        adc_ready,
    input  wire [7:0]  adc_data,
    input  wire        adc_otr,

    input  wire [6:0]  bus_addr,
    input  wire [31:0] bus_wdata,
    input  wire        bus_write,
    input  wire        bus_read_commit,
    output wire        bus_select,
    output reg  [31:0] bus_rdata
);

    localparam REG_CONTROL         = 7'h20; /* 采集启动和终止寄存器。 */
    localparam REG_STATUS          = 7'h21; /* 采集状态寄存器。 */
    localparam REG_SAMPLE_COUNT    = 7'h22; /* 目标采样点数寄存器。 */
    localparam REG_DECIMATION      = 7'h23; /* FPGA抽取倍数寄存器。 */
    localparam REG_TRIGGER         = 7'h24; /* 触发模式和阈值寄存器。 */
    localparam REG_READ_ADDR       = 7'h25; /* 采样缓存读地址寄存器。 */
    localparam REG_READ_DATA       = 7'h26; /* 采样缓存数据寄存器。 */
    localparam REG_CAPTURED_COUNT  = 7'h27; /* 实际已采集点数寄存器。 */
    localparam REG_OVERRANGE_COUNT = 7'h28; /* 越界采样时钟计数寄存器。 */
    localparam REG_SAMPLE_CLK_HZ   = 7'h29; /* ADC物理采样时钟寄存器。 */
    localparam REG_DEVICE_ID       = 7'h2A; /* AD9280采集单元设备标识。 */
    localparam REG_FIRMWARE_VERSION = 7'h2B; /* AD9280采集协议版本。 */
    localparam REG_MIN_MAX         = 7'h2C; /* 已存样本最小值和最大值。 */
    localparam REG_SUM             = 7'h2D; /* 已存样本累加和。 */
    localparam REG_LATEST          = 7'h2E; /* 最近一个ADC采样码。 */
    localparam REG_FILTER          = 7'h2F; /* bit0: enable G-mode CIC3/R16 anti-alias decimator. */

    localparam [31:0] DEVICE_ID       = 32'hAD928001; /* AD9280采集单元标识。 */
    localparam [31:0] FIRMWARE_VERSION = 32'h00010001; /* 当前采集协议版本。 */
    localparam [31:0] SAMPLE_CLK_HZ    = 32'd32000000; /* AD9280采样时钟。 */
    localparam [12:0] BUFFER_DEPTH     = 13'd4096; /* FPGA采样缓存深度。 */
    localparam [31:0] G_CAPTURE_VERSION = 32'h00010002;

    localparam [1:0] TRIGGER_IMMEDIATE = 2'd0; /* 立即开始采集。 */
    localparam [1:0] TRIGGER_RISING    = 2'd1; /* 阈值上升沿触发。 */
    localparam [1:0] TRIGGER_FALLING   = 2'd2; /* 阈值下降沿触发。 */
    localparam [1:0] TRIGGER_EITHER    = 2'd3; /* 阈值任意边沿触发。 */

    /* 配置时钟域寄存器。 */
    reg [12:0] sample_count_cfg;
    reg [15:0] decimation_cfg;
    reg [1:0] trigger_mode_cfg;
    reg [7:0] trigger_threshold_cfg;
    reg [11:0] read_addr_cfg;
    reg start_toggle_cfg;
    reg abort_toggle_cfg;
    reg [7:0] ram_read_data;
    reg capture_valid_cfg;
    reg filter_enable_cfg;

    /* ADC时钟域配置同步与锁存寄存器。 */
    reg [12:0] sample_count_meta;
    reg [12:0] sample_count_sync;
    reg [15:0] decimation_meta;
    reg [15:0] decimation_sync;
    reg [1:0] trigger_mode_meta;
    reg [1:0] trigger_mode_sync;
    reg [7:0] trigger_threshold_meta;
    reg [7:0] trigger_threshold_sync;
    reg filter_enable_meta;
    reg filter_enable_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] start_toggle_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] abort_toggle_sync;
    reg start_toggle_seen;
    reg abort_toggle_seen;
    reg [12:0] sample_count_latched;
    reg [15:0] decimation_latched;
    reg [1:0] trigger_mode_latched;
    reg [7:0] trigger_threshold_latched;

    reg filter_enable_latched;
    /* ADC时钟域采集状态与统计量。 */
    reg busy_adc;
    reg triggered_adc;
    reg overrange_adc;
    reg completion_toggle_adc;
    reg [12:0] captured_count_adc;
    reg [31:0] overrange_count_adc;
    reg [31:0] sample_sum_adc;
    reg [7:0] sample_min_adc;
    reg [7:0] sample_max_adc;
    reg [7:0] latest_sample_adc;
    reg [7:0] previous_sample_adc;
    reg [11:0] write_addr_adc;
    reg [15:0] decimation_count_adc;

    /* 块RAM写端口流水寄存器。 */
    reg ram_write_enable_adc;
    reg [11:0] ram_write_addr_adc;
    reg [7:0] ram_write_data_adc;

    /* 返回配置时钟域前的两级状态同步寄存器。 */
    (* ASYNC_REG = "TRUE" *) reg [1:0] busy_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] triggered_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] overrange_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] completion_toggle_sync;
    reg [12:0] captured_count_meta;
    reg [12:0] captured_count_sync;
    reg [31:0] overrange_count_meta;
    reg [31:0] overrange_count_sync;
    reg [31:0] sample_sum_meta;
    reg [31:0] sample_sum_sync;
    reg [7:0] sample_min_meta;
    reg [7:0] sample_min_sync;
    reg [7:0] sample_max_meta;
    reg [7:0] sample_max_sync;
    reg [7:0] latest_sample_meta;
    reg [7:0] latest_sample_sync;

    /* 4096×8位双时钟采样缓存，综合为FPGA块RAM。 */
    (* ram_style = "block" *) reg [7:0] sample_ram [0:4095];

    wire cic_output_valid;
    wire [7:0] cic_output_code;
    wire raw_store_sample;
    wire [7:0] stored_sample;
    /* 跨时钟控制事件和采集状态组合条件。 */
    wire start_event = start_toggle_sync[1] ^ start_toggle_seen;
    wire abort_event = abort_toggle_sync[1] ^ abort_toggle_seen;
    wire trigger_now = AD9280_TriggerHit(trigger_mode_latched,
                                         previous_sample_adc,
                                         adc_data,
                                         trigger_threshold_latched);
    wire capture_window = busy_adc && (triggered_adc || trigger_now);
    assign raw_store_sample = capture_window &&
                              ((!triggered_adc) ||
                               (decimation_count_adc == 16'd0));
    wire store_sample = filter_enable_latched ?
                        cic_output_valid : raw_store_sample;
    assign stored_sample = filter_enable_latched ?
                           cic_output_code : adc_data;
    wire capture_done_cfg = capture_valid_cfg &&
                            (completion_toggle_sync[2] == start_toggle_cfg);
    wire capture_busy_cfg = capture_valid_cfg && !capture_done_cfg;

    assign bus_select = (bus_addr >= REG_CONTROL) && (bus_addr <= REG_FILTER);

    /* 判断当前样本是否满足所选阈值触发条件。 */
    cic3_decimator_16 g_cic_decimator (
        .clk(adc_clk),
        .reset_n(reset_n),
        .clear(start_event || abort_event),
        .input_valid(capture_window && filter_enable_latched),
        .input_code(adc_data),
        .output_valid(cic_output_valid),
        .output_code(cic_output_code)
    );


    function AD9280_TriggerHit;
        /* 触发模式、相邻样本和比较阈值。 */
        input [1:0] mode;
        input [7:0] previous_sample;
        input [7:0] current_sample;
        input [7:0] threshold;
        begin
            case (mode)
                TRIGGER_IMMEDIATE:
                    AD9280_TriggerHit = 1'b1;
                TRIGGER_RISING:
                    AD9280_TriggerHit = (previous_sample < threshold) &&
                                        (current_sample >= threshold);
                TRIGGER_FALLING:
                    AD9280_TriggerHit = (previous_sample > threshold) &&
                                        (current_sample <= threshold);
                TRIGGER_EITHER:
                    AD9280_TriggerHit = ((previous_sample < threshold) &&
                                         (current_sample >= threshold)) ||
                                        ((previous_sample > threshold) &&
                                         (current_sample <= threshold));
                default:
                    AD9280_TriggerHit = 1'b0;
            endcase
        end
    endfunction

    /* 配置寄存器写入、缓存读取和读地址自动递增。 */
    always @(posedge cfg_clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_count_cfg <= 13'd1024;
            decimation_cfg <= 16'd1;
            trigger_mode_cfg <= TRIGGER_IMMEDIATE;
            trigger_threshold_cfg <= 8'd128;
            start_toggle_cfg <= 1'b0;
            abort_toggle_cfg <= 1'b0;
            capture_valid_cfg <= 1'b0;
            filter_enable_cfg <= 1'b0;
        end else begin
            if (bus_write) begin
                case (bus_addr)
                    REG_CONTROL: begin
                        if (bus_wdata[0]) begin
                            start_toggle_cfg <= ~start_toggle_cfg;
                            capture_valid_cfg <= 1'b1;
                        end
                        if (bus_wdata[1]) begin
                            abort_toggle_cfg <= ~abort_toggle_cfg;
                            capture_valid_cfg <= 1'b0;
                        end
                    end
                    REG_SAMPLE_COUNT:
                        sample_count_cfg <= bus_wdata[12:0];
                    REG_DECIMATION:
                        decimation_cfg <= bus_wdata[15:0];
                    REG_TRIGGER: begin
                        trigger_mode_cfg <= bus_wdata[9:8];
                        trigger_threshold_cfg <= bus_wdata[7:0];
                    end
                    REG_FILTER:
                        filter_enable_cfg <= bus_wdata[0];
                    default: begin end
                endcase
            end
        end
    end

    /* 管理块RAM读地址，读完一个样本后自动递增。 */
    always @(posedge cfg_clk) begin
        if (!reset_n) begin
            read_addr_cfg <= 12'd0;
        end else if (bus_write && (bus_addr == REG_CONTROL) &&
                     bus_wdata[0]) begin
            read_addr_cfg <= 12'd0;
        end else if (bus_write && (bus_addr == REG_READ_ADDR)) begin
            read_addr_cfg <= bus_wdata[11:0];
        end else if (bus_read_commit && (bus_addr == REG_READ_DATA)) begin
            read_addr_cfg <= read_addr_cfg + 1'b1;
        end
    end

    /* 将控制配置和启动事件同步到32 MHz采样时钟域。 */
    always @(posedge adc_clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_count_meta <= 13'd1024;
            sample_count_sync <= 13'd1024;
            decimation_meta <= 16'd1;
            decimation_sync <= 16'd1;
            trigger_mode_meta <= TRIGGER_IMMEDIATE;
            trigger_mode_sync <= TRIGGER_IMMEDIATE;
            trigger_threshold_meta <= 8'd128;
            trigger_threshold_sync <= 8'd128;
            start_toggle_sync <= 2'b00;
            filter_enable_meta <= 1'b0;
            filter_enable_sync <= 1'b0;
            abort_toggle_sync <= 2'b00;
        end else begin
            sample_count_meta <= sample_count_cfg;
            sample_count_sync <= sample_count_meta;
            decimation_meta <= decimation_cfg;
            decimation_sync <= decimation_meta;
            trigger_mode_meta <= trigger_mode_cfg;
            trigger_mode_sync <= trigger_mode_meta;
            trigger_threshold_meta <= trigger_threshold_cfg;
            filter_enable_meta <= filter_enable_cfg;
            filter_enable_sync <= filter_enable_meta;
            trigger_threshold_sync <= trigger_threshold_meta;
            start_toggle_sync <= {start_toggle_sync[0], start_toggle_cfg};
            abort_toggle_sync <= {abort_toggle_sync[0], abort_toggle_cfg};
        end
    end

    /* 在ADC时钟域执行触发、抽取、缓存和统计。 */
    always @(posedge adc_clk or negedge reset_n) begin
        if (!reset_n) begin
            start_toggle_seen <= 1'b0;
            abort_toggle_seen <= 1'b0;
            sample_count_latched <= 13'd1024;
            decimation_latched <= 16'd1;
            trigger_mode_latched <= TRIGGER_IMMEDIATE;
            filter_enable_latched <= 1'b0;
            trigger_threshold_latched <= 8'd128;
            busy_adc <= 1'b0;
            triggered_adc <= 1'b0;
            overrange_adc <= 1'b0;
            completion_toggle_adc <= 1'b0;
            captured_count_adc <= 13'd0;
            overrange_count_adc <= 32'd0;
            sample_sum_adc <= 32'd0;
            sample_min_adc <= 8'hFF;
            sample_max_adc <= 8'h00;
            latest_sample_adc <= 8'd128;
            previous_sample_adc <= 8'd128;
            decimation_count_adc <= 16'd0;
        end else begin
            latest_sample_adc <= adc_data;
            previous_sample_adc <= adc_data;

            if (!adc_ready) begin
                busy_adc <= 1'b0;
                triggered_adc <= 1'b0;
            end else if (abort_event) begin
                abort_toggle_seen <= abort_toggle_sync[1];
                /* Acknowledge the aborted start toggle so the next command
                 * cannot alias the stale completion token. */
                completion_toggle_adc <= start_toggle_seen;
                busy_adc <= 1'b0;
                triggered_adc <= 1'b0;
            end else if (start_event) begin
                start_toggle_seen <= start_toggle_sync[1];
                sample_count_latched <=
                    ((sample_count_sync == 13'd0) ||
                     (sample_count_sync > BUFFER_DEPTH)) ?
                    BUFFER_DEPTH : sample_count_sync;
                decimation_latched <=
                    (decimation_sync == 16'd0) ? 16'd1 : decimation_sync;
                trigger_mode_latched <= trigger_mode_sync;
                filter_enable_latched <= filter_enable_sync &&
                                         (decimation_sync == 16'd16);
                trigger_threshold_latched <= trigger_threshold_sync;
                busy_adc <= 1'b1;
                triggered_adc <= (trigger_mode_sync == TRIGGER_IMMEDIATE);
                overrange_adc <= 1'b0;
                captured_count_adc <= 13'd0;
                overrange_count_adc <= 32'd0;
                sample_sum_adc <= 32'd0;
                sample_min_adc <= 8'hFF;
                sample_max_adc <= 8'h00;
                decimation_count_adc <= 16'd0;
                previous_sample_adc <= adc_data;
            end else if (busy_adc) begin
                if (!triggered_adc && trigger_now) begin
                    triggered_adc <= 1'b1;
                end

                if (capture_window && adc_otr) begin
                    overrange_adc <= 1'b1;
                    overrange_count_adc <= overrange_count_adc + 1'b1;
                end

                if (store_sample) begin
                    sample_sum_adc <= sample_sum_adc + stored_sample;
                    if (captured_count_adc == 13'd0) begin
                        sample_min_adc <= stored_sample;
                        sample_max_adc <= stored_sample;
                    end else begin
                        if (stored_sample < sample_min_adc) begin
                            sample_min_adc <= stored_sample;
                        end
                        if (stored_sample > sample_max_adc) begin
                            sample_max_adc <= stored_sample;
                        end
                    end

                    captured_count_adc <= captured_count_adc + 1'b1;
                    if (!filter_enable_latched) begin
                        decimation_count_adc <= decimation_latched - 1'b1;
                    end

                    if ((captured_count_adc + 1'b1) >= sample_count_latched) begin
                        busy_adc <= 1'b0;
                        completion_toggle_adc <= start_toggle_seen;
                    end
                end else if (!filter_enable_latched &&
                             triggered_adc &&
                             (decimation_count_adc != 16'd0)) begin
                    decimation_count_adc <= decimation_count_adc - 1'b1;
                end
            end
        end
    end

    /* 双时钟块RAM写端口。 */
    always @(posedge adc_clk) begin
        if (!reset_n || !adc_ready) begin
            ram_write_enable_adc <= 1'b0;
            ram_write_addr_adc <= 12'd0;
            ram_write_data_adc <= 8'd128;
            write_addr_adc <= 12'd0;
        end else begin
            ram_write_enable_adc <= store_sample;
            if (start_event) begin
                write_addr_adc <= 12'd0;
            end else if (store_sample) begin
                ram_write_addr_adc <= write_addr_adc;
                ram_write_data_adc <= stored_sample;
                write_addr_adc <= write_addr_adc + 1'b1;
            end

            if (ram_write_enable_adc) begin
                sample_ram[ram_write_addr_adc] <= ram_write_data_adc;
            end
        end
    end

    /* 双时钟块RAM读端口。 */
    always @(posedge cfg_clk) begin
        ram_read_data <= sample_ram[read_addr_cfg];
    end

    /* 将稳定状态和统计快照同步回50 MHz配置时钟域。 */
    always @(posedge cfg_clk or negedge reset_n) begin
        if (!reset_n) begin
            busy_sync <= 2'b00;
            triggered_sync <= 2'b00;
            overrange_sync <= 2'b00;
            completion_toggle_sync <= 3'b000;
            captured_count_meta <= 13'd0;
            captured_count_sync <= 13'd0;
            overrange_count_meta <= 32'd0;
            overrange_count_sync <= 32'd0;
            sample_sum_meta <= 32'd0;
            sample_sum_sync <= 32'd0;
            sample_min_meta <= 8'hFF;
            sample_min_sync <= 8'hFF;
            sample_max_meta <= 8'h00;
            sample_max_sync <= 8'h00;
            latest_sample_meta <= 8'd128;
            latest_sample_sync <= 8'd128;
        end else begin
            busy_sync <= {busy_sync[0], busy_adc};
            triggered_sync <= {triggered_sync[0], triggered_adc};
            overrange_sync <= {overrange_sync[0], overrange_adc};
            completion_toggle_sync <= {completion_toggle_sync[1:0],
                                       completion_toggle_adc};
            captured_count_meta <= captured_count_adc;
            captured_count_sync <= captured_count_meta;
            overrange_count_meta <= overrange_count_adc;
            overrange_count_sync <= overrange_count_meta;
            sample_sum_meta <= sample_sum_adc;
            sample_sum_sync <= sample_sum_meta;
            sample_min_meta <= sample_min_adc;
            sample_min_sync <= sample_min_meta;
            sample_max_meta <= sample_max_adc;
            sample_max_sync <= sample_max_meta;
            latest_sample_meta <= latest_sample_adc;
            latest_sample_sync <= latest_sample_meta;
        end
    end

    /* AD9280寄存器读回多路选择。 */
    always @(*) begin
        case (bus_addr)
            REG_CONTROL:
                bus_rdata = 32'd0;
            REG_STATUS:
                bus_rdata = {26'd0,
                             busy_sync[1] && !triggered_sync[1],
                             overrange_sync[1],
                             triggered_sync[1],
                             capture_done_cfg,
                             capture_busy_cfg,
                             adc_ready};
            REG_SAMPLE_COUNT:
                bus_rdata = {19'd0, sample_count_cfg};
            REG_DECIMATION:
                bus_rdata = {16'd0, decimation_cfg};
            REG_TRIGGER:
                bus_rdata = {22'd0, trigger_mode_cfg,
                             trigger_threshold_cfg};
            REG_READ_ADDR:
                bus_rdata = {20'd0, read_addr_cfg};
            REG_READ_DATA:
                bus_rdata = {12'd0, read_addr_cfg, ram_read_data};
            REG_CAPTURED_COUNT:
                bus_rdata = {19'd0, captured_count_sync};
            REG_OVERRANGE_COUNT:
                bus_rdata = overrange_count_sync;
            REG_SAMPLE_CLK_HZ:
                bus_rdata = SAMPLE_CLK_HZ;
            REG_DEVICE_ID:
                bus_rdata = DEVICE_ID;
            REG_FIRMWARE_VERSION:
                bus_rdata = G_CAPTURE_VERSION;
            REG_MIN_MAX:
                bus_rdata = {16'd0, sample_max_sync, sample_min_sync};
            REG_SUM:
                bus_rdata = sample_sum_sync;
            REG_LATEST:
                bus_rdata = {24'd0, latest_sample_sync};
            REG_FILTER:
                bus_rdata = {31'd0, filter_enable_cfg};
            default:
                bus_rdata = 32'd0;
        endcase
    end

endmodule
