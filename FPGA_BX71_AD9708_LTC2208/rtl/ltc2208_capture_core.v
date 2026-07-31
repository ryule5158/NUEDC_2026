`timescale 1ns / 1ps

/*
 * LTC2208 capture, anti-interference filtering, block RAM and SPI registers.
 * The register map intentionally stays compatible with the proven AD9238
 * STM32 transport; device identity and firmware version prevent a mismatched
 * image from being accepted silently. READ_DATA uses a 12-bit address tag and
 * signed-20 Q3 sample, while MIN/MAX/SUM/LATEST retain raw S16 semantics.
 */
module ltc2208_capture_core(
    input  wire        cfg_clk,
    input  wire        adc_clk,
    input  wire        reset_n,
    input  wire        adc_ready,
    input  wire [15:0] adc_data,

    input  wire [6:0]  bus_addr,
    input  wire [31:0] bus_wdata,
    input  wire        bus_write,
    input  wire        bus_read_commit,
    output wire        bus_select,
    output reg  [31:0] bus_rdata
);
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

    localparam [31:0] DEVICE_ID = 32'h22080001;
    localparam [31:0] SAMPLE_CLK_HZ = 32'd130000000;
    localparam [31:0] G_CAPTURE_VERSION = 32'h00030002;
    localparam [12:0] BUFFER_DEPTH = 13'd4096;
    localparam [15:0] FILTER_DECIMATION = 16'd65;

    localparam [1:0] TRIGGER_IMMEDIATE = 2'd0;
    localparam [1:0] TRIGGER_RISING    = 2'd1;
    localparam [1:0] TRIGGER_FALLING   = 2'd2;
    localparam [1:0] TRIGGER_EITHER    = 2'd3;

    reg [12:0] sample_count_cfg;
    reg [15:0] decimation_cfg;
    reg [1:0] trigger_mode_cfg;
    reg [15:0] trigger_threshold_cfg;
    reg [11:0] read_addr_cfg;
    reg start_toggle_cfg;
    reg abort_toggle_cfg;
    reg signed [19:0] ram_read_data;
    reg capture_valid_cfg;
    reg filter_enable_cfg;

    (* ASYNC_REG = "TRUE" *) reg [12:0] sample_count_meta, sample_count_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] decimation_meta, decimation_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] trigger_mode_meta, trigger_mode_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] trigger_threshold_meta,
                                               trigger_threshold_sync;
    (* ASYNC_REG = "TRUE" *) reg filter_enable_meta, filter_enable_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] start_toggle_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] abort_toggle_sync;

    reg start_toggle_seen;
    reg abort_toggle_seen;
    reg [12:0] sample_count_latched;
    reg [15:0] decimation_latched;
    reg [1:0] trigger_mode_latched;
    reg [15:0] trigger_threshold_latched;
    reg filter_enable_latched;

    reg busy_adc;
    reg triggered_adc;
    reg overrange_adc;
    reg completion_toggle_adc;
    reg [12:0] captured_count_adc;
    reg [31:0] overrange_count_adc;
    reg signed [31:0] sample_sum_adc;
    reg signed [15:0] sample_min_adc;
    reg signed [15:0] sample_max_adc;
    reg signed [15:0] latest_sample_adc;
    reg [15:0] previous_sample_adc;
    reg [11:0] write_addr_adc;
    reg [15:0] decimation_count_adc;

    reg ram_write_enable_adc;
    reg [11:0] ram_write_addr_adc;
    reg signed [19:0] ram_write_data_adc;

    (* ASYNC_REG = "TRUE" *) reg [1:0] busy_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] triggered_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] overrange_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] completion_toggle_sync;
    (* ASYNC_REG = "TRUE" *) reg [12:0] captured_count_meta,
                                               captured_count_sync;
    (* ASYNC_REG = "TRUE" *) reg [31:0] overrange_count_meta,
                                               overrange_count_sync;
    (* ASYNC_REG = "TRUE" *) reg signed [31:0] sample_sum_meta,
                                                      sample_sum_sync;
    (* ASYNC_REG = "TRUE" *) reg signed [15:0] sample_min_meta,
                                                      sample_min_sync;
    (* ASYNC_REG = "TRUE" *) reg signed [15:0] sample_max_meta,
                                                      sample_max_sync;
    (* ASYNC_REG = "TRUE" *) reg signed [15:0] latest_sample_meta,
                                                      latest_sample_sync;

    (* ram_style = "block" *) reg signed [19:0] sample_ram [0:4095];

    wire signed [15:0] centered_adc_sample =
        $signed(adc_data ^ 16'h8000);
    wire signed [19:0] centered_adc_sample_q3 =
        $signed({centered_adc_sample[15], centered_adc_sample, 3'b000});
    wire filtered_output_valid;
    wire signed [19:0] filtered_output_sample;
    wire start_event = start_toggle_sync[1] ^ start_toggle_seen;
    wire abort_event = abort_toggle_sync[1] ^ abort_toggle_seen;
    wire trigger_now = LTC2208_TriggerHit(trigger_mode_latched,
                                           previous_sample_adc,
                                           adc_data,
                                           trigger_threshold_latched);
    wire capture_window = busy_adc && (triggered_adc || trigger_now);
    wire raw_store_sample = capture_window &&
                            ((!triggered_adc) ||
                             (decimation_count_adc == 16'd0));
    wire store_sample = filter_enable_latched ?
                        filtered_output_valid : raw_store_sample;
    wire signed [19:0] stored_sample = filter_enable_latched ?
                                      filtered_output_sample :
                                      centered_adc_sample_q3;
    wire signed [15:0] stored_sample_s16 = LTC2208_Q3ToS16(stored_sample);
    wire inferred_overrange = (adc_data <= 16'h0002) ||
                              (adc_data >= 16'hFFFD);
    wire capture_done_cfg = capture_valid_cfg &&
                            (completion_toggle_sync[2] == start_toggle_cfg);
    wire capture_busy_cfg = capture_valid_cfg && !capture_done_cfg;

    assign bus_select = (bus_addr >= REG_CONTROL) &&
                        (bus_addr <= REG_FILTER);

    ltc2208_cic5_fir81_decimator anti_interference_filter (
        .clk(adc_clk),
        .reset_n(reset_n),
        .clear(start_event || abort_event),
        .input_valid(capture_window && filter_enable_latched),
        .input_sample(centered_adc_sample),
        .output_valid(filtered_output_valid),
        .output_sample(filtered_output_sample)
    );

    function LTC2208_TriggerHit;
        input [1:0] mode;
        input [15:0] previous_sample;
        input [15:0] current_sample;
        input [15:0] threshold;
        begin
            case (mode)
                TRIGGER_IMMEDIATE:
                    LTC2208_TriggerHit = 1'b1;
                TRIGGER_RISING:
                    LTC2208_TriggerHit = (previous_sample < threshold) &&
                                         (current_sample >= threshold);
                TRIGGER_FALLING:
                    LTC2208_TriggerHit = (previous_sample > threshold) &&
                                         (current_sample <= threshold);
                TRIGGER_EITHER:
                    LTC2208_TriggerHit =
                        ((previous_sample < threshold) &&
                         (current_sample >= threshold)) ||
                        ((previous_sample > threshold) &&
                         (current_sample <= threshold));
                default:
                    LTC2208_TriggerHit = 1'b0;
            endcase
        end
    endfunction

    /*
     * Preserve the legacy statistics ABI in physical raw ADC counts.  Q3 is
     * rounded to the nearest S16 code (half ties away from zero), then safely
     * saturated. Raw bypass samples are exact multiples of eight.
     */
    function signed [15:0] LTC2208_Q3ToS16;
        input signed [19:0] sample_q3;
        reg signed [20:0] rounded_q3;
        reg signed [20:0] raw_integer;
        begin
            rounded_q3 = $signed({sample_q3[19], sample_q3}) +
                (sample_q3[19] ? 21'sd3 : 21'sd4);
            raw_integer = rounded_q3 >>> 3;
            if (raw_integer > 21'sd32767)
                LTC2208_Q3ToS16 = 16'sh7FFF;
            else if (raw_integer < -21'sd32768)
                LTC2208_Q3ToS16 = 16'sh8000;
            else
                LTC2208_Q3ToS16 = raw_integer[15:0];
        end
    endfunction

    always @(posedge cfg_clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_count_cfg <= 13'd1024;
            decimation_cfg <= 16'd1;
            trigger_mode_cfg <= TRIGGER_IMMEDIATE;
            trigger_threshold_cfg <= 16'h8000;
            start_toggle_cfg <= 1'b0;
            abort_toggle_cfg <= 1'b0;
            capture_valid_cfg <= 1'b0;
            filter_enable_cfg <= 1'b0;
        end else if (bus_write) begin
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
                    trigger_mode_cfg <= bus_wdata[17:16];
                    trigger_threshold_cfg <= bus_wdata[15:0];
                end
                REG_FILTER:
                    filter_enable_cfg <= bus_wdata[0];
                default: begin end
            endcase
        end
    end

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
    /*
     * Bundled-data CDC: software writes every configuration register before
     * toggling start. The two-stage data synchronizers settle before the
     * synchronized start event is consumed, and the values are latched for
     * the complete capture so later register writes cannot tear one frame.
     */

    always @(posedge adc_clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_count_meta <= 13'd1024;
            sample_count_sync <= 13'd1024;
            decimation_meta <= 16'd1;
            decimation_sync <= 16'd1;
            trigger_mode_meta <= TRIGGER_IMMEDIATE;
            trigger_mode_sync <= TRIGGER_IMMEDIATE;
            trigger_threshold_meta <= 16'h8000;
            trigger_threshold_sync <= 16'h8000;
            filter_enable_meta <= 1'b0;
            filter_enable_sync <= 1'b0;
            start_toggle_sync <= 2'b00;
            abort_toggle_sync <= 2'b00;
        end else begin
            sample_count_meta <= sample_count_cfg;
            sample_count_sync <= sample_count_meta;
            decimation_meta <= decimation_cfg;
            decimation_sync <= decimation_meta;
            trigger_mode_meta <= trigger_mode_cfg;
            trigger_mode_sync <= trigger_mode_meta;
            trigger_threshold_meta <= trigger_threshold_cfg;
            trigger_threshold_sync <= trigger_threshold_meta;
            filter_enable_meta <= filter_enable_cfg;
            filter_enable_sync <= filter_enable_meta;
            start_toggle_sync <= {start_toggle_sync[0], start_toggle_cfg};
            abort_toggle_sync <= {abort_toggle_sync[0], abort_toggle_cfg};
        end
    end

    always @(posedge adc_clk or negedge reset_n) begin
        if (!reset_n) begin
            start_toggle_seen <= 1'b0;
            abort_toggle_seen <= 1'b0;
            sample_count_latched <= 13'd1024;
            decimation_latched <= 16'd1;
            trigger_mode_latched <= TRIGGER_IMMEDIATE;
            trigger_threshold_latched <= 16'h8000;
            filter_enable_latched <= 1'b0;
            busy_adc <= 1'b0;
            triggered_adc <= 1'b0;
            overrange_adc <= 1'b0;
            completion_toggle_adc <= 1'b0;
            captured_count_adc <= 13'd0;
            overrange_count_adc <= 32'd0;
            sample_sum_adc <= 32'sd0;
            sample_min_adc <= 16'sh7FFF;
            sample_max_adc <= 16'sh8000;
            latest_sample_adc <= 16'sd0;
            previous_sample_adc <= 16'h8000;
            decimation_count_adc <= 16'd0;
        end else begin
            latest_sample_adc <= centered_adc_sample;
            previous_sample_adc <= adc_data;

            if (!adc_ready) begin
                busy_adc <= 1'b0;
                triggered_adc <= 1'b0;
            end else if (abort_event) begin
                abort_toggle_seen <= abort_toggle_sync[1];
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
                trigger_threshold_latched <= trigger_threshold_sync;
                filter_enable_latched <= filter_enable_sync &&
                                         (decimation_sync ==
                                          FILTER_DECIMATION);
                busy_adc <= 1'b1;
                triggered_adc <= (trigger_mode_sync == TRIGGER_IMMEDIATE);
                overrange_adc <= 1'b0;
                captured_count_adc <= 13'd0;
                overrange_count_adc <= 32'd0;
                sample_sum_adc <= 32'sd0;
                sample_min_adc <= 16'sh7FFF;
                sample_max_adc <= 16'sh8000;
                decimation_count_adc <= 16'd0;
                previous_sample_adc <= adc_data;
            end else if (busy_adc) begin
                if (!triggered_adc && trigger_now)
                    triggered_adc <= 1'b1;

                if (capture_window && inferred_overrange) begin
                    overrange_adc <= 1'b1;
                    overrange_count_adc <= overrange_count_adc + 1'b1;
                end

                if (store_sample) begin
                    sample_sum_adc <= sample_sum_adc +
                        {{16{stored_sample_s16[15]}}, stored_sample_s16};
                    if (captured_count_adc == 13'd0) begin
                        sample_min_adc <= stored_sample_s16;
                        sample_max_adc <= stored_sample_s16;
                    end else begin
                        if ($signed(stored_sample_s16) < $signed(sample_min_adc))
                            sample_min_adc <= stored_sample_s16;
                        if ($signed(stored_sample_s16) > $signed(sample_max_adc))
                            sample_max_adc <= stored_sample_s16;
                    end

                    captured_count_adc <= captured_count_adc + 1'b1;
                    if (!filter_enable_latched)
                        decimation_count_adc <= decimation_latched - 1'b1;

                    if ((captured_count_adc + 1'b1) >=
                        sample_count_latched) begin
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

    always @(posedge adc_clk) begin
        if (!reset_n || !adc_ready) begin
            ram_write_enable_adc <= 1'b0;
            ram_write_addr_adc <= 12'd0;
            ram_write_data_adc <= 20'sd0;
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

            if (ram_write_enable_adc)
                sample_ram[ram_write_addr_adc] <= ram_write_data_adc;
        end
    end

    always @(posedge cfg_clk) begin
        ram_read_data <= sample_ram[read_addr_cfg];
    end
    /*
     * Result fields stop changing before completion_toggle_adc changes. The
     * result buses use two stages while completion uses three, so software can
     * observe DONE only after a coherent final result has reached cfg_clk.
     * The integrated asynchronous-clock SPI regression checks this contract.
     */

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
            sample_sum_meta <= 32'sd0;
            sample_sum_sync <= 32'sd0;
            sample_min_meta <= 16'sh7FFF;
            sample_min_sync <= 16'sh7FFF;
            sample_max_meta <= 16'sh8000;
            sample_max_sync <= 16'sh8000;
            latest_sample_meta <= 16'sd0;
            latest_sample_sync <= 16'sd0;
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
                bus_rdata = {14'd0, trigger_mode_cfg,
                             trigger_threshold_cfg};
            REG_READ_ADDR:
                bus_rdata = {20'd0, read_addr_cfg};
            REG_READ_DATA:
                bus_rdata = {read_addr_cfg, ram_read_data};
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
                bus_rdata = {sample_max_sync, sample_min_sync};
            REG_SUM:
                bus_rdata = sample_sum_sync;
            REG_LATEST:
                bus_rdata = {{16{latest_sample_sync[15]}},
                             latest_sample_sync};
            REG_FILTER:
                bus_rdata = {31'd0, filter_enable_cfg};
            default:
                bus_rdata = 32'd0;
        endcase
    end
endmodule
