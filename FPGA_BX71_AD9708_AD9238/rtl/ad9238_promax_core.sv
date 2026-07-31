`timescale 1ns / 1ps

/* ProMax实时DSP桥：将50 MSPS ADC数据安全送入50 MHz并行DSP数据面。 */
module ad9238_promax_core #(
    parameter integer POWER_BLOCK_LOG2 = 10 /* 功率平均点数为2的该次幂。 */
)(
    input  wire        cfg_clk,
    input  wire        adc_clk,
    input  wire        reset_n,
    input  wire        adc_ready,
    input  wire [11:0] adc_data,

    input  wire [6:0]  bus_addr,
    input  wire [31:0] bus_wdata,
    input  wire        bus_write,
    output wire        bus_select,
    output reg  [31:0] bus_rdata
);

    /* ProMax固定能力：8路DDC、4组32抽头匹配滤波，不包含FFT和UART。 */
    localparam integer DDC_CHANNELS = 8;
    localparam integer MF_BANKS = 4;
    localparam integer MF_TAPS = 32;
    localparam integer DDC_WIDTH = 18;
    localparam integer MF_WIDTH = 24;
    localparam integer POWER_WIDTH = 37;

    /* 紧凑寄存器地址，不占用原AD9708和AD9238地址。 */
    localparam [6:0] REG_ID             = 7'h30;
    localparam [6:0] REG_VERSION        = 7'h31;
    localparam [6:0] REG_CAPABILITY     = 7'h32;
    localparam [6:0] REG_CONTROL        = 7'h33;
    localparam [6:0] REG_STATUS         = 7'h34;
    localparam [6:0] REG_SNAPSHOT       = 7'h35;
    localparam [6:0] REG_SAMPLE_RATE    = 7'h36;
    localparam [6:0] REG_FIFO_STATUS    = 7'h37;
    localparam [6:0] REG_PHASE_INC0     = 7'h38;
    localparam [6:0] REG_PHASE_OFFSET0  = 7'h40;
    localparam [6:0] REG_MF_SELECTOR    = 7'h48;
    localparam [6:0] REG_MF_COEFF       = 7'h49;
    localparam [6:0] REG_POWER_LO0      = 7'h50;
    localparam [6:0] REG_POWER_HI0      = 7'h58;
    localparam [6:0] REG_MF_SCORE0      = 7'h60;
    localparam [6:0] REG_MF_INDEX0      = 7'h64;
    localparam [6:0] REG_WAVE_CONTROL   = 7'h68;
    localparam [6:0] REG_WAVE_STATUS    = 7'h69;
    localparam [6:0] REG_WAVE_MIN_MAX   = 7'h6A;
    localparam [6:0] REG_WAVE_SUM_LO    = 7'h6B;
    localparam [6:0] REG_WAVE_SUM_HI    = 7'h6C;
    localparam [6:0] REG_WAVE_SQ_LO     = 7'h6D;
    localparam [6:0] REG_WAVE_SQ_HI     = 7'h6E;
    localparam [6:0] REG_WAVE_HIGH      = 7'h6F;
    localparam [6:0] REG_WAVE_RISING    = 7'h70;
    localparam [6:0] REG_WAVE_PERIOD    = 7'h71;
    localparam [6:0] REG_WAVE_SAMPLES   = 7'h72;

    localparam [31:0] DEVICE_ID = 32'h4258504D; /* ASCII "BXPM"。 */
    localparam [31:0] VERSION = 32'h00010100; /* ProMax协议版本1.1。 */
    localparam [31:0] CAPABILITY = 32'h29200408; /* 增加连续波形统计。 */
    localparam [31:0] SAMPLE_RATE_HZ = 32'd50000000; /* ADC有效采样率。 */

    /* 6字节总线直接维护DSP配置和结果快照。 */
    reg run_enable;
    reg soft_clear;
    reg snapshot_hold;
    reg power_snapshot_valid;
    reg score_snapshot_valid;
    reg [7:0] power_generation;
    reg [7:0] score_generation;
    reg [4:0] wave_block_log2;
    reg wave_clear;
    reg [DDC_CHANNELS*32-1:0] phase_inc_flat;
    reg [DDC_CHANNELS*32-1:0] phase_offset_flat;
    reg [9:0] mf_selector;
    reg mf_cfg_valid;
    reg [1:0] mf_cfg_bank;
    reg [4:0] mf_cfg_index;
    reg signed [17:0] mf_cfg_data;
    reg [POWER_WIDTH-1:0] power_snapshot [0:DDC_CHANNELS-1];
    reg signed [MF_WIDTH-1:0] score_snapshot [0:MF_BANKS-1];
    reg [31:0] peak_index_snapshot [0:MF_BANKS-1];

    /* ADC偏移二进制翻转符号位并左对齐，充分使用18位DSP动态范围。 */
    wire [17:0] fifo_write_data = {~adc_data[11], adc_data[10:0], 6'd0};
    wire [17:0] fifo_read_data;
    wire fifo_full;
    wire fifo_empty;
    wire fifo_overflow;
    wire fifo_write_busy;
    wire fifo_read_busy;
    wire fifo_data_valid;
    wire [5:0] fifo_read_count;
    wire fifo_read_enable = !fifo_empty && !fifo_read_busy;
    wire dsp_sample_valid = fifo_data_valid && run_enable;

    /* 50 MHz写、50 MHz读的官方异步FIFO，避免ADC样本裸跨时钟域。 */
    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("distributed"),
        .ECC_MODE("no_ecc"),
        .RELATED_CLOCKS(0),
        .FIFO_WRITE_DEPTH(32),
        .WRITE_DATA_WIDTH(18),
        .WR_DATA_COUNT_WIDTH(6),
        .PROG_FULL_THRESH(24),
        .FULL_RESET_VALUE(0),
        .USE_ADV_FEATURES("1707"),
        .READ_MODE("std"),
        .FIFO_READ_LATENCY(1),
        .READ_DATA_WIDTH(18),
        .RD_DATA_COUNT_WIDTH(6),
        .PROG_EMPTY_THRESH(4),
        .DOUT_RESET_VALUE("0"),
        .CDC_SYNC_STAGES(2),
        .WAKEUP_TIME(0)
    ) adc_async_fifo (
        .sleep(1'b0),
        .rst(~reset_n),
        .wr_clk(adc_clk),
        .wr_en(adc_ready && !fifo_write_busy),
        .din(fifo_write_data),
        .full(fifo_full),
        .prog_full(),
        .wr_data_count(),
        .overflow(fifo_overflow),
        .wr_rst_busy(fifo_write_busy),
        .almost_full(),
        .wr_ack(),
        .rd_clk(cfg_clk),
        .rd_en(fifo_read_enable),
        .dout(fifo_read_data),
        .empty(fifo_empty),
        .prog_empty(),
        .rd_data_count(fifo_read_count),
        .underflow(),
        .rd_rst_busy(fifo_read_busy),
        .almost_empty(),
        .data_valid(fifo_data_valid),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr(),
        .dbiterr()
    );

    /* FIFO写域异常通过粘滞位和两级同步返回寄存器域。 */
    reg fifo_clear_toggle_cfg;
    (* ASYNC_REG = "TRUE" *) reg [1:0] fifo_clear_toggle_adc;
    reg fifo_clear_toggle_seen;
    reg fifo_overflow_sticky_adc;
    reg fifo_write_busy_adc;
    (* ASYNC_REG = "TRUE" *) reg [1:0] fifo_overflow_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] fifo_full_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] fifo_write_busy_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] adc_ready_sync;

    always @(posedge adc_clk or negedge reset_n) begin
        if (!reset_n) begin
            fifo_clear_toggle_adc <= 2'b00;
            fifo_clear_toggle_seen <= 1'b0;
            fifo_overflow_sticky_adc <= 1'b0;
            fifo_write_busy_adc <= 1'b1;
        end else begin
            /* 先在写时钟域寄存XPM忙状态，避免组合逻辑直接进入跨域同步器。 */
            fifo_write_busy_adc <= fifo_write_busy;
            fifo_clear_toggle_adc <= {fifo_clear_toggle_adc[0],
                                      fifo_clear_toggle_cfg};
            if (fifo_clear_toggle_adc[1] != fifo_clear_toggle_seen) begin
                fifo_clear_toggle_seen <= fifo_clear_toggle_adc[1];
                fifo_overflow_sticky_adc <= 1'b0;
            end else if (fifo_overflow) begin
                fifo_overflow_sticky_adc <= 1'b1;
            end
        end
    end

    always @(posedge cfg_clk or negedge reset_n) begin
        if (!reset_n) begin
            fifo_overflow_sync <= 2'b00;
            fifo_full_sync <= 2'b00;
            fifo_write_busy_sync <= 2'b11;
            adc_ready_sync <= 2'b00;
        end else begin
            fifo_overflow_sync <= {fifo_overflow_sync[0],
                                   fifo_overflow_sticky_adc};
            fifo_full_sync <= {fifo_full_sync[0], fifo_full};
            fifo_write_busy_sync <= {fifo_write_busy_sync[0],
                                     fifo_write_busy_adc};
            adc_ready_sync <= {adc_ready_sync[0], adc_ready};
        end
    end

    /* ProMax实时数据面信号。 */
    wire ddc_valid;
    wire [DDC_CHANNELS*DDC_WIDTH-1:0] ddc_i_flat;
    wire [DDC_CHANNELS*DDC_WIDTH-1:0] ddc_q_flat;
    wire ddc_filtered_valid;
    wire [DDC_CHANNELS*DDC_WIDTH-1:0] ddc_filtered_i_flat;
    wire [DDC_CHANNELS*DDC_WIDTH-1:0] ddc_filtered_q_flat;
    wire mf_valid;
    wire [MF_BANKS*MF_WIDTH-1:0] mf_score_flat;
    wire mf_peak_valid;
    wire [MF_BANKS*MF_WIDTH-1:0] mf_peak_abs_flat;
    wire [MF_BANKS*MF_WIDTH-1:0] mf_peak_score_flat;
    wire [MF_BANKS*32-1:0] mf_peak_index_flat;
    wire band_power_valid;
    wire [DDC_CHANNELS*POWER_WIDTH-1:0] band_power_flat;

    wire wave_result_valid;
    wire wave_threshold_settled;
    wire [7:0] wave_generation;
    wire signed [7:0] wave_min;
    wire signed [7:0] wave_max;
    wire signed [7:0] wave_threshold;
    wire signed [39:0] wave_sum;
    wire [47:0] wave_sum_square;
    wire [31:0] wave_high_count;
    wire [31:0] wave_rising_count;
    wire [31:0] wave_period_sum;
    wire [31:0] wave_sample_count;
    wire signed [7:0] dsp_sample_code =
        $signed(fifo_read_data[17:10]);

    dsp_promax_benchmark #(
        .DDC_CHANNELS(DDC_CHANNELS),
        .MF_BANKS(MF_BANKS),
        .MF_TAPS(MF_TAPS),
        .POWER_BLOCK_LOG2(POWER_BLOCK_LOG2)
    ) realtime_dsp (
        .clk(cfg_clk),
        .reset_n(reset_n),
        .state_clear(soft_clear),
        .s_valid(dsp_sample_valid),
        .s_data($signed(fifo_read_data)),
        .ddc_phase_inc_flat(phase_inc_flat),
        .ddc_phase_offset_flat(phase_offset_flat),
        .mf_cfg_valid(mf_cfg_valid),
        .mf_cfg_bank(mf_cfg_bank),
        .mf_cfg_index(mf_cfg_index),
        .mf_cfg_data(mf_cfg_data),
        .ddc_valid(ddc_valid),
        .ddc_i_flat(ddc_i_flat),
        .ddc_q_flat(ddc_q_flat),
        .ddc_filtered_valid(ddc_filtered_valid),
        .ddc_filtered_i_flat(ddc_filtered_i_flat),
        .ddc_filtered_q_flat(ddc_filtered_q_flat),
        .mf_valid(mf_valid),
        .mf_score_flat(mf_score_flat),
        .mf_peak_valid(mf_peak_valid),
        .mf_peak_abs_flat(mf_peak_abs_flat),
        .mf_peak_score_flat(mf_peak_score_flat),
        .mf_peak_index_flat(mf_peak_index_flat),
        .band_power_valid(band_power_valid),
        .band_power_flat(band_power_flat)
    );

    dsp_stream_stats wave_stats (
        .clk(cfg_clk),
        .reset_n(reset_n),
        .clear(soft_clear | wave_clear),
        .hold(snapshot_hold),
        .s_valid(dsp_sample_valid),
        .s_data(dsp_sample_code),
        .block_log2(wave_block_log2),
        .result_valid(wave_result_valid),
        .threshold_settled(wave_threshold_settled),
        .generation(wave_generation),
        .result_min(wave_min),
        .result_max(wave_max),
        .result_threshold(wave_threshold),
        .result_sum(wave_sum),
        .result_sum_square(wave_sum_square),
        .result_high_count(wave_high_count),
        .result_rising_count(wave_rising_count),
        .result_period_sum(wave_period_sum),
        .result_sample_count(wave_sample_count)
    );

    assign bus_select = (bus_addr >= REG_ID) &&
                        (bus_addr <= REG_WAVE_SAMPLES);

    /* DSP寄存器读回，保留地址始终返回0。 */
    integer read_index;
    always @(*) begin
        bus_rdata = 32'd0;
        read_index = 0;
        case (bus_addr)
            REG_ID: bus_rdata = DEVICE_ID;
            REG_VERSION: bus_rdata = VERSION;
            REG_CAPABILITY: bus_rdata = CAPABILITY;
            REG_CONTROL: bus_rdata = {31'd0, run_enable};
            REG_STATUS: bus_rdata = {24'd0, wave_result_valid,
                                     adc_ready_sync[1], fifo_empty,
                                     fifo_full_sync[1], fifo_overflow_sync[1],
                                     score_snapshot_valid,
                                     power_snapshot_valid, run_enable};
            REG_SNAPSHOT: bus_rdata = {8'd0, score_generation,
                                       power_generation, 7'd0, snapshot_hold};
            REG_SAMPLE_RATE: bus_rdata = SAMPLE_RATE_HZ;
            REG_FIFO_STATUS: bus_rdata = {16'd0, 2'd0, fifo_read_count,
                                          3'd0, fifo_read_busy,
                                          fifo_write_busy_sync[1],
                                          fifo_overflow_sync[1],
                                          fifo_full_sync[1], fifo_empty};
            REG_MF_SELECTOR: bus_rdata = {22'd0, mf_selector};
            REG_MF_COEFF: bus_rdata = {{14{mf_cfg_data[17]}}, mf_cfg_data};
            REG_WAVE_CONTROL: bus_rdata = {27'd0, wave_block_log2};
            REG_WAVE_STATUS: bus_rdata = {11'd0, wave_block_log2,
                                           wave_generation, 6'd0,
                                           wave_threshold_settled,
                                           wave_result_valid};
            REG_WAVE_MIN_MAX: bus_rdata = {8'd0, wave_threshold,
                                            wave_max, wave_min};
            REG_WAVE_SUM_LO: bus_rdata = wave_sum[31:0];
            REG_WAVE_SUM_HI: bus_rdata = {{24{wave_sum[39]}},
                                           wave_sum[39:32]};
            REG_WAVE_SQ_LO: bus_rdata = wave_sum_square[31:0];
            REG_WAVE_SQ_HI: bus_rdata = {16'd0,
                                          wave_sum_square[47:32]};
            REG_WAVE_HIGH: bus_rdata = wave_high_count;
            REG_WAVE_RISING: bus_rdata = wave_rising_count;
            REG_WAVE_PERIOD: bus_rdata = wave_period_sum;
            REG_WAVE_SAMPLES: bus_rdata = wave_sample_count;
            default: begin
                if ((bus_addr >= REG_PHASE_INC0) &&
                    (bus_addr < REG_PHASE_INC0 + DDC_CHANNELS)) begin
                    bus_rdata = phase_inc_flat[
                        (bus_addr-REG_PHASE_INC0)*32 +: 32];
                end else if ((bus_addr >= REG_PHASE_OFFSET0) &&
                             (bus_addr < REG_PHASE_OFFSET0 + DDC_CHANNELS)) begin
                    bus_rdata = phase_offset_flat[
                        (bus_addr-REG_PHASE_OFFSET0)*32 +: 32];
                end else if ((bus_addr >= REG_POWER_LO0) &&
                             (bus_addr < REG_POWER_LO0 + DDC_CHANNELS)) begin
                    read_index = bus_addr - REG_POWER_LO0;
                    bus_rdata = power_snapshot[read_index][31:0];
                end else if ((bus_addr >= REG_POWER_HI0) &&
                             (bus_addr < REG_POWER_HI0 + DDC_CHANNELS)) begin
                    read_index = bus_addr - REG_POWER_HI0;
                    bus_rdata = {{27{1'b0}}, power_snapshot[read_index][36:32]};
                end else if ((bus_addr >= REG_MF_SCORE0) &&
                             (bus_addr < REG_MF_SCORE0 + MF_BANKS)) begin
                    read_index = bus_addr - REG_MF_SCORE0;
                    bus_rdata = {{8{score_snapshot[read_index][23]}},
                                 score_snapshot[read_index]};
                end else if ((bus_addr >= REG_MF_INDEX0) &&
                             (bus_addr < REG_MF_INDEX0 + MF_BANKS)) begin
                    read_index = bus_addr - REG_MF_INDEX0;
                    bus_rdata = peak_index_snapshot[read_index];
                end
            end
        endcase
    end

    /* 配置写入及结果快照均位于50 MHz域，无需额外多位CDC。 */
    integer channel_index;
    integer bank_index;
    always @(posedge cfg_clk or negedge reset_n) begin
        if (!reset_n) begin
            run_enable <= 1'b0;
            soft_clear <= 1'b0;
            snapshot_hold <= 1'b0;
            power_snapshot_valid <= 1'b0;
            score_snapshot_valid <= 1'b0;
            power_generation <= 8'd0;
            score_generation <= 8'd0;
            wave_block_log2 <= 5'd12;
            wave_clear <= 1'b0;
            phase_inc_flat <= {(DDC_CHANNELS*32){1'b0}};
            phase_offset_flat <= {(DDC_CHANNELS*32){1'b0}};
            mf_selector <= 10'd0;
            mf_cfg_valid <= 1'b0;
            mf_cfg_bank <= 2'd0;
            mf_cfg_index <= 5'd0;
            mf_cfg_data <= 18'sd0;
            fifo_clear_toggle_cfg <= 1'b0;
            for (channel_index = 0; channel_index < DDC_CHANNELS;
                 channel_index = channel_index + 1)
                power_snapshot[channel_index] <= {POWER_WIDTH{1'b0}};
            for (bank_index = 0; bank_index < MF_BANKS;
                 bank_index = bank_index + 1) begin
                score_snapshot[bank_index] <= {MF_WIDTH{1'b0}};
                peak_index_snapshot[bank_index] <= 32'd0;
            end
        end else begin
            soft_clear <= 1'b0;
            mf_cfg_valid <= 1'b0;
            wave_clear <= 1'b0;

            if (band_power_valid && !snapshot_hold) begin
                for (channel_index = 0; channel_index < DDC_CHANNELS;
                     channel_index = channel_index + 1)
                    power_snapshot[channel_index] <= band_power_flat[
                        channel_index*POWER_WIDTH +: POWER_WIDTH];
                power_snapshot_valid <= 1'b1;
                power_generation <= power_generation + 1'b1;
            end

            if (mf_peak_valid && !snapshot_hold) begin
                for (bank_index = 0; bank_index < MF_BANKS;
                     bank_index = bank_index + 1) begin
                    score_snapshot[bank_index] <= $signed(mf_peak_score_flat[
                        bank_index*MF_WIDTH +: MF_WIDTH]);
                    peak_index_snapshot[bank_index] <= mf_peak_index_flat[
                        bank_index*32 +: 32];
                end
                score_snapshot_valid <= 1'b1;
                score_generation <= score_generation + 1'b1;
            end

            if (bus_write && bus_select) begin
                if ((bus_addr >= REG_PHASE_INC0) &&
                    (bus_addr < REG_PHASE_INC0 + DDC_CHANNELS)) begin
                    phase_inc_flat[(bus_addr-REG_PHASE_INC0)*32 +: 32]
                        <= bus_wdata;
                end else if ((bus_addr >= REG_PHASE_OFFSET0) &&
                             (bus_addr < REG_PHASE_OFFSET0 + DDC_CHANNELS)) begin
                    phase_offset_flat[(bus_addr-REG_PHASE_OFFSET0)*32 +: 32]
                        <= bus_wdata;
                end else begin
                    case (bus_addr)
                        REG_CONTROL: begin
                            run_enable <= bus_wdata[0];
                            if (bus_wdata[1]) begin
                                soft_clear <= 1'b1;
                                snapshot_hold <= 1'b0;
                                power_snapshot_valid <= 1'b0;
                                score_snapshot_valid <= 1'b0;
                                power_generation <= 8'd0;
                                score_generation <= 8'd0;
                                fifo_clear_toggle_cfg <= ~fifo_clear_toggle_cfg;
                                for (channel_index = 0;
                                     channel_index < DDC_CHANNELS;
                                     channel_index = channel_index + 1)
                                    power_snapshot[channel_index]
                                        <= {POWER_WIDTH{1'b0}};
                                for (bank_index = 0; bank_index < MF_BANKS;
                                     bank_index = bank_index + 1) begin
                                    score_snapshot[bank_index]
                                        <= {MF_WIDTH{1'b0}};
                                    peak_index_snapshot[bank_index] <= 32'd0;
                                end
                            end
                        end
                        REG_SNAPSHOT: snapshot_hold <= bus_wdata[0];
                        REG_MF_SELECTOR: begin
                            mf_selector <= {bus_wdata[9:8], 3'd0,
                                            bus_wdata[4:0]};
                        end
                        REG_MF_COEFF: begin
                            mf_cfg_bank <= mf_selector[9:8];
                            mf_cfg_index <= mf_selector[4:0];
                            mf_cfg_data <= bus_wdata[17:0];
                            mf_cfg_valid <= 1'b1;
                        end
                        REG_WAVE_CONTROL: begin
                            if (bus_wdata[4:0] < 5'd6)
                                wave_block_log2 <= 5'd6;
                            else if (bus_wdata[4:0] > 5'd25)
                                wave_block_log2 <= 5'd25;
                            else
                                wave_block_log2 <= bus_wdata[4:0];
                            if (bus_wdata[8])
                                wave_clear <= 1'b1;
                        end
                        default: begin
                        end
                    endcase
                end
            end
        end
    end

endmodule
