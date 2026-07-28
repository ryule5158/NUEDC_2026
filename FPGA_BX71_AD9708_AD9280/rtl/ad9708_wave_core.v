`timescale 1ns / 1ps

/* AD9708波形核心：完成DDS、任意波、幅度偏置处理和硬件扫频。 */
module ad9708_wave_core #(
    parameter RAM_ADDR_WIDTH = 10,       /* 波形RAM地址位宽。 */
    parameter DAC_MID_CODE = 8'd128      /* DAC零偏中心码。 */
)(
    input  wire                       cfg_clk,
    input  wire                       reset_n,
    input  wire                       dac_clk,

    input  wire                       cfg_commit_toggle,
    input  wire                       enable_cfg,
    input  wire [2:0]                 mode_cfg,
    input  wire [31:0]                freq_word_cfg,
    input  wire [31:0]                phase_offset_cfg,
    input  wire [7:0]                 constant_code_cfg,
    input  wire [10:0]                ram_points_cfg,
    input  wire [7:0]                 amplitude_cfg,
    input  wire [7:0]                 amplitude_fraction_cfg,
    input  wire [7:0]                 offset_cfg,
    input  wire [7:0]                 offset_fraction_cfg,
    input  wire                       phase_reset_toggle,
    input  wire [1:0]                 mod_mode_cfg,
    input  wire [31:0]                mod_freq_word_cfg,
    input  wire [15:0]                am_carrier_gain_cfg,
    input  wire [15:0]                am_sideband_gain_cfg,
    input  wire [31:0]                fm_deviation_word_cfg,
    input  wire [31:0]                mod_phase_offset_cfg,

    input  wire                       sweep_enable_cfg,
    input  wire [1:0]                 sweep_mode_cfg,
    input  wire                       sweep_hold_cfg,
    input  wire                       sweep_direction_cfg,
    input  wire [31:0]                sweep_low_cfg,
    input  wire [31:0]                sweep_high_cfg,
    input  wire [31:0]                sweep_step_cfg,
    input  wire [31:0]                sweep_dwell_cfg,
    input  wire                       sweep_start_toggle,

    input  wire                       ram_wr_en,
    input  wire [RAM_ADDR_WIDTH-1:0]  ram_wr_addr,
    input  wire [7:0]                 ram_wr_data,

    (* IOB = "TRUE" *)
    output reg  [7:0]                 dac_data,
    output reg  [31:0]                phase_debug,
    output reg  [31:0]                current_freq_word,
    output reg                        sweep_running,
    output reg                        sweep_done,
    output reg                        sweep_direction_up
);

    /* 波形模式编码。 */
    localparam MODE_CONST  = 3'd0; /* 恒定码。 */
    localparam MODE_RAM    = 3'd1; /* 任意波RAM。 */
    localparam MODE_SAW    = 3'd2; /* 锯齿波。 */
    localparam MODE_TRI    = 3'd3; /* 三角波。 */
    localparam MODE_SQUARE = 3'd4; /* 方波。 */
    localparam MOD_OFF     = 2'd0;
    localparam MOD_AM      = 2'd1;
    localparam MOD_FM      = 2'd2;

    /* 线性扫频模式编码。 */
    localparam SWEEP_BIDIR  = 2'd0; /* 往返扫频。 */
    localparam SWEEP_LOOP   = 2'd1; /* 单向循环扫频。 */
    localparam SWEEP_ONCE   = 2'd2; /* 单次扫频。 */
    localparam SWEEP_MANUAL = 2'd3; /* 手动方向扫频。 */

    localparam RAM_DEPTH = (1 << RAM_ADDR_WIDTH); /* 波形RAM深度。 */

    /* 1/4周期Q1.15正弦表；象限折叠后只需65个常量。 */
    function [15:0] sine_quarter_q15;
        input [6:0] index;
        begin
            case (index)
                7'd0: sine_quarter_q15 = 16'd0;
                7'd1: sine_quarter_q15 = 16'd804;
                7'd2: sine_quarter_q15 = 16'd1608;
                7'd3: sine_quarter_q15 = 16'd2410;
                7'd4: sine_quarter_q15 = 16'd3212;
                7'd5: sine_quarter_q15 = 16'd4011;
                7'd6: sine_quarter_q15 = 16'd4808;
                7'd7: sine_quarter_q15 = 16'd5602;
                7'd8: sine_quarter_q15 = 16'd6393;
                7'd9: sine_quarter_q15 = 16'd7179;
                7'd10: sine_quarter_q15 = 16'd7962;
                7'd11: sine_quarter_q15 = 16'd8739;
                7'd12: sine_quarter_q15 = 16'd9512;
                7'd13: sine_quarter_q15 = 16'd10278;
                7'd14: sine_quarter_q15 = 16'd11039;
                7'd15: sine_quarter_q15 = 16'd11793;
                7'd16: sine_quarter_q15 = 16'd12539;
                7'd17: sine_quarter_q15 = 16'd13279;
                7'd18: sine_quarter_q15 = 16'd14010;
                7'd19: sine_quarter_q15 = 16'd14732;
                7'd20: sine_quarter_q15 = 16'd15446;
                7'd21: sine_quarter_q15 = 16'd16151;
                7'd22: sine_quarter_q15 = 16'd16846;
                7'd23: sine_quarter_q15 = 16'd17530;
                7'd24: sine_quarter_q15 = 16'd18204;
                7'd25: sine_quarter_q15 = 16'd18868;
                7'd26: sine_quarter_q15 = 16'd19519;
                7'd27: sine_quarter_q15 = 16'd20159;
                7'd28: sine_quarter_q15 = 16'd20787;
                7'd29: sine_quarter_q15 = 16'd21403;
                7'd30: sine_quarter_q15 = 16'd22005;
                7'd31: sine_quarter_q15 = 16'd22594;
                7'd32: sine_quarter_q15 = 16'd23170;
                7'd33: sine_quarter_q15 = 16'd23731;
                7'd34: sine_quarter_q15 = 16'd24279;
                7'd35: sine_quarter_q15 = 16'd24811;
                7'd36: sine_quarter_q15 = 16'd25329;
                7'd37: sine_quarter_q15 = 16'd25832;
                7'd38: sine_quarter_q15 = 16'd26319;
                7'd39: sine_quarter_q15 = 16'd26790;
                7'd40: sine_quarter_q15 = 16'd27245;
                7'd41: sine_quarter_q15 = 16'd27683;
                7'd42: sine_quarter_q15 = 16'd28105;
                7'd43: sine_quarter_q15 = 16'd28510;
                7'd44: sine_quarter_q15 = 16'd28898;
                7'd45: sine_quarter_q15 = 16'd29268;
                7'd46: sine_quarter_q15 = 16'd29621;
                7'd47: sine_quarter_q15 = 16'd29956;
                7'd48: sine_quarter_q15 = 16'd30273;
                7'd49: sine_quarter_q15 = 16'd30571;
                7'd50: sine_quarter_q15 = 16'd30852;
                7'd51: sine_quarter_q15 = 16'd31113;
                7'd52: sine_quarter_q15 = 16'd31356;
                7'd53: sine_quarter_q15 = 16'd31580;
                7'd54: sine_quarter_q15 = 16'd31785;
                7'd55: sine_quarter_q15 = 16'd31971;
                7'd56: sine_quarter_q15 = 16'd32137;
                7'd57: sine_quarter_q15 = 16'd32285;
                7'd58: sine_quarter_q15 = 16'd32412;
                7'd59: sine_quarter_q15 = 16'd32521;
                7'd60: sine_quarter_q15 = 16'd32609;
                7'd61: sine_quarter_q15 = 16'd32678;
                7'd62: sine_quarter_q15 = 16'd32728;
                7'd63: sine_quarter_q15 = 16'd32757;
                default: sine_quarter_q15 = 16'd32767;
            endcase
        end
    endfunction

    /* STM32写、DAC时钟读的双时钟波形RAM。 */
    (* ram_style = "block" *) reg [7:0] wave_ram [0:RAM_DEPTH-1];
    reg [RAM_ADDR_WIDTH-1:0] ram_rd_addr;
    reg [7:0] ram_sample;

    /* 经提交翻转同步后一次性更新的DAC域配置。 */
    reg enable_dac;
    reg [2:0] mode_dac;
    reg [31:0] freq_word_dac;
    reg [31:0] phase_offset_dac;
    reg [7:0] constant_code_dac;
    reg [10:0] ram_points_dac;
    reg [7:0] amplitude_dac;
    reg [7:0] amplitude_fraction_dac;
    reg [7:0] offset_dac;
    reg [7:0] offset_fraction_dac;
    reg [1:0] mod_mode_dac;
    reg [31:0] mod_freq_word_dac;
    reg [15:0] am_carrier_gain_dac;
    reg [15:0] am_sideband_gain_dac;
    reg [31:0] fm_deviation_word_dac;
    reg [31:0] mod_phase_offset_dac;
    reg sweep_enable_dac;
    reg [1:0] sweep_mode_dac;
    reg sweep_hold_dac;
    reg sweep_direction_dac;
    reg [31:0] sweep_low_dac;
    reg [31:0] sweep_high_dac;
    reg [31:0] sweep_step_dac;
    reg [31:0] sweep_dwell_dac;

    /* 三个异步控制翻转量的同步寄存器。 */
    (* ASYNC_REG = "TRUE" *) reg [2:0] cfg_commit_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] phase_reset_sync;
    (* ASYNC_REG = "TRUE" *) reg [3:0] sweep_start_sync;

    /* DDS相位、扫频驻留和当前原始采样。 */
    reg [31:0] phase_acc;
    reg [31:0] mod_phase_acc;
    reg [31:0] sweep_dwell_count;
    reg [7:0] selected_sample;

    /*
     * AM采用两级乘法流水线：
     * factor = carrier_gain + sideband_gain*sin(mod_phase)；
     * output = 128 + (carrier-128)*factor。
     * 两个增益之和不超过32767，因此100%调制度也不会数字削顶。
     */
    reg signed [8:0] am_carrier_stage1;
    (* use_dsp = "yes" *) reg signed [33:0] am_side_product_stage1;
    reg signed [8:0] am_carrier_stage2;
    reg signed [17:0] am_factor_stage2;
    (* use_dsp = "yes" *) reg signed [27:0] am_product_stage3;
    reg [7:0] am_sample_stage4;

    /* RAM寻址与Q8.8幅度换算流水寄存器。 */
    reg [42:0] ram_phase_product_reg;
    reg [7:0] raw_sample_reg;
    (* use_dsp = "yes" *) reg [24:0] scaled_product_reg;
    reg signed [16:0] scaled_low_q_reg;
    reg [16:0] divided_product_q_reg;
    reg signed [16:0] divided_low_q_reg;
    reg [7:0] scaled_sample_reg;
    reg [7:0] division_error_reg;
    reg [7:0] code_quantization_error_reg;

    /* 同步翻转量转换出的单周期控制事件。 */
    wire cfg_commit_req = cfg_commit_sync[2] ^ cfg_commit_sync[1];
    wire phase_reset_req = phase_reset_sync[2] ^ phase_reset_sync[1];
    wire sweep_start_req = sweep_start_sync[3] ^ sweep_start_sync[2];

    /* 由DDS高位直接生成的基础波形采样。 */
    wire [7:0] saw_sample = phase_acc[31:24];
    wire [7:0] tri_sample = phase_acc[31] ?
                            ~phase_acc[30:23] : phase_acc[30:23];
    wire [7:0] square_sample = phase_acc[31] ? 8'hFF : 8'h00;

    wire [31:0] mod_phase_lookup =
        mod_phase_acc + mod_phase_offset_dac;
    wire [1:0] mod_quadrant = mod_phase_lookup[31:30];
    wire [6:0] mod_quarter_index =
        ((mod_quadrant == 2'd1) || (mod_quadrant == 2'd3)) ?
        (7'd64 - {1'b0, mod_phase_lookup[29:24]}) :
        {1'b0, mod_phase_lookup[29:24]};
    wire [15:0] mod_sine_magnitude =
        sine_quarter_q15(mod_quarter_index);
    wire signed [16:0] mod_sine_q15 =
        mod_quadrant[1] ? -$signed({1'b0, mod_sine_magnitude}) :
                          $signed({1'b0, mod_sine_magnitude});
    wire signed [34:0] am_side_scaled =
        am_side_product_stage1 >>> 15;
    wire signed [35:0] am_factor_sum =
        $signed({20'd0, am_carrier_gain_dac}) + am_side_scaled;
    wire signed [28:0] am_scaled_center = am_product_stage3 >>> 15;
    wire signed [29:0] am_code_unclamped =
        am_scaled_center + 30'sd128;

    /* FM直接调制每个125 MHz DAC周期的相位增量。 */
    wire signed [49:0] fm_product =
        $signed({1'b0, fm_deviation_word_dac}) * mod_sine_q15;
    wire signed [34:0] fm_delta_word = fm_product >>> 15;
    wire signed [34:0] fm_sum_word =
        $signed({3'b000, current_freq_word}) + fm_delta_word;
    wire [31:0] phase_step_word =
        (mod_mode_dac != MOD_FM) ? current_freq_word :
        (fm_sum_word < 0) ? 32'd0 :
        (fm_sum_word > 35'h0FFFFFFFF) ? 32'hFFFFFFFF :
        fm_sum_word[31:0];

    /* Q8.8幅度A对应端点[offset-A, offset+A-1]。 */
    wire [15:0] amplitude_q = {amplitude_dac, amplitude_fraction_dac};
    wire [15:0] offset_q = {offset_dac, offset_fraction_dac};
    wire [16:0] amplitude_span_q = (amplitude_q == 16'd0) ? 17'd0 :
                                   ({1'b0, amplitude_q} << 1) - 17'd256;
    wire signed [16:0] scaled_low_q =
        $signed({1'b0, offset_q}) - $signed({1'b0, amplitude_q});

    /*
     * 将raw*span_q/255的余数反馈到下一采样，再对Q8.8结果误差反馈量化。
     * 24位数除以255可折叠三个字节，避免组合除法器。
     */
    wire [24:0] division_value =
        scaled_product_reg + {17'd0, division_error_reg};
    wire [7:0] division_byte2 = division_value[23:16];
    wire [7:0] division_byte1 = division_value[15:8];
    wire [7:0] division_byte0 = division_value[7:0];
    wire [9:0] division_fold = {2'd0, division_byte2} +
                               {2'd0, division_byte1} +
                               {2'd0, division_byte0};
    wire [1:0] division_fold_quotient =
        (division_fold >= 10'd510) ? 2'd2 :
        (division_fold >= 10'd255) ? 2'd1 : 2'd0;
    wire [9:0] division_fold_remainder =
        division_fold - ((division_fold_quotient == 2'd2) ? 10'd510 :
                         (division_fold_quotient == 2'd1) ? 10'd255 :
                         10'd0);
    wire [16:0] division_quotient =
        {1'b0, division_byte2, 8'd0} +
        {9'd0, division_byte2} +
        {9'd0, division_byte1} +
        {15'd0, division_fold_quotient};
    wire [7:0] division_remainder = division_fold_remainder[7:0];
    wire signed [17:0] interpolated_q =
        $signed({divided_low_q_reg[16], divided_low_q_reg}) +
        $signed({1'b0, divided_product_q_reg});
    wire signed [18:0] quantization_value =
        $signed({interpolated_q[17], interpolated_q}) +
        $signed({11'd0, code_quantization_error_reg});
    wire [7:0] limited_sample = (quantization_value < 19'sd0) ? 8'd0 :
                                (quantization_value > 19'sd65535) ? 8'd255 :
                                quantization_value[15:8];

    /* 仿真和综合共用的波形RAM初始化索引。 */
    integer init_i;
    /* 上电后将任意波RAM初始化为中点码。 */
    initial begin
        for (init_i = 0; init_i < RAM_DEPTH; init_i = init_i + 1) begin
            wave_ram[init_i] = DAC_MID_CODE;
        end
    end

    /* 根据当前模式选择满幅原始采样，随后统一执行幅度和偏置处理。 */
    always @(*) begin
        case (mode_dac)
            MODE_RAM:    selected_sample = ram_sample;
            MODE_SAW:    selected_sample = saw_sample;
            MODE_TRI:    selected_sample = tri_sample;
            MODE_SQUARE: selected_sample = square_sample;
            default:     selected_sample = constant_code_dac;
        endcase
    end

    /* 波形RAM写端口，写入期间应用层会先停止DAC输出。 */
    always @(posedge cfg_clk) begin
        if (ram_wr_en) begin
            wave_ram[ram_wr_addr] <= ram_wr_data;
        end
    end

    /* 同步控制翻转，并在提交事件到达时原子捕获全部多位配置。 */
    always @(posedge dac_clk or negedge reset_n) begin
        if (!reset_n) begin
            cfg_commit_sync <= 3'b000;
            phase_reset_sync <= 3'b000;
            sweep_start_sync <= 4'b0000;
            enable_dac <= 1'b0;
            mode_dac <= MODE_CONST;
            freq_word_dac <= 32'd0;
            phase_offset_dac <= 32'd0;
            constant_code_dac <= DAC_MID_CODE;
            ram_points_dac <= 11'd1024;
            amplitude_dac <= 8'd128;
            amplitude_fraction_dac <= 8'd0;
            offset_dac <= DAC_MID_CODE;
            offset_fraction_dac <= 8'd0;
            mod_mode_dac <= MOD_OFF;
            mod_freq_word_dac <= 32'd0;
            am_carrier_gain_dac <= 16'd32767;
            am_sideband_gain_dac <= 16'd0;
            fm_deviation_word_dac <= 32'd0;
            mod_phase_offset_dac <= 32'd0;
            sweep_enable_dac <= 1'b0;
            sweep_mode_dac <= SWEEP_BIDIR;
            sweep_hold_dac <= 1'b0;
            sweep_direction_dac <= 1'b1;
            sweep_low_dac <= 32'd0;
            sweep_high_dac <= 32'd0;
            sweep_step_dac <= 32'd0;
            sweep_dwell_dac <= 32'd1;
        end else begin
            cfg_commit_sync <= {cfg_commit_sync[1:0], cfg_commit_toggle};
            phase_reset_sync <= {phase_reset_sync[1:0], phase_reset_toggle};
            sweep_start_sync <= {sweep_start_sync[2:0], sweep_start_toggle};

            if (cfg_commit_req) begin
                enable_dac <= enable_cfg;
                mode_dac <= mode_cfg;
                freq_word_dac <= freq_word_cfg;
                phase_offset_dac <= phase_offset_cfg;
                constant_code_dac <= constant_code_cfg;
                ram_points_dac <= ((ram_points_cfg >= 11'd2) &&
                                   (ram_points_cfg <= 11'd1024)) ?
                                  ram_points_cfg : 11'd1024;
                if ((amplitude_cfg == 8'd0) &&
                    (amplitude_fraction_cfg < 8'd128)) begin
                    amplitude_dac <= 8'd0;
                    amplitude_fraction_dac <= 8'd0;
                end else if (amplitude_cfg < 8'd128) begin
                    amplitude_dac <= amplitude_cfg;
                    amplitude_fraction_dac <= amplitude_fraction_cfg;
                end else begin
                    amplitude_dac <= 8'd128;
                    amplitude_fraction_dac <= 8'd0;
                end
                offset_dac <= offset_cfg;
                offset_fraction_dac <= offset_fraction_cfg;
                mod_mode_dac <= (mod_mode_cfg <= MOD_FM) ?
                                mod_mode_cfg : MOD_OFF;
                mod_freq_word_dac <= mod_freq_word_cfg;
                am_carrier_gain_dac <= am_carrier_gain_cfg;
                am_sideband_gain_dac <= am_sideband_gain_cfg;
                fm_deviation_word_dac <= fm_deviation_word_cfg;
                mod_phase_offset_dac <= mod_phase_offset_cfg;
                sweep_enable_dac <= sweep_enable_cfg;
                sweep_mode_dac <= sweep_mode_cfg;
                sweep_hold_dac <= sweep_hold_cfg;
                sweep_direction_dac <= sweep_direction_cfg;
                sweep_low_dac <= sweep_low_cfg;
                sweep_high_dac <= sweep_high_cfg;
                sweep_step_dac <= sweep_step_cfg;
                sweep_dwell_dac <= (sweep_dwell_cfg == 32'd0) ?
                                   32'd1 : sweep_dwell_cfg;
            end
        end
    end

    /* DDS、任意点数RAM寻址和硬件线性扫频状态机。 */
    always @(posedge dac_clk or negedge reset_n) begin
        if (!reset_n) begin
            phase_acc <= 32'd0;
            mod_phase_acc <= 32'd0;
            phase_debug <= 32'd0;
            ram_phase_product_reg <= 43'd0;
            ram_rd_addr <= {RAM_ADDR_WIDTH{1'b0}};
            ram_sample <= DAC_MID_CODE;
            raw_sample_reg <= DAC_MID_CODE;
            am_carrier_stage1 <= 9'sd0;
            am_side_product_stage1 <= 34'sd0;
            am_carrier_stage2 <= 9'sd0;
            am_factor_stage2 <= 18'sd32767;
            am_product_stage3 <= 28'sd0;
            am_sample_stage4 <= DAC_MID_CODE;
            scaled_product_reg <= 25'd0;
            scaled_low_q_reg <= 17'sd0;
            divided_product_q_reg <= 17'd0;
            divided_low_q_reg <= 17'sd0;
            scaled_sample_reg <= DAC_MID_CODE;
            division_error_reg <= 8'd0;
            code_quantization_error_reg <= 8'd0;
            current_freq_word <= 32'd0;
            sweep_dwell_count <= 32'd0;
            sweep_running <= 1'b0;
            sweep_done <= 1'b0;
            sweep_direction_up <= 1'b1;
            dac_data <= DAC_MID_CODE;
        end else begin
            ram_phase_product_reg <= phase_acc * ram_points_dac;
            ram_rd_addr <= ram_phase_product_reg[41:32];
            ram_sample <= wave_ram[ram_rd_addr];

            am_carrier_stage1 <=
                $signed({1'b0, selected_sample}) - 9'sd128;
            am_side_product_stage1 <=
                $signed({1'b0, am_sideband_gain_dac}) * mod_sine_q15;
            am_carrier_stage2 <= am_carrier_stage1;
            if (am_factor_sum < 0) begin
                am_factor_stage2 <= 18'sd0;
            end else if (am_factor_sum > 36'sd32767) begin
                am_factor_stage2 <= 18'sd32767;
            end else begin
                am_factor_stage2 <= am_factor_sum[17:0];
            end
            am_product_stage3 <= am_carrier_stage2 * am_factor_stage2;
            if (am_code_unclamped < 0) begin
                am_sample_stage4 <= 8'd0;
            end else if (am_code_unclamped > 30'sd255) begin
                am_sample_stage4 <= 8'd255;
            end else begin
                am_sample_stage4 <= am_code_unclamped[7:0];
            end
            raw_sample_reg <= (mod_mode_dac == MOD_AM) ?
                              am_sample_stage4 : selected_sample;
            scaled_product_reg <= raw_sample_reg * amplitude_span_q;
            scaled_low_q_reg <= scaled_low_q;
            divided_product_q_reg <= division_quotient;
            divided_low_q_reg <= scaled_low_q_reg;
            scaled_sample_reg <= limited_sample;

            if (!enable_dac || (mode_dac == MODE_CONST) ||
                (amplitude_q == 16'd0) || cfg_commit_req ||
                phase_reset_req) begin
                division_error_reg <= 8'd0;
            end else begin
                division_error_reg <= division_remainder;
            end

            if (!enable_dac || (mode_dac == MODE_CONST) ||
                cfg_commit_req || phase_reset_req) begin
                code_quantization_error_reg <= 8'd0;
            end else begin
                code_quantization_error_reg <= quantization_value[7:0];
            end

            if (!sweep_enable_dac) begin
                current_freq_word <= freq_word_dac;
                sweep_dwell_count <= 32'd0;
                sweep_running <= 1'b0;
                sweep_done <= 1'b0;
                sweep_direction_up <= 1'b1;
            end else if (sweep_start_req) begin
                sweep_direction_up <= (sweep_mode_dac == SWEEP_MANUAL) ?
                                      sweep_direction_dac : 1'b1;
                current_freq_word <= ((sweep_mode_dac == SWEEP_MANUAL) &&
                                      !sweep_direction_dac) ?
                                     sweep_high_dac : sweep_low_dac;
                sweep_dwell_count <= 32'd0;
                sweep_running <= 1'b1;
                sweep_done <= 1'b0;
            end else if (sweep_running && enable_dac && !sweep_hold_dac) begin
                if ((sweep_mode_dac == SWEEP_MANUAL) &&
                    (sweep_direction_up != sweep_direction_dac)) begin
                    sweep_direction_up <= sweep_direction_dac;
                    sweep_dwell_count <= 32'd0;
                    sweep_done <= 1'b0;
                end else if (sweep_dwell_count >= (sweep_dwell_dac - 1'b1)) begin
                    sweep_dwell_count <= 32'd0;

                    if (sweep_direction_up) begin
                        if ((current_freq_word >= sweep_high_dac) ||
                            (sweep_step_dac >=
                             (sweep_high_dac - current_freq_word))) begin
                            current_freq_word <= sweep_high_dac;
                            case (sweep_mode_dac)
                                SWEEP_BIDIR: begin
                                    sweep_direction_up <= 1'b0;
                                end
                                SWEEP_LOOP: begin
                                    current_freq_word <=
                                        (current_freq_word >= sweep_high_dac) ?
                                        sweep_low_dac : sweep_high_dac;
                                end
                                SWEEP_ONCE: begin
                                    sweep_running <= 1'b0;
                                    sweep_done <= 1'b1;
                                end
                                default: begin
                                    sweep_done <= 1'b1;
                                end
                            endcase
                        end else begin
                            current_freq_word <= current_freq_word + sweep_step_dac;
                            sweep_done <= 1'b0;
                        end
                    end else begin
                        if ((current_freq_word <= sweep_low_dac) ||
                            (sweep_step_dac >=
                             (current_freq_word - sweep_low_dac))) begin
                            current_freq_word <= sweep_low_dac;
                            if (sweep_mode_dac == SWEEP_BIDIR) begin
                                sweep_direction_up <= 1'b1;
                            end else begin
                                sweep_done <= 1'b1;
                            end
                        end else begin
                            current_freq_word <= current_freq_word - sweep_step_dac;
                            sweep_done <= 1'b0;
                        end
                    end
                end else begin
                    sweep_dwell_count <= sweep_dwell_count + 1'b1;
                end
            end

            if (phase_reset_req) begin
                phase_acc <= phase_offset_dac;
                mod_phase_acc <= 32'd0;
            end else if (enable_dac) begin
                phase_acc <= phase_acc + phase_step_word;
                if (mod_mode_dac != MOD_OFF) begin
                    mod_phase_acc <= mod_phase_acc + mod_freq_word_dac;
                end
            end
            phase_debug <= phase_acc;

            if (!enable_dac) begin
                dac_data <= DAC_MID_CODE;
            end else if (mode_dac == MODE_CONST) begin
                dac_data <= constant_code_dac;
            end else begin
                dac_data <= scaled_sample_reg;
            end
        end
    end

endmodule
