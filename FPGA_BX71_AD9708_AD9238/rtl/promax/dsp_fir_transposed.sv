`timescale 1ns/1ps

// 转置型全并行FIR：每个抽头独立乘加，系数和累加器构成主要状态。
// Fully parallel transposed-form FIR.
// One multiplier is inferred per tap. There is no long combinational adder tree:
// every tap owns an accumulator register, which is friendly to high clock rates.
module dsp_fir_transposed #(
    parameter integer TAPS     = 16,
    parameter integer IN_W     = 18,
    parameter integer COEFF_W  = 18,
    parameter integer ACC_W    = 48,
    parameter integer OUT_W    = 18,
    parameter integer OUT_SHIFT = 16
) (
    input  wire                         clk,
    input  wire                         reset_n,
    input  wire                         state_clear,

    input  wire                         cfg_valid,
    input  wire [$clog2(TAPS)-1:0]      cfg_index,
    input  wire signed [COEFF_W-1:0]    cfg_data,

    input  wire                         s_valid,
    input  wire signed [IN_W-1:0]       s_data,
    output wire                         m_valid,
    output wire signed [OUT_W-1:0]      m_data
);
    reg signed [COEFF_W-1:0] coeff_mem [0:TAPS-1];
    reg signed [ACC_W-1:0] stage_acc [0:TAPS-1];
    reg valid_d;
    wire signed [OUT_W-1:0] rounded_output;

    integer i;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            valid_d <= 1'b0;
            for (i = 0; i < TAPS; i = i + 1) begin
                coeff_mem[i] <= {COEFF_W{1'b0}};
                stage_acc[i] <= {ACC_W{1'b0}};
            end
        end else begin
            // Coefficients are configuration, not run state.  A software
            // clear flushes only the delay/accumulator pipeline and valid bit;
            // loaded templates remain ready for the next acquisition.
            if (cfg_valid)
                coeff_mem[cfg_index] <= cfg_data;

            if (state_clear) begin
                valid_d <= 1'b0;
                for (i = 0; i < TAPS; i = i + 1)
                    stage_acc[i] <= {ACC_W{1'b0}};
            end else begin
                valid_d <= s_valid;
            if (s_valid) begin
                for (i = 0; i < TAPS-1; i = i + 1)
                    stage_acc[i] <= ($signed(s_data) * $signed(coeff_mem[i]))
                                    + stage_acc[i+1];
                stage_acc[TAPS-1] <= $signed(s_data) * $signed(coeff_mem[TAPS-1]);
            end
            end
        end
    end

    dsp_round_sat #(.IN_W(ACC_W), .OUT_W(OUT_W), .SHIFT(OUT_SHIFT)) u_output_round (
        .din(stage_acc[0]), .dout(rounded_output)
    );

    assign m_valid = valid_d;
    assign m_data  = rounded_output;
endmodule
