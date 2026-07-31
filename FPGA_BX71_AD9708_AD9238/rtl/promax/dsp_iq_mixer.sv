`timescale 1ns/1ps

// I/Q混频器：用两路DSP乘法器将输入样本并行下变频到复基带。
// Parallel complex down-converter. With LO_W=18, Vivado maps the I and Q
// multipliers naturally to two DSP48E1 slices. LO inputs are Q1.(LO_W-1).
module dsp_iq_mixer #(
    parameter integer IN_W   = 18,
    parameter integer LO_W   = 18,
    parameter integer OUT_W  = 18
) (
    input  wire                        clk,
    input  wire                        reset_n,
    input  wire                        state_clear,
    input  wire                        s_valid,
    input  wire signed [IN_W-1:0]      s_data,
    input  wire signed [LO_W-1:0]      lo_sin,
    input  wire signed [LO_W-1:0]      lo_cos,
    output wire                        m_valid,
    output wire signed [OUT_W-1:0]     m_i,
    output wire signed [OUT_W-1:0]     m_q
);
    localparam integer MULT_W = IN_W + LO_W;
    reg signed [MULT_W-1:0] mult_i;
    reg signed [MULT_W-1:0] mult_q;
    reg valid_d;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            mult_i  <= {MULT_W{1'b0}};
            mult_q  <= {MULT_W{1'b0}};
            valid_d <= 1'b0;
        end else if (state_clear) begin
            mult_i  <= {MULT_W{1'b0}};
            mult_q  <= {MULT_W{1'b0}};
            valid_d <= 1'b0;
        end else begin
            valid_d <= s_valid;
            if (s_valid) begin
                mult_i <= s_data * lo_cos;
                mult_q <= -(s_data * lo_sin);
            end
        end
    end

    dsp_round_sat #(.IN_W(MULT_W), .OUT_W(OUT_W), .SHIFT(LO_W-1)) u_round_i (
        .din(mult_i), .dout(m_i)
    );
    dsp_round_sat #(.IN_W(MULT_W), .OUT_W(OUT_W), .SHIFT(LO_W-1)) u_round_q (
        .din(mult_q), .dout(m_q)
    );

    assign m_valid = valid_d;
endmodule
