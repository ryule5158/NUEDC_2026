`timescale 1ns/1ps

// CIC抽取器：无乘法完成积分、降采样和梳状滤波，状态数组保存各级历史量。
// Multiplier-free CIC decimator. RATE should normally be a power of two so
// GAIN_SHIFT can exactly remove the DC gain RATE^STAGES.
module dsp_cic_decimator #(
    parameter integer IN_W       = 18,
    parameter integer OUT_W      = 18,
    parameter integer RATE       = 16,
    parameter integer STAGES     = 3,
    parameter integer ACC_W      = 48,
    parameter integer GAIN_SHIFT = 12
) (
    input  wire                         clk,
    input  wire                         reset_n,
    input  wire                         state_clear,
    input  wire                         s_valid,
    input  wire signed [IN_W-1:0]       s_data,
    output wire                         m_valid,
    output wire signed [OUT_W-1:0]      m_data
);
    localparam integer COUNT_W = (RATE <= 1) ? 1 : $clog2(RATE);
    reg [COUNT_W-1:0] rate_count;
    reg signed [ACC_W-1:0] integrator [0:STAGES-1];
    reg signed [ACC_W-1:0] comb_delay [0:STAGES-1];
    reg signed [ACC_W-1:0] comb_stage [0:STAGES-1];
    reg [STAGES-1:0] integrator_valid;
    reg [STAGES-1:0] comb_valid;
    wire signed [ACC_W-1:0] sample_ext =
        {{(ACC_W-IN_W){s_data[IN_W-1]}}, s_data};
    wire signed [OUT_W-1:0] scaled_output;

    dsp_round_sat #(.IN_W(ACC_W), .OUT_W(OUT_W), .SHIFT(GAIN_SHIFT)) u_gain_scale (
        .din(comb_stage[STAGES-1]), .dout(scaled_output)
    );

    assign m_valid = comb_valid[STAGES-1];
    assign m_data  = scaled_output;

    integer i;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rate_count <= {COUNT_W{1'b0}};
            integrator_valid <= {STAGES{1'b0}};
            comb_valid <= {STAGES{1'b0}};
            for (i = 0; i < STAGES; i = i + 1) begin
                integrator[i] <= {ACC_W{1'b0}};
                comb_delay[i] <= {ACC_W{1'b0}};
                comb_stage[i] <= {ACC_W{1'b0}};
            end
        end else if (state_clear) begin
            rate_count <= {COUNT_W{1'b0}};
            integrator_valid <= {STAGES{1'b0}};
            comb_valid <= {STAGES{1'b0}};
            for (i = 0; i < STAGES; i = i + 1) begin
                integrator[i] <= {ACC_W{1'b0}};
                comb_delay[i] <= {ACC_W{1'b0}};
                comb_stage[i] <= {ACC_W{1'b0}};
            end
        end else begin
            // Register every CIC stage.  The previous implementation formed
            // all three 48-bit integrator adds and all three comb subtracts in
            // one combinational path (59 logic levels).  This pipeline keeps
            // one-sample-per-clock throughput while each timing stage contains
            // only one wide add/subtract.
            integrator_valid[0] <= s_valid;
            if (s_valid) begin
                integrator[0] <= integrator[0] + sample_ext;
            end
            for (i = 1; i < STAGES; i = i + 1) begin
                integrator_valid[i] <= integrator_valid[i-1];
                if (integrator_valid[i-1])
                    integrator[i] <= integrator[i] + integrator[i-1];
            end

            comb_valid[0] <= 1'b0;
            for (i = 1; i < STAGES; i = i + 1) begin
                comb_valid[i] <= comb_valid[i-1];
                if (comb_valid[i-1]) begin
                    comb_stage[i] <= comb_stage[i-1] - comb_delay[i];
                    comb_delay[i] <= comb_stage[i-1];
                end
            end

            // Count samples only after they have traversed all registered
            // integrator stages.  integrator[STAGES-1] then belongs to the
            // same valid item represented by integrator_valid[STAGES-1].
            if (integrator_valid[STAGES-1]) begin
                if (rate_count == RATE-1) begin
                    rate_count <= {COUNT_W{1'b0}};
                    comb_stage[0] <= integrator[STAGES-1] - comb_delay[0];
                    comb_delay[0] <= integrator[STAGES-1];
                    comb_valid[0] <= 1'b1;
                end else begin
                    rate_count <= rate_count + 1'b1;
                end
            end
        end
    end
endmodule
