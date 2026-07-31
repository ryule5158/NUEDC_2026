`timescale 1ns/1ps

// 功率计：对2^BLOCK_LOG2个有效样本求平方均值，供多频点能量判决。
// Mean-square estimator over a 2^BLOCK_LOG2 sample block.
// RMS can be obtained later with a CORDIC/square-root stage; many contest
// decisions only need power ratios, so keeping RMS squared saves logic.
module dsp_power_meter #(
    parameter integer IN_W       = 18,
    parameter integer BLOCK_LOG2 = 10
) (
    input  wire                         clk,
    input  wire                         reset_n,
    input  wire                         state_clear,
    input  wire                         s_valid,
    input  wire signed [IN_W-1:0]       s_data,
    output reg                          m_valid,
    output reg [2*IN_W-1:0]             mean_square
);
    localparam integer BLOCK_SIZE = (1 << BLOCK_LOG2);
    localparam integer SUM_W = 2*IN_W + BLOCK_LOG2;
    reg [BLOCK_LOG2-1:0] sample_count;
    reg [SUM_W-1:0] sum_square;
    wire signed [2*IN_W-1:0] square_signed = s_data * s_data;
    wire [2*IN_W-1:0] square_value = square_signed;
    wire [SUM_W-1:0] sum_next = sum_square + {{BLOCK_LOG2{1'b0}}, square_value};

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_count <= {BLOCK_LOG2{1'b0}};
            sum_square <= {SUM_W{1'b0}};
            mean_square <= {(2*IN_W){1'b0}};
            m_valid <= 1'b0;
        end else if (state_clear) begin
            sample_count <= {BLOCK_LOG2{1'b0}};
            sum_square <= {SUM_W{1'b0}};
            mean_square <= {(2*IN_W){1'b0}};
            m_valid <= 1'b0;
        end else begin
            m_valid <= 1'b0;
            if (s_valid) begin
                if (sample_count == BLOCK_SIZE-1) begin
                    mean_square <= sum_next >> BLOCK_LOG2;
                    sample_count <= {BLOCK_LOG2{1'b0}};
                    sum_square <= {SUM_W{1'b0}};
                    m_valid <= 1'b1;
                end else begin
                    sample_count <= sample_count + 1'b1;
                    sum_square <= sum_next;
                end
            end
        end
    end
endmodule
