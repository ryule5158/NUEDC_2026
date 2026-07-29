`timescale 1ns / 1ps

/*
 * Three-stage CIC decimator used by the 2026 G signal analyzer.
 * Input:  unsigned offset-binary AD9280 code at 32 MSPS.
 * Output: unsigned offset-binary code at 2 MSPS (R=16).
 *
 * DC gain is 16^3=4096, removed by the arithmetic right shift.  The
 * integrators intentionally use two's-complement modular arithmetic; 24 bits
 * are wider than the 9 + 3*log2(16) = 21 bits required by this CIC.
 */
module cic3_decimator_16(
    input  wire       clk,
    input  wire       reset_n,
    input  wire       clear,
    input  wire       input_valid,
    input  wire [7:0] input_code,
    output reg        output_valid,
    output reg  [7:0] output_code
);

    localparam [4:0] DECIMATION = 5'd16;
    localparam [2:0] WARMUP_OUTPUTS = 3'd3;

    reg signed [23:0] integrator1;
    reg signed [23:0] integrator2;
    reg signed [23:0] integrator3;
    reg signed [23:0] comb_delay1;
    reg signed [23:0] comb_delay2;
    reg signed [23:0] comb_delay3;
    reg [3:0] phase;
    reg [2:0] warmup_count;

    wire signed [8:0] centered =
        $signed({1'b0, input_code}) - 9'sd128;
    wire signed [23:0] centered_ext =
        {{15{centered[8]}}, centered};
    wire signed [23:0] integrator1_next =
        integrator1 + centered_ext;
    wire signed [23:0] integrator2_next =
        integrator2 + integrator1_next;
    wire signed [23:0] integrator3_next =
        integrator3 + integrator2_next;
    wire signed [23:0] comb1_next = integrator3_next - comb_delay1;
    wire signed [23:0] comb2_next = comb1_next - comb_delay2;
    wire signed [23:0] comb3_next = comb2_next - comb_delay3;

    function [7:0] CIC_ToOffsetBinary;
        input signed [23:0] value;
        reg signed [23:0] scaled;
        begin
            scaled = value >>> 12;
            if (scaled > 24'sd127)
                CIC_ToOffsetBinary = 8'hFF;
            else if (scaled < -24'sd128)
                CIC_ToOffsetBinary = 8'h00;
            else
                CIC_ToOffsetBinary = scaled[7:0] + 8'd128;
        end
    endfunction

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            integrator1 <= 24'sd0;
            integrator2 <= 24'sd0;
            integrator3 <= 24'sd0;
            comb_delay1 <= 24'sd0;
            comb_delay2 <= 24'sd0;
            comb_delay3 <= 24'sd0;
            phase <= 4'd0;
            warmup_count <= 3'd0;
            output_valid <= 1'b0;
            output_code <= 8'd128;
        end else begin
            output_valid <= 1'b0;

            if (clear) begin
                integrator1 <= 24'sd0;
                integrator2 <= 24'sd0;
                integrator3 <= 24'sd0;
                comb_delay1 <= 24'sd0;
                comb_delay2 <= 24'sd0;
                comb_delay3 <= 24'sd0;
                phase <= 4'd0;
                warmup_count <= 3'd0;
                output_code <= 8'd128;
            end else if (input_valid) begin
                integrator1 <= integrator1_next;
                integrator2 <= integrator2_next;
                integrator3 <= integrator3_next;

                if (phase == (DECIMATION - 1'b1)) begin
                    phase <= 4'd0;
                    comb_delay1 <= integrator3_next;
                    comb_delay2 <= comb1_next;
                    comb_delay3 <= comb2_next;

                    if (warmup_count < WARMUP_OUTPUTS) begin
                        warmup_count <= warmup_count + 1'b1;
                    end else begin
                        output_code <= CIC_ToOffsetBinary(comb3_next);
                        output_valid <= 1'b1;
                    end
                end else begin
                    phase <= phase + 1'b1;
                end
            end
        end
    end

endmodule
