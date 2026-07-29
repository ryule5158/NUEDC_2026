`timescale 1ns / 1ps

/*
 * Fixed three-stage CIC decimator for the G analyzer.
 *
 * Input:  unsigned offset-binary AD9280 code at 32 MSPS.
 * Output: signed two's-complement Q8.8 sample at 2 MSPS (R=16).
 *
 * The host compensates the exact deterministic magnitude response
 *
 *   |H(f)| = |sin(pi*f*R/Fs)/(R*sin(pi*f/Fs))|^3
 *
 * and the linear phase -2*pi*f*[3*(R-1)/2]/Fs.  The three integrators use
 * two's-complement modular arithmetic; 24 bits exceed the 9+3*log2(16)=21
 * bits required to preserve the full CIC result.
 *
 * for every reported spectral line. Keeping the CIC here preserves the
 * oversampling/noise benefit of the 8-bit ADC without leaving its passband
 * droop in the final voltage result.
 */
module cic3_decimator_16(
    input  wire                      clk,
    input  wire                      reset_n,
    input  wire                      clear,
    input  wire                      input_valid,
    input  wire [7:0]                input_code,
    output reg                       output_valid,
    output reg signed [15:0]         output_sample_q8_8
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

    /*
     * Normalize R^3=4096 while retaining eight fractional bits:
     * q8_8 = value*256/4096 = value/16.
     * Magnitude-domain round-to-nearest, ties-to-even keeps the quantizer
     * symmetric, followed by signed saturation.
     */
    function signed [15:0] CIC_ToQ8_8;
        input signed [23:0] value;
        reg [24:0] magnitude;
        reg [24:0] rounded_magnitude;
        reg signed [25:0] scaled;
        begin
            if (value[23])
                magnitude = {1'b0, ~value} + 25'd1;
            else
                magnitude = {1'b0, value};

            rounded_magnitude = magnitude >> 4;
            if ((magnitude[3:0] > 4'd8) ||
                ((magnitude[3:0] == 4'd8) &&
                 rounded_magnitude[0])) begin
                rounded_magnitude = rounded_magnitude + 1'b1;
            end

            if (value[23])
                scaled = -$signed({1'b0, rounded_magnitude});
            else
                scaled = $signed({1'b0, rounded_magnitude});

            if (scaled > 26'sd32767)
                CIC_ToQ8_8 = 16'sh7FFF;
            else if (scaled < -26'sd32768)
                CIC_ToQ8_8 = 16'sh8000;
            else
                CIC_ToQ8_8 = scaled[15:0];
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
            output_sample_q8_8 <= 16'sd0;
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
                output_sample_q8_8 <= 16'sd0;
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
                        output_sample_q8_8 <= CIC_ToQ8_8(comb3_next);
                        output_valid <= 1'b1;
                    end
                end else begin
                    phase <= phase + 1'b1;
                end
            end
        end
    end

endmodule
