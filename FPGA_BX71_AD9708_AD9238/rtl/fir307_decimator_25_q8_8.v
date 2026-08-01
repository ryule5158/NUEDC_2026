`timescale 1ns / 1ps

/*
 * Resource-shared CH2 anti-alias reference filter.
 *
 * The 307-tap FIR is scheduled in transposed polyphase form. Each incoming
 * sample is multiplied by the taps that contribute to the next 13 R25 output
 * instants. Thirteen registered multipliers and accumulators replace a second
 * 154-DSP full-rate FIR while producing the same convolution outputs.
 *
 * The 13 paths are intentionally written explicitly. Vivado 2018.3 can crash
 * while optimizing the equivalent rotating unpacked-register arrays.
 */
module fir307_decimator_25_q8_8(
    input  wire              clk,
    input  wire              reset_n,
    input  wire              clear,
    input  wire              input_valid,
    input  wire [11:0]       input_code,
    output reg               output_valid,
    output reg signed [15:0] output_sample_q8_8
);
    reg [8:0] valid_sample_count;
    reg [4:0] input_phase;
    reg product_valid;
    reg [4:0] product_phase;
    reg product_output_eligible;

    reg signed [39:0] accumulator_0, accumulator_1, accumulator_2;
    reg signed [39:0] accumulator_3, accumulator_4, accumulator_5;
    reg signed [39:0] accumulator_6, accumulator_7, accumulator_8;
    reg signed [39:0] accumulator_9, accumulator_10, accumulator_11;
    reg signed [39:0] accumulator_12;
    reg signed [39:0] product_0, product_1, product_2, product_3;
    reg signed [39:0] product_4, product_5, product_6, product_7;
    reg signed [39:0] product_8, product_9, product_10, product_11;
    reg signed [39:0] product_12;

    wire signed [12:0] centered_input =
        $signed({1'b0, input_code}) - 13'sd2048;

    function signed [17:0] FIR307_Coefficient;
        input [7:0] coefficient_index;
        begin
            case (coefficient_index)
`include "fir307_antialias_coeffs.vh"
            endcase
        end
    endfunction

    function signed [17:0] FIR307_FullCoefficient;
        input [8:0] tap_index;
        begin
            if (tap_index <= 9'd153)
                FIR307_FullCoefficient =
                    FIR307_Coefficient(tap_index[7:0]);
            else if (tap_index <= 9'd306)
                FIR307_FullCoefficient =
                    FIR307_Coefficient(9'd306 - tap_index);
            else
                FIR307_FullCoefficient = 18'sd0;
        end
    endfunction

    function signed [15:0] FIR307_ToQ8_8;
        input signed [39:0] accumulator_q16;
        reg signed [40:0] magnitude;
        reg signed [40:0] rounded_q8_8;
        begin
            if (accumulator_q16 >= 0) begin
                rounded_q8_8 =
                    ($signed({1'b0, accumulator_q16}) + 41'sd2048) >>> 12;
            end else begin
                magnitude = -$signed({accumulator_q16[39],
                                      accumulator_q16});
                rounded_q8_8 = -((magnitude + 41'sd2048) >>> 12);
            end
            if (rounded_q8_8 > 41'sd32767)
                FIR307_ToQ8_8 = 16'sh7FFF;
            else if (rounded_q8_8 < -41'sd32768)
                FIR307_ToQ8_8 = 16'sh8000;
            else
                FIR307_ToQ8_8 = rounded_q8_8[15:0];
        end
    endfunction

    wire [8:0] base_tap = 9'd24 - {4'd0, input_phase};
    wire signed [17:0] coefficient_0 = FIR307_FullCoefficient(base_tap);
    wire signed [17:0] coefficient_1 = FIR307_FullCoefficient(base_tap + 9'd25);
    wire signed [17:0] coefficient_2 = FIR307_FullCoefficient(base_tap + 9'd50);
    wire signed [17:0] coefficient_3 = FIR307_FullCoefficient(base_tap + 9'd75);
    wire signed [17:0] coefficient_4 = FIR307_FullCoefficient(base_tap + 9'd100);
    wire signed [17:0] coefficient_5 = FIR307_FullCoefficient(base_tap + 9'd125);
    wire signed [17:0] coefficient_6 = FIR307_FullCoefficient(base_tap + 9'd150);
    wire signed [17:0] coefficient_7 = FIR307_FullCoefficient(base_tap + 9'd175);
    wire signed [17:0] coefficient_8 = FIR307_FullCoefficient(base_tap + 9'd200);
    wire signed [17:0] coefficient_9 = FIR307_FullCoefficient(base_tap + 9'd225);
    wire signed [17:0] coefficient_10 = FIR307_FullCoefficient(base_tap + 9'd250);
    wire signed [17:0] coefficient_11 = FIR307_FullCoefficient(base_tap + 9'd275);
    wire signed [17:0] coefficient_12 = FIR307_FullCoefficient(base_tap + 9'd300);

    wire signed [30:0] next_0 = centered_input * coefficient_0;
    wire signed [30:0] next_1 = centered_input * coefficient_1;
    wire signed [30:0] next_2 = centered_input * coefficient_2;
    wire signed [30:0] next_3 = centered_input * coefficient_3;
    wire signed [30:0] next_4 = centered_input * coefficient_4;
    wire signed [30:0] next_5 = centered_input * coefficient_5;
    wire signed [30:0] next_6 = centered_input * coefficient_6;
    wire signed [30:0] next_7 = centered_input * coefficient_7;
    wire signed [30:0] next_8 = centered_input * coefficient_8;
    wire signed [30:0] next_9 = centered_input * coefficient_9;
    wire signed [30:0] next_10 = centered_input * coefficient_10;
    wire signed [30:0] next_11 = centered_input * coefficient_11;
    wire signed [30:0] next_12 = centered_input * coefficient_12;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            valid_sample_count <= 9'd0;
            input_phase <= 5'd0;
            product_valid <= 1'b0;
            product_phase <= 5'd0;
            product_output_eligible <= 1'b0;
            output_valid <= 1'b0;
            output_sample_q8_8 <= 16'sd0;
            accumulator_0 <= 0; accumulator_1 <= 0; accumulator_2 <= 0;
            accumulator_3 <= 0; accumulator_4 <= 0; accumulator_5 <= 0;
            accumulator_6 <= 0; accumulator_7 <= 0; accumulator_8 <= 0;
            accumulator_9 <= 0; accumulator_10 <= 0; accumulator_11 <= 0;
            accumulator_12 <= 0;
            product_0 <= 0; product_1 <= 0; product_2 <= 0; product_3 <= 0;
            product_4 <= 0; product_5 <= 0; product_6 <= 0; product_7 <= 0;
            product_8 <= 0; product_9 <= 0; product_10 <= 0;
            product_11 <= 0; product_12 <= 0;
        end else begin
            output_valid <= 1'b0;
            if (clear) begin
                valid_sample_count <= 9'd0;
                input_phase <= 5'd0;
                product_valid <= 1'b0;
                product_phase <= 5'd0;
                product_output_eligible <= 1'b0;
                output_sample_q8_8 <= 16'sd0;
                accumulator_0 <= 0; accumulator_1 <= 0; accumulator_2 <= 0;
                accumulator_3 <= 0; accumulator_4 <= 0; accumulator_5 <= 0;
                accumulator_6 <= 0; accumulator_7 <= 0; accumulator_8 <= 0;
                accumulator_9 <= 0; accumulator_10 <= 0; accumulator_11 <= 0;
                accumulator_12 <= 0;
                product_0 <= 0; product_1 <= 0; product_2 <= 0; product_3 <= 0;
                product_4 <= 0; product_5 <= 0; product_6 <= 0; product_7 <= 0;
                product_8 <= 0; product_9 <= 0; product_10 <= 0;
                product_11 <= 0; product_12 <= 0;
            end else begin
                if (product_valid) begin
                    if (product_phase == 5'd24) begin
                        output_sample_q8_8 <= FIR307_ToQ8_8(
                            accumulator_0 + product_0);
                        output_valid <= product_output_eligible;
                        accumulator_0 <= accumulator_1 + product_1;
                        accumulator_1 <= accumulator_2 + product_2;
                        accumulator_2 <= accumulator_3 + product_3;
                        accumulator_3 <= accumulator_4 + product_4;
                        accumulator_4 <= accumulator_5 + product_5;
                        accumulator_5 <= accumulator_6 + product_6;
                        accumulator_6 <= accumulator_7 + product_7;
                        accumulator_7 <= accumulator_8 + product_8;
                        accumulator_8 <= accumulator_9 + product_9;
                        accumulator_9 <= accumulator_10 + product_10;
                        accumulator_10 <= accumulator_11 + product_11;
                        accumulator_11 <= accumulator_12 + product_12;
                        accumulator_12 <= 40'sd0;
                    end else begin
                        accumulator_0 <= accumulator_0 + product_0;
                        accumulator_1 <= accumulator_1 + product_1;
                        accumulator_2 <= accumulator_2 + product_2;
                        accumulator_3 <= accumulator_3 + product_3;
                        accumulator_4 <= accumulator_4 + product_4;
                        accumulator_5 <= accumulator_5 + product_5;
                        accumulator_6 <= accumulator_6 + product_6;
                        accumulator_7 <= accumulator_7 + product_7;
                        accumulator_8 <= accumulator_8 + product_8;
                        accumulator_9 <= accumulator_9 + product_9;
                        accumulator_10 <= accumulator_10 + product_10;
                        accumulator_11 <= accumulator_11 + product_11;
                        accumulator_12 <= accumulator_12 + product_12;
                    end
                end

                if (input_valid) begin
                    product_0 <= {{9{next_0[30]}}, next_0};
                    product_1 <= {{9{next_1[30]}}, next_1};
                    product_2 <= {{9{next_2[30]}}, next_2};
                    product_3 <= {{9{next_3[30]}}, next_3};
                    product_4 <= {{9{next_4[30]}}, next_4};
                    product_5 <= {{9{next_5[30]}}, next_5};
                    product_6 <= {{9{next_6[30]}}, next_6};
                    product_7 <= {{9{next_7[30]}}, next_7};
                    product_8 <= {{9{next_8[30]}}, next_8};
                    product_9 <= {{9{next_9[30]}}, next_9};
                    product_10 <= {{9{next_10[30]}}, next_10};
                    product_11 <= {{9{next_11[30]}}, next_11};
                    product_12 <= {{9{next_12[30]}}, next_12};
                    product_valid <= 1'b1;
                    product_phase <= input_phase;
                    product_output_eligible <=
                        (input_phase == 5'd24) &&
                        (valid_sample_count >= 9'd306);
                    if (valid_sample_count < 9'd307)
                        valid_sample_count <= valid_sample_count + 1'b1;
                    if (input_phase == 5'd24)
                        input_phase <= 5'd0;
                    else
                        input_phase <= input_phase + 1'b1;
                end else begin
                    product_valid <= 1'b0;
                end
            end
        end
    end
endmodule
