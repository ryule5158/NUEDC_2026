`timescale 1ns/1ps

// 定点舍入饱和模块：统一DSP链路的右移舍入和有符号限幅规则。
// Arithmetic right shift with round-to-nearest (ties away from zero) and
// signed saturation.
// SHIFT must be at least 1. Keeping this operation in one reviewed module avoids
// repeating subtly different truncation rules throughout the DSP chain.
module dsp_round_sat #(
    parameter integer IN_W  = 36,
    parameter integer OUT_W = 18,
    parameter integer SHIFT = 17
) (
    input  wire signed [IN_W-1:0]  din,
    output reg  signed [OUT_W-1:0] dout
);
    localparam integer WORK_W = IN_W + 1;
    localparam signed [WORK_W-1:0] ROUND_BIAS =
        ({{(WORK_W-1){1'b0}}, 1'b1} <<< (SHIFT-1));
    localparam signed [WORK_W-1:0] MAX_OUT =
        {{(WORK_W-OUT_W){1'b0}}, 1'b0, {(OUT_W-1){1'b1}}};
    localparam signed [WORK_W-1:0] MIN_OUT =
        {{(WORK_W-OUT_W){1'b1}}, 1'b1, {(OUT_W-1){1'b0}}};

    reg signed [WORK_W-1:0] extended;
    reg signed [WORK_W-1:0] rounded;
    reg signed [WORK_W-1:0] shifted;

    always @* begin
        extended = {din[IN_W-1], din};
        // 负数不能直接减半个 LSB 再算术右移：那会把恰好为整数的负数也
        // 多减 1（例如 -1.000 会错误变成 -2）。负数加 (half-1) 后再
        // floor，才与正数的“最近值、半值远离零”规则对称。
        rounded  = (extended >= 0) ? (extended + ROUND_BIAS)
                                   : (extended + ROUND_BIAS - 1'b1);
        shifted  = rounded >>> SHIFT;

        if (shifted > MAX_OUT)
            dout = {1'b0, {(OUT_W-1){1'b1}}};
        else if (shifted < MIN_OUT)
            dout = {1'b1, {(OUT_W-1){1'b0}}};
        else
            dout = shifted[OUT_W-1:0];
    end
endmodule
