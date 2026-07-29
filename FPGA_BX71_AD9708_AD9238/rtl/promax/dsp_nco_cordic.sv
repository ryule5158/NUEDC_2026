`timescale 1ns/1ps

// CORDIC数控振荡器：32位相位累加，流水输出18位正弦和余弦。
// 32-bit phase-accumulator NCO with a fully pipelined CORDIC rotator.
// Phase uses binary angle measurement (BAM): 2^32 represents one full turn.
// After the pipeline fills, one sin/cos pair is produced for every valid input.
module dsp_nco_cordic #(
    parameter integer PHASE_W = 32,
    parameter integer AMP_W   = 18,
    parameter integer ITER    = 16
) (
    input  wire                       clk,
    input  wire                       reset_n,
    input  wire                       state_clear,
    input  wire                       s_valid,
    input  wire [PHASE_W-1:0]         phase_inc,
    input  wire [PHASE_W-1:0]         phase_offset,
    output wire                       m_valid,
    output wire signed [AMP_W-1:0]    sin_out,
    output wire signed [AMP_W-1:0]    cos_out
);
    localparam integer XY_W = AMP_W + 2;
    // 0.607253 * (2^(AMP_W-1)-1)，补偿 CORDIC 的固有增益。
    // 用 64 位常量计算，避免原先只在 AMP_W=18 时才正确的魔数。
    localparam signed [XY_W-1:0] CORDIC_K =
        (64'sd607253 * ((64'sd1 <<< (AMP_W-1)) - 1) + 64'sd500000)
        / 64'sd1000000;
    localparam signed [PHASE_W-1:0] QUARTER_TURN = 32'sh4000_0000;

    reg [PHASE_W-1:0] phase_acc;
    reg signed [XY_W-1:0] x_pipe [0:ITER];
    reg signed [XY_W-1:0] y_pipe [0:ITER];
    reg signed [PHASE_W-1:0] z_pipe [0:ITER];
    reg [ITER:0] valid_pipe;
    reg [ITER:0] flip_pipe;

    wire signed [PHASE_W-1:0] phase_bam = $signed(phase_acc + phase_offset);
    wire signed [XY_W-1:0] cos_wide = flip_pipe[ITER] ? -x_pipe[ITER] : x_pipe[ITER];
    wire signed [XY_W-1:0] sin_wide = flip_pipe[ITER] ? -y_pipe[ITER] : y_pipe[ITER];

    function automatic signed [PHASE_W-1:0] atan_bam(input integer index);
        begin
            case (index)
                0:  atan_bam = 32'sh2000_0000;
                1:  atan_bam = 32'sh12E4_051E;
                2:  atan_bam = 32'sh09FB_385B;
                3:  atan_bam = 32'sh0511_11D4;
                4:  atan_bam = 32'sh028B_0D43;
                5:  atan_bam = 32'sh0145_D7E1;
                6:  atan_bam = 32'sh00A2_F61E;
                7:  atan_bam = 32'sh0051_7C55;
                8:  atan_bam = 32'sh0028_BE53;
                9:  atan_bam = 32'sh0014_5F2F;
                10: atan_bam = 32'sh000A_2F98;
                11: atan_bam = 32'sh0005_17CC;
                12: atan_bam = 32'sh0002_8BE6;
                13: atan_bam = 32'sh0001_45F3;
                14: atan_bam = 32'sh0000_A2FA;
                15: atan_bam = 32'sh0000_517D;
                default: atan_bam = {PHASE_W{1'b0}};
            endcase
        end
    endfunction

    integer i;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            phase_acc  <= {PHASE_W{1'b0}};
            valid_pipe <= {(ITER+1){1'b0}};
            flip_pipe  <= {(ITER+1){1'b0}};
            for (i = 0; i <= ITER; i = i + 1) begin
                x_pipe[i] <= {XY_W{1'b0}};
                y_pipe[i] <= {XY_W{1'b0}};
                z_pipe[i] <= {PHASE_W{1'b0}};
            end
        end else if (state_clear) begin
            // Synchronous run-state clear.  This is intentionally separate
            // from reset_n so a software clear never creates an asynchronous
            // reset pulse inside the FPGA clock domain.
            phase_acc  <= {PHASE_W{1'b0}};
            valid_pipe <= {(ITER+1){1'b0}};
            flip_pipe  <= {(ITER+1){1'b0}};
            for (i = 0; i <= ITER; i = i + 1) begin
                x_pipe[i] <= {XY_W{1'b0}};
                y_pipe[i] <= {XY_W{1'b0}};
                z_pipe[i] <= {PHASE_W{1'b0}};
            end
        end else begin
            valid_pipe[0] <= s_valid;
            if (s_valid) begin
                phase_acc <= phase_acc + phase_inc;
                x_pipe[0] <= CORDIC_K;
                y_pipe[0] <= {XY_W{1'b0}};
                // CORDIC converges in +/-90 degrees. For quadrants II/III,
                // rotate by pi now and invert both outputs at the end.
                if ((phase_bam > QUARTER_TURN) ||
                    (phase_bam < -QUARTER_TURN)) begin
                    z_pipe[0] <= phase_bam ^ 32'h8000_0000;
                    flip_pipe[0] <= 1'b1;
                end else begin
                    z_pipe[0] <= phase_bam;
                    flip_pipe[0] <= 1'b0;
                end
            end

            for (i = 0; i < ITER; i = i + 1) begin
                valid_pipe[i+1] <= valid_pipe[i];
                flip_pipe[i+1]  <= flip_pipe[i];
                if (valid_pipe[i]) begin
                    if (z_pipe[i] >= 0) begin
                        x_pipe[i+1] <= x_pipe[i] - (y_pipe[i] >>> i);
                        y_pipe[i+1] <= y_pipe[i] + (x_pipe[i] >>> i);
                        z_pipe[i+1] <= z_pipe[i] - atan_bam(i);
                    end else begin
                        x_pipe[i+1] <= x_pipe[i] + (y_pipe[i] >>> i);
                        y_pipe[i+1] <= y_pipe[i] - (x_pipe[i] >>> i);
                        z_pipe[i+1] <= z_pipe[i] + atan_bam(i);
                    end
                end
            end
        end
    end

    assign m_valid = valid_pipe[ITER];
    assign cos_out = cos_wide[AMP_W-1:0];
    assign sin_out = sin_wide[AMP_W-1:0];
endmodule
