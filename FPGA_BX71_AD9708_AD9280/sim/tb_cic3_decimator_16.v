`timescale 1ns / 1ps

module tb_cic3_decimator_16;
    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg clear = 1'b0;
    reg input_valid = 1'b0;
    reg [7:0] input_code = 8'd128;
    wire output_valid;
    wire [7:0] output_code;
    integer input_index;
    integer output_count;

    always #15.625 clk = ~clk;

    cic3_decimator_16 dut (
        .clk(clk),
        .reset_n(reset_n),
        .clear(clear),
        .input_valid(input_valid),
        .input_code(input_code),
        .output_valid(output_valid),
        .output_code(output_code)
    );

    task ClearFilter;
        begin
            @(negedge clk);
            clear = 1'b1;
            @(negedge clk);
            clear = 1'b0;
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        reset_n = 1'b1;
        ClearFilter();

        /* A constant input must retain its offset and amplitude after R^3 scaling. */
        input_valid = 1'b1;
        input_code = 8'd160;
        output_count = 0;
        while (output_count < 8) begin
            @(posedge clk);
            #1;
            if (output_valid) begin
                if (output_code != 8'd160)
                    $fatal(1, "CIC DC gain mismatch: %0d", output_code);
                output_count = output_count + 1;
            end
        end

        /* A 2 MHz square wave has one full period per R=16 block and must null. */
        ClearFilter();
        output_count = 0;
        input_index = 0;
        while (output_count < 8) begin
            @(negedge clk);
            input_code = ((input_index % 16) < 8) ? 8'd192 : 8'd64;
            input_index = input_index + 1;
            @(posedge clk);
            #1;
            if (output_valid) begin
                if ((output_code < 8'd127) || (output_code > 8'd129))
                    $fatal(1, "CIC first-null mismatch: %0d", output_code);
                output_count = output_count + 1;
            end
        end

        $display("G_CIC3_DECIMATOR_TEST_PASS");
        $finish;
    end
endmodule
