// tb_ntt64_core_s.v -- module-level equivalence check: serial ntt64_core_s vs
// the already-verified combinational ntt64_core.
//
// Eight random 64-point columns are driven through both cores.  The
// combinational core is the golden model; the serial core's dout is sampled
// on its done pulse and compared word for word.
`timescale 1ns/1ps

module tb_ntt64_core_s;

    localparam CW = 48;
    // Absolute ROM path keeps the test runnable from any shell cwd.
    localparam ROM_A = "C:/Users/Yi/WorkBuddy/2026-10-03-09-14-19/hedge-rtl/sub_a_plus_c/ntt64_twiddles.hex";

    reg              clk = 1'b0;
    reg              rst_n = 1'b0;
    reg              start = 1'b0;
    reg  [64*CW-1:0] din = {64*CW{1'b0}};
    wire [64*CW-1:0] dout_comb;
    wire [64*CW-1:0] dout_ser;
    wire             done;

    ntt64_core #(
        .CW(CW),
        .ROM_HEX(ROM_A)
    ) u_comb (
        .din(din),
        .dout(dout_comb)
    );

    ntt64_core_s #(
        .CW(CW),
        .ROM_HEX(ROM_A)
    ) u_ser (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .din(din),
        .dout(dout_ser),
        .done(done)
    );

    integer round;
    integer j;
    integer errors = 0;

    always #5 clk = ~clk;

    // Compare on the done pulse, after the serial pipeline has written back.
    always @(posedge done) begin
        for (j = 0; j < 64; j = j + 1) begin
            if (dout_comb[j*CW +: CW] !== dout_ser[j*CW +: CW]) begin
                errors = errors + 1;
                $display("  MISMATCH round=%0d k=%0d comb=%h ser=%h",
                         round, j, dout_comb[j*CW +: CW], dout_ser[j*CW +: CW]);
            end
        end
    end

    initial begin
        #2 rst_n = 1'b1;
        for (round = 0; round < 8; round = round + 1) begin
            // Keep each input below q: 32-bit random values zero-extended to
            // 48 bits are always < q = 2^47 - 2^20 + 1.
            for (j = 0; j < 64; j = j + 1)
                din[j*CW +: CW] = {$random};
            #1 start = 1'b1;
            @(posedge clk);
            #1 start = 1'b0;
            @(posedge done);
            @(posedge clk);
        end

        if (errors == 0)
            $display("tb_ntt64_core_s: PASS 8/8 columns bit-exact");
        else
            $display("tb_ntt64_core_s: FAIL %0d/512 words mismatched", errors);
        $finish;
    end

endmodule
