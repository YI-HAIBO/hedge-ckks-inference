//=============================================================================
// tb_ntt64_core.v -- Golden-vector testbench for the 64-point cross NTT core.
//
// Loads ntt64_golden_in.hex (64 random inputs) produced by
// hedge-scripts/gen_ntt64_golden.py, runs them through the combinational core,
// and compares against ntt64_golden_out.hex (Python ntt_dit result cross-
// checked against brute-force DFT in the generator).
//
// Phase 2 verification: NPAR=64 uses ntt64_core (6-stage DIT, w1=omega^128).
//=============================================================================
`timescale 1ns/1ps

module tb_ntt64_core;

    localparam CW   = 48;
    localparam BASE = "C:/Users/Yi/WorkBuddy/2026-10-03-09-14-19/hedge-rtl/sub_a_plus_c/";

    reg  [64*CW-1:0] din;
    wire [64*CW-1:0] dout;

    reg  [CW-1:0] golden_in  [0:63];
    reg  [CW-1:0] golden_out [0:63];

    integer i, errors;

    ntt64_core #(
        .CW(CW),
        .ROM_HEX({BASE, "ntt64_twiddles.hex"})
    ) dut (
        .din(din),
        .dout(dout)
    );

    initial begin
        $display("==============================================================");
        $display("TB ntt64_core: 64-point cross NTT vs Python golden");
        $display("==============================================================");

        $readmemh({BASE, "ntt64_golden_in.hex"},  golden_in);
        $readmemh({BASE, "ntt64_golden_out.hex"}, golden_out);

        din = {64*CW{1'b0}};
        for (i = 0; i < 64; i = i + 1)
            din[i*CW +: CW] = golden_in[i];

        #10;

        errors = 0;
        for (i = 0; i < 64; i = i + 1) begin
            if (dout[i*CW +: CW] !== golden_out[i]) begin
                $display("  MISMATCH k1=%0d: got=0x%H expected=0x%H",
                         i, dout[i*CW +: CW], golden_out[i]);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("  PASS: all 64 outputs match Python ntt_dit golden");
        else
            $display("  FAIL: %d/64 mismatches", errors);

        $display("==============================================================");
        $finish;
    end

    initial begin
        #(1000000);
        $display("GLOBAL TIMEOUT");
        $finish;
    end

endmodule
