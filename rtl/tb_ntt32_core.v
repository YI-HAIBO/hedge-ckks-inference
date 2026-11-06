//=============================================================================
// tb_ntt32_core.v -- Golden-vector testbench for the 32-point cross NTT core.
//
// Loads ntt32_golden_in.hex (32 random inputs) produced by
// hedge-scripts/gen_ntt32_rom.py, runs them through the combinational core,
// and compares against ntt32_golden_out.hex (the Python ntt_dit result, which
// is itself cross-checked against brute-force DFT in the generator).
//=============================================================================
`timescale 1ns/1ps

module tb_ntt32_core;

    localparam CW   = 48;
    localparam BASE = "C:/Users/Yi/WorkBuddy/2026-10-03-09-14-19/hedge-rtl/sub_a_plus_c/";

    reg  [32*CW-1:0] din;
    wire [32*CW-1:0] dout;

    reg  [CW-1:0] golden_in  [0:31];
    reg  [CW-1:0] golden_out [0:31];

    integer i, errors;

    ntt32_core #(
        .CW(CW),
        .ROM_HEX({BASE, "ntt32_twiddles.hex"})
    ) dut (
        .din(din),
        .dout(dout)
    );

    initial begin
        $display("==============================================================");
        $display("TB ntt32_core: 32-point cross NTT vs Python golden");
        $display("==============================================================");

        $readmemh({BASE, "ntt32_golden_in.hex"},  golden_in);
        $readmemh({BASE, "ntt32_golden_out.hex"}, golden_out);

        din = {32*CW{1'b0}};
        for (i = 0; i < 32; i = i + 1)
            din[i*CW +: CW] = golden_in[i];

        #10;

        errors = 0;
        for (i = 0; i < 32; i = i + 1) begin
            if (dout[i*CW +: CW] !== golden_out[i]) begin
                $display("  MISMATCH k1=%0d: got=0x%H expected=0x%H",
                         i, dout[i*CW +: CW], golden_out[i]);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("  PASS: all 32 outputs match Python ntt_dit golden");
        else
            $display("  FAIL: %d/32 mismatches", errors);

        $display("==============================================================");
        $finish;
    end

    initial begin
        #(1000000);
        $display("GLOBAL TIMEOUT");
        $finish;
    end

endmodule
