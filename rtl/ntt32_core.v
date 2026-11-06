//=============================================================================
// ntt32_core.v  --  Combinational 32-point NTT over Z_q (step-1 cross transform)
//
// One column of the step-1 cross stage of the four-step mixed-radix NTT:
//   A1[k1] = sum_{n1=0..31} a[n1] * w1^(n1*k1),   w1 = omega^256 (order 32)
//
// Ports are FLAT: bit slice [n*CW +: CW] is element n (n1 in, k1 out).
//
// Structure is an exact RTL mirror of the verified Python ntt_dit()
// (hedge-scripts/four_step_ntt_ref.py, hedge-scripts/gen_ntt32_rom.py):
//   * radix-2 DIT, 5 stages
//   * input bit-reversed, output natural order
//   * stage s: half = 2^s, m = 2^(s+1), twiddle exponent e = j << (L-1-s)
//   * twiddles are w1^0 .. w1^15, held in a 16-entry ROM
//
// Modular arithmetic is the same two-step Barrett reduction already proven on
// this project (q = 2^47 - 2^20 + 1, C = 2^20 - 1):
//   2^47 = q + (2^20 - 1)   =>   x = hi*2^47 + lo  ->  hi*C + lo  (twice)
//
// Fully combinational: functional correctness first; pipeline registers can be
// dropped between the staged wires later for timing closure.
//=============================================================================
`include "ntt_twiddles.vh"

module ntt32_core #(
    parameter CW      = 48,
    parameter ROM_HEX = "ntt32_twiddles.hex"
) (
    input  wire [32*CW-1:0] din,    // [n1*CW +: CW] = a[n1],  natural order
    output wire [32*CW-1:0] dout    // [k1*CW +: CW] = A1[k1], natural order
);

    localparam [CW-1:0] Q    = `HEDGE_Q;
    localparam [CW-1:0] REDC = `HEDGE_REDC;

    //--------------------------------------------------------------------------
    // Modular helpers (identical arithmetic to ac_modules_real.sv)
    //--------------------------------------------------------------------------
    function automatic [CW-1:0] qreduce(input [2*CW-1:0] x);
        reg [67:0] hi1, t1, hi2, t2;
        reg [46:0] lo1, lo2;
        reg [CW-1:0] r;
        begin
            hi1 = x[95:47];
            lo1 = x[46:0];
            t1  = hi1 * REDC + lo1;
            hi2 = t1[67:47];
            lo2 = t1[46:0];
            t2  = hi2 * REDC + lo2;
            r   = t2[CW-1:0];
            if (r >= Q) r = r - Q;
            qreduce = r;
        end
    endfunction

    function automatic [CW-1:0] modmul(input [CW-1:0] a, input [CW-1:0] b);
        reg [2*CW-1:0] p;
        begin
            p = a * b;                     // 96-bit context, full product
            modmul = qreduce(p);
        end
    endfunction

    function automatic [CW-1:0] modadd(input [CW-1:0] a, input [CW-1:0] b);
        reg [CW-1:0] s;
        begin
            s = a + b;                    // < 2q < 2^48
            modadd = (s >= Q) ? s - Q : s;
        end
    endfunction

    function automatic [CW-1:0] modsub(input [CW-1:0] a, input [CW-1:0] b);
        reg [CW-1:0] d;
        begin
            d = a - b;                    // > -q
            modsub = (a < b) ? d + Q : d;
        end
    endfunction

    //--------------------------------------------------------------------------
    // Twiddle ROM: tw[i] = w1^i, i = 0..15
    //--------------------------------------------------------------------------
    reg [CW-1:0] tw_rom [0:15];
    initial begin
        $readmemh(ROM_HEX, tw_rom);
    end

    //--------------------------------------------------------------------------
    // Bit reversal (5 bits) for the DIT input ordering
    //--------------------------------------------------------------------------
    function automatic [4:0] bitrev5(input [4:0] x);
        begin
            bitrev5 = {x[0], x[1], x[2], x[3], x[4]};
        end
    endfunction

    //--------------------------------------------------------------------------
    // Stage 0: bit-reverse the natural-order input
    //--------------------------------------------------------------------------
    wire [CW-1:0] s0 [0:31];
    genvar g;
    generate
        for (g = 0; g < 32; g = g + 1) begin : g_br
            assign s0[bitrev5(g[4:0])][47:0] = din[g*CW +: CW];
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Butterfly stages.
    //   stage ST: half = 2^ST, m = 2^(ST+1), shift = 4-ST
    //   top index t: (t % m) < half, pairs with t+half, j = t % half,
    //   twiddle index e = j << shift.  Each index is the top of exactly one
    //   butterfly, so every staged wire is driven exactly once.
    //--------------------------------------------------------------------------
    wire [CW-1:0] s1 [0:31];
    generate
        for (g = 0; g < 32; g = g + 1) begin : g_st0
            if (g % 2 == 0) begin : b0
                wire [CW-1:0] u = s0[g];
                wire [CW-1:0] v = modmul(s0[g+1], tw_rom[0]);
                assign s1[g]   = modadd(u, v);
                assign s1[g+1] = modsub(u, v);
            end
        end
    endgenerate

    wire [CW-1:0] s2 [0:31];
    generate
        for (g = 0; g < 32; g = g + 1) begin : g_st1
            if (g % 4 < 2) begin : b1
                localparam E1 = (g % 2) * 8;
                wire [CW-1:0] u = s1[g];
                wire [CW-1:0] v = modmul(s1[g+2], tw_rom[E1]);
                assign s2[g]   = modadd(u, v);
                assign s2[g+2] = modsub(u, v);
            end
        end
    endgenerate

    wire [CW-1:0] s3 [0:31];
    generate
        for (g = 0; g < 32; g = g + 1) begin : g_st2
            if (g % 8 < 4) begin : b2
                localparam E2 = (g % 4) * 4;
                wire [CW-1:0] u = s2[g];
                wire [CW-1:0] v = modmul(s2[g+4], tw_rom[E2]);
                assign s3[g]   = modadd(u, v);
                assign s3[g+4] = modsub(u, v);
            end
        end
    endgenerate

    wire [CW-1:0] s4 [0:31];
    generate
        for (g = 0; g < 32; g = g + 1) begin : g_st3
            if (g % 16 < 8) begin : b3
                localparam E3 = (g % 8) * 2;
                wire [CW-1:0] u = s3[g];
                wire [CW-1:0] v = modmul(s3[g+8], tw_rom[E3]);
                assign s4[g]   = modadd(u, v);
                assign s4[g+8] = modsub(u, v);
            end
        end
    endgenerate

    wire [CW-1:0] s5 [0:31];
    generate
        for (g = 0; g < 32; g = g + 1) begin : g_st4
            if (g % 32 < 16) begin : b4
                localparam E4 = (g % 16);
                wire [CW-1:0] u = s4[g];
                wire [CW-1:0] v = modmul(s4[g+16], tw_rom[E4]);
                assign s5[g]    = modadd(u, v);
                assign s5[g+16] = modsub(u, v);
            end
        end
    endgenerate

    generate
        for (g = 0; g < 32; g = g + 1) begin : g_out
            assign dout[g*CW +: CW] = s5[g];
        end
    endgenerate

endmodule
