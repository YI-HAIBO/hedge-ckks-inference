//=============================================================================
// ntt64_core.v  --  Combinational 64-point NTT over Z_q (step-1 cross transform)
//
// One column of the step-1 cross stage of the four-step mixed-radix NTT for
// NPAR=64 machines:
//   A1[k1] = sum_{n1=0..63} a[n1] * w1^(n1*k1),   w1 = omega^128 (order 64)
//
// Ports are FLAT: bit slice [n*CW +: CW] is element n (n1 in, k1 out).
//
// Structure is an exact RTL mirror of the verified Python ntt_dit()
// (hedge-scripts/four_step_ntt_ref.py, hedge-scripts/gen_ntt32_rom.py pattern):
//   * radix-2 DIT, 6 stages
//   * input bit-reversed, output natural order
//   * stage s: half = 2^s, m = 2^(s+1), twiddle exponent e = j << (L-1-s)
//   * twiddles are w1^0 .. w1^31, held in a 32-entry ROM
//
// Modular arithmetic is the same two-step Barrett reduction already proven on
// this project (q = 2^47 - 2^20 + 1, C = 2^20 - 1):
//   2^47 = q + (2^20 - 1)   =>   x = hi*2^47 + lo  ->  hi*C + lo  (twice)
//
// Fully combinational: functional correctness first; pipeline registers can be
// dropped between the staged wires later for timing closure.
//
// Verified: against hedge-scripts/four_step_ntt_ref.py ntt_dit with w1=omega^128.
//=============================================================================
`include "ntt_twiddles.vh"

module ntt64_core #(
    parameter CW      = 48,
    parameter ROM_HEX = "ntt64_twiddles.hex"
) (
    input  wire [64*CW-1:0] din,    // [n1*CW +: CW] = a[n1],  natural order
    output wire [64*CW-1:0] dout    // [k1*CW +: CW] = A1[k1], natural order
);

    localparam [CW-1:0] Q    = `HEDGE_Q;
    localparam [CW-1:0] REDC = `HEDGE_REDC;

    //--------------------------------------------------------------------------
    // Modular helpers (identical arithmetic to ntt32_core.v)
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
    // Twiddle ROM: tw[i] = w1^i, i = 0..31  (w1 = omega^128, order 64)
    //--------------------------------------------------------------------------
    reg [CW-1:0] tw_rom [0:31];
    initial begin
        $readmemh(ROM_HEX, tw_rom);
    end

    //--------------------------------------------------------------------------
    // Bit reversal (6 bits) for the DIT input ordering
    //--------------------------------------------------------------------------
    function automatic [5:0] bitrev6(input [5:0] x);
        begin
            bitrev6 = {x[0], x[1], x[2], x[3], x[4], x[5]};
        end
    endfunction

    //--------------------------------------------------------------------------
    // Stage 0: bit-reverse the natural-order input
    //--------------------------------------------------------------------------
    wire [CW-1:0] s0 [0:63];
    genvar g;
    generate
        for (g = 0; g < 64; g = g + 1) begin : g_br
            assign s0[bitrev6(g[5:0])][47:0] = din[g*CW +: CW];
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Butterfly stages.
    //   stage ST: half = 2^ST, m = 2^(ST+1), shift = 5-ST
    //   top index t: (t % m) < half, pairs with t+half, j = t % half,
    //   twiddle index e = j << shift.  Each index is the top of exactly one
    //   butterfly, so every staged wire is driven exactly once.
    //
    //   Stage 0: half=1,  m=2,  E0 = (g%2)*32  = 0
    //   Stage 1: half=2,  m=4,  E1 = (g%2)*16  = 0,16
    //   Stage 2: half=4,  m=8,  E2 = (g%4)*8   = 0,8,16,24
    //   Stage 3: half=8,  m=16, E3 = (g%8)*4   = 0,4,8,12,16,20,24,28
    //   Stage 4: half=16, m=32, E4 = (g%16)*2  = 0,2,4,...,30
    //   Stage 5: half=32, m=64, E5 = (g%32)     = 0,1,2,...,31
    //--------------------------------------------------------------------------

    // Stage 0: half=1, m=2, shift=5, twiddle e=0 only
    wire [CW-1:0] s1 [0:63];
    generate
        for (g = 0; g < 64; g = g + 1) begin : g_st0
            if (g % 2 == 0) begin : b0
                wire [CW-1:0] u = s0[g];
                wire [CW-1:0] v = modmul(s0[g+1], tw_rom[0]);
                assign s1[g]   = modadd(u, v);
                assign s1[g+1] = modsub(u, v);
            end
        end
    endgenerate

    // Stage 1: half=2, m=4, shift=4, twiddle e=0,16
    wire [CW-1:0] s2 [0:63];
    generate
        for (g = 0; g < 64; g = g + 1) begin : g_st1
            if (g % 4 < 2) begin : b1
                localparam E1 = (g % 2) * 16;
                wire [CW-1:0] u = s1[g];
                wire [CW-1:0] v = modmul(s1[g+2], tw_rom[E1]);
                assign s2[g]   = modadd(u, v);
                assign s2[g+2] = modsub(u, v);
            end
        end
    endgenerate

    // Stage 2: half=4, m=8, shift=3, twiddle e=0,8,16,24
    wire [CW-1:0] s3 [0:63];
    generate
        for (g = 0; g < 64; g = g + 1) begin : g_st2
            if (g % 8 < 4) begin : b2
                localparam E2 = (g % 4) * 8;
                wire [CW-1:0] u = s2[g];
                wire [CW-1:0] v = modmul(s2[g+4], tw_rom[E2]);
                assign s3[g]   = modadd(u, v);
                assign s3[g+4] = modsub(u, v);
            end
        end
    endgenerate

    // Stage 3: half=8, m=16, shift=2, twiddle e=0,4,8,12,16,20,24,28
    wire [CW-1:0] s4 [0:63];
    generate
        for (g = 0; g < 64; g = g + 1) begin : g_st3
            if (g % 16 < 8) begin : b3
                localparam E3 = (g % 8) * 4;
                wire [CW-1:0] u = s3[g];
                wire [CW-1:0] v = modmul(s3[g+8], tw_rom[E3]);
                assign s4[g]   = modadd(u, v);
                assign s4[g+8] = modsub(u, v);
            end
        end
    endgenerate

    // Stage 4: half=16, m=32, shift=1, twiddle e=0,2,4,...,30
    wire [CW-1:0] s5 [0:63];
    generate
        for (g = 0; g < 64; g = g + 1) begin : g_st4
            if (g % 32 < 16) begin : b4
                localparam E4 = (g % 16) * 2;
                wire [CW-1:0] u = s4[g];
                wire [CW-1:0] v = modmul(s4[g+16], tw_rom[E4]);
                assign s5[g]   = modadd(u, v);
                assign s5[g+16] = modsub(u, v);
            end
        end
    endgenerate

    // Stage 5: half=32, m=64, shift=0, twiddle e=0,1,2,...,31
    wire [CW-1:0] s6 [0:63];
    generate
        for (g = 0; g < 64; g = g + 1) begin : g_st5
            if (g % 64 < 32) begin : b5
                localparam E5 = (g % 32);
                wire [CW-1:0] u = s5[g];
                wire [CW-1:0] v = modmul(s5[g+32], tw_rom[E5]);
                assign s6[g]    = modadd(u, v);
                assign s6[g+32] = modsub(u, v);
            end
        end
    endgenerate

    // Output
    generate
        for (g = 0; g < 64; g = g + 1) begin : g_out
            assign dout[g*CW +: CW] = s6[g];
        end
    endgenerate

endmodule