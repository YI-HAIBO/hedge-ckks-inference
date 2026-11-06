//=============================================================================
// ntt64_core_s.v -- SERIAL 64-point NTT over Z_q (step-1 cross transform)
//
// r37 follow-up for the KS_REAL=1 datapath.  The original ntt64_core is fully
// combinational: 192 modmuls in parallel per 64-point column, 5 cores per
// nptt_cross, 2 crosses in keyswitch_pipe -> 3.78M logic LUTs = 77% of the
// 4.92M ac_adapter total.  This version keeps ONE pipelined modmul per core
// and sequences the 192 butterflies through it:
//
//   * radix-2 DIT, 6 stages, 32 butterflies per stage (same indexing as the
//     combinational core: t = m*q + r, e = r << (5-stage))
//   * 4-stage pipeline: issue -> a*b -> Barrett step1 -> Barrett step2, with
//     one butterfly issued per cycle; 4 drain cycles between stages so every
//     in-flight writeback lands before the next stage reads the file
//   * bit-reversed load (rf[bitrev(g)] = din[g]), natural-order output
//   * latency ~207 cycles per column vs 2 cycles combinational; nptt_cross
//     runs one column per beat-lane, 26 columns/frame -> ~5.4k cycles/frame,
//     invisible next to the 100k+ cycle engine stage
//
// Bit-exact by construction: same Barrett two-step reduction (q = 2^47-2^20+1,
// C = 2^20-1) and the same twiddle ROM as the verified combinational core.
//
// Interface: start pulse latches din and begins; done pulses when dout holds
// the transformed column.
//=============================================================================
`include "ntt_twiddles.vh"

module ntt64_core_s #(
    parameter CW      = 48,
    parameter ROM_HEX = "ntt64_twiddles.hex"
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              start,        // 1-cycle pulse: latch din, begin
    input  wire [64*CW-1:0]  din,          // a[n1], natural order
    output wire [64*CW-1:0]  dout,         // A1[k1], natural order (valid on done)
    output reg               done          // 1-cycle pulse
);

    localparam [CW-1:0] Q    = `HEDGE_Q;
    localparam [CW-1:0] REDC = `HEDGE_REDC;

    //--------------------------------------------------------------------------
    // Modular helpers (identical arithmetic to ntt64_core.v / ac_modules_real).
    // qreduce is inlined in the pipe slots below (s2_t1 / s3_r) so the modmul
    // is split across the s0->s3 pipeline stages.
    //--------------------------------------------------------------------------
    function automatic [CW-1:0] modadd(input [CW-1:0] a, input [CW-1:0] b);
        reg [CW-1:0] s;
        begin
            s = a + b;
            modadd = (s >= Q) ? s - Q : s;
        end
    endfunction

    function automatic [CW-1:0] modsub(input [CW-1:0] a, input [CW-1:0] b);
        reg [CW-1:0] d;
        begin
            d = a - b;
            modsub = (a < b) ? d + Q : d;
        end
    endfunction

    function automatic [5:0] bitrev6(input [5:0] x);
        begin
            bitrev6 = {x[0], x[1], x[2], x[3], x[4], x[5]};
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
    // Working register file: natural index, bit-reversed load
    //--------------------------------------------------------------------------
    reg [CW-1:0] rf [0:63];
    integer m;

    genvar g;
    generate
        for (g = 0; g < 64; g = g + 1) begin : g_dout
            assign dout[g*CW +: CW] = rf[g];
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Butterfly schedule (combinational, derived from the flat butterfly
    // counter bf = stage*32 + j):
    //   stage s: half = 2^s, m = 2^(s+1), j = 0..31
    //   t = m*(j>>s) + (j & (half-1)),  twiddle index e = (j&(half-1))<<(5-s)
    //--------------------------------------------------------------------------
    localparam ST_IDLE = 2'd0;
    localparam ST_RUN  = 2'd1;
    localparam ST_FINI = 2'd2;

    reg [1:0]  st;
    reg [7:0]  bf;          // 0..191
    reg [2:0]  drain;       // inter-stage / final bubble counter

    wire [2:0] w_stg  = bf[7:5];
    wire [4:0] w_j    = bf[4:0];
    wire [5:0] w_half = 6'd1 << w_stg;
    wire [5:0] w_m    = w_half << 1;
    wire [5:0] w_q    = {1'b0, w_j} >> w_stg;
    wire [5:0] w_r    = {1'b0, w_j} & (w_half - 6'd1);
    wire [5:0] w_t    = w_m * w_q + w_r;                  // 0..63
    wire [5:0] w_shft = 6'd5 - {3'd0, w_stg};
    wire [5:0] w_e    = w_r << w_shft;                    // 0..31

    reg [CW-1:0] rd_u, rd_b;
    always @(*) begin
        rd_u = rf[w_t[5:0]];
        rd_b = rf[w_t[5:0] + w_half];
    end

    //--------------------------------------------------------------------------
    // 4-stage butterfly pipeline.
    //   issue@k -> s0 -> s1 (a*b) -> s2 (Barrett step1) -> s3 (Barrett step2)
    //   writeback at k+4: rf[t] <= u + v, rf[t+half] <= u - v
    // In-flight butterflies of one stage touch disjoint pairs, so a 4-cycle
    // drain between stages is all the ordering the file needs.
    //--------------------------------------------------------------------------
    reg        s0_v;
    reg [5:0]  s0_t;
    reg [2:0]  s0_s;
    reg [CW-1:0] s0_u;
    reg [CW-1:0] s0_b;      // modmul operand a (bottom path value)
    reg [CW-1:0] s0_tw;     // modmul operand b (twiddle)

    reg        s1_v;
    reg [5:0]  s1_t;
    reg [2:0]  s1_s;
    reg [CW-1:0] s1_u;
    reg [2*CW-1:0] s1_prod;

    reg        s2_v;
    reg [5:0]  s2_t;
    reg [2:0]  s2_s;
    reg [CW-1:0] s2_u;
    reg [67:0] s2_t1;

    reg        s3_v;
    reg [5:0]  s3_t;
    reg [2:0]  s3_s;
    reg [CW-1:0] s3_u;
    reg [CW-1:0] s3_r;

    // Barrett step2 of the s2 slot (combinational -> sampled into s3_r)
    wire [67:0] s2_hi2 = s2_t1[67:47];
    wire [46:0] s2_lo2 = s2_t1[46:0];
    wire [67:0] s2_t2  = s2_hi2 * REDC + s2_lo2;
    wire [CW-1:0] s2_red = (s2_t2[CW-1:0] >= Q) ? s2_t2[CW-1:0] - Q
                                                : s2_t2[CW-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st    <= ST_IDLE;
            done  <= 1'b0;
            bf    <= 8'd0;
            drain <= 3'd0;
            s0_v  <= 1'b0; s1_v <= 1'b0; s2_v <= 1'b0; s3_v <= 1'b0;
            for (m = 0; m < 64; m = m + 1)
                rf[m] <= {CW{1'b0}};
        end else begin
            done <= 1'b0;

            //----------------------------------
            // writeback from the deepest slot
            //----------------------------------
            if (s3_v) begin
                rf[s3_t]                   <= modadd(s3_u, s3_r);
                rf[s3_t + (6'd1 << s3_s)]  <= modsub(s3_u, s3_r);
            end

            //----------------------------------
            // pipeline advance
            //----------------------------------
            s3_v  <= s2_v;
            s3_t  <= s2_t;
            s3_s  <= s2_s;
            s3_u  <= s2_u;
            s3_r  <= s2_red;

            s2_v  <= s1_v;
            s2_t  <= s1_t;
            s2_s  <= s1_s;
            s2_u  <= s1_u;
            s2_t1 <= s1_prod[95:47] * REDC + s1_prod[46:0];

            s1_v    <= s0_v;
            s1_t    <= s0_t;
            s1_s    <= s0_s;
            s1_u    <= s0_u;
            s1_prod <= s0_b * s0_tw;

            //----------------------------------
            // issue + FSM
            //----------------------------------
            if (st == ST_RUN && drain == 3'd0) begin
                s0_v  <= 1'b1;
                s0_t  <= w_t[5:0];
                s0_s  <= w_stg[2:0];
                s0_u  <= rd_u;
                s0_b  <= rd_b;
                s0_tw <= tw_rom[w_e[4:0]];
                if (bf == 8'd191) begin
                    st    <= ST_FINI;
                    drain <= 3'd5;      // let the last 4 writebacks land
                end else begin
                    bf <= bf + 8'd1;
                    if (w_j == 5'd31)
                        drain <= 3'd4;  // inter-stage bubble
                end
            end else begin
                s0_v <= 1'b0;
                if (st == ST_RUN) begin
                    drain <= drain - 3'd1;
                end else if (st == ST_FINI) begin
                    if (drain != 3'd0)
                        drain <= drain - 3'd1;
                    else begin
                        done <= 1'b1;   // single-cycle pulse (cleared above)
                        st   <= ST_IDLE;
                    end
                end
            end

            //----------------------------------
            // load: start pulse (only safe in IDLE, pipeline empty)
            //----------------------------------
            if (st == ST_IDLE && start) begin
                for (m = 0; m < 64; m = m + 1)
                    rf[bitrev6(m[5:0])] <= din[m*CW +: CW];
                bf    <= 8'd0;
                drain <= 3'd0;
                st    <= ST_RUN;
            end
        end
    end

endmodule
