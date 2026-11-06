//=============================================================================
// File    : ntt_unit.v
// Project : 玄算（HEDGE） - A+C NTT / INTT engine (REAL implementation)
// Level   : Module Level
// Role    : rtl-designer (葛逻辑)
// Status  : v1.0 - real Cooley-Tukey radix-2 DIT, N=8192, q=2^47-2^20+1
//           Algorithm cross-checked against a schoolbook DFT on a small ring
//           (N=8) and by an N=8192 round trip INTT(NTT(a))==a in Python BEFORE
//           any RTL was written (see hedge-scripts/_gen_ntt_vectors.py).
//
// Purpose:
//   Forward / inverse number-theoretic transform over Z/qZ for the CKKS ring
//   R = Z_q[X]/(X^N + 1), N = 2^13 = 8192, q = 2^47 - 2^20 + 1 (prime).
//   This is the block that keyswitch_pipe needs for "all multiplications
//   happen in the NTT domain".
//
// Interface (Contract v1.0 §3.1, §A.1 -- identical to the stub):
//   AXI4-Stream in/out, ntt_dir (0=NTT, 1=INTT), start, status, busy, err.
//   START_LATENCY is retained from the frozen parameter list and is unused.
//
// Algorithm (must stay bit-identical to _gen_ntt_vectors.py):
//   load  : mem[bitrev(i)] = a[i]
//   DIT   : for s in 0..L-1 { half=2^s; m=2*half;
//             for k in 0..N-1 step m { for j in 0..half-1 {
//               w = omega^( j*2^(L-1-s) )          (inverse: omega^(-that))
//               u = mem[k+j]; v = mem[k+j+half]*w
//               mem[k+j]      = u+v   (mod q)
//               mem[k+j+half] = u-v   (mod q) } } }
//   out   : A[i] = mem[i]            (natural order)
//   INTT  : same with inverse twiddles, then every coeff *= N^-1 (mod q)
//
// Timing discipline (carried over from the six poly_mult_real closure rounds -
// every one of these was measured, none is decorative):
//   1. BRAM read addresses are REGISTERS, never combinational fan-out of a
//      counter; the data is taken from a second register stage (rd_qa/rd_qb).
//   2. A cycle carries at most ONE bare 48x48 multiply OR ONE Barrett half
//      step -- never both.  The butterfly is therefore 9 phases.
//   3. The twiddle is a ROM lookup (address = shifted index), not a running
//      recurrence: a recurrence would need modmul(w, wlen) per butterfly,
//      i.e. a second 48x48 multiply in the same loop.
//   4. Twiddle ROM is shared by both directions: omega^(-x) == omega^(N-x),
//      so the inverse only negates the address.  One 8192-entry table instead
//      of two.
//
// Storage:
//   memP, memQ  : 8192 x 48-bit ping-pong coefficient buffers (block RAM)
//                 13 stages is odd, so the transform result lands in memQ.
//   tw_rom      : 8192 x 48-bit omega^k table (block RAM, $readmemh)
//
// Related Documents:
//   - AC-HEDGE-Interface-Contract-v1.0.md §3.1, §A.1
//   - Keyswitch-Rescale-Design-v0.1.md §5.2
//
// Change Log:
//   v1.0  2026-10-06  葛逻辑  First real implementation (Plan C). Replaces the
//                 ac_modules_stub.v placeholder. Latency is NOT the Contract
//                 §6.1 target -- see the LATENCY note in the header body.
//=============================================================================

`default_nettype wire

`ifndef HEDGE_NTT_L
`include "ntt_twiddles.vh"
`endif

module ntt_unit #(
    parameter AXI_DATA_W    = 256,
    parameter COEF_WIDTH    = 48,
    parameter POLY_ORDER    = 13,               // 2^13 = 8192 coefficients
    parameter START_LATENCY = 3,                // Contract §A.1, unused
    // Twiddle ROM file.  A parameter (not a literal) so a small-ring
    // testbench can point at its own table; synthesis uses the default.
    parameter ROM_HEX       = "ntt_twiddle_rom.hex",
    // N^-1 mod q for THIS ring; 0 means "use HEDGE_NINV (the 2^13 inverse)".
    // A POLY_ORDER < 13 regression TB must pass its own N^-1 here.
    parameter [COEF_WIDTH-1:0] NINV_OVR = {COEF_WIDTH{1'b0}}
) (
    input  wire                     clk_main,
    input  wire                     rst_n,
    input  wire                     axi_s_axis_TVALID,
    input  wire [AXI_DATA_W-1:0]    axi_s_axis_TDATA,
    output wire                     axi_s_axis_TREADY,
    input  wire                     axi_s_axis_TLAST,
    output wire                     axi_m_axis_TVALID,
    output wire [AXI_DATA_W-1:0]    axi_m_axis_TDATA,
    input  wire                     axi_m_axis_TREADY,
    output wire                     axi_m_axis_TLAST,
    input  wire                     ntt_dir,         // 0=NTT, 1=INTT
    input  wire                     start,
    output wire [7:0]               status,
    output wire                     busy,
    output wire                     err
);

    //==================================================
    // Fixed geometry
    //==================================================
    localparam L              = POLY_ORDER;
    localparam N              = 1 << L;                    // 8192
    localparam CW             = COEF_WIDTH;                // 48
    localparam CPB            = AXI_DATA_W / CW;           // 5 coefs per beat
    localparam BEATS          = (N + CPB - 1) / CPB;       // 1639
    localparam HALF_N         = N >> 1;                    // 4096 butterflies/stage

    localparam [CW-1:0] Q     = `HEDGE_Q;
    // N^-1 mod q.  HEDGE_NINV is the inverse of 2^13, which is correct for the
    // production geometry.  A reduced-order regression ring (POLY_ORDER < 13)
    // must override it -- the header only carries the production value, and
    // using it on a smaller ring scales by 1/8192 instead of 1/N.  That is not
    // an RTL bug: it is the constant that belongs to a different ring.
    localparam [CW-1:0] NINV  = (NINV_OVR != {CW{1'b0}}) ? NINV_OVR : `HEDGE_NINV;
    localparam [CW-1:0] REDC  = `HEDGE_REDC;               // 2^20 - 1

    //==================================================
    // FSM
    //==================================================
    localparam [3:0] S_IDLE  = 4'd0;
    localparam [3:0] S_LOAD  = 4'd1;
    localparam [3:0] S_XFORM = 4'd2;
    localparam [3:0] S_SCALE = 4'd3;    // INTT only: multiply by N^-1
    localparam [3:0] S_STORE = 4'd4;
    localparam [3:0] S_DONE  = 4'd5;
    localparam [3:0] S_ERROR = 4'd6;

    reg [3:0] state_r;

    //==================================================
    // Storage
    //==================================================
    (* ram_style = "block" *)
    reg [CW-1:0] memP [0:N-1];
    (* ram_style = "block" *)
    reg [CW-1:0] memQ [0:N-1];
    (* ram_style = "block" *)
    reg [CW-1:0] tw_rom [0:N-1];

    initial $readmemh(ROM_HEX, tw_rom);

    //==================================================
    // Modular arithmetic over Z/qZ, q = 2^47 - 2^20 + 1.
    // Same division-free reduction as poly_mult_real: 2^47 == REDC (mod q),
    // so x = hi*2^47 + lo  =>  x == REDC*hi + lo (mod q), applied twice.
    // Inputs are always products of two residues < q < 2^47, so the 68-bit
    // intermediate cannot overflow (2^47 * 2^20 = 2^67).
    //==================================================
    function automatic [CW-1:0] qreduce;
        input [2*CW-1:0] x;
        reg [67:0] hi1, hi2, t1, t2;
        reg [CW-2:0] lo1, lo2;
        reg [CW-1:0] r;
        begin
            hi1 = x[2*CW-1:CW-1];
            lo1 = x[CW-2:0];
            t1  = hi1 * REDC + lo1;
            hi2 = t1[67:47];
            lo2 = t1[46:0];
            t2  = hi2 * REDC + lo2;
            r   = t2[CW-1:0];
            if (r >= Q) r = r - Q;
            qreduce = r;
        end
    endfunction

    // Split halves: one Barrett step per cycle (poly_mult_real closure rounds
    // showed the chained version alone was 34 logic levels / 9.455 ns).
    function automatic [67:0] qreduce_pre;
        input [2*CW-1:0] x;
        reg [67:0] hi1;
        reg [CW-2:0] lo1;
        begin
            hi1 = x[2*CW-1:CW-1];
            lo1 = x[CW-2:0];
            qreduce_pre = hi1 * REDC + lo1;
        end
    endfunction

    function automatic [CW-1:0] qreduce_post;
        input [67:0] t1;
        reg [67:0] hi2, t2;
        reg [CW-2:0] lo1;
        reg [CW-1:0] r;
        begin
            hi2 = t1[67:47];
            lo1 = t1[46:0];
            t2  = hi2 * REDC + lo1;
            r   = t2[CW-1:0];
            if (r >= Q) r = r - Q;
            qreduce_post = r;
        end
    endfunction

    function automatic [CW-1:0] modadd;
        input [CW-1:0] a;
        input [CW-1:0] b;
        reg [CW:0] s;                    // 49 bits, a+b < 2q < 2^48
        begin
            s = a + b;
            modadd = (s >= {1'b0, Q}) ? s[CW-1:0] - Q : s[CW-1:0];
        end
    endfunction

    function automatic [CW-1:0] modsub;
        input [CW-1:0] a;
        input [CW-1:0] b;
        reg [CW-1:0] d;
        begin
            d = a - b;                   // wraps when a < b
            modsub = (a < b) ? d + Q : d;
        end
    endfunction

    function automatic [L-1:0] bitrev;
        input [L-1:0] x;
        integer k;
        reg [L-1:0] r;
        begin
            r = {L{1'b0}};
            // Index the argument; never assign to it (some elaborators drop
            // writes to automatic-function inputs -- poly_mult_real hit this).
            for (k = 0; k < L; k = k + 1)
                r = (r << 1) | x[k];
            bitrev = r;
        end
    endfunction

    //==================================================
    // Datapath registers
    //==================================================
    reg                 res_bank;        // bank holding the current result
    reg                 dir_r;           // latched ntt_dir

    reg [L-1:0]         rd_addr_a;       // registered BRAM read address (port A)
    reg [L-1:0]         rd_addr_b;       // registered BRAM read address (port B)
    reg [L-1:0]         tw_addr_r;       // registered ROM read address
    reg [CW-1:0]        rd_qa, rd_qb;    // registered BRAM readout
    reg [CW-1:0]        tw_q;            // registered ROM readout

    reg [3:0]           stg;             // stage index 0..L-1
    reg [L-1:0]         k_r;             // group base
    reg [L-1:0]         j_r;             // index within group
    reg [3:0]           bph;             // butterfly phase 0..8

    reg [CW-1:0]        u_r, v_r, w_r;
    reg [2*CW-1:0]      vp_raw;
    reg [67:0]          vp_t1;
    reg [CW-1:0]        vp_q;
    reg [CW-1:0]        sum_r, dif_r;

    reg [13:0]          beat_cnt;        // 0..BEATS-1
    reg [2:0]           ldp;             // 0..CPB  (load lane / sub-phase)
    reg [AXI_DATA_W-1:0] beat_buf;
    reg                 l_got;           // beat captured

    reg [L-1:0]         sc_cnt;          // SCALE sweep index
    reg [2:0]           sc_ph;           // SCALE sub-phase 0..4

    reg [L-1:0]         st_base;         // first coeff index of this beat
    reg [2:0]           st_lane;         // 0..CPB-1
    reg [1:0]           st_ph;           // 0=issue 1=wait 2=capture 3=emit
    reg [CPB*CW-1:0]    pack;
    reg                 pack_v;
    reg                 st_last;
    reg                 st_emitted;      // beat completed, waiting for TREADY

    reg                 wr_en;
    reg                 wr_bank;
    reg [L-1:0]         wr_addr;
    reg [CW-1:0]        wr_data;

    reg                 err_r;

    //==================================================
    // Bank wiring
    //==================================================
    // XFORM ping-pongs: reads res_bank, writes dst_bank, and flips res_bank on
    // the last stage.  LOAD and STORE always work on res_bank.  S_SCALE is an
    // in-place sweep -- it reads and writes the SAME location sc_cnt, so it
    // must use res_bank for BOTH the read port and the write port.  v1.1 briefly
    // pointed src_bank at dst_bank during SCALE, which is wrong precisely
    // because res_bank has already flipped: dst_bank is then the stale bank,
    // and the rescaled coefficients land where STORE will never look.
    wire dst_bank = ~res_bank;
    wire src_bank = res_bank;

    //==================================================
    // Memory ports (two registered read ports + one write port)
    //==================================================
    always @(posedge clk_main) begin
        if (state_r == S_XFORM || state_r == S_LOAD || state_r == S_SCALE || state_r == S_STORE)
            rd_qa <= src_bank ? memQ[rd_addr_a] : memP[rd_addr_a];
    end

    always @(posedge clk_main) begin
        if (state_r == S_XFORM)
            rd_qb <= src_bank ? memQ[rd_addr_b] : memP[rd_addr_b];
    end

    always @(posedge clk_main) begin
        if (wr_en) begin
            if (wr_bank) memQ[wr_addr] <= wr_data;
            else         memP[wr_addr] <= wr_data;
        end
    end

    always @(posedge clk_main) begin
        tw_q <= tw_rom[tw_addr_r];
    end

    //==================================================
    // Twiddle address: omega^(j*2^(L-1-s)); inverse uses omega^(N - that)
    //==================================================
    wire [L-1:0] tw_idx_fwd = j_r << (L-1-stg);
    wire [L-1:0] tw_idx     = dir_r ? ((N - tw_idx_fwd) & (N-1)) : tw_idx_fwd;

    //==================================================
    // Outputs
    //==================================================
    assign busy   = (state_r != S_IDLE) && (state_r != S_DONE) && (state_r != S_ERROR);
    assign err    = err_r;
    assign status = {4'h0, state_r};

    assign axi_s_axis_TREADY = (state_r == S_LOAD) && (ldp == 3'd0);
    assign axi_m_axis_TVALID = pack_v && (state_r == S_STORE);
    assign axi_m_axis_TLAST  = pack_v && (state_r == S_STORE) && st_last;
    assign axi_m_axis_TDATA  = {{(AXI_DATA_W-CPB*CW){1'b0}}, pack};

    //==================================================
    // Main FSM
    //==================================================
    integer lane;
    reg [15:0] coef_idx;

    always @(posedge clk_main or negedge rst_n) begin
        if (!rst_n) begin
            state_r   <= S_IDLE;
            res_bank  <= 1'b0;
            dir_r     <= 1'b0;
            wr_en     <= 1'b0;
            err_r     <= 1'b0;
            beat_cnt  <= 14'd0;
            ldp       <= 3'd0;
            l_got     <= 1'b0;
            stg       <= 4'd0;
            k_r       <= {L{1'b0}};
            j_r       <= {L{1'b0}};
            bph       <= 4'd0;
            sc_cnt    <= {L{1'b0}};
            sc_ph     <= 3'd0;
            st_base   <= {L{1'b0}};
            st_lane   <= 3'd0;
            st_ph     <= 2'd0;
            pack      <= {(CPB*CW){1'b0}};
            pack_v    <= 1'b0;
            st_last   <= 1'b0;
            st_emitted <= 1'b0;
            rd_addr_a <= {L{1'b0}};
            rd_addr_b <= {L{1'b0}};
            tw_addr_r <= {L{1'b0}};
        end else begin
            wr_en <= 1'b0;               // default: one-cycle write pulse

            case (state_r)

            //-------------------------------------------------- IDLE
            S_IDLE: begin
                if (start) begin
                    dir_r    <= ntt_dir;
                    res_bank <= 1'b0;      // stimulus always lands in memP
                    beat_cnt <= 14'd0;
                    ldp      <= 3'd0;
                    l_got    <= 1'b0;
                    stg      <= 4'd0;
                    k_r      <= {L{1'b0}};
                    j_r      <= {L{1'b0}};
                    bph      <= 4'd0;
                    sc_cnt   <= {L{1'b0}};
                    sc_ph    <= 3'd0;
                    st_base  <= {L{1'b0}};
                    st_lane  <= 3'd0;
                    st_ph    <= 2'd0;
                    pack_v   <= 1'b0;
                    st_emitted <= 1'b0;
                    st_last  <= 1'b0;
                    state_r  <= S_LOAD;
                end
            end

            //-------------------------------------------------- LOAD
            // One beat carries CPB coefficients; the single write port needs
            // CPB cycles to deposit them, so the load advances CPB+1 cycles
            // per beat and TREADY is raised only in sub-phase 0.
            S_LOAD: begin
                if (ldp == 3'd0) begin
                    if (axi_s_axis_TVALID && axi_s_axis_TREADY) begin
                        beat_buf <= axi_s_axis_TDATA;
                        l_got    <= 1'b1;
                        ldp      <= 3'd1;
                        // Beat count / TLAST agreement is checked here: a
                        // short or long frame is a real error, not a warning.
                        if (axi_s_axis_TLAST && (beat_cnt != BEATS-1)) begin
                            err_r   <= 1'b1;
                            state_r <= S_ERROR;
                        end
                    end
                end else begin
                    coef_idx = beat_cnt * CPB + (ldp - 1);
                    if (coef_idx < N) begin
                        wr_en    <= 1'b1;
                        wr_bank  <= res_bank;
                        wr_addr  <= bitrev(coef_idx[L-1:0]);
                        wr_data  <= beat_buf[(ldp-1)*CW +: CW];
                    end
                    if (ldp == CPB) begin
                        ldp <= 3'd0;
                        if (beat_cnt == BEATS-1) begin
                            state_r <= S_XFORM;
                            stg     <= 4'd0;
                            k_r     <= {L{1'b0}};
                            j_r     <= {L{1'b0}};
                            bph     <= 4'd0;
                        end else begin
                            beat_cnt <= beat_cnt + 14'd1;
                        end
                    end else begin
                        ldp <= ldp + 3'd1;
                    end
                end
            end

            //-------------------------------------------------- XFORM
            // 9 phases per butterfly; each carries at most one 48x48 multiply
            // or one Barrett half step.
            S_XFORM: begin
                case (bph)
                    4'd0: begin
                        // Issue both read addresses and the twiddle address.
                        rd_addr_a <= k_r + j_r;
                        rd_addr_b <= k_r + j_r + (1 << stg);
                        tw_addr_r <= tw_idx;
                        bph       <= 4'd1;
                    end
                    4'd1: begin
                        bph <= 4'd2;                 // BRAM + ROM register stage
                    end
                    4'd2: begin
                        u_r <= rd_qa;
                        v_r <= rd_qb;
                        w_r <= tw_q;
                        bph <= 4'd3;
                    end
                    4'd3: begin
                        vp_raw <= v_r * w_r;         // the one bare 48x48
                        bph    <= 4'd4;
                    end
                    4'd4: begin
                        vp_t1 <= qreduce_pre(vp_raw);
                        bph   <= 4'd5;
                    end
                    4'd5: begin
                        vp_q <= qreduce_post(vp_t1);
                        bph  <= 4'd6;
                    end
                    4'd6: begin
                        sum_r <= modadd(u_r, vp_q);
                        dif_r <= modsub(u_r, vp_q);
                        bph   <= 4'd7;
                    end
                    4'd7: begin
                        wr_en   <= 1'b1;
                        wr_bank <= dst_bank;
                        wr_addr <= k_r + j_r;
                        wr_data <= sum_r;
                        bph     <= 4'd8;
                    end
                    default: begin                   // 4'd8
                        wr_en   <= 1'b1;
                        wr_bank <= dst_bank;
                        wr_addr <= k_r + j_r + (1 << stg);
                        wr_data <= dif_r;
                        bph     <= 4'd0;
                        // advance j / k / stage
                        if (j_r == (1 << stg) - 1) begin
                            j_r <= {L{1'b0}};
                            if (k_r == N - (1 << (stg+1))) begin
                                k_r     <= {L{1'b0}};
                                res_bank<= dst_bank;
                                if (stg == L-1) begin
                                    if (dir_r) begin
                                        state_r <= S_SCALE;
                                        sc_cnt  <= {L{1'b0}};
                                        sc_ph   <= 3'd0;
                                    end else begin
                                        state_r  <= S_STORE;
                                        st_base  <= {L{1'b0}};
                                        st_lane  <= 3'd0;
                                        st_ph    <= 2'd0;
                                        pack_v   <= 1'b0;
                                        st_emitted <= 1'b0;
                                    end
                                end else begin
                                    stg <= stg + 4'd1;
                                end
                            end else begin
                                k_r <= k_r + (1 << (stg+1));
                            end
                        end else begin
                            j_r <= j_r + {{(L-1){1'b0}}, 1'b1};
                        end
                    end
                endcase
            end

            //-------------------------------------------------- SCALE (INTT)
            // Every coefficient *= N^-1, in place: read sc_cnt, write sc_cnt,
            // same bank.  Kept as a separate sweep so the butterfly stays a
            // single-multiply datapath.  Do NOT flip res_bank here -- the
            // rescaled result is already where STORE expects it.
            S_SCALE: begin
                case (sc_ph)
                    3'd0: begin
                        rd_addr_a <= sc_cnt;
                        sc_ph     <= 3'd1;
                    end
                    3'd1: begin
                        sc_ph <= 3'd2;               // BRAM register stage
                    end
                    3'd2: begin
                        vp_raw <= rd_qa * NINV;
                        sc_ph  <= 3'd3;
                    end
                    3'd3: begin
                        vp_t1 <= qreduce_pre(vp_raw);
                        sc_ph <= 3'd4;
                    end
                    default: begin                   // 3'd4
                        wr_en   <= 1'b1;
                        wr_bank <= res_bank;
                        wr_addr <= sc_cnt;
                        wr_data <= qreduce_post(vp_t1);
                        sc_ph   <= 3'd0;
                        if (sc_cnt == N-1) begin
                            state_r  <= S_STORE;
                            st_base  <= {L{1'b0}};
                            st_lane  <= 3'd0;
                            st_ph    <= 2'd0;
                            pack_v   <= 1'b0;
                            st_emitted <= 1'b0;
                        end else begin
                            sc_cnt <= sc_cnt + {{(L-1){1'b0}}, 1'b1};
                        end
                    end
                endcase
            end

            //-------------------------------------------------- STORE
            // Three cycles per coefficient lane: issue the address, wait out
            // the BRAM read latency, then capture.  v1.0 used two phases
            // (issue / capture) and therefore captured the PREVIOUS lane's
            // word every time -- a one-cycle pipeline error, exactly the class
            // of bug the 9-phase butterfly in S_XFORM was built to avoid.
            S_STORE: begin
                if (!pack_v) begin
                    case (st_ph)
                        2'd0: begin
                            rd_addr_a <= st_base + st_lane;
                            st_ph     <= 2'd1;
                        end
                        2'd1: begin
                            st_ph <= 2'd2;               // BRAM register stage
                        end
                        2'd2: begin
                            coef_idx = st_base + st_lane;
                            if (coef_idx < N)
                                pack[st_lane*CW +: CW] <= rd_qa;
                            else
                                pack[st_lane*CW +: CW] <= {CW{1'b0}};
                            st_ph <= 2'd3;
                        end
                        default: begin                // 2'd3: close the beat
                            if (st_lane == CPB-1) begin
                                pack_v     <= 1'b1;
                                st_last    <= (st_base + CPB >= N);
                                st_lane    <= 3'd0;
                                st_emitted <= 1'b1;
                            end else begin
                                st_lane <= st_lane + 3'd1;
                            end
                            st_ph <= 2'd0;
                        end
                    endcase
                end else if (axi_m_axis_TREADY) begin
                    pack_v <= 1'b0;
                    if (st_last) begin
                        state_r <= S_DONE;
                    end else begin
                        st_base <= st_base + CPB[L-1:0];
                    end
                end
            end

            //-------------------------------------------------- DONE
            S_DONE: begin
                state_r <= S_IDLE;
            end

            //-------------------------------------------------- ERROR
            S_ERROR: begin
                // Terminal; requires reset to re-arm.
            end

            default: state_r <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
