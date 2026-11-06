//=============================================================================
// tb_keyswitch_m35.v -- M3.5: k-digit golden test for keyswitch_pipe v0.8
//
// Purpose
//   End-to-end numeric verification of the M3.5 digit-serial keyswitch:
//   the DUT computes, over Z_q (q = 2^47 - 2^20 + 1 = 140737487306753),
//
//       c1' = ( c1 + SUM_{d<k} INTT( NTT(key_d) * NTT(dig_d) ) ) mod q
//
//   with NUM_DIGITS = k (host-side decomposition, M3.5 decision).  The
//   digit streams are consumed SERIALLY: after each FRAME_IN-beat digit
//   segment the FSM drops axi_s_axis_TREADY, runs the NTT round for that
//   digit, accumulates the pointwise term, and re-asserts TREADY for the
//   next segment -- a streaming host is interleaved purely by
//   backpressure, which this TB exercises by driving the whole
//   (1+k)*FRAME_IN stream without waiting between segments.
//
//   k is selected at compile time: iverilog -DKS_K=4 selects k=4; the
//   default is k=2 (the M3.5 default).  All three k values share ONE
//   golden vector set (seed 20261024) -- the k-digit golden is the sum
//   truncated to the first k terms -- so the three regressions are
//   mutually independent checks of the same vector pool.
//
// Golden source
//   hedge-scripts/gen_keyswitch_m35_golden.py, seed 20261024.  Vectors are
//   produced by hedge-tb/golden_ntt.py and cross-checked against
//   hedge-scripts/four_step_ntt_ref.py for every k before being dumped.
//
// Test plan
//   TC0  zero-key identity: all k key digits zero -> c1' == c1
//   TC2  k-frame key_load handshake, key_loaded_r latches on the LAST frame
//   TC3  full keyswitch: (1+k) x FRAME_IN input beats, 8192-coefficient
//        golden comparison, FRAME_IN output beats, exactly one TLAST,
//        exactly k NTT rounds observed (digit-loop check)
//   TC4  error: start without a loaded key -> E_KEY_NOT_LOADED (0x12)
//   TC5  error: digit_num_zero -> E_MODCHAIN_EXHAUSTED (0x10)
//
// Handshake discipline (the 2026-10-19 iverilog lesson): every TB output
// the DUT samples at a posedge (TVALID / TDATA / TLAST / key_load_*) is
// assigned at least #1 AFTER the @(posedge clk) resume point.
//
// Run:  python .workbuddy/run_ks_m35.py           (k=2, k=4)
//=============================================================================
`timescale 1ns / 1ps
`default_nettype wire

module tb_keyswitch_m35;

    // k (NUM_DIGITS) is a COMPILE-TIME select: iverilog -DKS_K=4 runs the
    // k=4 regression; the default is the M3.5 default k=2.
`ifdef KS_K
    localparam K = `KS_K;
`else
    localparam K = 2;
`endif

    //==================================================
    // Geometry (NPAR=64, 256-bit wrapper beats)
    //==================================================
    localparam N1            = 64;
    localparam N2            = 128;
    localparam N             = N1 * N2;                 // 8192
    localparam COEF_W        = 48;
    localparam DATA_W        = 256;
    localparam CPB           = DATA_W / COEF_W;         // 5
    localparam FRAME_IN      = (N + CPB - 1) / CPB;     // 1639 natural beats
    localparam INPUT_BEATS   = (1 + K) * FRAME_IN;      // c1 then k digits
    localparam KMAX_FILES    = 4;                       // vector pool depth
    localparam BEATS_PER_ENG = (N2 + CPB - 1) / CPB;    // 26
    localparam TIMEOUT       = 40_000_000;              // ns hard watchdog
    localparam BASE = "C:/Users/Yi/WorkBuddy/2026-10-03-09-14-19/hedge-rtl/sub_a_plus_c/";

    //==================================================
    // Clock & reset (100 MHz)
    //==================================================
    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    //==================================================
    // 256-bit DUT interface (real datapath)
    //==================================================
    reg  [DATA_W-1:0] s_TDATA  = {DATA_W{1'b0}};
    wire             s_TREADY;
    reg              s_TVALID = 1'b0;
    reg              s_TLAST  = 1'b0;

    wire [DATA_W-1:0] m_TDATA;
    wire             m_TVALID;
    wire             m_TLAST;
    reg              m_TREADY = 1'b1;

    reg              kl_start = 1'b0;
    reg              kl_valid = 1'b0;
    wire             kl_ready;
    reg  [DATA_W-1:0] kl_TDATA = {DATA_W{1'b0}};

    reg              start      = 1'b0;
    reg              digit_zero = 1'b0;
    reg              stream_en  = 1'b0;
    wire [7:0]       status;
    wire             busy;
    wire             err;
    wire [31:0]      err_code;

    keyswitch_pipe #(
        .AXI_DATA_W      (DATA_W),
        .COEF_WIDTH      (COEF_W),
        .NUM_ENGINES     (N1),
        .COEFS_PER_ENGINE(N2),
        .REAL_DATAPATH   (1),
        .NUM_DIGITS      (K)
    ) dut (
        .clk_main            (clk),
        .rst_n               (rst_n),
        .axi_s_axis_TVALID   (s_TVALID),
        .axi_s_axis_TDATA    (s_TDATA),
        .axi_s_axis_TREADY   (s_TREADY),
        .axi_s_axis_TLAST    (s_TLAST),
        .axi_m_axis_TVALID   (m_TVALID),
        .axi_m_axis_TDATA    (m_TDATA),
        .axi_m_axis_TREADY   (m_TREADY),
        .axi_m_axis_TLAST    (m_TLAST),
        .key_load_start      (kl_start),
        .key_load_valid      (kl_valid),
        .key_load_ready      (kl_ready),
        .key_load_TDATA      (kl_TDATA),
        .start               (start),
        .status              (status),
        .busy                (busy),
        .err                 (err),
        .err_code            (err_code),
        .stream_enable       (stream_en),
        .digit_num_zero      (digit_zero)
    );

    //==================================================
    // Test data
    //==================================================
    // iverilog rejects $readmemh on a ROW of a 2-D array ("needs 2
    // indices"), so the four key / digit vectors are flat 1-D memories
    // reached through the key_get / dig_get accessors below.
    reg [COEF_W-1:0] key0 [0:N-1];   // NTT(key_0)
    reg [COEF_W-1:0] key1 [0:N-1];   // NTT(key_1)
    reg [COEF_W-1:0] key2 [0:N-1];   // NTT(key_2)
    reg [COEF_W-1:0] key3 [0:N-1];   // NTT(key_3)
    reg [COEF_W-1:0] dig0 [0:N-1];   // digit 0 of the decomposed c2
    reg [COEF_W-1:0] dig1 [0:N-1];   // digit 1
    reg [COEF_W-1:0] dig2 [0:N-1];   // digit 2
    reg [COEF_W-1:0] dig3 [0:N-1];   // digit 3

    function [COEF_W-1:0] key_get(input integer d, input integer i);
        begin
            case (d)
                0:  key_get = key0[i];
                1:  key_get = key1[i];
                2:  key_get = key2[i];
                default: key_get = key3[i];
            endcase
        end
    endfunction

    function [COEF_W-1:0] dig_get(input integer d, input integer i);
        begin
            case (d)
                0:  dig_get = dig0[i];
                1:  dig_get = dig1[i];
                2:  dig_get = dig2[i];
                default: dig_get = dig3[i];
            endcase
        end
    endfunction
    reg [COEF_W-1:0] c1_poly  [0:N-1];
    reg [COEF_W-1:0] exp_poly [0:N-1];                    // expected c1'

    // Expected index-0 values of the committed vector set (seed 20261024),
    // printed by hedge-scripts/gen_keyswitch_m35_golden.py.  These pin the
    // TB to that exact generation run per k: a missing, stubbed or STALE
    // hex file fails here instead of producing a vacuous numeric pass.
    localparam [COEF_W-1:0] EXP_C1_0 = 48'h169496b7e0b4;
    localparam [COEF_W-1:0] EXP_OUT_0 =
        (K == 1) ? 48'h617d24fada89 :
        (K == 2) ? 48'h7ed304a6f852 :
                   48'h65cf40eda527;   // K == 4

    integer nnz_key = 0;
    integer nnz_c1  = 0;
    integer nnz_dig = 0;
    integer nnz_out = 0;

    //==================================================
    // TC0 sanity switch (all k key digits zero -> c1' == c1)
    //==================================================
    reg     zero_key_mode   = 1'b0;
    reg     expect_identity = 1'b0;

    initial begin
        integer j, d;
        $readmemh({BASE, "ks_m35_key_d0.hex"}, key0);
        $readmemh({BASE, "ks_m35_key_d1.hex"}, key1);
        $readmemh({BASE, "ks_m35_key_d2.hex"}, key2);
        $readmemh({BASE, "ks_m35_key_d3.hex"}, key3);
        $readmemh({BASE, "ks_m35_dig_d0.hex"}, dig0);
        $readmemh({BASE, "ks_m35_dig_d1.hex"}, dig1);
        $readmemh({BASE, "ks_m35_dig_d2.hex"}, dig2);
        $readmemh({BASE, "ks_m35_dig_d3.hex"}, dig3);
        $readmemh({BASE, "ks_m35_c1.hex"},  c1_poly);
        if (K == 1)
            $readmemh({BASE, "ks_m35_out_k1.hex"}, exp_poly);
        else if (K == 2)
            $readmemh({BASE, "ks_m35_out_k2.hex"}, exp_poly);
        else
            $readmemh({BASE, "ks_m35_out_k4.hex"}, exp_poly);
        // $readmemh FAILS SILENTLY on a missing file and leaves the array at
        // its declared value (0).  Counting non-zero entries catches a
        // stubbed set; note $readmemh has no return value in iverilog.
        for (j = 0; j < N; j = j + 1) begin
            if (c1_poly[j]  != {COEF_W{1'b0}}) nnz_c1  = nnz_c1  + 1;
            if (exp_poly[j] != {COEF_W{1'b0}}) nnz_out = nnz_out + 1;
            for (d = 0; d < KMAX_FILES; d = d + 1) begin
                if (key_get(d, j) != {COEF_W{1'b0}}) nnz_key = nnz_key + 1;
                if (dig_get(d, j) != {COEF_W{1'b0}}) nnz_dig = nnz_dig + 1;
            end
        end
    end

    //==================================================
    // Result tallies + digit-loop monitor
    //==================================================
    integer pass_cnt    = 0;
    integer fail_cnt    = 0;
    integer tc          = 0;
    integer mism_cnt    = 0;
    integer m_beats     = 0;
    integer m_got_cnt   = 0;
    integer m_last_cnt  = 0;
    integer c1_mism_cnt = 0;
    // Count the S_NTT_FWD (state 3) entries: exactly K rounds must happen,
    // one per digit.  Too few means the digit loop short-circuited; too
    // many means it never terminated.
    reg [7:0] prev_status = 8'd0;
    integer   ntt_rounds  = 0;
    integer   combine_visits = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            if (prev_status != 8'd3 && status === 8'd3)
                ntt_rounds = ntt_rounds + 1;
            if (prev_status != 8'd5 && status === 8'd5) begin
                combine_visits = combine_visits + 1;
                // [DEBUG M3.5] dump the round state at every S_COMBINE entry
                $display("  [ROUND %0d] ntt_out_bram[0]=0x%064H", combine_visits, dut.ntt_out_bram[0]);
                $display("           key_bank=%0d pw_out_bram[0]=0x%064H", dut.proc_digit_r, dut.pw_out_bram[0]);
                $display("           in_bram[0]=0x%064H", dut.in_bram[0]);
                // [DEBUG M3.5] full-frame dumps for offline pattern analysis.
                // Compile with -DKS_DBG_DUMP to enable; the golden runs keep
                // them off so the testbench does not litter the run directory.
`ifdef KS_DBG_DUMP
                if (combine_visits == 1) $writememh("dbg_ntt_out_r1.hex", dut.ntt_out_bram);
                if (combine_visits == 2) $writememh("dbg_ntt_out_r2.hex", dut.ntt_out_bram);
                if (combine_visits == 3) $writememh("dbg_ntt_out_r3.hex", dut.ntt_out_bram);
                if (combine_visits == 4) $writememh("dbg_ntt_out_r4.hex", dut.ntt_out_bram);
`endif
            end
            prev_status <= status;
        end
    end

    integer k;
    integer waitd;
    integer lane_i;
    integer gidx_i;

    task automatic check_pass(input [255:0] name, input cond);
        begin
            tc = tc + 1;
            if (cond) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] TC%0d: %s", tc, name);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] TC%0d: %s", tc, name);
            end
        end
    endtask

    //==================================================
    // Pack one natural-order 256-bit beat.
    //   key frames   : beat j of digit d holds key_poly_d[d][j*CPB+l]
    //   input stream : global beat j; segment 0 (j < FRAME_IN) is c1,
    //                  segment 1+d (j >= FRAME_IN) is dig_poly_d[d], with
    //                  d = (j / FRAME_IN) - 1 and the coefficient index
    //                  restarting at 0 inside every segment.  Lanes past
    //                  the last real coefficient of a segment are zero
    //                  (the last natural beat carries only N mod CPB = 2).
    //==================================================
    task automatic pack_key_beat(input integer d, input integer j,
                                 output reg [DATA_W-1:0] beat);
        integer l, gi;
        begin
            beat = {DATA_W{1'b0}};
            for (l = 0; l < CPB; l = l + 1) begin
                gi = j * CPB + l;
                if (gi < N && !zero_key_mode)
                    beat[l*COEF_W +: COEF_W] = key_get(d, gi);
            end
        end
    endtask

    task automatic pack_in_beat(input integer j,
                                output reg [DATA_W-1:0] beat);
        integer l, gi, d, src;
        reg [COEF_W-1:0] c;
        begin
            beat = {DATA_W{1'b0}};
            src = j % FRAME_IN;              // beat inside the segment
            d   = (j / FRAME_IN) - 1;        // -1 = c1 segment
            for (l = 0; l < CPB; l = l + 1) begin
                gi = src * CPB + l;
                c  = {COEF_W{1'b0}};
                if (gi < N) begin
                    if (d < 0)
                        c = c1_poly[gi];
                    else
                        c = dig_get(d, gi);
                end
                beat[l*COEF_W +: COEF_W] = c;
            end
        end
    endtask

    //==================================================
    // Drive one 256-bit beat through a valid/ready handshake.
    //   The DUT drops TREADY between digit segments (M3.5 backpressure);
    //   the while loop parks on that, holding TVALID and the CURRENT
    //   beat on the axis until the round finishes.
    //==================================================
    task automatic send_beat(input integer j, input integer last_of_stream);
        reg [DATA_W-1:0] b;
        begin
            pack_in_beat(j, b);
            #1;
            s_TDATA  = b;
            s_TLAST  = last_of_stream ? 1'b1 : 1'b0;
            s_TVALID = 1'b1;
            @(posedge clk);
            while (!s_TREADY) @(posedge clk);
            #1;
            s_TVALID = 1'b0;
            s_TLAST  = 1'b0;
        end
    endtask

    //==================================================
    // Drive the full k-frame key load (one frame per digit).
    //   Frame d fills key_bram_a[d]; key_loaded_r latches on the last
    //   beat of frame K-1, returning the FSM to S_IDLE.
    //==================================================
    task automatic load_keys;
        integer d, kk;
        reg [DATA_W-1:0] b;
        begin
            for (d = 0; d < K; d = d + 1) begin
                for (kk = 0; kk < FRAME_IN; kk = kk + 1) begin
                    pack_key_beat(d, kk, b);
                    #1;
                    kl_TDATA = b;
                    kl_valid = 1'b1;
                    @(posedge clk);
                    while (!kl_ready) @(posedge clk);
                    #1;
                    kl_valid = 1'b0;
                end
            end
        end
    endtask

    //==================================================
    // Poll the FSM to a given state id
    //==================================================
    task automatic wait_state(input integer st, output integer waited);
        begin
            waited = 0;
            while (status !== st[7:0]) begin
                @(posedge clk);
                waited = waited + 1;
                if (waited > 5_000_000) begin
                    $display("  [WARN] wait_state(%0d) timed out, status=0x%02h", st, status);
                    break;
                end
            end
        end
    endtask

    //==================================================
    // Output monitor (separate process, never races the drive tasks)
    //==================================================
    reg [COEF_W-1:0] got_c;

    always @(posedge clk) begin
        if (rst_n && m_TVALID && m_TREADY) begin
            m_beats    = m_beats + 1;
            if (m_TLAST) m_last_cnt = m_last_cnt + 1;

            if (m_beats <= FRAME_IN) begin
                for (lane_i = 0; lane_i < CPB; lane_i = lane_i + 1) begin
                    gidx_i = (m_beats - 1) * CPB + lane_i;
                    got_c  = m_TDATA[lane_i*COEF_W +: COEF_W];
                    if (gidx_i < N) begin
                        if (expect_identity) begin
                            if (got_c !== c1_poly[gidx_i]) begin
                                if (c1_mism_cnt < 24)
                                    $display("  ID-MISMATCH beat=%0d lane=%0d gidx=%0d: got=0x%012H exp_c1=0x%012H",
                                             m_beats - 1, lane_i, gidx_i, got_c, c1_poly[gidx_i]);
                                c1_mism_cnt = c1_mism_cnt + 1;
                            end
                        end else begin
                            if (got_c !== exp_poly[gidx_i]) begin
                                if (mism_cnt < 24)
                                    $display("  MISMATCH beat=%0d lane=%0d gidx=%0d: got=0x%012H exp=0x%012H",
                                             m_beats - 1, lane_i, gidx_i, got_c, exp_poly[gidx_i]);
                                mism_cnt = mism_cnt + 1;
                            end
                        end
                        m_got_cnt = m_got_cnt + 1;
                    end else if (got_c !== {COEF_W{1'b0}}) begin
                        // padding lane of the final natural beat must be 0
                        if (expect_identity) begin
                            if (c1_mism_cnt < 24)
                                $display("  PAD-LANE beat=%0d lane=%0d gidx=%0d got=0x%012H (expected 0)",
                                         m_beats - 1, lane_i, gidx_i, got_c);
                            c1_mism_cnt = c1_mism_cnt + 1;
                        end else begin
                            if (mism_cnt < 24)
                                $display("  PAD-LANE beat=%0d lane=%0d gidx=%0d got=0x%012H (expected 0)",
                                         m_beats - 1, lane_i, gidx_i, got_c);
                            mism_cnt = mism_cnt + 1;
                        end
                    end
                end
            end else if (expect_identity) begin
                if (c1_mism_cnt < 24)
                    $display("  EXTRA OUT BEAT %0d (expected %0d)", m_beats, FRAME_IN);
                c1_mism_cnt = c1_mism_cnt + 1;
            end else if (mism_cnt < 24) begin
                $display("  EXTRA OUT BEAT %0d (expected %0d)", m_beats, FRAME_IN);
                mism_cnt = mism_cnt + 1;
            end
        end
    end

    //==================================================
    // Watchdog
    //==================================================
    initial begin
        #TIMEOUT;
        $display("[TIMEOUT] exceeded %0d ns", TIMEOUT);
        $display("  K=%0d m_beats=%0d/%0d  m_got=%0d/%0d  mism=%0d  status=0x%02h err=%b code=0x%08h",
                 K, m_beats, FRAME_IN, m_got_cnt, N, mism_cnt, status, err, err_code);
        $display("  ntt_rounds=%0d (expect %0d)  combine_visits=%0d",
                 ntt_rounds, K, combine_visits);
        $finish;
    end

    //==================================================
    // Main sequence
    //==================================================
    initial begin
        $display("============================================================");
        $display("tb_keyswitch_m35: M3.5 k-digit golden test, keyswitch_pipe v0.8");
        $display("  K=%0d  N=%0d  q=0x%012X  CPB=%0d  FRAME_IN=%0d  INPUT_BEATS=%0d",
                 K, N, 140737487306753, CPB, FRAME_IN, INPUT_BEATS);
        $display("============================================================");

        // ------------------------------------------------------------
        // Reset
        // ------------------------------------------------------------
        rst_n      = 1'b0;
        kl_start   = 1'b0;
        kl_valid   = 1'b0;
        start      = 1'b0;
        digit_zero = 1'b0;
        stream_en  = 1'b0;
        s_TVALID   = 1'b0;
        s_TLAST    = 1'b0;
        m_TREADY   = 1'b1;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // ------------------------------------------------------------
        // Golden-vector guard: refuse to run on a stubbed or stale set.
        // ------------------------------------------------------------
        if (nnz_key < KMAX_FILES*N/2 || nnz_c1 < N/2 || nnz_dig < KMAX_FILES*N/2
            || nnz_out < N/2) begin
            $display("[FATAL] golden vectors look stubbed: nnz key=%0d c1=%0d dig=%0d out=%0d",
                     nnz_key, nnz_c1, nnz_dig, nnz_out);
            $display("        fix: python hedge-scripts/gen_keyswitch_m35_golden.py");
            $finish;
        end
        if (c1_poly[0] !== EXP_C1_0 || exp_poly[0] !== EXP_OUT_0) begin
            $display("[FATAL] golden vectors are not the seed-20261024 set for K=%0d", K);
            $display("        c1[0]=0x%012H (expect 0x%012H)  out[0]=0x%012H (expect 0x%012H)",
                     c1_poly[0], EXP_C1_0, exp_poly[0], EXP_OUT_0);
            $finish;
        end
        $display("  golden vectors OK: c1[0]=0x%012H out[0]=0x%012H",
                 c1_poly[0], exp_poly[0]);

        // ------------------------------------------------------------
        // TC1: no X after reset
        // ------------------------------------------------------------
        $display("--- TC1: no X after reset ---");
        check_pass("busy/err/status not X",
                   (busy !== 1'bx) && (err !== 1'bx) && (status !== 8'bx));
        check_pass("start in S_IDLE", status === 8'd0);
        check_pass("not busy at idle", busy === 1'b0);

        // ------------------------------------------------------------
        // TC4: error path -- start without a loaded key
        // ------------------------------------------------------------
        $display("--- TC4: start without key -> E_KEY_NOT_LOADED ---");
        #1;
        start = 1'b1;
        @(posedge clk);
        #1;
        start = 1'b0;
        repeat (3) @(posedge clk);
        check_pass("err raised",    err === 1'b1);
        check_pass("code 0x12",     err_code === 32'h0000_0012);
        check_pass("state S_ERROR", status === 8'd15);

        // ------------------------------------------------------------
        // TC5: error path -- digit_num_zero
        // ------------------------------------------------------------
        $display("--- TC5: digit_num_zero -> E_MODCHAIN_EXHAUSTED ---");
        #1;
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        #1;
        digit_zero = 1'b1;
        start      = 1'b1;
        @(posedge clk);
        #1;
        start      = 1'b0;
        digit_zero = 1'b0;
        repeat (3) @(posedge clk);
        check_pass("err raised",    err === 1'b1);
        check_pass("code 0x10",     err_code === 32'h0000_0010);
        check_pass("state S_ERROR", status === 8'd15);

        // ------------------------------------------------------------
        // Clean slate
        // ------------------------------------------------------------
        #1;
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        m_beats = 0; m_got_cnt = 0; m_last_cnt = 0; mism_cnt = 0; c1_mism_cnt = 0;
        ntt_rounds = 0; combine_visits = 0; prev_status = 8'd0;

        // ------------------------------------------------------------
        // TC0: SANITY -- all k key digits zero -> c1' == c1 (identity).
        //   pw accumulates k zero terms, INTT(0) = 0, so acc = c1.
        // ------------------------------------------------------------
        $display("--- TC0: zero-key identity (c1' == c1), K=%0d ---", K);

        expect_identity = 1'b1;

        #1;
        kl_start = 1'b1;
        @(posedge clk);
        #1;
        kl_start = 1'b0;
        wait_state(1, waitd);

        zero_key_mode = 1'b1;
        load_keys;
        zero_key_mode = 1'b0;
        wait_state(0, waitd);
        check_pass("TC0: k key frames loaded, back to S_IDLE", status === 8'd0);

        #1;
        start = 1'b1;
        @(posedge clk);
        #1;
        start = 1'b0;
        wait_state(2, waitd);
        check_pass("TC0: reached S_LOAD_INPUT", status === 8'd2);

        for (k = 0; k < INPUT_BEATS; k = k + 1)
            send_beat(k, (k == INPUT_BEATS - 1));

        wait_state(9, waitd);
        check_pass("TC0: reached S_DONE",       status === 8'd9);
        check_pass("TC0: err stayed clear",     err === 1'b0);
        wait_state(0, waitd);
        check_pass("TC0: returned to S_IDLE",   status === 8'd0);

        repeat (5) @(posedge clk);

        $display("--- TC0 detail ---");
        $display("  m_beats=%0d  m_got=%0d  c1_mism=%0d  TLAST=%0d  ntt_rounds=%0d",
                 m_beats, m_got_cnt, c1_mism_cnt, m_last_cnt, ntt_rounds);
        check_pass("TC0: beats == FRAME_IN",   m_beats   === FRAME_IN);
        check_pass("TC0: coefs == N",          m_got_cnt === N);
        check_pass("TC0: c1_mism == 0",        c1_mism_cnt === 0);
        check_pass("TC0: k NTT rounds ran",    ntt_rounds === K);

        expect_identity = 1'b0;

        // Reset again for TC2/TC3 (the real key run).
        #1;
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        m_beats = 0; m_got_cnt = 0; m_last_cnt = 0; mism_cnt = 0;
        ntt_rounds = 0; combine_visits = 0; prev_status = 8'd0;

        // ------------------------------------------------------------
        // TC2: k-frame key-load handshake
        // ------------------------------------------------------------
        $display("--- TC2: k-frame key_load_start -> S_LOAD_KEY ---");
        #1;
        kl_start = 1'b1;
        @(posedge clk);
        #1;
        kl_start = 1'b0;
        wait_state(1, waitd);
        check_pass("reached S_LOAD_KEY",      status === 8'd1);
        check_pass("key_load_ready asserted", kl_ready === 1'b1);

        // ------------------------------------------------------------
        // TC3: the real k-digit keyswitch
        // ------------------------------------------------------------
        $display("--- TC3: real k-digit keyswitch, 8192-coef golden compare ---");
        load_keys;
        wait_state(0, waitd);
        check_pass("k key frames loaded, back to S_IDLE", status === 8'd0);

        #1;
        start = 1'b1;
        @(posedge clk);
        #1;
        start = 1'b0;
        wait_state(2, waitd);
        check_pass("reached S_LOAD_INPUT", status === 8'd2);

        // (1 + K) x FRAME_IN beats, TLAST only on the very last.  The
        // send_beat loop rides the M3.5 backpressure between digits.
        for (k = 0; k < INPUT_BEATS; k = k + 1)
            send_beat(k, (k == INPUT_BEATS - 1));
        check_pass("fed (1+K) x FRAME_IN input beats", k === INPUT_BEATS);

        wait_state(9, waitd);
        check_pass("reached S_DONE",    status === 8'd9);
        check_pass("err stayed clear",  err === 1'b0);
        wait_state(0, waitd);
        check_pass("returned to S_IDLE", status === 8'd0);

        // Let the monitor drain any late beat.
        repeat (5) @(posedge clk);

        check_pass("output beats == FRAME_IN", m_beats   === FRAME_IN);
        check_pass("coefficients == N",        m_got_cnt === N);
        check_pass("exactly one TLAST",        m_last_cnt === 1);
        check_pass("0 / 8192 mismatches",      mism_cnt === 0);
        check_pass("k NTT rounds ran",         ntt_rounds === K);
        check_pass("k S_COMBINE visits",       combine_visits === K);

        $display("--- TC3 detail ---");
        $display("  m_beats=%0d  m_got=%0d  mism=%0d  TLAST=%0d  ntt_rounds=%0d  combine=%0d",
                 m_beats, m_got_cnt, mism_cnt, m_last_cnt, ntt_rounds, combine_visits);
        // iverilog rejects a variable part-select on a memory element, so
        // whole 256-bit beats are printed (the low 2 lanes carry gidx 0/5).
        $display("  proc_digit (final)=%0d  load_phase=%0d",
                 dut.proc_digit_r, dut.load_phase_r);
        $display("  key_bram_a[0][0]    : 0x%064H", dut.key_bram_a[0][0]);
        $display("  key_bram_a[K-1][0]  : 0x%064H", dut.key_bram_a[K-1][0]);
        $display("  pw_out_bram[0]      : 0x%064H", dut.pw_out_bram[0]);
        $display("  acc_bram[0]         : 0x%064H", dut.acc_bram[0]);
        $display("  exp_poly @g0,g5: 0x%012H 0x%012H", exp_poly[0], exp_poly[5]);

        // ------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------
        $display("============================================================");
        $display("tb_keyswitch_m35 (K=%0d): PASS=%0d FAIL=%0d", K, pass_cnt, fail_cnt);
        $display("  real DP : m_beats=%0d/%0d  coefs=%0d/%0d  mism=%0d  rounds=%0d/%0d",
                 m_beats, FRAME_IN, m_got_cnt, N, mism_cnt, ntt_rounds, K);
        $display("  status=0x%02h busy=%b err=%b code=0x%08h",
                 status, busy, err, err_code);
        $display("============================================================");
        if (fail_cnt == 0)
            $display("ALL TESTS PASSED - M3.5 K=%0d KEYSWITCH GOLDEN VERIFIED", K);
        else
            $display("SOME TESTS FAILED");
        $finish;
    end

endmodule

`default_nettype wire
