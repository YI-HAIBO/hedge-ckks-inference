//=============================================================================
// tb_keyswitch_real.v -- M5: system-level golden test for keyswitch_pipe v0.7
//
// Purpose
//   End-to-end numeric verification of the REAL 256-bit / NPAR=64 keyswitch
//   datapath (REAL_DATAPATH=1, AXI_DATA_W=256).  This is the first TB in the
//   project that exercises both nptt_wrapper instances (INV=0 forward and
//   INV=1 inverse), both nptt_stream_unpack instances, the
//   engine-interleaved -> natural repack, the real POINTWISE and the real
//   ACCUMULATE of the keyswitch.
//
//   The RTL computes, over Z_q  (q = 2^47 - 2^20 + 1 = 140737487306753):
//
//       c1' = ( c1 + INTT( NTT(a_k) * NTT(c2) ) ) mod q
//
//   a_k is loaded already in NTT domain through key_load, so key_bram_a holds
//   NTT(a_k) and no third wrapper is needed.  NUM_DIGITS = 1, so the digit d
//   of the decomposed c2 is c2 itself.
//
// Golden source
//   hedge-scripts/gen_keyswitch_real_golden.py, seed 20261023.  The vectors
//   are produced by hedge-tb/golden_ntt.py and cross-checked against
//   hedge-scripts/four_step_ntt_ref.py (0/8192 differences in both NTT and
//   INTT) before being dumped, so a mismatch here can only be a DUT bug.
//
// Test plan
//   TC1  no X on any status / handshake line after reset
//   TC2  key_load_start reaches S_LOAD_KEY and key_load_ready asserts
//   TC3  real keyswitch: 2 x FRAME_IN input beats, 8192-coefficient golden
//        comparison of c1', FRAME_IN output beats, exactly one TLAST
//   TC4  error path: start without a loaded key -> E_KEY_NOT_LOADED (0x12)
//   TC5  error path: digit_num_zero -> E_MODCHAIN_EXHAUSTED (0x10)
//   TC6  pass-through build (REAL_DATAPATH=0, AXI_DATA_W=64) still echoes
//        its frame, so the 64-bit ac_adapter front end keeps working
//
// Handshake discipline (the 2026-10-19 iverilog lesson, also recorded in
// tb_nptt_wrapper_npar64_golden.v): every TB output the DUT samples at a
// posedge (TVALID / TDATA / TLAST / key_load_*) is assigned at least #1
// AFTER the @(posedge clk) resume point.  Assigning in the same timestep as
// the acceptance edge lets iverilog schedule the TB continuation before the
// DUT's always block, which shifts every stored beat by one -- a silent
// 8192/8192 mismatch instead of an obvious error.
//
// Run:  .workbuddy/run_ks_real_npar64.py
//=============================================================================
`timescale 1ns / 1ps
`default_nettype wire

module tb_keyswitch_real;

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
    localparam INPUT_BEATS   = 2 * FRAME_IN;            // c1 stream then c2 stream
    // Pass-through build geometry.  At AXI_DATA_W=64 the 64-bit build has
    // CPB_pt = 64/48 = 1, so its OWN FRAME_IN is N = 8192 single-coefficient
    // beats, not the 1639 packed beats of the 256-bit build.  The key-load
    // FSM counts its own FRAME_IN and latches key_loaded_r on beat
    // LIM_IN-1 = 8191, so TC6 must feed PT_FRAME beats or it times out.
    localparam CPB_PT        = 64 / COEF_W;                  // 1
    localparam PT_FRAME      = (N + CPB_PT - 1) / CPB_PT;    // 8192
    localparam BEATS_PER_ENG = (N2 + CPB - 1) / CPB;    // 26
    localparam TIMEOUT       = 6_000_000;               // ns hard watchdog
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
        .NUM_DIGITS      (1)
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
    // 64-bit DUT interface (pass-through build, as ac_adapter consumes it)
    //==================================================
    reg  [63:0] pt_s_TDATA = 64'd0;
    wire        pt_s_TREADY;
    reg         pt_s_TVALID = 1'b0;
    reg         pt_s_TLAST  = 1'b0;
    wire [63:0] pt_m_TDATA;
    wire        pt_m_TVALID;
    wire        pt_m_TLAST;
    reg         pt_m_TREADY = 1'b1;
    reg         pt_kl_start = 1'b0;
    reg         pt_kl_valid = 1'b0;
    wire        pt_kl_ready;
    reg  [63:0] pt_kl_TDATA = 64'd0;
    reg         pt_start    = 1'b0;
    reg         pt_digit    = 1'b0;
    wire [7:0]  pt_status;
    wire        pt_busy;
    wire        pt_err;
    wire [31:0] pt_err_code;

    keyswitch_pipe #(
        .AXI_DATA_W      (64),
        .COEF_WIDTH      (48),
        .NUM_ENGINES     (N1),
        .COEFS_PER_ENGINE(N2),
        .REAL_DATAPATH   (0),
        .NUM_DIGITS      (1)
    ) dut_pt (
        .clk_main            (clk),
        .rst_n               (rst_n),
        .axi_s_axis_TVALID   (pt_s_TVALID),
        .axi_s_axis_TDATA    (pt_s_TDATA),
        .axi_s_axis_TREADY   (pt_s_TREADY),
        .axi_s_axis_TLAST    (pt_s_TLAST),
        .axi_m_axis_TVALID   (pt_m_TVALID),
        .axi_m_axis_TDATA    (pt_m_TDATA),
        .axi_m_axis_TREADY   (pt_m_TREADY),
        .axi_m_axis_TLAST    (pt_m_TLAST),
        .key_load_start      (pt_kl_start),
        .key_load_valid      (pt_kl_valid),
        .key_load_ready      (pt_kl_ready),
        .key_load_TDATA      (pt_kl_TDATA),
        .start               (pt_start),
        .status              (pt_status),
        .busy                (pt_busy),
        .err                 (pt_err),
        .err_code            (pt_err_code),
        .stream_enable       (1'b0),
        .digit_num_zero      (pt_digit)
    );

    //==================================================
    // Test data
    //==================================================
    reg [COEF_W-1:0] key_poly [0:N-1];     // NTT(a_k), fed through key_load
    reg [COEF_W-1:0] c1_poly  [0:N-1];
    reg [COEF_W-1:0] c2_poly  [0:N-1];
    reg [COEF_W-1:0] exp_poly [0:N-1];     // expected c1'

    // Expected index-0 values of the committed vector set (seed 20261023),
    // printed by hedge-scripts/gen_keyswitch_real_golden.py.  These pin the
    // TB to that exact generation run: a missing, stubbed or STALE hex file
    // fails here instead of producing a vacuous numeric pass.
    localparam [COEF_W-1:0] EXP_C1_0  = 48'h7b48a0df75f4;
    localparam [COEF_W-1:0] EXP_OUT_0 = 48'h4e807b3faf46;

    integer nnz_key = 0;
    integer nnz_c1  = 0;
    integer nnz_c2  = 0;
    integer nnz_out = 0;

    //==================================================
    // TC0 sanity switch (a_k = 0  ->  c1' == c1)
    //
    //   When zero_key_mode is set, pack_beat drives key_poly with all
    //   zeros through key_load.  The DUT then computes
    //
    //       c1' = c1 + INTT( NTT(0) * NTT(c2) ) = c1 + INTT(0) = c1
    //
    //   so the output stream must match c1_poly coefficient-for-
    //   coefficient.  This test isolates four stages (padding zero-
    //   fill, POINTWISE mult-by-zero, INTT of zero, ACCUMULATE, STORE
    //   out path) from any bug in the key path (key_load indexing,
    //   key_bram_a -> POINTWISE mapping).  If TC0 passes but TC3
    //   fails, the bug is key-side; if both fail with the same
    //   signature, the bug is in the transform or accumulator chain.
    //==================================================
    reg     zero_key_mode   = 1'b0;   // pack_beat: emit zero for key stream
    reg     expect_identity = 1'b0;   // monitor:  compare against c1_poly

    initial begin
        integer j;
        $readmemh({BASE, "ks_golden_key.hex"}, key_poly);
        $readmemh({BASE, "ks_golden_c1.hex"},  c1_poly);
        $readmemh({BASE, "ks_golden_c2.hex"},  c2_poly);
        $readmemh({BASE, "ks_golden_out.hex"}, exp_poly);
        // $readmemh FAILS SILENTLY on a missing file and leaves the array at
        // its declared value (0).  Counting non-zero entries catches a
        // stubbed set; note $readmemh has no return value in iverilog.
        for (j = 0; j < N; j = j + 1) begin
            if (key_poly[j] != {COEF_W{1'b0}}) nnz_key = nnz_key + 1;
            if (c1_poly[j]  != {COEF_W{1'b0}}) nnz_c1  = nnz_c1  + 1;
            if (c2_poly[j]  != {COEF_W{1'b0}}) nnz_c2  = nnz_c2  + 1;
            if (exp_poly[j] != {COEF_W{1'b0}}) nnz_out = nnz_out + 1;
        end
    end

    //==================================================
    // Result tallies
    //==================================================
    integer pass_cnt    = 0;
    integer fail_cnt    = 0;
    integer tc          = 0;
    integer mism_cnt    = 0;
    integer m_beats     = 0;      // 256-bit output beats received
    integer m_got_cnt   = 0;      // coefficients compared
    integer m_last_cnt  = 0;      // TLAST pulses seen
    integer c1_mism_cnt = 0;      // mismatches vs c1_poly (TC0 identity)
    integer pt_beats    = 0;      // 64-bit output beats received
    integer pt_out_val  = 0;      // mismatches in the pass-through echo

    integer k;
    integer waitd;
    integer lane_i;
    integer gidx_i;
    integer off_i;

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
    // Pack one natural-order 256-bit beat from the selected stream.
    //   lane l occupies bits [l*48 +: 48];  256 = 5*48 + 16, so the top 16
    //   bits are dead and are zeroed explicitly.
    //   For the key stream (use_key=1) the beat holds key_poly[j*5 + l].
    //   For the input stream (use_key=0) beats 0..FRAME_IN-1 hold c1 and
    //   beats FRAME_IN..2*FRAME_IN-1 hold c2; lanes past the last real
    //   coefficient of either stream are zero-filled (the last natural beat
    //   carries only N mod CPB = 2 real coefficients).
    //==================================================
    //==================================================
    // Pack one 256-bit (5-coefficient) beat for the real-datapath build.
    //
    //   The real keyswitch_pipe takes TWO concatenated FRAME_IN-beat
    //   streams: beats 0..FRAME_IN-1 carry c1, beats FRAME_IN..2*FRAME_IN-1
    //   carry c2.  So the source-array index is (j % FRAME_IN) and the
    //   coefficient index within the source is gi = (j % FRAME_IN)*CPB + l.
    //   The last natural beat of each stream carries only N mod CPB = 2 real
    //   coefficients (8192 = 1638*5 + 2), so lanes 2..4 are zero-filled.
    //
    //   RC4 (2026-10-23): an earlier version used a single GLOBAL coefficient
    //   index gi = j*CPB + l for the whole 2*FRAME_IN stream.  That is wrong
    //   for j >= FRAME_IN: gi = 1639*5 + l = 8195..8199 lies beyond N = 8192,
    //   so the `gi < N` guard silently zero-filled every c2 beat.  The DUT
    //   then computed acc = c1 + INTT(NTT(a_k) * NTT(0)) = c1, which looked
    //   like a valid 0/8192-shaped output but was 100% mismatch and hid the
    //   error behind the "exactly c1" signature.  Fixed by splitting the
    //   stream index, mirroring the DUT's own two-stream contract.
    //==================================================
    task automatic pack_beat(input integer j, input integer use_key,
                             output reg [DATA_W-1:0] beat);
        integer l, gi, src;
        reg [COEF_W-1:0] c;
        begin
            beat = {DATA_W{1'b0}};
            if (use_key)
                src = j;                    // key stream: 0..FRAME_IN-1
            else
                src = j % FRAME_IN;        // c1 stream, then c2 stream
            for (l = 0; l < CPB; l = l + 1) begin
                gi = src * CPB + l;
                c  = {COEF_W{1'b0}};
                if (gi < N) begin
                    if (use_key && !zero_key_mode)
                        c = key_poly[gi];
                    else if (!use_key && j < FRAME_IN)
                        c = c1_poly[gi];
                    else if (!use_key)
                        c = c2_poly[gi];
                    // use_key && zero_key_mode  ->  c stays 0
                end
                beat[l*COEF_W +: COEF_W] = c;
            end
        end
    endtask

    //==================================================
    // Drive one 256-bit beat through a valid/ready handshake.
    //   use_key=1 packs key_poly, 0 packs the c1/c2 input stream.
    //   last_of_stream=1 asserts TLAST (only meaningful for the input).
    //==================================================
    task automatic send_beat(input integer j, input integer use_key,
                             input integer last_of_stream);
        reg [DATA_W-1:0] b;
        begin
            pack_beat(j, use_key, b);
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
    // Drive one 64-bit beat (pass-through build)
    //   pt_in_beat[j] is the expected echo for output beat j.  Indexing
    //   by beat number -- rather than comparing against a single "last
    //   driven" latch -- makes the monitor meaningful even if the DUT
    //   misbehaves: an infinite re-emit of beat 0 shows up as a run of
    //   identical-value mismatches against DIFFERENT expectations, which
    //   is the signature of the v0.7 S_STORE_OUT re-prime loop (RC3) and
    //   not a data-correctness problem.
    //==================================================
    reg [63:0] pt_in_beat [0:127];
    integer    pt_nbeats_drv = 0;
    task automatic send_beat_pt(input integer j, input integer last_of_stream);
        begin
            pt_in_beat[pt_nbeats_drv] = {8'd0, key_poly[j]};
            pt_nbeats_drv = pt_nbeats_drv + 1;
            #1;
            pt_s_TDATA  = pt_in_beat[pt_nbeats_drv - 1];
            pt_s_TLAST  = last_of_stream ? 1'b1 : 1'b0;
            pt_s_TVALID = 1'b1;
            @(posedge clk);
            while (!pt_s_TREADY) @(posedge clk);
            #1;
            pt_s_TVALID = 1'b0;
            pt_s_TLAST  = 1'b0;
        end
    endtask

    //==================================================
    // Poll the 256-bit FSM to a given state id
    //==================================================
    task automatic wait_state(input integer st, output integer waited);
        begin
            waited = 0;
            while (status !== st[7:0]) begin
                @(posedge clk);
                waited = waited + 1;
                if (waited > 3_000_000) begin
                    $display("  [WARN] wait_state(%0d) timed out, status=0x%02h", st, status);
                    break;
                end
            end
        end
    endtask

    //==================================================
    // Poll the 64-bit FSM
    //==================================================
    task automatic wait_state_pt(input integer st, output integer waited);
        begin
            waited = 0;
            while (pt_status !== st[7:0]) begin
                @(posedge clk);
                waited = waited + 1;
                if (waited > 100_000) begin
                    $display("  [WARN] wait_state_pt(%0d) timed out, status=0x%02h", st, pt_status);
                    break;
                end
            end
        end
    endtask

    //==================================================
    // Output monitors.  Kept as separate always processes so they never
    // race the drive-side tasks.
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
                            // TC0: expected output is c1_poly[gidx] (a_k=0)
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

    always @(posedge clk) begin
        if (rst_n && pt_m_TVALID && pt_m_TREADY) begin
            if (pt_beats < pt_nbeats_drv &&
                pt_m_TDATA !== pt_in_beat[pt_beats]) begin
                if (pt_out_val < 24)
                    $display("  [PT] echo mismatch beat=%0d got=0x%016H exp=0x%016H",
                             pt_beats, pt_m_TDATA, pt_in_beat[pt_beats]);
                pt_out_val = pt_out_val + 1;
            end
            pt_beats = pt_beats + 1;
        end
    end

    //==================================================
    // Watchdog
    //==================================================
    initial begin
        #TIMEOUT;
        $display("[TIMEOUT] exceeded %0d ns", TIMEOUT);
        $display("  m_beats=%0d/%0d  m_got=%0d/%0d  mism=%0d  status=0x%02h err=%b code=0x%08h",
                 m_beats, FRAME_IN, m_got_cnt, N, mism_cnt, status, err, err_code);
        $finish;
    end

    //==================================================
    // Main sequence
    //==================================================
    initial begin
        $display("============================================================");
        $display("tb_keyswitch_real: M5 golden test, keyswitch_pipe v0.7");
        $display("  N=%0d  q=0x%012X  CPB=%0d  FRAME_IN=%0d  INPUT_BEATS=%0d",
                 N, 140737487306753, CPB, FRAME_IN, INPUT_BEATS);
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
        // Golden-vector guard: refuse to run the numeric compare on a
        // stubbed or stale vector set.  $readmemh fails SILENTLY on a
        // missing file and leaves the array at 0, so all-zero c1/c2/key
        // would yield an all-zero output and a VACUOUS "0 / 8192
        // mismatches" that looks exactly like a pass.
        // ------------------------------------------------------------
        if (nnz_key < N/2 || nnz_c1 < N/2 || nnz_c2 < N/2 || nnz_out < N/2) begin
            $display("[FATAL] golden vectors look stubbed: nnz key=%0d c1=%0d c2=%0d out=%0d of %0d",
                     nnz_key, nnz_c1, nnz_c2, nnz_out, N);
            $display("        fix: python hedge-scripts/gen_keyswitch_real_golden.py");
            $finish;
        end
        if (c1_poly[0] !== EXP_C1_0 || exp_poly[0] !== EXP_OUT_0) begin
            $display("[FATAL] golden vectors are not the seed-20261023 set -- files missing or stale");
            $display("        c1[0]=0x%012H (expect 0x%012H)  out[0]=0x%012H (expect 0x%012H)",
                     c1_poly[0], EXP_C1_0, exp_poly[0], EXP_OUT_0);
            $finish;
        end
        $display("  golden vectors OK: %0d coefs each, c1[0]=0x%012H out[0]=0x%012H",
                 N, c1_poly[0], exp_poly[0]);

        // ------------------------------------------------------------
        // TC1: no X after reset
        // ------------------------------------------------------------
        $display("--- TC1: no X after reset ---");
        check_pass("busy/err/status not X",
                   (busy !== 1'bx) && (err !== 1'bx) && (status !== 8'bx));
        check_pass("start in S_IDLE", status === 8'd0);
        check_pass("not busy at idle", busy === 1'b0);
        check_pass("no err at idle",   err  === 1'b0);

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
        // S_ERROR is terminal: recover with a reset.
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
        // Clean slate for the real datapath run
        // ------------------------------------------------------------
        #1;
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        m_beats = 0; m_got_cnt = 0; m_last_cnt = 0; mism_cnt = 0; c1_mism_cnt = 0;

        // ------------------------------------------------------------
        // TC0: SANITY — zero key  ->  c1' == c1  (identity).
        //
        //   Loads key_poly = 0 through key_load, feeds the SAME c1/c2
        //   streams as TC3, and requires the output to match c1_poly
        //   coefficient-for-coefficient.  Expected math:
        //
        //       c1' = c1 + INTT( NTT(0) * NTT(c2) )
        //           = c1 + INTT( 0 )
        //           = c1
        //
        //   Isolates: padding zero-fill, POINTWISE mult-by-zero, INTT
        //   of zero, ACCUMULATE with zero, and the STORE_OUT path.
        //   If this passes but TC3 fails with the same signature, the
        //   bug is confined to the key path (key_bram_a indexing /
        //   key -> POINTWISE mapping).  If this also fails, the bug
        //   is in the transform chain or accumulator, independent of
        //   key contents.
        // ------------------------------------------------------------
        $display("--- TC0: zero key -> identity (c1' == c1) ---");

        expect_identity = 1'b1;

        #1;
        kl_start = 1'b1;
        @(posedge clk);
        #1;
        kl_start = 1'b0;
        wait_state(1, waitd);

        zero_key_mode = 1'b1;
        for (k = 0; k < FRAME_IN; k = k + 1) begin
            pack_beat(k, 1, kl_TDATA);   // use_key=1, zero_key_mode=1 -> all zero
            #1;
            kl_valid = 1'b1;
            @(posedge clk);
            while (!kl_ready) @(posedge clk);
            #1;
            kl_valid = 1'b0;
        end
        zero_key_mode = 1'b0;
        wait_state(0, waitd);
        check_pass("TC0: key(=0) loaded, back to S_IDLE", status === 8'd0);

        #1;
        start = 1'b1;
        @(posedge clk);
        #1;
        start = 1'b0;
        wait_state(2, waitd);
        check_pass("TC0: reached S_LOAD_INPUT", status === 8'd2);

        for (k = 0; k < INPUT_BEATS; k = k + 1)
            send_beat(k, 0, (k == INPUT_BEATS - 1));

        wait_state(9, waitd);
        check_pass("TC0: reached S_DONE",       status === 8'd9);
        check_pass("TC0: err stayed clear",     err === 1'b0);
        wait_state(0, waitd);
        check_pass("TC0: returned to S_IDLE",   status === 8'd0);

        repeat (5) @(posedge clk);

        $display("--- TC0 detail ---");
        $display("  m_beats=%0d  m_got=%0d  c1_mism=%0d  TLAST=%0d",
                 m_beats, m_got_cnt, c1_mism_cnt, m_last_cnt);
        check_pass("TC0: beats == FRAME_IN",   m_beats   === FRAME_IN);
        check_pass("TC0: coefs == N",          m_got_cnt === N);
        check_pass("TC0: c1_mism == 0",        c1_mism_cnt === 0);

        // DIAG: same stage dump for the identity run.  If TC0 fails,
        // this tells us which BRAM stage first deviates from c1.
        $display("--- TC0 diag: beat 0 / beat 1 ---");
        $display("  base_bram   : 0x%064H  |  0x%064H   (c1)",
                 dut.base_bram[0], dut.base_bram[1]);
        $display("  key_bram_a  : 0x%064H  |  0x%064H   (NTT(0) should be 0)",
                 dut.key_bram_a[0][0], dut.key_bram_a[0][1]);
        $display("  in_bram     : 0x%064H  |  0x%064H   (c2)",
                 dut.in_bram[0], dut.in_bram[1]);
        $display("  ntt_out_bram: 0x%064H  |  0x%064H   (NTT(c2))",
                 dut.ntt_out_bram[0], dut.ntt_out_bram[1]);
        $display("  pw_out_bram : 0x%064H  |  0x%064H   (should be 0)",
                 dut.pw_out_bram[0], dut.pw_out_bram[1]);
        $display("  intt_out_bram:0x%064H  |  0x%064H   (should be 0)",
                 dut.intt_out_bram[0], dut.intt_out_bram[1]);
        $display("  acc_bram    : 0x%064H  |  0x%064H   (should == c1)",
                 dut.acc_bram[0], dut.acc_bram[1]);
        $display("  exp_c1      : 0x%012H @g0   0x%012H @g5",
                 c1_poly[0], c1_poly[5]);

        expect_identity = 1'b0;

        // Reset again for TC2/TC3 (the real key run).
        #1;
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        m_beats = 0; m_got_cnt = 0; m_last_cnt = 0; mism_cnt = 0;

        // ------------------------------------------------------------
        // TC2: key-load handshake
        // ------------------------------------------------------------
        $display("--- TC2: key_load_start -> S_LOAD_KEY, key_load_ready ---");
        #1;
        kl_start = 1'b1;
        @(posedge clk);
        #1;
        kl_start = 1'b0;
        wait_state(1, waitd);
        check_pass("reached S_LOAD_KEY",      status === 8'd1);
        check_pass("key_load_ready asserted", kl_ready === 1'b1);

        // ------------------------------------------------------------
        // TC3: the real keyswitch
        // ------------------------------------------------------------
        $display("--- TC3: real keyswitch, 8192-coef golden compare ---");

        // FRAME_IN key beats, natural order.  key_loaded_r latches on the
        // last one, which returns the FSM to S_IDLE.
        for (k = 0; k < FRAME_IN; k = k + 1) begin
            pack_beat(k, 1, kl_TDATA);
            #1;
            kl_valid = 1'b1;
            @(posedge clk);
            while (!kl_ready) @(posedge clk);
            #1;
            kl_valid = 1'b0;
        end
        wait_state(0, waitd);
        check_pass("key loaded, back to S_IDLE", status === 8'd0);

        #1;
        start = 1'b1;
        @(posedge clk);
        #1;
        start = 1'b0;
        wait_state(2, waitd);
        check_pass("reached S_LOAD_INPUT", status === 8'd2);

        // c1 stream (beats 0..FRAME_IN-1) then c2 stream, TLAST on the last.
        for (k = 0; k < INPUT_BEATS; k = k + 1)
            send_beat(k, 0, (k == INPUT_BEATS - 1));
        check_pass("fed 2 x FRAME_IN input beats", k === INPUT_BEATS);

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

        $display("--- TC3 detail ---");
        $display("  m_beats=%0d  m_got=%0d  mism=%0d  TLAST=%0d",
                 m_beats, m_got_cnt, mism_cnt, m_last_cnt);

        // ------------------------------------------------------------
        // DIAG: hierarchical dump of every BRAM stage, beat 0 and beat 1.
        //
        //   Two iverilog limits shaped this block:
        //     * a variable-base part-select on a memory element cannot be
        //       bound ("Part select is not valid for this kind of object"),
        //       so whole 256-bit beats are printed and the low two lanes
        //       are the ones that matter for gidx 0 and 5;
        //     * a hierarchical reference cannot pass THROUGH a generate
        //       block ("Unable to bind wire/reg/memory"), so the real-DP
        //       arrays are reached only when dut.genblk1 exists, i.e. from
        //       inside a `if (REAL_DP)`.  That check is a simulation-time
        //       runtime condition, not a compile-time bind, so it is legal.
        //
        //   Isolates which stage first becomes 0 so the mismatch can be
        //   pinned without a waveform tool.
        // ------------------------------------------------------------
        // ------------------------------------------------------------
        // DIAG: hierarchical dump of every BRAM stage, beat 0 and beat 1.
        //
        //   All seven BRAM arrays and every replay counter are declared at
        //   MODULE scope in keyswitch_pipe.v (only the wrapper instances
        //   and the wrapper-facing always blocks live inside the generate
        //   block, and iverilog refuses to bind a hierarchical reference
        //   that passes THROUGH a generate-if).  A variable-base
        //   part-select on a memory element is also rejected, so whole
        //   256-bit beats are printed and the low two lanes carry gidx 0
        //   and gidx 5.
        //
        //   Isolates which stage first becomes 0 so the mismatch can be
        //   pinned without a waveform tool.
        // ------------------------------------------------------------
        $display("--- TC3 diag: beat 0 / beat 1 (low 2 lanes) ---");
        $display("  base_bram   : 0x%064H  |  0x%064H   (c1)",
                 dut.base_bram[0], dut.base_bram[1]);
        $display("  key_bram_a  : 0x%064H  |  0x%064H   (NTT(a_k))",
                 dut.key_bram_a[0][0], dut.key_bram_a[0][1]);
        $display("  in_bram     : 0x%064H  |  0x%064H   (c2)",
                 dut.in_bram[0], dut.in_bram[1]);
        $display("  ntt_out_bram: 0x%064H  |  0x%064H   (NTT(c2))",
                 dut.ntt_out_bram[0], dut.ntt_out_bram[1]);
        $display("  pw_out_bram : 0x%064H  |  0x%064H   (N^-1*NTT(c2)*NTT(a_k))",
                 dut.pw_out_bram[0], dut.pw_out_bram[1]);
        $display("  intt_out_bram:0x%064H  |  0x%064H   (INTT(pw))",
                 dut.intt_out_bram[0], dut.intt_out_bram[1]);
        $display("  acc_bram    : 0x%064H  |  0x%064H   (c1 + INTT(pw))",
                 dut.acc_bram[0], dut.acc_bram[1]);
        $display("  exp_poly    : 0x%012H @g0   0x%012H @g5   (golden out)",
                 exp_poly[0], exp_poly[5]);
        $display("  counters: ntt_outcnt=%0d ntt_coefcnt=%0d intt_outcnt=%0d intt_coefcnt=%0d",
                 dut.ntt_outcnt_r, dut.ntt_coefcnt_r,
                 dut.intt_outcnt_r, dut.intt_coefcnt_r);

        // ------------------------------------------------------------
        // TC6: pass-through build (64-bit) still echoes its frame
        //   NOTE: at AXI_DATA_W=64 the DUT's own CPB is 1, so its own
        //   FRAME_IN is PT_FRAME = 8192 single-coefficient beats and
        //   key_loaded_r latches only on the 8192nd.  The pass-through
        //   path never reads key_bram_a, so the values are only there to
        //   drive the handshake -- but the handshake length is not.
        // ------------------------------------------------------------
        $display("--- TC6: 64-bit pass-through build echoes the frame ---");
        pt_beats = 0; pt_out_val = 0;

        #1;
        pt_kl_start = 1'b1;
        @(posedge clk);
        #1;
        pt_kl_start = 1'b0;
        wait_state_pt(1, waitd);
        check_pass("PT: reached S_LOAD_KEY", pt_status === 8'd1);

        for (k = 0; k < PT_FRAME; k = k + 1) begin
            #1;
            pt_kl_TDATA = {8'd0, key_poly[k]};
            pt_kl_valid = 1'b1;
            @(posedge clk);
            while (!pt_kl_ready) @(posedge clk);
            #1;
            pt_kl_valid = 1'b0;
        end
        wait_state_pt(0, waitd);
        check_pass("PT: key loaded, back to S_IDLE", pt_status === 8'd0);

        #1;
        pt_start = 1'b1;
        @(posedge clk);
        #1;
        pt_start = 1'b0;
        wait_state_pt(2, waitd);
        check_pass("PT: reached S_LOAD_INPUT", pt_status === 8'd2);

        // Any beat count is legal in pass-through mode; 8 beats is enough
        // to prove multi-beat echo including the TLAST boundary.
        for (k = 0; k < 8; k = k + 1)
            send_beat_pt(k, (k == 7));

        wait_state_pt(9, waitd);
        check_pass("PT: reached S_DONE",     pt_status === 8'd9);
        check_pass("PT: no err",             pt_err === 1'b0);
        wait_state_pt(0, waitd);

        repeat (5) @(posedge clk);
        check_pass("PT: 8 beats echoed",     pt_beats === 8);
        check_pass("PT: echo matches input", pt_out_val === 0);

        // ------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------
        $display("============================================================");
        $display("tb_keyswitch_real: PASS=%0d FAIL=%0d", pass_cnt, fail_cnt);
        $display("  real DP : m_beats=%0d/%0d  coefs=%0d/%0d  mism=%0d",
                 m_beats, FRAME_IN, m_got_cnt, N, mism_cnt);
        $display("  pt DP   : beats=%0d/8  mism=%0d", pt_beats, pt_out_val);
        $display("  status=0x%02h busy=%b err=%b code=0x%08h",
                 status, busy, err, err_code);
        $display("============================================================");
        if (fail_cnt == 0)
            $display("ALL TESTS PASSED - M5 REAL KEYSWITCH GOLDEN VERIFIED");
        else
            $display("SOME TESTS FAILED");
        $finish;
    end

endmodule

`default_nettype wire
