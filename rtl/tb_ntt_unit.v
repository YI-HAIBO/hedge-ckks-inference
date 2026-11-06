//=============================================================================
// File    : tb_ntt_unit.v
// Project : 玄算（HEDGE）
// Role    : sim-verifier (魏波形)
// Status  : v1.0
//
// Purpose:
//   Verify ntt_unit.v against reference vectors produced by
//   hedge-scripts/_gen_ntt_vectors.py.  The testbench performs NO modular
//   arithmetic of its own -- expected outputs are read from files, so a bug
//   in the RTL cannot be masked by the same bug in a golden model.
//
//   TC1  forward : NTT(stimulus)          == tb_ntt_vec_fwd.hex
//   TC2  inverse : INTT(NTT(stimulus))    == tb_ntt_vec_in.hex  (round trip)
//
//   Both directions are run back to back on the same instance and the
//   cycle count from `start` to `status == S_DONE` is reported, because the
//   Contract §6.1 latency budget is an open issue (R1) and needs real numbers.
//
// Run: hedge-scripts/run_sim_ntt.py   (cwd = hedge-rtl/sub_a_plus_c so that
//      the relative $readmemh paths resolve)
//=============================================================================

`timescale 1ns/1ps

module tb_ntt_unit;

    localparam CW   = 48;
    localparam L    = 13;
    localparam N    = 8192;
    localparam DW   = 256;
    localparam CPB  = DW / CW;          // 5
    localparam BEATS = (N + CPB - 1) / CPB;   // 1639

    localparam [3:0] S_DONE = 4'd5;

    reg                  clk_main = 1'b0;
    reg                  rst_n    = 1'b0;
    reg                  s_tvalid = 1'b0;
    reg  [DW-1:0]        s_tdata  = {DW{1'b0}};
    reg                  s_tlast  = 1'b0;
    wire                 s_tready;
    wire                 m_tvalid;
    wire [DW-1:0]        m_tdata;
    reg                  m_tready = 1'b0;
    wire                 m_tlast;
    reg                  ntt_dir  = 1'b0;
    reg                  start    = 1'b0;
    wire [7:0]           status;
    wire                 busy;
    wire                 err;

    reg [CW-1:0] vec_in  [0:N-1];
    reg [CW-1:0] vec_exp [0:N-1];
    reg [CW-1:0] vec_got [0:N-1];

    integer i, fails, b, l, idx, cyc, tc_pass, tc_fail;
    integer cycle_mark;

    //==================================================
    // DUT
    //==================================================
    ntt_unit #(
        .AXI_DATA_W   (DW),
        .COEF_WIDTH   (CW),
        .POLY_ORDER   (L),
        .START_LATENCY(3)
        // NINV_OVR is left at 0 on purpose: HEDGE_NINV IS the inverse of
        // 2^13, which is the right constant for this production geometry.  The
        // N=8 regression TB has to override it with 8^-1 -- see
        // tb_ntt_unit_small.v.
    ) u_ntt (
        .clk_main          (clk_main),
        .rst_n             (rst_n),
        .axi_s_axis_TVALID (s_tvalid),
        .axi_s_axis_TDATA  (s_tdata),
        .axi_s_axis_TREADY (s_tready),
        .axi_s_axis_TLAST  (s_tlast),
        .axi_m_axis_TVALID (m_tvalid),
        .axi_m_axis_TDATA  (m_tdata),
        .axi_m_axis_TREADY (m_tready),
        .axi_m_axis_TLAST  (m_tlast),
        .ntt_dir           (ntt_dir),
        .start             (start),
        .status            (status),
        .busy              (busy),
        .err               (err)
    );

    always #5 clk_main = ~clk_main;      // 100 MHz

    //==================================================
    // stimulus / capture tasks
    //==================================================
    // AXI4-Stream source handshake -- three phases, and all three are load
    // bearing.  Two earlier versions each failed differently, so keep them:
    //
    //   v1  checked TREADY at @(posedge).  At a posedge the DUT's state_r and
    //       ldp have already advanced (non-blocking), so TREADY reads the NEXT
    //       cycle's value: the TB never sees a transfer and hangs forever.
    //
    //   v2  drove TVALID, then waited a negedge, then consumed a posedge, then
    //       released TVALID immediately.  The release lands in the same
    //       timestep the DUT samples, so the DUT sees valid already low and
    //       silently drops the beat while the TB has already moved on.  With
    //       N=8 that showed up as "stalled at beat 1"; with N=8192 the same
    //       bug showed up as a stall on beat 1638 (the last beat).
    //
    //   v3  (this one, verified bit-exact on the N=8 ring):
    //       1. park on a negedge and drive TVALID/TDATA/TLAST there
    //       2. read TREADY on THAT SAME negedge -- it depends only on DUT
    //          registers, so it is already settled and it is exactly the value
    //          the DUT will sample at the next posedge
    //       3. consume that posedge, then park one more negedge before
    //          touching the handshake signals again
    task send_poly;
        begin
            for (b = 0; b < BEATS; b = b + 1) begin
                @(negedge clk_main);
                for (l = 0; l < CPB; l = l + 1) begin
                    idx = b * CPB + l;
                    if (idx < N) s_tdata[l*CW +: CW] = vec_in[idx];
                    else         s_tdata[l*CW +: CW] = {CW{1'b0}};
                end
                s_tvalid   = 1'b1;
                s_tlast    = (b == BEATS - 1);

                cyc = 0;
                while (1) begin
                    if (s_tready) begin
                        @(posedge clk_main);   // <-- the transfer edge
                        @(negedge clk_main);   // park before releasing TVALID
                        break;
                    end
                    @(negedge clk_main);
                    cyc = cyc + 1;
                    if (cyc > 5000) begin
                        $display("[FAIL] send_poly stalled at beat %0d: status=%0h s_tready=%b",
                                 b, status, s_tready);
                        $finish;
                    end
                end

                s_tvalid = 1'b0;
                s_tlast  = 1'b0;
            end
        end
    endtask

    // AXI4-Stream sink handshake, same discipline.  The DUT drives TVALID from
    // the pack_v register, so TVALID/TDATA are stable for a whole cycle and are
    // sampled on the negedge.  After the transfer edge we must advance a full
    // negedge before looking again, otherwise the same beat is captured twice.
    task recv_poly;
        begin
            m_tready = 1'b1;
            for (b = 0; b < BEATS; b = b + 1) begin
                cyc = 0;
                @(negedge clk_main);
                while (1) begin
                    if (m_tvalid) begin
                        for (l = 0; l < CPB; l = l + 1) begin
                            idx = b * CPB + l;
                            if (idx < N) vec_got[idx] = m_tdata[l*CW +: CW];
                        end
                        @(posedge clk_main);   // <-- the transfer edge
                        @(negedge clk_main);   // next beat is in the next cycle
                        break;
                    end
                    @(negedge clk_main);
                    cyc = cyc + 1;
                    if (cyc > 2000000) begin
                        $display("[FAIL] timeout waiting for output beat %0d", b);
                        $finish;
                    end
                end
            end
            m_tready = 1'b0;
        end
    endtask

    task run_dir;
        input       dir;
        input [8*24:1] name;
        begin
            ntt_dir = dir;

            // S_DONE lasts exactly one cycle and then the DUT falls back to
            // S_IDLE by itself.  Arming `start` while it is still in S_DONE is
            // a silent no-op -- the S_IDLE branch never runs -- so wait for the
            // return to IDLE before starting the next direction.
            cyc = 0;
            while (status[3:0] != 4'd0) begin
                @(posedge clk_main);
                cyc = cyc + 1;
                if (cyc > 5000) begin
                    $display("[FAIL] %0s : DUT never returned to S_IDLE (status=%0h)",
                             name, status[3:0]);
                    tc_fail = tc_fail + 1;
                    disable run_dir;
                end
            end

            // Raise `start` while parked on a negedge so it is stable for the
            // whole following cycle and the S_IDLE branch certainly sees it.
            // Raising and clearing it inside a single posedge window is a race.
            @(negedge clk_main);
            start   = 1'b1;
            cycle_mark = 0;
            @(posedge clk_main);        // DUT samples start=1 here -> S_LOAD
            @(negedge clk_main);
            start   = 1'b0;

            send_poly;
            recv_poly;

            // wait for the FSM to park in DONE (output beats are already out)
            cyc = 0;
            while (status[3:0] != S_DONE) begin
                @(posedge clk_main);
                cyc = cyc + 1;
                if (cyc > 5000) begin
                    $display("[FAIL] %0s : never reached S_DONE (status=%0h)", name, status);
                    tc_fail = tc_fail + 1;
                    disable run_dir;
                end
            end

            fails = 0;
            for (i = 0; i < N; i = i + 1)
                if (vec_got[i] !== vec_exp[i]) fails = fails + 1;

            if (fails == 0) begin
                $display("[PASS] %0s : %0d/%0d coefficients match, latency ~%0d cycles",
                         name, N, N, cycle_mark);
                tc_pass = tc_pass + 1;
            end else begin
                $display("[FAIL] %0s : %0d/%0d mismatches; first at i=%0d got=%h exp=%h",
                         name, fails, N, first_bad(), vec_got[first_bad()], vec_exp[first_bad()]);
                tc_fail = tc_fail + 1;
            end
            @(posedge clk_main);
        end
    endtask

    function integer first_bad;
        integer k;
        begin
            first_bad = 0;
            for (k = 0; k < N; k = k + 1)
                if (vec_got[k] !== vec_exp[k]) begin
                    first_bad = k;
                    k = N;               // break
                end
        end
    endfunction

    //==================================================
    // cycle counter (latency measurement)
    //==================================================
    always @(posedge clk_main) begin
        if (start || busy) cycle_mark <= cycle_mark + 1;
    end


    //==================================================
    // main
    //==================================================
    initial begin
        $readmemh("tb_ntt_vec_in.hex",  vec_in);
        $readmemh("tb_ntt_vec_fwd.hex", vec_exp);

        tc_pass = 0;
        tc_fail = 0;

        rst_n = 1'b0;
        repeat (8) @(posedge clk_main);
        rst_n = 1'b1;
        repeat (4) @(posedge clk_main);

        // TC1: forward NTT
        run_dir(1'b0, "TC1 forward NTT  ");

        // TC2: inverse of the forward output must return the stimulus
        for (i = 0; i < N; i = i + 1) vec_in[i] = vec_exp[i];   // in  <- NTT(a)
        $readmemh("tb_ntt_vec_in.hex", vec_exp);                 // exp <- a
        run_dir(1'b1, "TC2 inverse INTT ");

        $display("");
        $display("Test Summary: PASS=%0d, FAIL=%0d", tc_pass, tc_fail);
        $finish;
    end

    // overall watchdog
    initial begin
        #400000000;   // 400 ms
        $display("[FAIL] global watchdog timeout");
        $finish;
    end

endmodule
