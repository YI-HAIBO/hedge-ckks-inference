//=============================================================================
// File    : tb_ntt_unit_small.v
// Project : 玄算（HEDGE）
// Role    : sim-verifier (魏波形)
// Status  : v1.0 -- fast regression TB for the LOAD handshake
//
// Same DUT as tb_ntt_unit.v but POLY_ORDER=3 (N=8), so the whole forward +
// inverse run is ~200 cycles and every beat can be printed.  This is how the
// beat-1638 stall is diagnosed: the full-size TB stalls on the LAST beat and
// prints nothing per cycle, so the bug is invisible until it is reproduced at
// N=8 where the handshake is a handful of lines.
//
// Prints every send cycle: t, b, s_tvalid, s_tready, s_tlast, status.
//=============================================================================

`timescale 1ns/1ps

module tb_ntt_unit_small;

    localparam CW   = 48;
    localparam L    = 3;
    localparam N    = 8;
    localparam DW   = 256;
    localparam CPB  = DW / CW;          // 5
    localparam BEATS = (N + CPB - 1) / CPB;   // 2

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
    integer cycle_count = 0;

    ntt_unit #(
        .AXI_DATA_W   (DW),
        .COEF_WIDTH   (CW),
        .POLY_ORDER   (L),
        .START_LATENCY(3),
        .ROM_HEX      ("ntt_twiddle_rom_n8.hex"),
        // 8^-1 mod q.  MUST NOT be left at HEDGE_NINV: that constant is the
        // inverse of 2^13 and belongs to the production ring, so the INTT
        // rescale would divide by 8192 instead of by 8.
        .NINV_OVR     (48'h6FFFFFF20001)
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

    always #5 clk_main = ~clk_main;

    // cycle counter, readable from the waveform viewer at any time
    always @(posedge clk_main) cycle_count <= cycle_count + 1;

    // AXI4-Stream source handshake, three-phase (the rule this TB got wrong
    // twice -- recording it so it is not re-broken):
    //   phase 1  drive TVALID/TDATA/TLAST, then park on a NEGEDGE.  On the
    //            negedge the combinational TREADY has settled, so its value
    //            is the one the DUT will sample at the NEXT posedge.
    //   phase 2  read TREADY on THAT SAME negedge -- do not wait for another
    //            one.  TREADY is a function of DUT registers only, so it is
    //            already settled here.  This is the exact spot v1 got wrong:
    //            the TB drove TVALID at t=115, first looked at TREADY at
    //            t=130, and by then the DUT had already taken the beat at
    //            t=125.  The TB therefore ran one beat behind forever and
    //            waited for a handshake that had already happened -- the DUT
    //            consumed all BEATS correctly and still "stalled".
    //   phase 3  consume that posedge (the transfer edge), then wait one more
    //            NEGEDGE before touching TVALID/TLAST again so the DUT never
    //            samples a signal in the same timestep the TB changed it.
    task send_poly;
        begin
            for (b = 0; b < BEATS; b = b + 1) begin
                @(negedge clk_main);          // park: TREADY settles
                for (l = 0; l < CPB; l = l + 1) begin
                    idx = b * CPB + l;
                    if (idx < N) s_tdata[l*CW +: CW] = vec_in[idx];
                    else         s_tdata[l*CW +: CW] = {CW{1'b0}};
                end
                s_tvalid   = 1'b1;
                s_tlast    = (b == BEATS - 1);

                // No #1 delay here on purpose.  We are parked ON the negedge,
                // so the data and TREADY are both already valid; the DUT will
                // sample them at the next posedge.  A delay here would let the
                // TB start changing the payload before that posedge arrives.
                cyc = 0;
                while (1) begin
                    if (s_tready) begin
                        @(posedge clk_main);   // <-- the transfer edge
                        @(negedge clk_main);   // park BEFORE releasing TVALID:
                        break;                 // otherwise the TB would change
                    end                        // TVALID in the same timestep the
                    @(negedge clk_main);       // DUT samples it -> the DUT sees
                    cyc = cyc + 1;             // valid already deasserted and
                    if (cyc > 200) begin       // silently drops the beat.
                        $display("[FAIL] send_poly stalled at beat %0d", b);
                        $finish;
                    end
                end

                s_tvalid = 1'b0;
                s_tlast  = 1'b0;
            end
        end
    endtask

    // AXI4-Stream sink handshake, same discipline as send_poly.  The DUT drives
    // TVALID from a register (pack_v), so TVALID/TDATA hold for the whole cycle
    // and are safely sampled on the negedge.  After the transfer edge we must
    // advance a full negedge before looking again, otherwise the same beat is
    // captured twice.
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
                        @(negedge clk_main);   // next beat lives in the NEXT cycle
                        break;
                    end
                    @(negedge clk_main);
                    cyc = cyc + 1;
                    if (cyc > 20000) begin
                        $display("[FAIL] recv timeout beat %0d", b);
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
            // S_DONE lasts exactly one cycle, then the DUT falls back to
            // S_IDLE by itself.  Arming `start` while it is still in S_DONE is
            // a no-op (the S_IDLE branch never runs), so wait for IDLE first.
            cyc = 0;
            while (status[3:0] != 4'd0) begin
                @(posedge clk_main);
                cyc = cyc + 1;
                if (cyc > 20000) begin
                    $display("[FAIL] %0s : DUT never returned to S_IDLE", name);
                    tc_fail = tc_fail + 1;
                    disable run_dir;
                end
            end
            // Raise `start` while parked on a negedge so it is stable for the
            // whole following cycle and the DUT's S_IDLE branch certainly
            // sees it.  Raising and dropping it inside a single posedge-to-
            // posedge window is a race: the DUT may sample the already-cleared
            // value and the whole transform silently never starts.
            @(negedge clk_main);
            start   = 1'b1;
            cycle_mark = 0;
            @(posedge clk_main);       // DUT samples start=1 here -> S_LOAD
            @(negedge clk_main);
            start   = 1'b0;

            send_poly;
            recv_poly;

            cyc = 0;
            while (status[3:0] != S_DONE) begin
                @(posedge clk_main);
                cyc = cyc + 1;
                if (cyc > 20000) begin
                    $display("[FAIL] %0s : never reached S_DONE", name);
                    tc_fail = tc_fail + 1;
                    disable run_dir;
                end
            end

            fails = 0;
            for (i = 0; i < N; i = i + 1)
                if (vec_got[i] !== vec_exp[i]) fails = fails + 1;

            if (fails == 0) begin
                $display("[PASS] %0s : %0d/%0d match, latency ~%0d cycles",
                         name, N, N, cycle_mark);
                tc_pass = tc_pass + 1;
            end else begin
                $display("[FAIL] %0s : %0d mismatches", name, fails);
                tc_fail = tc_fail + 1;
            end
            @(posedge clk_main);
        end
    endtask

    always @(posedge clk_main) begin
        if (start || busy) cycle_mark <= cycle_mark + 1;
    end

    initial begin
        $readmemh("tb_ntt_vec_in_n8.hex",  vec_in);
        $readmemh("tb_ntt_vec_fwd_n8.hex", vec_exp);

        tc_pass = 0;
        tc_fail = 0;

        rst_n = 1'b0;
        repeat (8) @(posedge clk_main);
        rst_n = 1'b1;
        repeat (4) @(posedge clk_main);

        run_dir(1'b0, "TC1 forward NTT  ");

        for (i = 0; i < N; i = i + 1) vec_in[i] = vec_exp[i];
        $readmemh("tb_ntt_vec_in_n8.hex", vec_exp);
        run_dir(1'b1, "TC2 inverse INTT ");

        $display("");
        $display("Test Summary: PASS=%0d, FAIL=%0d", tc_pass, tc_fail);
        $finish;
    end

    initial begin
        #20000000;
        $display("[FAIL] global watchdog timeout");
        $finish;
    end

endmodule