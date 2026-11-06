//=============================================================================
// File    : tb_keyswitch_pipe.v
// Project : HEDGE - KeySwitch skeleton verification (v0.3 pass-through)
// Status  : 2026-10-10 - 4 TC: happy / E_KEY_NOT_LOADED / E_MODCHAIN_EXHAUSTED
//           / stream_enable no-op
// Notes   : keyswitch_pipe v0.3 is a PASS-THROUGH skeleton (no real NTT/INTT).
//           Verification target = FSM transitions + AXI handshake + err_code
//           + echo datapath.  NOT a numerical relin test.
//=============================================================================

`timescale 1ns / 1ps

module tb_keyswitch_pipe;

    localparam AXI_DATA_W = 256;
    localparam COEF_WIDTH = 48;

    reg  clk = 1'b0;
    reg  rst_n = 1'b0;

    // AXI input
    reg                in_valid = 1'b0;
    reg  [AXI_DATA_W-1:0] in_data = {AXI_DATA_W{1'b0}};
    wire               in_ready;
    reg                in_last  = 1'b0;

    // AXI output
    wire               out_valid;
    wire [AXI_DATA_W-1:0] out_data;
    wire               out_last;
    reg                out_ready = 1'b1;

    // Key load
    reg                kl_start = 1'b0;
    reg                kl_valid = 1'b0;
    wire               kl_ready;
    reg  [AXI_DATA_W-1:0] kl_data = 32'hDEADBEEF;

    // Control
    reg                start = 1'b0;
    wire [7:0]         status;
    wire               busy;
    wire               err;
    wire [31:0]        err_code;
    reg                stream_enable = 1'b0;
    reg                digit_num_zero = 1'b0;

    always #5 clk = ~clk;

    keyswitch_pipe #(
        .AXI_DATA_W(AXI_DATA_W),
        .COEF_WIDTH(COEF_WIDTH),
        .N_PAR(1),
        .COEFS_PER_BEAT(AXI_DATA_W/COEF_WIDTH)
    ) dut (
        .clk_main(clk),
        .rst_n(rst_n),
        .axi_s_axis_TVALID(in_valid),
        .axi_s_axis_TDATA(in_data),
        .axi_s_axis_TREADY(in_ready),
        .axi_s_axis_TLAST(in_last),
        .axi_m_axis_TVALID(out_valid),
        .axi_m_axis_TDATA(out_data),
        .axi_m_axis_TREADY(out_ready),
        .axi_m_axis_TLAST(out_last),
        .key_load_start(kl_start),
        .key_load_valid(kl_valid),
        .key_load_ready(kl_ready),
        .key_load_TDATA(kl_data),
        .start(start),
        .status(status),
        .busy(busy),
        .err(err),
        .err_code(err_code),
        .stream_enable(stream_enable),
        .digit_num_zero(digit_num_zero)
    );

    // Watchdog
    reg [31:0] pass_cnt = 0;
    reg [31:0] fail_cnt = 0;

    initial begin
        $display("TB keyswitch_pipe: start");
        rst_n = 1'b0;
        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // TC1: load key + start + 1-beat echo
        $display("=== TC1: happy path (load key, echo 1 beat) ===");
        run_tc1;

        // TC2: start without key -> E_KEY_NOT_LOADED (0x12)
        $display("=== TC2: start without key -> E_KEY_NOT_LOADED ===");
        run_tc2;

        // TC3: start with digit_num_zero -> E_MODCHAIN_EXHAUSTED (0x10)
        $display("=== TC3: digit_num_zero -> E_MODCHAIN_EXHAUSTED ===");
        run_tc3;

        // TC4: stream_enable=1 during full flow -> still works
        $display("=== TC4: stream_enable=1 no-op ===");
        run_tc4;

        $display("---- SUMMARY: pass=%0d fail=%0d ----", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("[ALL PASS]");
        else               $display("[FAIL]");
        $finish;
    end

    // TC1: full happy path
    task run_tc1;
        reg [AXI_DATA_W-1:0] tx_data;
        integer cyc;
        begin
            // Reset err state first
            rst_n = 0; repeat (4) @(posedge clk); rst_n = 1; repeat (4) @(posedge clk);
            kl_start = 0; kl_valid = 0; start = 0; in_valid = 0; in_last = 0;
            digit_num_zero = 0;

            // (a) Load key
            kl_start = 1'b1;
            @(posedge clk);
            kl_start = 1'b0;
            // Now in ST_LOAD_KEY, key_load_ready=1
            repeat (2) @(posedge clk);
            kl_valid = 1'b1;
            @(posedge clk);
            kl_valid = 1'b0;
            repeat (3) @(posedge clk);

            // (b) Start
            tx_data = 32'hCAFEF00D;
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            repeat (2) @(posedge clk);

            // (c) Send 1 beat with TLAST
            in_valid = 1'b1;
            in_data  = tx_data;
            in_last  = 1'b1;
            while (!in_ready) @(posedge clk);
            @(posedge clk);
            in_valid = 1'b0;
            in_last  = 1'b0;

            // (d) Wait for output (ST_POINTWISE -> ST_COMBINE -> ST_INTT -> ST_STORE)
            cyc = 0;
            while (out_valid == 0 && cyc < 200) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
            // Now out_valid should be 1, accept it
            if (out_valid && out_last) begin
                if (out_data === in_data) begin
                    pass_cnt = pass_cnt + 1;
                    $display("[PASS] TC1 echo OK: out=%h", out_data);
                end else begin
                    fail_cnt = fail_cnt + 1;
                    $display("[FAIL] TC1 echo mismatch: got=%h exp=%h", out_data, tx_data);
                end
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] TC1: no output after %0d cycles (status=%h busy=%b err=%b)",
                         cyc, status, busy, err);
            end

            // Cleanup
            repeat (10) @(posedge clk);
        end
    endtask

    // TC2: no key loaded -> E_KEY_NOT_LOADED
    task run_tc2;
        begin
            rst_n = 0; repeat (4) @(posedge clk); rst_n = 1; repeat (4) @(posedge clk);
            kl_start = 0; kl_valid = 0; start = 0; in_valid = 0;
            digit_num_zero = 0;

            // Start WITHOUT loading key first
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            repeat (3) @(posedge clk);

            if (err == 1'b1 && err_code == 32'h0000_0012) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] TC2 err_code=0x12 (E_KEY_NOT_LOADED)");
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] TC2 err=%b err_code=0x%08h status=0x%02h", err, err_code, status);
            end
            repeat (10) @(posedge clk);
        end
    endtask

    // TC3: digit_num_zero -> E_MODCHAIN_EXHAUSTED
    task run_tc3;
        begin
            rst_n = 0; repeat (4) @(posedge clk); rst_n = 1; repeat (4) @(posedge clk);
            kl_start = 0; kl_valid = 0; start = 0; in_valid = 0;
            digit_num_zero = 1'b0;

            // Load key first (to ensure key_loaded path is available)
            kl_start = 1'b1;
            @(posedge clk);
            kl_start = 1'b0;
            repeat (2) @(posedge clk);
            kl_valid = 1'b1;
            @(posedge clk);
            kl_valid = 1'b0;
            repeat (3) @(posedge clk);

            // Now start with digit_num_zero
            digit_num_zero = 1'b1;
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            repeat (3) @(posedge clk);

            if (err == 1'b1 && err_code == 32'h0000_0010) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] TC3 err_code=0x10 (E_MODCHAIN_EXHAUSTED)");
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] TC3 err=%b err_code=0x%08h status=0x%02h", err, err_code, status);
            end
            digit_num_zero = 0;
            repeat (10) @(posedge clk);
        end
    endtask

    // TC4: stream_enable=1 no-op - full flow still works
    task run_tc4;
        reg [AXI_DATA_W-1:0] tx_data;
        integer cyc;
        begin
            rst_n = 0; repeat (4) @(posedge clk); rst_n = 1; repeat (4) @(posedge clk);
            kl_start = 0; kl_valid = 0; start = 0; in_valid = 0; in_last = 0;
            digit_num_zero = 0;
            stream_enable = 1'b1;

            // Load key
            kl_start = 1'b1;
            @(posedge clk);
            kl_start = 1'b0;
            repeat (2) @(posedge clk);
            kl_valid = 1'b1;
            @(posedge clk);
            kl_valid = 1'b0;
            repeat (3) @(posedge clk);

            tx_data = 32'h5EA110CC;
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            repeat (2) @(posedge clk);

            in_valid = 1'b1;
            in_data  = tx_data;
            in_last  = 1'b1;
            while (!in_ready) @(posedge clk);
            @(posedge clk);
            in_valid = 1'b0;
            in_last  = 1'b0;

            cyc = 0;
            while (out_valid == 0 && cyc < 200) begin
                @(posedge clk);
                cyc = cyc + 1;
            end

            if (out_valid && out_last && (out_data === tx_data)) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] TC4 stream_enable=1 no-op OK (echo=%h)", out_data);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] TC4 out=%h exp=%h valid=%b last=%b after %0d cy",
                         out_data, tx_data, out_valid, out_last, cyc);
            end
            stream_enable = 1'b0;
            repeat (10) @(posedge clk);
        end
    endtask

endmodule
