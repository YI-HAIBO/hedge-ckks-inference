//=============================================================================
// File    : keyswitch_pipe.v
// Project : 玄算（HEDGE） - A+C Keyswitch Pipeline
// Level   : Module Level (real NTT/INTT datapath)
// Status  : v0.7 - M3 digit decomposition, real pointwise / accumulate
//
// v0.7 change summary (2026-10-23)
//   * PREFLIGHT FIX: the v0.6 POINTWISE / ACCUMULATE loops wrapped their
//     `for ... begin : g_pw end` in an inner `generate ... endgenerate`.
//     A generate KEYWORD PAIR nested inside an already-labelled generate
//     block (g_real_datapath) is rejected by xvlog in Verilog-2001 mode:
//         [VRFC 10-4982] syntax error near 'generate'
//         [VRFC 10-8549] keyword 'generate' used in incorrect context
//     9 errors, all in this file.  Fixed by dropping the inner keywords and
//     keeping only the labelled `for` loop, which is a legal generate item.
//   * REPACK FIX: the wrapper output repack now reuses nptt_stream_unpack.v
//     (round65v33 #101, verified 17/17 against the wrapper golden TB)
//     instead of a hand-rolled one-lane-per-cycle loop.  The unpack module
//     already encodes the proven scatter geometry and the ragged-tail
//     correction (the last NUM_ENGINES beats carry TAIL = 3 valid lanes for
//     NPAR=64, not 5), so the keyswitch inherits both for free.
//   * STORE FIX: v0.6's S_STORE_OUT advanced out_beat_r and re-loaded
//     out_buf from acc_bram[out_beat_r] in the SAME NBA cycle, so the axis
//     re-presented beat 0 and silently dropped the last beat (an off-by-one
//     that would have shipped as "PASS except first/last beat").  The
//     advance now reads out_src_next = acc_bram[out_beat_r + 1].
//   * BRAM COUNT: 7 x FRAME_IN x 256 bit = 7 x 1639 x 256 = ~1,236 Kbit
//     ~= 8.6 of the 36Kb BRAM tiles.  ntt_out_bram holds the raw NTT(c2)
//     and pw_out_bram holds the NTT-domain product; keeping the two apart
//     is what makes the M3.5 k-digit sum a drop-in
//     (pw = SUM_d ntt(d_d) * key_d).
//
// v0.6 change summary (2026-10-23, superseded by v0.7)
//   * REAL_DATAPATH=1 became a genuine 256-bit / NPAR=64 keyswitch
//     datapath: 2x nptt_wrapper (INV=0 forward NTT, INV=1 inverse INTT),
//     explicit engine-interleaved -> natural packing REPACK, real
//     POINTWISE = (a*b) mod q and real ACCUMULATE = (x + c1) mod q.
//   * Beat-geometry split (LIM_IN / LIM_OUT) introduced.  This closed the
//     fatal M2 defect: M2 used beat_limit = 1664 for BOTH the wrapper
//     A-port feed and the wrapper output, but the wrapper A port takes
//     FRAME_IN = 1639 NATURAL beats and returns TOTAL_BEATS = 1664
//     ENGINE-INTERLEAVED beats -- the two packings are not interchangeable,
//     so M2 could not produce a correct NTT even though preflight (a pure
//     parse/elab gate) stayed green.
//
// Keyswitch math (relin / keyswitch, digit d of the decomposed c2):
//       c1' = c1 + a_k * d
//   In this M3 scope NUM_DIGITS = 1, so d == c2, i.e. c1' = c1 + a_k * c2.
//   c0' = c0 + b_k * c2 is the symmetric path and is out of M3 scope.
//   All arithmetic over Z_q,  q = 140737487306753 = 2^47 - 2^20 + 1.
//
// Beat geometry (the reason for two different limits)
//   CPB           = AXI_DATA_W / COEF_WIDTH        = 256/48 = 5 coefs/beat
//   FRAME_IN      = ceil(N / CPB)                  = ceil(8192/5) = 1639
//                   -> nptt_wrapper A-port packing (NATURAL coefficient order)
//   BEATS_PER_ENG = ceil(COEFS_PER_ENGINE / CPB)   = ceil(128/5)  = 26
//   TOTAL_BEATS   = NUM_ENGINES * BEATS_PER_ENG    = 64*26        = 1664
//                   -> nptt_wrapper m_axis packing (ENGINE-INTERLEAVED order)
//   TOTAL_COEFS   = N = 8192
//                   -> the unpack module emits exactly this many coefficients
//
// Wrapper control contract (nptt_wrapper.v, verified by the wrapper golden
// TBs): axi_s_axis_A_TREADY is asserted for the whole fill phase
// (wst_r == W_FILL && wr_cnt < FRAME_IN), and the transform fires on
// `frame_done && armed`, where frame_done comes from wr_cnt reaching
// FRAME_IN and `armed` latches pipe_start in any state.  A_TLAST is
// therefore NOT required to start the transform, but is still asserted on
// the final beat for well-behaved downstream logic.
//
// Resource note (impl-engineer handoff)
//   7 x FRAME_IN x AXI_DATA_W bit arrays = 7 x 1639 x 256 = ~1,236 Kbit
//   (~8.6 of the 36Kb BRAM tiles).  Measured at R (route) the BD wrapper
//   uses 24.5 / 312 tiles (7.85%), so the full keyswitch subtree (2 extra
//   wrappers plus these arrays) stays well inside the device budget.  See
//   hedge-spec/HEDGE-Delivery-Report-v1.0.md §5.3.
//
//   CAVEAT for the impl engineer: pw_out_bram / intt_out_bram / acc_bram
//   take PARTIAL-WORD writes (one 48-bit lane of a 256-bit word per cycle).
//   Vivado cannot give a 256-bit BRAM word a 48-bit write mask, so these
//   arrays are likely to fall back to distributed RAM / shift registers
//   rather than BRAM36E1.  Functional correctness is unaffected; if area or
//   timing suffers, restructure to full-word writes by buffering an entire
//   destination beat in a 5-lane shadow register before committing.
//
// Latency (M3, one keyswitch call, NPAR=64)
//   S_LOAD_INPUT  2 x 1639          beats
//   S_NTT_FWD      ~1641 fill + transform (~28K) + 1664 x 7 unpack
//   S_POINTWISE    1639
//   S_INTT_INV     ~1641 fill + transform (~28K) + 1664 x 7 unpack
//   S_ACCUMULATE   1639
//   S_STORE_OUT    1639
//   The 7 cycles per unpack beat is 5 lanes + 2 handshake bubbles; unpack
//   clears in_vld one cycle after its last lane fires, so m_TREADY cannot
//   be asserted combinationally without adding a bypass.  Not on the
//   critical path for a single transform.
//
// Contract v1.0 compliance
//   * Port list UNCHANGED (axi s/m axis, key_load_*, start, status, busy,
//     err, err_code, stream_enable, digit_num_zero).
//   * stream_enable retained as a no-op (U3 closed, Contract v1.0 §10).
//   * Error codes E_KEY_NOT_LOADED / E_MODCHAIN_EXHAUSTED unchanged.
//   * FSM: 11 states (IDLE, LOAD_KEY, LOAD_INPUT, NTT_FWD, POINTWISE,
//     COMBINE, INTT_INV, ACCUMULATE, STORE_OUT, DONE, ERROR).
//
// Deferred (M3.5 / later)
//   * NUM_DIGITS > 1 (real digit decomposition with k-digit summation).
//   * The 256-bit ac_adapter integration (ac_adapter still instantiates
//     this module at AXI_DATA_W=64, which selects the pass-through build).
//
//=============================================================================
`timescale 1ns / 1ps
`default_nettype wire

`ifndef HEDGE_ERR_CODES_DEFINED
`define HEDGE_ERR_CODES_DEFINED
`define E_KEY_NOT_LOADED     32'h0000_0012
`define E_MODCHAIN_EXHAUSTED 32'h0000_0010
`endif

module keyswitch_pipe #(
    parameter AXI_DATA_W       = 256,
    parameter COEF_WIDTH       = 48,
    parameter NUM_ENGINES      = 64,
    parameter COEFS_PER_ENGINE = (8192 / NUM_ENGINES),          // 128 (NPAR=64)
    parameter CPB              = AXI_DATA_W / COEF_WIDTH,       // 5
    parameter BEATS_PER_ENGINE = (COEFS_PER_ENGINE + CPB - 1) / CPB, // 26
    parameter TOTAL_BEATS      = NUM_ENGINES * BEATS_PER_ENGINE,// 1664 wrapper out
    parameter TOTAL_COEFS      = NUM_ENGINES * COEFS_PER_ENGINE,// 8192
    parameter FRAME_IN         = (TOTAL_COEFS + CPB - 1) / CPB, // 1639 wrapper in
    parameter REAL_DATAPATH    = 1,  // 1 = real NTT/INTT path, 0 = pass-through
    parameter NUM_DIGITS       = 2,  // [M3.5] digit count k of the decomposed c2
    parameter ROM_HEX          = "ntt64_twiddles.hex"  // vestigial: nptt_top self-selects
) (
    input  wire                     clk_main,
    input  wire                     rst_n,

    input  wire                     axi_s_axis_TVALID,
    input  wire [AXI_DATA_W-1:0]    axi_s_axis_TDATA,
    output wire                     axi_s_axis_TREADY,
    input  wire                     axi_s_axis_TLAST,

    // AXI4-Stream: output ciphertext polynomial (c1' for M3)
    output wire                     axi_m_axis_TVALID,
    output wire [AXI_DATA_W-1:0]    axi_m_axis_TDATA,
    input  wire                     axi_m_axis_TREADY,
    output wire                     axi_m_axis_TLAST,

    // Key load (one-time load of the NTT-domain key DIGITS, cached in
    //   BRAM).  NUM_DIGITS consecutive FRAME_IN-beat frames, natural
    //   coefficient order, lane 0 = lowest COEF_WIDTH bits; frame d fills
    //   key_bram_a[d].  key_loaded_r latches on the last beat of the last
    //   frame.
    input  wire                     key_load_start,
    input  wire                     key_load_valid,
    output wire                     key_load_ready,
    input  wire [AXI_DATA_W-1:0]    key_load_TDATA,

    // Control / status
    input  wire                     start,
    output wire [7:0]               status,
    output wire                     busy,
    output wire                     err,
    output wire [31:0]              err_code,

    // Contract v1.0 §10: retained, no-op (U3 closed)
    input  wire                     stream_enable,

    // Modulus-chain-exhausted flag from the modmanager
    input  wire                     digit_num_zero
);

    //==================================================
    // Derived geometry
    //==================================================
    localparam [13:0] LIM_IN   = FRAME_IN[13:0];          // 1639 natural beats
    localparam [13:0] LIM_OUT  = TOTAL_BEATS[13:0];       // 1664 interleaved beats
    localparam [13:0] LIM_COEF = TOTAL_COEFS[13:0];       // 8192 coefficients
    // Natural-order BRAM has FRAME_IN*CPB slots, of which the last
    // NAT_SLOTS - TOTAL_COEFS (=3 for NPAR=64) belong to the ragged
    // tail beat and must be zero-filled explicitly by the write path.
    localparam [13:0] NAT_SLOTS = FRAME_IN * CPB;         // 8195
    localparam [13:0] LAST_NBT  = FRAME_IN - 14'd1;       // 1638
    // The natural coefficient count 8192 does not divide by CPB=5, so
    // the last beat (LAST_NBT) has (TOTAL_COEFS % CPB) = 2 real lanes
    // and (CPB - 2) = 3 padding lanes that must be zero-filled.
    localparam [3:0]  FIRST_PAD_L = TOTAL_COEFS % CPB;    // 2
    localparam [3:0]  LAST_PAD_L  = CPB - 4'd1;            // 4
    // The real datapath is only legal at the wrapper's native 256-bit width.
    localparam REAL_DP = REAL_DATAPATH && (AXI_DATA_W == 256);

    //==================================================
    // CKKS modulus
    //==================================================
    localparam [47:0] Q = 48'd140737487306753;   // 2^47 - 2^20 + 1
    // 1/N mod q = pow(8192, -1, q).  The four-step inverse path uses
    // conjugate twiddle ROMs only, so it produces N*a rather than a
    // (see the note on S_POINTWISE).  Folding N^-1 into the pointwise
    // multiply is legitimate because the INTT is linear over Z_q.
    localparam [47:0] N_INV = 48'd140720307437697;  // = pow(8192, -1, Q)

    //==================================================
    // FSM encoding (11 states)
    //==================================================
    localparam [3:0] S_IDLE       = 4'd0;
    localparam [3:0] S_LOAD_KEY   = 4'd1;
    localparam [3:0] S_LOAD_INPUT = 4'd2;
    localparam [3:0] S_NTT_FWD    = 4'd3;
    localparam [3:0] S_POINTWISE  = 4'd4;
    localparam [3:0] S_COMBINE    = 4'd5;
    localparam [3:0] S_INTT_INV   = 4'd6;
    localparam [3:0] S_ACCUMULATE = 4'd7;
    localparam [3:0] S_STORE_OUT  = 4'd8;
    localparam [3:0] S_DONE       = 4'd9;
    localparam [3:0] S_ERROR      = 4'd15;

    reg [3:0]  state_r      = S_IDLE;
    reg [31:0] err_code_r   = 32'd0;
    reg        key_loaded_r = 1'b0;

    //==================================================
    // Status / handshake
    //==================================================
    assign status           = {3'b000, state_r};
    assign busy             = (state_r != S_IDLE) && (state_r != S_DONE) && (state_r != S_ERROR);
    assign err              = (state_r == S_ERROR);
    assign err_code         = err_code_r;
    assign key_load_ready   = (state_r == S_LOAD_KEY);
    assign axi_s_axis_TREADY = (state_r == S_LOAD_INPUT);

    // Contract v1.0 §10: stream_enable is a retained no-op (U3 closed).
    wire _unused_stream_enable = stream_enable;

    //==================================================
    // BRAMs (all sized FRAME_IN, natural coefficient order throughout)
    //==================================================
    // [M3.5] one key bank per digit: key_bram_a[d] = NTT(key_d)
    // [2026-10-25] Flat one-dimensional key bank.  A [NUM_DIGITS][FRAME_IN]
    // 2-D array is treated by Vivado as a "3D-RAM" and dissolved into
    // NUM_DIGITS*FRAME_IN*AXI_DATA_W registers ([Synth 8-11357]: 419,584 FFs
    // at NUM_DIGITS=1).  Flattening keeps one write port with one access per
    // cycle, which does infer block RAM.  Index = digit*FRAME_IN + beat.
    (* ram_style = "block" *) reg [AXI_DATA_W-1:0] key_bram_a
                                 [0:NUM_DIGITS*FRAME_IN-1];
    reg [AXI_DATA_W-1:0] in_bram       [0:FRAME_IN-1];  // c2 (or pass-through data)
    reg [AXI_DATA_W-1:0] base_bram     [0:FRAME_IN-1];  // c1
    reg [AXI_DATA_W-1:0] ntt_out_bram  [0:FRAME_IN-1];  // NTT(c2), natural order
    reg [AXI_DATA_W-1:0] pw_out_bram   [0:FRAME_IN-1];  // NTT(c2) * NTT(a_k) mod q
    reg [AXI_DATA_W-1:0] intt_out_bram [0:FRAME_IN-1];  // INTT(product) + padding zeros
    reg [AXI_DATA_W-1:0] acc_bram      [0:FRAME_IN-1];  // INTT(product) + c1, mod q

    //==================================================
    // Counters / latches
    //==================================================
    // [M3.5] digit bookkeeping
    //   key_digit_r  : frame counter during S_LOAD_KEY (0..NUM_DIGITS-1)
    //   load_phase_r : input-stream segment counter during S_LOAD_INPUT
    //                  (0 = c1, 1..NUM_DIGITS = digit phase-1); survives
    //                  the NTT/POINTWISE round trips between digits.
    //   proc_digit_r : digit index currently being transformed/accumulated
    //                  (latched at the end of each digit segment)
    reg [7:0]  key_digit_r   = 8'd0;
    reg [7:0]  load_phase_r  = 8'd0;
    reg [7:0]  proc_digit_r  = 8'd0;
    reg [13:0] key_beat_r    = 14'd0;
    reg [13:0] in_beat_r     = 14'd0;
    reg [13:0] pt_nbeats_r   = 14'd0;
    reg [13:0] ntt_replay_r  = 14'd0;
    reg [13:0] intt_replay_r = 14'd0;
    reg [13:0] ntt_outcnt_r  = 14'd0;   // interleaved beats accepted from the wrapper
    reg [13:0] ntt_coefcnt_r = 14'd0;   // natural coefficients emitted by the unpack
    reg [13:0] intt_outcnt_r = 14'd0;
    reg [13:0] intt_coefcnt_r= 14'd0;
    // Pad-write counters: after the unpack has emitted all TOTAL_COEFS
    // real coefficients, (NAT_SLOTS - TOTAL_COEFS) additional zero
    // writes fill the trailing partial natural beat so the wrapper's
    // A-port replay never sees an X lane.
    reg [3:0]  ntt_pad_r     = 4'd0;
    reg [3:0]  intt_pad_r    = 4'd0;
    reg [13:0] pw_cnt_r      = 14'd0;
    reg [13:0] acc_cnt_r     = 14'd0;
    reg [13:0] out_beat_r    = 14'd0;

    reg [AXI_DATA_W-1:0] out_buf_r = {AXI_DATA_W{1'b0}};
    reg                  out_vld_r = 1'b0;
    reg                  out_lst_r = 1'b0;

    integer pw_i;
    integer acc_i;
    genvar  gi;

    //==================================================
    // Wrapper + unpack interface wires (module scope so the FSM and the
    // datapath blocks can reference them in both builds; driven inside the
    // generate below, with explicit zero fallbacks when not built)
    //==================================================
    wire                ntt_A_TREADY, ntt_m_TVALID, ntt_m_TLAST;
    wire [AXI_DATA_W-1:0] ntt_m_TDATA;
    wire                intt_A_TREADY, intt_m_TVALID, intt_m_TLAST;
    wire [AXI_DATA_W-1:0] intt_m_TDATA;
    wire                ntt_unpack_in_TREADY, intt_unpack_in_TREADY;
    wire                ntt_unpack_out_TVALID, intt_unpack_out_TVALID;
    wire [63:0]         ntt_unpack_out_TDATA, intt_unpack_out_TDATA;
    // Scatter positions exposed by the unpack instances.  See the nptt_stream_unpack
    // port header for the inverse mapping: gidx = e + N1*k2,
    //   e  = beat % NUM_ENGINES,  ib = beat / NUM_ENGINES,
    //   k2 = ib*CPB + lane.
    wire [15:0]         ntt_unpack_cur_beat, intt_unpack_cur_beat;
    wire [3:0]          ntt_unpack_cur_lane, intt_unpack_cur_lane;
    // [2026-10-25] These eight were only ever IMPLICIT nets: iverilog and
    // Vivado both auto-create a 1-bit wire for an undeclared identifier
    // ([Synth 8-11241]), which silently hid the omission.  Declaring them
    // keeps the module lint-clean and safe under `default_nettype none.
    wire                ntt_A_TVALID, ntt_A_TLAST, ntt_pipe_start, ntt_m_TREADY;
    wire                intt_A_TVALID, intt_A_TLAST, intt_pipe_start,
                        intt_m_TREADY;

    //--------------------------------------------------
    // Wrapper A-port driving and m-axis acceptance.
    //   A_TVALID/A_TLAST/pipe_start are combinational off state_r so the
    //   wrapper fill and the replay counter share one set of clock values
    //   (no cross-always race).  m_TREADY is held while the unpack module
    //   still holds a beat, so the wrapper cannot present a beat we have
    //   not finished draining -- and drops when we leave the state, so no
    //   stray handshake can ever latch phantom data into a parked unpack.
    //--------------------------------------------------
    assign ntt_A_TVALID   = REAL_DP && (state_r == S_NTT_FWD)
                            && (ntt_replay_r <  LIM_IN);
    assign ntt_A_TLAST    = REAL_DP && (state_r == S_NTT_FWD)
                            && (ntt_replay_r == LIM_IN - 14'd1);
    assign ntt_pipe_start = REAL_DP && (state_r == S_NTT_FWD);
    assign ntt_m_TREADY   = REAL_DP && (state_r == S_NTT_FWD)
                            && ntt_unpack_in_TREADY;

    assign intt_A_TVALID  = REAL_DP && (state_r == S_INTT_INV)
                            && (intt_replay_r <  LIM_IN);
    assign intt_A_TLAST   = REAL_DP && (state_r == S_INTT_INV)
                            && (intt_replay_r == LIM_IN - 14'd1);
    assign intt_pipe_start= REAL_DP && (state_r == S_INTT_INV);
    assign intt_m_TREADY  = REAL_DP && (state_r == S_INTT_INV)
                            && intt_unpack_in_TREADY;

    // A-port TDATA is selected by the replay counter (combinational read).
    wire [AXI_DATA_W-1:0] ntt_A_TDATA  = in_bram[ntt_replay_r];
    wire [AXI_DATA_W-1:0] intt_A_TDATA = pw_out_bram[intt_replay_r];

    // Unpack output is gated by our own ack: we only pull a coefficient
    // while the corresponding transform state is active.
    wire ntt_unpack_out_TREADY  = REAL_DP && (state_r == S_NTT_FWD);
    wire intt_unpack_out_TREADY = REAL_DP && (state_r == S_INTT_INV);

    //==================================================
    // Output source mux: real datapath -> acc_bram, pass-through -> in_bram
    //   out_src  : the beat currently addressed by out_beat_r (first prime)
    //   out_src_next : the beat addressed by out_beat_r + 1 (advance path).
    //   Splitting the two avoids the v0.6 off-by-one where the advance
    //   re-loaded the beat that was already on the axis.
    //==================================================
    wire [13:0] out_lim      = REAL_DP ? LIM_IN : pt_nbeats_r;
    wire [13:0] out_beat_next = out_beat_r + 14'd1;
    wire [AXI_DATA_W-1:0] out_src;
    wire [AXI_DATA_W-1:0] out_src_next;
    generate
    if (REAL_DP) begin : g_out_src_real
        assign out_src      = acc_bram[out_beat_r];
        assign out_src_next = acc_bram[out_beat_next];
    end else begin : g_out_src_pt
        assign out_src      = in_bram[out_beat_r];
        assign out_src_next = in_bram[out_beat_next];
    end
    endgenerate

    assign axi_m_axis_TVALID = (state_r == S_STORE_OUT) && out_vld_r;
    assign axi_m_axis_TLAST  = (state_r == S_STORE_OUT) && out_vld_r && out_lst_r;
    assign axi_m_axis_TDATA  = out_buf_r;

    //==================================================
    // Real datapath: wrappers + unpack + pointwise + accumulate
    //==================================================
    generate
    if (REAL_DP) begin : g_real_datapath

        //------------------------------------------------------
        // Forward NTT (INV=0)
        //------------------------------------------------------
        nptt_wrapper #(
            .AXI_DATA_W       (AXI_DATA_W),
            .COEF_WIDTH       (COEF_WIDTH),
            .NUM_ENGINES      (NUM_ENGINES),
            .COEFS_PER_ENGINE (COEFS_PER_ENGINE),
            .ROM_HEX          (ROM_HEX),
            .INV              (0)
        ) u_ntt_fwd (
            .clk_main           (clk_main),
            .rst_n              (rst_n),
            .axi_s_axis_A_TVALID (ntt_A_TVALID),
            .axi_s_axis_A_TDATA  (ntt_A_TDATA),
            .axi_s_axis_A_TREADY (ntt_A_TREADY),
            .axi_s_axis_A_TLAST  (ntt_A_TLAST),
            .axi_s_axis_B_TVALID (1'b0),
            .axi_s_axis_B_TDATA  ({AXI_DATA_W{1'b0}}),
            .axi_s_axis_B_TREADY (),
            .axi_s_axis_B_TLAST  (1'b0),
            .axi_m_axis_TVALID   (ntt_m_TVALID),
            .axi_m_axis_TDATA    (ntt_m_TDATA),
            .axi_m_axis_TREADY   (ntt_m_TREADY),
            .axi_m_axis_TLAST    (ntt_m_TLAST),
            .mode               (2'b00),
            .pipe_start         (ntt_pipe_start),
            .pipe_stall         (1'b0),
            .status             (),
            .busy               (),
            .err                (),
            .err_code           ()
        );

        //------------------------------------------------------
        // Inverse INTT (INV=1) - uses dedicated INTT wrapper with reverse pipeline
        //------------------------------------------------------
        nptt_intt_wrapper #(
            .AXI_DATA_W       (AXI_DATA_W),
            .COEF_WIDTH       (COEF_WIDTH),
            .NUM_ENGINES      (NUM_ENGINES),
            .COEFS_PER_ENGINE (COEFS_PER_ENGINE),
            .ROM_HEX          (ROM_HEX)
        ) u_intt_inv (
            .clk_main           (clk_main),
            .rst_n              (rst_n),
            .axi_s_axis_A_TVALID (intt_A_TVALID),
            .axi_s_axis_A_TDATA  (intt_A_TDATA),
            .axi_s_axis_A_TREADY (intt_A_TREADY),
            .axi_s_axis_A_TLAST  (intt_A_TLAST),
            .axi_s_axis_B_TVALID (1'b0),
            .axi_s_axis_B_TDATA  ({AXI_DATA_W{1'b0}}),
            .axi_s_axis_B_TREADY (),
            .axi_s_axis_B_TLAST  (1'b0),
            .axi_m_axis_TVALID   (intt_m_TVALID),
            .axi_m_axis_TDATA    (intt_m_TDATA),
            .axi_m_axis_TREADY   (intt_m_TREADY),
            .axi_m_axis_TLAST    (intt_m_TLAST),
            .mode               (2'b00),
            .pipe_start         (intt_pipe_start),
            .pipe_stall         (1'b0),
            .status             (),
            .busy               (),
            .err                (),
            .err_code           ()
        );

        //------------------------------------------------------
        // Output repack (reuses the #101-verified scatter inverse).
        //   Wrapper m_axis beat b, lane l  ->  k2 = (b/N1)*CPB + l,
        //                                     gidx = e + N1*k2,
        //                                     e = b % N1
        //   One coefficient is emitted per cycle, natural order, low
        //   COEF_WIDTH bits of out_TDATA.
        //------------------------------------------------------
        nptt_stream_unpack #(
            .IN_DATA_W     (AXI_DATA_W),
            .OUT_DATA_W    (64),
            .IN_COEF_W     (COEF_WIDTH),
            .NUM_ENGINES   (NUM_ENGINES),
            .BEATS_PER_ENG (BEATS_PER_ENGINE),
            .COEFS_PER_ENG (COEFS_PER_ENGINE)
        ) u_ntt_unpack (
            .clk_main         (clk_main),
            .rst_n            (rst_n),
            .in_TVALID        (ntt_m_TVALID),
            .in_TDATA         (ntt_m_TDATA),
            .in_TLAST         (ntt_m_TLAST),
            .in_TREADY        (ntt_unpack_in_TREADY),
            .out_TVALID       (ntt_unpack_out_TVALID),
            .out_TDATA        (ntt_unpack_out_TDATA),
            .out_TLAST        (),
            .out_TREADY       (ntt_unpack_out_TREADY),
            .unpack_beats     (),
            .unpack_coefs     (),
            .unpack_full      (),
            .unpack_last_seen (),
            .unpack_err       (),
            .cur_beat         (ntt_unpack_cur_beat),
            .cur_lane         (ntt_unpack_cur_lane)
        );

        nptt_stream_unpack #(
            .IN_DATA_W     (AXI_DATA_W),
            .OUT_DATA_W    (64),
            .IN_COEF_W     (COEF_WIDTH),
            .NUM_ENGINES   (NUM_ENGINES),
            .BEATS_PER_ENG (BEATS_PER_ENGINE),
            .COEFS_PER_ENG (COEFS_PER_ENGINE)
        ) u_intt_unpack (
            .clk_main         (clk_main),
            .rst_n            (rst_n),
            .in_TVALID        (intt_m_TVALID),
            .in_TDATA         (intt_m_TDATA),
            .in_TLAST         (intt_m_TLAST),
            .in_TREADY        (intt_unpack_in_TREADY),
            .out_TVALID       (intt_unpack_out_TVALID),
            .out_TDATA        (intt_unpack_out_TDATA),
            .out_TLAST        (),
            .out_TREADY       (intt_unpack_out_TREADY),
            .unpack_beats     (),
            .unpack_coefs     (),
            .unpack_full      (),
            .unpack_last_seen (),
            .unpack_err       (),
            .cur_beat         (intt_unpack_cur_beat),
            .cur_lane         (intt_unpack_cur_lane)
        );

        // NOTE: ntt_unpack_in_TREADY / ntt_unpack_out_TVALID /
        // ntt_unpack_out_TDATA are driven by the instance port connections
        // above (lines for .in_TREADY / .out_TVALID / .out_TDATA) and are
        // tied to zero in the pass-through stub below.  Hierarchical reads
        // of the instance's port nets (u_ntt_unpack.unpack_in_TREADY) are
        // neither legal nor synthesizable -- the port connection IS the
        // association -- so no explicit assign is needed here.

        //------------------------------------------------------
        // Replay counters (drive the wrapper A ports from BRAM)
        //------------------------------------------------------
        always @(posedge clk_main or negedge rst_n) begin
            if (!rst_n)                    ntt_replay_r <= 14'd0;
            else if (state_r != S_NTT_FWD) ntt_replay_r <= 14'd0;
            else if (ntt_A_TREADY && (ntt_replay_r < LIM_IN))
                                         ntt_replay_r <= ntt_replay_r + 14'd1;
        end

        always @(posedge clk_main or negedge rst_n) begin
            if (!rst_n)                     intt_replay_r <= 14'd0;
            else if (state_r != S_INTT_INV) intt_replay_r <= 14'd0;
            else if (intt_A_TREADY && (intt_replay_r < LIM_IN))
                                           intt_replay_r <= intt_replay_r + 14'd1;
        end

        //------------------------------------------------------
        // Forward-domain repack: scatter (b,l) -> natural gidx -> beat/lane.
        //
        //   The wrapper m_axis emits coefficients in the nptt_scatter
        //   round-robin order (engine e, block ib, lane l):
        //       gidx_scatter = e + N1 * (ib * CPB + l)
        //   where  e  = cur_beat % NUM_ENGINES,
        //          ib = cur_beat / NUM_ENGINES.
        //   The unpack module is a width adapter only -- it preserves this
        //   scatter order -- so writing coefficient k to beat k/CPB /
        //   lane k%CPB would land scatter-order values into natural
        //   positions and scramble the entire NTT(c2) matrix (this was
        //   the v0.7 numeric bug that TC3 caught 8192/8192 wrong).
        //
        //   We therefore compute the NATURAL gidx from the scatter
        //   position the unpack tells us, and write to (gidx/CPB,
        //   gidx%CPB).  All 8192 valid natural positions are then
        //   filled exactly once each.
        //
        //   Padding:  ntt_out_bram has FRAME_IN * CPB = 1639 * 5 = 8195
        //   slot positions, of which the last 3 (natural gidx 8192..8194)
        //   correspond to coefficients beyond N = 8192 and are never
        //   written by the unpack path.  Without an explicit zero-fill
        //   those lanes stay at the BRAM's X initial value and, when the
        //   wrapper A-port replays ntt_out_bram into the INTT, the X
        //   propagates through the entire inverse transform.  A separate
        //   ntt_pad_r counter emits the 3 pad writes immediately after
        //   the real stream completes.
        //------------------------------------------------------
        wire [15:0]  ntt_scatter_b = ntt_unpack_cur_beat;
        wire [3:0]   ntt_scatter_l = ntt_unpack_cur_lane;
        wire [15:0]  ntt_e         = ntt_scatter_b % NUM_ENGINES;
        wire [15:0]  ntt_ib        = ntt_scatter_b / NUM_ENGINES;
        wire [15:0]  ntt_k2        = ntt_ib * CPB + ntt_scatter_l;
        wire [15:0]  ntt_nat_gidx  = ntt_e + NUM_ENGINES * ntt_k2;
        wire [13:0]  ntt_rp_nbt    = ntt_nat_gidx[13:0] / CPB;
        wire [13:0]  ntt_rp_nl     = ntt_nat_gidx[13:0] % CPB;
        wire [COEF_WIDTH-1:0] ntt_coef_val = ntt_unpack_out_TDATA[COEF_WIDTH-1:0];

        // Pad phase writes to lanes FIRST_PAD_L..LAST_PAD_L of beat
        // LAST_NBT, which correspond to natural gidx 8192..8194.
        wire [13:0] ntt_pad_off_raw = LAST_NBT * CPB + FIRST_PAD_L + ntt_pad_r;
        wire [13:0] ntt_pad_gidx    = ntt_pad_off_raw[13:0];
        wire [3:0]  ntt_pad_l_raw   = FIRST_PAD_L + ntt_pad_r;
        wire [2:0]  ntt_pad_l       = ntt_pad_l_raw[2:0];
        wire        ntt_pad_valid   = (ntt_pad_r <= (LAST_PAD_L - FIRST_PAD_L));

        always @(posedge clk_main or negedge rst_n) begin
            if (!rst_n) begin
                ntt_outcnt_r  <= 14'd0;
                ntt_coefcnt_r <= 14'd0;
                ntt_pad_r     <= 4'd0;
            end else if (state_r != S_NTT_FWD) begin
                ntt_outcnt_r  <= 14'd0;
                ntt_coefcnt_r <= 14'd0;
                ntt_pad_r     <= 4'd0;
            end else begin
                if (ntt_m_TVALID && ntt_m_TREADY)
                    ntt_outcnt_r  <= ntt_outcnt_r + 14'd1;
                if (ntt_unpack_out_TVALID && ntt_unpack_out_TREADY) begin
                    ntt_out_bram[ntt_rp_nbt][ntt_rp_nl[2:0]*COEF_WIDTH +: COEF_WIDTH]
                        <= ntt_coef_val % Q;
                    ntt_coefcnt_r <= ntt_coefcnt_r + 14'd1;
                end
                // Pad writes happen strictly after the last real emission:
                // the counter has finished but state_r is still S_NTT_FWD
                // (the FSM advance is one NBA cycle later, gated on
                // ntt_coefcnt_r >= LIM_COEF && ntt_pad_r ==
                // (LAST_PAD_L - FIRST_PAD_L)).
                if (ntt_coefcnt_r >= LIM_COEF && ntt_pad_valid) begin
                    ntt_out_bram[LAST_NBT]
                        [ntt_pad_l*COEF_WIDTH +: COEF_WIDTH]
                        <= {COEF_WIDTH{1'b0}};
                    ntt_pad_r <= ntt_pad_r + 4'd1;
                end
            end
        end

        //------------------------------------------------------
        // INTT-domain repack: scatter (b,l) -> natural gidx -> beat/lane.
        //   Same fix as ntt_out_bram: the wrapper emits the inverse
        //   transform in engine-interleaved scatter order and we must
        //   apply the scatter inverse before writing to intt_out_bram.
        //   Without this fix INTT(pw_out) values would land at wrong
        //   natural positions, producing a garbled c1'.
        //------------------------------------------------------
        wire [15:0]  intt_scatter_b = intt_unpack_cur_beat;
        wire [3:0]   intt_scatter_l = intt_unpack_cur_lane;
        wire [15:0]  intt_e         = intt_scatter_b % NUM_ENGINES;
        wire [15:0]  intt_ib        = intt_scatter_b / NUM_ENGINES;
        wire [15:0]  intt_k2        = intt_ib * CPB + intt_scatter_l;
        // [2026-10-24 BUG FIX] INTT 输出是时域：(engine n1, intra n2) 持有
        //   a[n1*N2 + n2]（连续收集），不是正向 NTT 频谱输出的 stride 映射
        //   (k1 + N1*k2)。旧代码沿用正向 stride 映射，导致 intt_out[g] 实际
        //   落在 prod[128*g] 的位置 —— 值正确、位置全错。
        wire [15:0]  intt_nat_gidx  = intt_e * COEFS_PER_ENGINE + intt_k2;
        // scatter 帧第 26 beat 的 lane 3/4 是 intra 128/129 零填充；连续映射下
        //   它们会落到 natural 128(e+1) 与真实数据冲突，必须丢弃。
        wire         intt_intra_valid = (intt_k2 < COEFS_PER_ENGINE);
        wire [13:0]  intt_rp_nbt    = intt_nat_gidx[13:0] / CPB;
        wire [13:0]  intt_rp_nl     = intt_nat_gidx[13:0] % CPB;
        wire [COEF_WIDTH-1:0] intt_coef_val = intt_unpack_out_TDATA[COEF_WIDTH-1:0];

        wire intt_pad_valid_mask = (intt_pad_r <= (LAST_PAD_L - FIRST_PAD_L));
        wire [3:0] intt_pad_l_raw = FIRST_PAD_L + intt_pad_r;
        wire [2:0] intt_pad_l     = intt_pad_l_raw[2:0];

        always @(posedge clk_main or negedge rst_n) begin
            if (!rst_n) begin
                intt_outcnt_r   <= 14'd0;
                intt_coefcnt_r  <= 14'd0;
                intt_pad_r      <= 4'd0;
            end else if (state_r != S_INTT_INV) begin
                intt_outcnt_r   <= 14'd0;
                intt_coefcnt_r  <= 14'd0;
                intt_pad_r      <= 4'd0;
            end else begin
                if (intt_m_TVALID && intt_m_TREADY)
                    intt_outcnt_r <= intt_outcnt_r + 14'd1;
                if (intt_unpack_out_TVALID && intt_unpack_out_TREADY
                    && intt_intra_valid) begin
                    intt_out_bram[intt_rp_nbt][intt_rp_nl[2:0]*COEF_WIDTH +: COEF_WIDTH]
                        <= intt_coef_val % Q;
                    intt_coefcnt_r <= intt_coefcnt_r + 14'd1;
                end
                if (intt_coefcnt_r >= LIM_COEF && intt_pad_valid_mask) begin
                    intt_out_bram[LAST_NBT]
                        [intt_pad_l*COEF_WIDTH +: COEF_WIDTH]
                        <= {COEF_WIDTH{1'b0}};
                    intt_pad_r <= intt_pad_r + 4'd1;
                end
            end
        end

        //------------------------------------------------------
        // POINTWISE [M3.5]: cross-digit accumulation.
        //   pw_out[b][l] = SUM_d ( N^-1 * NTT(d_d)[b][l]
        //                          * NTT(key_d)[b][l] )  mod Q
        //   computed one digit per visit: digit 0 SEEDS the accumulator,
        //   digits 1..k-1 READ-MODIFY-WRITE the running sum.  Reads
        //   ntt_out_bram + key_bram_a[proc_digit_r], writes pw_out_bram --
        //   the pw_out read (pw_prev) and write are the same array, but the
        //   read is combinational and the write is NBA, so the
        //   read-modify-write sees the pre-update value.
        //   Doing the multiply inside the unpack pass AND here would square
        //   the key, so exactly one of the two places may perform it.
        //------------------------------------------------------
        // pw_prod      = NTT(d_d) * NTT(key_d), full 96-bit product
        // pw_prod_q    = pw_prod mod q
        // pw_prod_norm = pw_prod_q * N_INV, full 96-bit product
        // pw_term      = pw_prod_norm mod q   (the N^-1-scaled term)
        // pw_sum       = pw_term + pw_prev    (49-bit, reduced on write)
        //
        // The extra multiply is the 1/N normalisation that the four-step
        // inverse path does not apply on its own.  Folding N^-1 into the
        // per-term multiply is legitimate because the INTT is Z_q-linear:
        // INTT(N^-1 * SUM t_d) = SUM INTT(N^-1 t_d) = SUM INTT_true(t_d).
        // Both qreduces share the 5-wide (CPB) lane structure of the beat,
        // so no datapath widening is required -- one extra 48x48 multiply
        // per lane per beat beyond the v0.7 pointwise.
        wire [COEF_WIDTH-1:0]   pw_key_l    [0:CPB-1];
        wire [COEF_WIDTH-1:0]   pw_prev     [0:CPB-1];
        wire [2*COEF_WIDTH-1:0] pw_prod     [0:CPB-1];
        wire [2*COEF_WIDTH-1:0] pw_prod_q   [0:CPB-1];
        wire [2*COEF_WIDTH-1:0] pw_prod_norm[0:CPB-1];
        wire [COEF_WIDTH-1:0]   pw_term     [0:CPB-1];
        wire [COEF_WIDTH:0]     pw_sum      [0:CPB-1];
        for (gi = 0; gi < CPB; gi = gi + 1) begin : g_pw
            assign pw_key_l[gi] = key_bram_a[proc_digit_r*FRAME_IN + pw_cnt_r]
                                       [gi*COEF_WIDTH +: COEF_WIDTH];
            assign pw_prev[gi]  = pw_out_bram[pw_cnt_r]
                                       [gi*COEF_WIDTH +: COEF_WIDTH];
            assign pw_prod[gi]  = { {COEF_WIDTH{1'b0}},
                                    ntt_out_bram[pw_cnt_r][gi*COEF_WIDTH +: COEF_WIDTH] } *
                                  { {COEF_WIDTH{1'b0}}, pw_key_l[gi] };
            assign pw_prod_q[gi]    = pw_prod[gi] % Q;
            assign pw_prod_norm[gi] = { {COEF_WIDTH{1'b0}}, pw_prod_q[gi] } *
                                      { {COEF_WIDTH{1'b0}}, N_INV };
            assign pw_term[gi] = pw_prod_norm[gi] % Q;   // 96-bit modulus, result < Q
            assign pw_sum[gi]  = {1'b0, pw_term[gi]} + {1'b0, pw_prev[gi]};
        end

        always @(posedge clk_main or negedge rst_n) begin
            if (!rst_n)
                pw_cnt_r <= 14'd0;
            else if (state_r != S_POINTWISE)
                pw_cnt_r <= 14'd0;
            else if (pw_cnt_r < LIM_IN)
                pw_cnt_r <= pw_cnt_r + 14'd1;
        end

        // [2026-10-25] The memory write lives in a RESET-FREE process:
        // Vivado cannot infer a block RAM that is sensitive to an
        // asynchronous reset ([Synth 8-4767] -> [Synth 8-3391]).
        always @(posedge clk_main) begin
            if (state_r == S_POINTWISE && pw_cnt_r < LIM_IN) begin
                for (pw_i = 0; pw_i < CPB; pw_i = pw_i + 1) begin
                    if (pw_cnt_r*CPB + pw_i < TOTAL_COEFS) begin
                        if (proc_digit_r == 8'd0)
                            // digit 0: seed the accumulator
                            pw_out_bram[pw_cnt_r][pw_i*COEF_WIDTH +: COEF_WIDTH]
                                <= pw_term[pw_i];
                        else
                            // digits 1..k-1: accumulate into the running sum
                            pw_out_bram[pw_cnt_r][pw_i*COEF_WIDTH +: COEF_WIDTH]
                                <= pw_sum[pw_i] % Q;
                    end else
                        // padding lane of the tail beat: forced to 0 every
                        // digit so the sum cannot gain a phantom term
                        pw_out_bram[pw_cnt_r][pw_i*COEF_WIDTH +: COEF_WIDTH]
                            <= {COEF_WIDTH{1'b0}};
                end
            end
        end

        //------------------------------------------------------
        // ACCUMULATE: acc[b][l] = (intt_out[b][l] + base[b][l]) mod Q
        //------------------------------------------------------
        wire [COEF_WIDTH+1:0] acc_sum [0:CPB-1];
        for (gi = 0; gi < CPB; gi = gi + 1) begin : g_acc
            assign acc_sum[gi] =
                {1'b0, intt_out_bram[acc_cnt_r][gi*COEF_WIDTH +: COEF_WIDTH]} +
                {1'b0, base_bram[acc_cnt_r][gi*COEF_WIDTH +: COEF_WIDTH]};
        end

        always @(posedge clk_main or negedge rst_n) begin
            if (!rst_n)
                acc_cnt_r <= 14'd0;
            else if (state_r != S_ACCUMULATE)
                acc_cnt_r <= 14'd0;
            else if (acc_cnt_r < LIM_IN)
                acc_cnt_r <= acc_cnt_r + 14'd1;
        end

        // [2026-10-25] The memory write lives in a RESET-FREE process:
        // Vivado cannot infer a block RAM that is sensitive to an
        // asynchronous reset ([Synth 8-4767] -> [Synth 8-3391]).
        always @(posedge clk_main) begin
            if (state_r == S_ACCUMULATE && acc_cnt_r < LIM_IN) begin
                for (acc_i = 0; acc_i < CPB; acc_i = acc_i + 1) begin
                    if (acc_cnt_r*CPB + acc_i < TOTAL_COEFS)
                        acc_bram[acc_cnt_r][acc_i*COEF_WIDTH +: COEF_WIDTH]
                            <= acc_sum[acc_i] % Q;
                    else
                        acc_bram[acc_cnt_r][acc_i*COEF_WIDTH +: COEF_WIDTH]
                            <= {COEF_WIDTH{1'b0}};
                end
            end
        end

    end else begin : g_no_real

        //------------------------------------------------------
        // Wrappers not built: keep every interface wire deterministic so
        // the FSM and the counter blocks elaborate identically.
        //------------------------------------------------------
        assign ntt_A_TREADY  = 1'b0;
        assign ntt_m_TVALID  = 1'b0;
        assign ntt_m_TDATA   = {AXI_DATA_W{1'b0}};
        assign ntt_m_TLAST   = 1'b0;
        assign ntt_unpack_in_TREADY  = 1'b0;
        assign ntt_unpack_out_TVALID = 1'b0;
        assign ntt_unpack_out_TDATA  = 64'd0;
        assign ntt_unpack_cur_beat   = 16'd0;
        assign ntt_unpack_cur_lane   = 4'd0;

        assign intt_A_TREADY = 1'b0;
        assign intt_m_TVALID = 1'b0;
        assign intt_m_TDATA  = {AXI_DATA_W{1'b0}};
        assign intt_m_TLAST  = 1'b0;
        assign intt_unpack_in_TREADY  = 1'b0;
        assign intt_unpack_out_TVALID = 1'b0;
        assign intt_unpack_out_TDATA  = 64'd0;
        assign intt_unpack_cur_beat   = 16'd0;
        assign intt_unpack_cur_lane   = 4'd0;

    end
    endgenerate

    //==================================================
    // Key load (shared: both builds cache the key digits in key_bram_a)
    //   [M3.5] NUM_DIGITS consecutive frames: frame d fills key_bram_a[d].
    //   key_loaded_r latches only on the last beat of the LAST frame, so
    //   the FSM stays in S_LOAD_KEY until all k banks are cached.
    //==================================================
    always @(posedge clk_main or negedge rst_n) begin
        if (!rst_n) begin
            key_beat_r   <= 14'd0;
            key_digit_r  <= 8'd0;
            key_loaded_r <= 1'b0;
        end else if (state_r == S_IDLE && key_load_start) begin
            // (re)arm a key load
            key_beat_r   <= 14'd0;
            key_digit_r  <= 8'd0;
            key_loaded_r <= 1'b0;
        end else if (state_r == S_LOAD_KEY && key_load_valid && key_load_ready) begin
            if (key_beat_r == LIM_IN - 14'd1) begin
                if (key_digit_r >= NUM_DIGITS - 1) begin
                    key_loaded_r <= 1'b1;
                    key_beat_r   <= 14'd0;
                    key_digit_r  <= 8'd0;
                end else begin
                    key_beat_r   <= 14'd0;
                    key_digit_r  <= key_digit_r + 8'd1;
                end
            end else begin
                key_beat_r <= key_beat_r + 14'd1;
            end
        end else if (state_r != S_LOAD_KEY) begin
            key_beat_r  <= 14'd0;
            key_digit_r <= 8'd0;
        end
    end

    // [2026-10-25] Key-bank write lives in its own RESET-FREE process: a
    // memory written from a process sensitive to an asynchronous reset cannot
    // map to block RAM ([Synth 8-3391]).  One write per cycle -> 1-port BRAM.
    always @(posedge clk_main) begin
        if (state_r == S_LOAD_KEY && key_load_valid && key_load_ready)
            key_bram_a[key_digit_r*FRAME_IN + key_beat_r] <= key_load_TDATA;
    end

    //==================================================
    // Input load (shared)
    //   Real datapath  : (1 + NUM_DIGITS) x FRAME_IN beats, consumed in
    //                    segments.  Segment 0 is c1 -> base_bram.  Segment
    //                    p (1..NUM_DIGITS) is digit (p-1) -> in_bram; the
    //                    FSM then leaves S_LOAD_INPUT (TREADY drops, the
    //                    host stalls on backpressure), runs the NTT_FWD +
    //                    POINTWISE round for that digit, and returns to
    //                    S_LOAD_INPUT for the next segment.  load_phase_r
    //                    tracks the segment and survives the transform
    //                    round trips; in_beat_r restarts at 0 per segment.
    //   Pass-through   : all beats -> in_bram
    //==================================================
    wire input_hs       = axi_s_axis_TVALID && axi_s_axis_TREADY;
    wire input_seg_last = input_hs && (in_beat_r == LIM_IN - 14'd1);

    always @(posedge clk_main or negedge rst_n) begin
        if (!rst_n) begin
            in_beat_r    <= 14'd0;
            pt_nbeats_r  <= 14'd0;
            load_phase_r <= 8'd0;
            proc_digit_r <= 8'd0;
        // pt_nbeats_r feeds out_lim on the pass-through build, so it must
        // survive S_STORE_OUT as well as every other non-load state; see
        // the note above the shared output store block.
        end else if (state_r == S_IDLE) begin
            // arm a fresh run: c1 segment first, digit 0 first
            in_beat_r    <= 14'd0;
            pt_nbeats_r  <= 14'd0;
            load_phase_r <= 8'd0;
            proc_digit_r <= 8'd0;
        end else if (state_r != S_LOAD_INPUT && state_r != S_STORE_OUT) begin
            in_beat_r   <= 14'd0;
            pt_nbeats_r <= 14'd0;
            // load_phase_r / proc_digit_r KEEP their values across the
            // NTT/POINTWISE digit round trips.
        end else if (input_hs) begin
            if (REAL_DP) begin
                if (input_seg_last) begin
                    in_beat_r <= 14'd0;
                    if (load_phase_r == 8'd0) begin
                        // c1 segment done; first digit segment comes next
                        load_phase_r <= 8'd1;
                    end else begin
                        // digit (load_phase_r-1) captured: hand it to the
                        // transform round, advance the segment counter.
                        proc_digit_r <= load_phase_r - 8'd1;
                        load_phase_r <= load_phase_r + 8'd1;
                    end
                end else begin
                    in_beat_r <= in_beat_r + 14'd1;
                end
            end else begin
                in_beat_r   <= in_beat_r + 14'd1;
                pt_nbeats_r <= in_beat_r + 14'd1;
            end
        end
    end

    // [2026-10-25] RAM writes live in a RESET-FREE process: Vivado cannot
    // infer a block RAM that is sensitive to an asynchronous reset
    // ([Synth 8-4767] -> [Synth 8-3391] "number of bits too large").
    // Segment 0 -> base_bram (c1), segments 1..k -> in_bram (digits).
    always @(posedge clk_main) begin
        if (input_hs) begin
            if (REAL_DP) begin
                if (load_phase_r == 8'd0)
                    base_bram[in_beat_r] <= axi_s_axis_TDATA;
                else
                    in_bram[in_beat_r] <= axi_s_axis_TDATA;
            end else begin
                in_bram[in_beat_r] <= axi_s_axis_TDATA;
            end
        end
    end

    //==================================================
    // Output store (shared): emit out_src, one beat per cycle
    //==================================================
    always @(posedge clk_main or negedge rst_n) begin
        if (!rst_n) begin
            out_buf_r  <= {AXI_DATA_W{1'b0}};
            out_vld_r  <= 1'b0;
            out_lst_r  <= 1'b0;
            out_beat_r <= 14'd0;
        end else if (state_r != S_STORE_OUT) begin
            out_vld_r  <= 1'b0;
            out_lst_r  <= 1'b0;
            out_beat_r <= 14'd0;
        end else if (out_vld_r && axi_m_axis_TREADY) begin
            out_lst_r  <= 1'b0;
            if (out_beat_next < out_lim) begin
                // The beat currently on the axis is being consumed; load the
                // NEXT one.  out_src_next is read at out_beat_r + 1 so the
                // sequence is beat 0, 1, ..., out_lim-1 with no duplicate and
                // no missing tail (the v0.6 bug).
                out_buf_r  <= out_src_next;
                out_vld_r  <= 1'b1;
                out_lst_r  <= (out_beat_next == out_lim - 14'd1);
                out_beat_r <= out_beat_next;
            end else begin
                out_vld_r <= 1'b0;
            end
        end else if (!out_vld_r) begin
            // Prime the first beat.
            out_buf_r  <= out_src;
            out_vld_r  <= 1'b1;
            out_lst_r  <= (out_beat_r == out_lim - 14'd1);
        end
    end

    //==================================================
    // FSM
    //==================================================
    always @(posedge clk_main or negedge rst_n) begin
        if (!rst_n) begin
            state_r    <= S_IDLE;
            err_code_r <= 32'd0;
        end else begin
            state_r    <= state_r;                 // hold by default
            if (state_r != S_ERROR)
                err_code_r <= 32'd0;

            case (state_r)

                //----------------------------------------------------
                S_IDLE: begin
                    if (key_load_start) begin
                        state_r <= S_LOAD_KEY;
                    end else if (start) begin
                        if (digit_num_zero) begin
                            state_r    <= S_ERROR;
                            err_code_r <= `E_MODCHAIN_EXHAUSTED;
                        end else if (!key_loaded_r) begin
                            state_r    <= S_ERROR;
                            err_code_r <= `E_KEY_NOT_LOADED;
                        end else begin
                            state_r <= S_LOAD_INPUT;
                        end
                    end
                end

                //----------------------------------------------------
                // Beat capture is done by the key_load block; it also
                // raises key_loaded_r, which triggers the return here.
                S_LOAD_KEY: begin
                    if (key_loaded_r)
                        state_r <= S_IDLE;
                end

                //----------------------------------------------------
                // Beat capture is done by the input_load block.  [M3.5]
                // segmentation is by beat COUNT, not TLAST: each FRAME_IN-
                // beat digit segment ends on input_seg_last and starts an
                // NTT round; TLAST only marks the last beat of the whole
                // (1+NUM_DIGITS)-segment stream.  The pass-through build
                // still ends on TLAST.
                S_LOAD_INPUT: begin
                    if (input_seg_last && REAL_DP) begin
                        if (load_phase_r >= 8'd1)
                            state_r <= S_NTT_FWD;
                    end else if (input_hs && axi_s_axis_TLAST && !REAL_DP) begin
                        state_r <= S_STORE_OUT;
                    end
                end

                //----------------------------------------------------
                // Forward NTT: BRAM replay into the wrapper A port, the
                // wrapper output repacked by the unpack module.  Leave only
                // once every interleaved beat AND every natural coefficient
                // has been consumed.
                S_NTT_FWD: begin
                    if (ntt_outcnt_r >= LIM_OUT
                        && ntt_coefcnt_r >= LIM_COEF
                        && ntt_pad_r      >  (LAST_PAD_L - FIRST_PAD_L))
                        state_r <= S_POINTWISE;
                end

                //----------------------------------------------------
                // Element-wise (a*b) mod q in the NTT domain.
                S_POINTWISE: begin
                    if (pw_cnt_r >= LIM_IN)
                        state_r <= S_COMBINE;
                end

                //----------------------------------------------------
                // [M3.5] digit loop.  The POINTWISE pass for digit
                // proc_digit_r is done: digits remain -> back to
                // S_LOAD_INPUT (TREADY re-asserts, the host resumes the
                // digit stream); last digit -> the accumulated pw_out goes
                // to the INTT.  NUM_DIGITS=1 keeps the v0.7 single-product
                // handoff.
                S_COMBINE: begin
                    if (proc_digit_r >= NUM_DIGITS - 1)
                        state_r <= S_INTT_INV;
                    else
                        state_r <= S_LOAD_INPUT;
                end

                //----------------------------------------------------
                // Inverse INTT: replay pw_out_bram, repack the output.
                S_INTT_INV: begin
                    if (intt_outcnt_r >= LIM_OUT
                        && intt_coefcnt_r >= LIM_COEF
                        && intt_pad_r      >  (LAST_PAD_L - FIRST_PAD_L))
                        state_r <= S_ACCUMULATE;
                end

                //----------------------------------------------------
                // (intt_out + c1) mod q.
                S_ACCUMULATE: begin
                    if (acc_cnt_r >= LIM_IN)
                        state_r <= S_STORE_OUT;
                end

                //----------------------------------------------------
                S_STORE_OUT: begin
                    if (out_vld_r && axi_m_axis_TREADY && out_lst_r)
                        state_r <= S_DONE;
                end

                //----------------------------------------------------
                S_DONE: begin
                    state_r <= S_IDLE;
                end

                //----------------------------------------------------
                S_ERROR: begin
                    // Terminal; requires rst_n to re-arm.
                end

                default: state_r <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
