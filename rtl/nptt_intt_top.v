//=============================================================================
// File    : nptt_intt_top.v
// Project : 玄算（HEDGE） - Four-step INTT top level (NPAR=32/64)
// Level   : Top Module
// Status  : v1.0 - REAL four-step INTT pipeline: unscatter -> engine -> twist
//            -> cross.  Reverse order of nptt_top.v with inverse twiddles.
//
// Data flow (all engines lockstep):
//   1 AXI in  -> nptt_unscatter (step 4', scatter inverse)
//              -> N x nptt_engine_core (step 3', inverse local NTT)
//              -> N x nptt_twist (step 2', inverse twist)
//              -> nptt_cross (step 1', inverse cross-engine NTT)
//   N AXI out
//
// Control: only the engine cores are batch processors (start/done).
// unscatter, twist and cross are pure streaming with backpressure, so the
// pipeline self-flows once every engine core is armed.  A one-cycle
// eng_start pulse is issued on the external start; top-level done fires
// when all N cores report done (the cross may still be draining).
//
// NINV handling: Per design spec Option B, POINTWISE absorbs N^-1, so
// engine_core must NOT apply any additional N^-1 scaling.  We pass
// NINV_OVR=0 to disable the S_SCALE state.
//
// NINV_OVR values (for reference):
//   NPAR=32 (COEFS_PER_ENGINE=256): 128^-1 = 0x7efffff02001
//   NPAR=64 (COEFS_PER_ENGINE=128): 256^-1 = 0x7F7FFFF01001
//   Both disabled here (NINV_OVR=0) since POINTWISE already applies N^-1.
//=============================================================================

`default_nettype wire

module nptt_intt_top #(
    parameter AXI_DATA_W        = 256,
    parameter COEF_WIDTH        = 48,
    parameter NUM_ENGINES       = 32,
    // [Phase 2] COEFS_PER_ENGINE = 8192 / NUM_ENGINES (256 for NPAR=32, 128 for NPAR=64)
    parameter COEFS_PER_ENGINE  = (8192 / NUM_ENGINES),
    parameter CPB               = AXI_DATA_W / COEF_WIDTH,           // 5
    parameter BEATS_PER_ENGINE  = (COEFS_PER_ENGINE + CPB - 1) / CPB, // 52 or 26
    parameter TOTAL_BEATS       = NUM_ENGINES * BEATS_PER_ENGINE      // 1664 or 1664
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Single AXI4-Stream slave port (scatter-order input)
    input  wire                     s_axis_TVALID,
    input  wire [AXI_DATA_W-1:0]    s_axis_TDATA,
    output wire                     s_axis_TREADY,
    input  wire                     s_axis_TLAST,

    // Single AXI4-Stream master port (scatter-order output, INTT result)
    output wire                     m_axis_TVALID,
    output wire [AXI_DATA_W-1:0]    m_axis_TDATA,
    output wire                     m_axis_TLAST,
    input  wire                     m_axis_TREADY,

    // Control
    input  wire                     start,
    output wire                     done
);

    // [Phase 2] Engine-core POLY_ORDER and ROM_HEX follow COEFS_PER_ENGINE:
    //   NPAR=32 -> COEFS_PER_ENGINE=256 -> POLY_ORDER=8, 256-entry ROM
    //   NPAR=64 -> COEFS_PER_ENGINE=128 -> POLY_ORDER=7, 128-entry ROM
    localparam ENG_POLY_ORDER = (COEFS_PER_ENGINE == 256) ? 8 : (COEFS_PER_ENGINE == 128) ? 7 : 8;
    localparam ENG_ROM_HEX    = (COEFS_PER_ENGINE == 256) ? "ntt_engine_twiddle_rom.hex"
                                                        : (COEFS_PER_ENGINE == 128) ? "ntt_engine_twiddle_rom_n128.hex"
                                                                                    : "ntt_engine_twiddle_rom.hex";
    // [Phase INTT] Cross/twist ROM selection: inverse twiddles
    localparam CROSS_ROM_HEX  = (NUM_ENGINES == 32) ? "ntt32_twiddles_inv.hex" : "ntt64_twiddles_inv.hex";

    //==================================================
    // Stage 1: unscatter (step 4') -- 1 in -> N out
    //==================================================
    wire [NUM_ENGINES-1:0]            un_m_TVALID;
    wire [NUM_ENGINES*AXI_DATA_W-1:0] un_m_TDATA;
    wire [NUM_ENGINES-1:0]            un_m_TREADY;
    wire [NUM_ENGINES-1:0]            un_m_TLAST;

    nptt_unscatter #(
        .AXI_DATA_W(AXI_DATA_W),
        .COEF_WIDTH(COEF_WIDTH),
        .NUM_ENGINES(NUM_ENGINES),
        .BEATS_PER_ENGINE(BEATS_PER_ENGINE)
    ) u_unscatter (
        .clk(clk),
        .rst_n(rst_n),
        .axi_s_TVALID(s_axis_TVALID),
        .axi_s_TDATA (s_axis_TDATA),
        .axi_s_TREADY(s_axis_TREADY),
        .axi_s_TLAST (s_axis_TLAST),
        .axi_m_TVALID(un_m_TVALID),
        .axi_m_TDATA (un_m_TDATA),
        .axi_m_TREADY(un_m_TREADY),
        .axi_m_TLAST (un_m_TLAST)
    );

    //==================================================
    // Stage 2: engine core (step 3') -- N x inverse local NTT (batch)
    //==================================================
    wire [NUM_ENGINES-1:0]            ec_m_TVALID;
    wire [NUM_ENGINES*AXI_DATA_W-1:0] ec_m_TDATA;
    wire [NUM_ENGINES-1:0]            ec_m_TLAST;
    wire [NUM_ENGINES-1:0]            ec_m_TREADY;   // driven by twist (see gen_twist below)
    wire [NUM_ENGINES-1:0]            eng_start;
    wire [NUM_ENGINES-1:0]            eng_done;
    wire [NUM_ENGINES-1:0]            eng_busy;
    wire [NUM_ENGINES-1:0]            eng_err;

    genvar i;
    generate
        for (i = 0; i < NUM_ENGINES; i = i + 1) begin : gen_engine
            nptt_engine_core #(
                .AXI_DATA_W(AXI_DATA_W),
                .COEF_WIDTH(COEF_WIDTH),
                .POLY_ORDER(ENG_POLY_ORDER),
                .COEFS_PER_ENGINE(COEFS_PER_ENGINE),
                .ROM_HEX(ENG_ROM_HEX),
                .NINV_OVR(48'h0)  // Disable S_SCALE: POINTWISE already applies N^-1
            ) u_eng_i (
                .clk(clk),
                .rst_n(rst_n),
                .nptt_mode(1'b1),      // Inverse mode
                .start(eng_start[i]),
                .busy(eng_busy[i]),
                .done(eng_done[i]),
                .err(eng_err[i]),
                .status(),
                .axi_s_axis_TVALID(un_m_TVALID[i]),
                .axi_s_axis_TDATA (un_m_TDATA[i*AXI_DATA_W +: AXI_DATA_W]),
                .axi_s_axis_TREADY(un_m_TREADY[i]),
                .axi_s_axis_TLAST (un_m_TLAST[i]),
                .axi_m_axis_TVALID(ec_m_TVALID[i]),
                .axi_m_axis_TDATA (ec_m_TDATA[i*AXI_DATA_W +: AXI_DATA_W]),
                .axi_m_axis_TREADY(ec_m_TREADY[i]),
                .axi_m_axis_TLAST (ec_m_TLAST[i])
            );
        end
    endgenerate

    //==================================================
    // Stage 3: twist (step 2') -- N x inverse twist
    //==================================================
    wire [NUM_ENGINES-1:0]            tw_m_TVALID;
    wire [NUM_ENGINES*AXI_DATA_W-1:0] tw_m_TDATA;
    wire [NUM_ENGINES-1:0]            tw_m_TLAST;
    wire [NUM_ENGINES-1:0]            tw_m_TREADY;

    generate
        for (i = 0; i < NUM_ENGINES; i = i + 1) begin : gen_twist
            // engine core output -> twist instance input;
            // twist drives ec_m_TREADY[i] via its axi_s_axis_TREADY output
            nptt_twist #(
                .AXI_DATA_W(AXI_DATA_W),
                .COEF_WIDTH(COEF_WIDTH),
                .COEFS_PER_BLOCK(COEFS_PER_ENGINE),
                .ENGINE_ID(i[5:0]),
                .L((NUM_ENGINES == 64) ? 7 : 8),
                .ROM_HEX(                          (i == 0)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng00.hex" : "ntt_twist32_inv_eng00.hex" :
                          (i == 1)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng01.hex" : "ntt_twist32_inv_eng01.hex" :
                          (i == 2)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng02.hex" : "ntt_twist32_inv_eng02.hex" :
                          (i == 3)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng03.hex" : "ntt_twist32_inv_eng03.hex" :
                          (i == 4)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng04.hex" : "ntt_twist32_inv_eng04.hex" :
                          (i == 5)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng05.hex" : "ntt_twist32_inv_eng05.hex" :
                          (i == 6)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng06.hex" : "ntt_twist32_inv_eng06.hex" :
                          (i == 7)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng07.hex" : "ntt_twist32_inv_eng07.hex" :
                          (i == 8)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng08.hex" : "ntt_twist32_inv_eng08.hex" :
                          (i == 9)  ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng09.hex" : "ntt_twist32_inv_eng09.hex" :
                          (i == 10) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng10.hex" : "ntt_twist32_inv_eng10.hex" :
                          (i == 11) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng11.hex" : "ntt_twist32_inv_eng11.hex" :
                          (i == 12) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng12.hex" : "ntt_twist32_inv_eng12.hex" :
                          (i == 13) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng13.hex" : "ntt_twist32_inv_eng13.hex" :
                          (i == 14) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng14.hex" : "ntt_twist32_inv_eng14.hex" :
                          (i == 15) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng15.hex" : "ntt_twist32_inv_eng15.hex" :
                          (i == 16) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng16.hex" : "ntt_twist32_inv_eng16.hex" :
                          (i == 17) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng17.hex" : "ntt_twist32_inv_eng17.hex" :
                          (i == 18) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng18.hex" : "ntt_twist32_inv_eng18.hex" :
                          (i == 19) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng19.hex" : "ntt_twist32_inv_eng19.hex" :
                          (i == 20) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng20.hex" : "ntt_twist32_inv_eng20.hex" :
                          (i == 21) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng21.hex" : "ntt_twist32_inv_eng21.hex" :
                          (i == 22) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng22.hex" : "ntt_twist32_inv_eng22.hex" :
                          (i == 23) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng23.hex" : "ntt_twist32_inv_eng23.hex" :
                          (i == 24) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng24.hex" : "ntt_twist32_inv_eng24.hex" :
                          (i == 25) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng25.hex" : "ntt_twist32_inv_eng25.hex" :
                          (i == 26) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng26.hex" : "ntt_twist32_inv_eng26.hex" :
                          (i == 27) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng27.hex" : "ntt_twist32_inv_eng27.hex" :
                          (i == 28) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng28.hex" : "ntt_twist32_inv_eng28.hex" :
                          (i == 29) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng29.hex" : "ntt_twist32_inv_eng29.hex" :
                          (i == 30) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng30.hex" : "ntt_twist32_inv_eng30.hex" :
                          (i == 31) ? (NUM_ENGINES == 64) ? "ntt_twist_inv_eng31.hex" : "ntt_twist32_inv_eng31.hex" :
                          (i == 32)  ? "ntt_twist_inv_eng32.hex" :
                          (i == 33)  ? "ntt_twist_inv_eng33.hex" :
                          (i == 34)  ? "ntt_twist_inv_eng34.hex" :
                          (i == 35)  ? "ntt_twist_inv_eng35.hex" :
                          (i == 36)  ? "ntt_twist_inv_eng36.hex" :
                          (i == 37)  ? "ntt_twist_inv_eng37.hex" :
                          (i == 38)  ? "ntt_twist_inv_eng38.hex" :
                          (i == 39)  ? "ntt_twist_inv_eng39.hex" :
                          (i == 40)  ? "ntt_twist_inv_eng40.hex" :
                          (i == 41)  ? "ntt_twist_inv_eng41.hex" :
                          (i == 42)  ? "ntt_twist_inv_eng42.hex" :
                          (i == 43)  ? "ntt_twist_inv_eng43.hex" :
                          (i == 44)  ? "ntt_twist_inv_eng44.hex" :
                          (i == 45)  ? "ntt_twist_inv_eng45.hex" :
                          (i == 46)  ? "ntt_twist_inv_eng46.hex" :
                          (i == 47)  ? "ntt_twist_inv_eng47.hex" :
                          (i == 48)  ? "ntt_twist_inv_eng48.hex" :
                          (i == 49)  ? "ntt_twist_inv_eng49.hex" :
                          (i == 50)  ? "ntt_twist_inv_eng50.hex" :
                          (i == 51)  ? "ntt_twist_inv_eng51.hex" :
                          (i == 52)  ? "ntt_twist_inv_eng52.hex" :
                          (i == 53)  ? "ntt_twist_inv_eng53.hex" :
                          (i == 54)  ? "ntt_twist_inv_eng54.hex" :
                          (i == 55)  ? "ntt_twist_inv_eng55.hex" :
                          (i == 56)  ? "ntt_twist_inv_eng56.hex" :
                          (i == 57)  ? "ntt_twist_inv_eng57.hex" :
                          (i == 58)  ? "ntt_twist_inv_eng58.hex" :
                          (i == 59)  ? "ntt_twist_inv_eng59.hex" :
                          (i == 60)  ? "ntt_twist_inv_eng60.hex" :
                          (i == 61)  ? "ntt_twist_inv_eng61.hex" :
                          (i == 62)  ? "ntt_twist_inv_eng62.hex" :
                          (i == 63)  ? "ntt_twist_inv_eng63.hex" :
                          "ntt_twist_inv_eng00.hex")
            ) u_twist_i (
                .clk(clk),
                .rst_n(rst_n),
                .axi_s_axis_TVALID(ec_m_TVALID[i]),
                .axi_s_axis_TDATA (ec_m_TDATA[i*AXI_DATA_W +: AXI_DATA_W]),
                .axi_s_axis_TREADY(ec_m_TREADY[i]),
                .axi_s_axis_TLAST (ec_m_TLAST[i]),
                .axi_m_axis_TVALID(tw_m_TVALID[i]),
                .axi_m_axis_TDATA (tw_m_TDATA[i*AXI_DATA_W +: AXI_DATA_W]),
                .axi_m_axis_TREADY(tw_m_TREADY[i]),
                .axi_m_axis_TLAST (tw_m_TLAST[i])
            );
        end
    endgenerate

    //==================================================
    // Stage 4: cross (step 1') -- N in -> N out (inverse cross-engine NTT)
    //==================================================
    wire [NUM_ENGINES-1:0]            cr_m_TVALID;
    wire [NUM_ENGINES*AXI_DATA_W-1:0] cr_m_TDATA;
    wire [NUM_ENGINES-1:0]            cr_m_TLAST;
    wire [NUM_ENGINES-1:0]            cr_m_TREADY;

    nptt_cross #(
        .AXI_DATA_W(AXI_DATA_W),
        .COEF_WIDTH(COEF_WIDTH),
        .NUM_ENGINES(NUM_ENGINES),
        .COEFS_PER_ENGINE(COEFS_PER_ENGINE),
        .ROM_HEX(CROSS_ROM_HEX)
    ) u_cross (
        .clk(clk),
        .rst_n(rst_n),
        .s_axis_TVALID(tw_m_TVALID),
        .s_axis_TDATA(tw_m_TDATA),
        .s_axis_TREADY(tw_m_TREADY),
        .s_axis_TLAST(tw_m_TLAST),
        .m_axis_TVALID(cr_m_TVALID),
        .m_axis_TDATA(cr_m_TDATA),
        .m_axis_TREADY(cr_m_TREADY),
        .m_axis_TLAST(cr_m_TLAST)
    );

    //==================================================
    // Stage 5: scatter -- N in -> 1 out (re-interleave to scatter order)
    //==================================================
    wire [NUM_ENGINES-1:0] sc_s_TREADY;

    nptt_scatter #(
        .AXI_DATA_W(AXI_DATA_W),
        .COEF_WIDTH(COEF_WIDTH),
        .NUM_ENGINES(NUM_ENGINES),
        .BEATS_PER_ENGINE(BEATS_PER_ENGINE)
    ) u_scatter (
        .clk(clk),
        .rst_n(rst_n),
        .axi_s_TVALID(cr_m_TVALID),
        .axi_s_TDATA (cr_m_TDATA),
        .axi_s_TREADY(cr_m_TREADY),
        .axi_s_TLAST (cr_m_TLAST),
        .axi_m_TVALID(m_axis_TVALID),
        .axi_m_TDATA (m_axis_TDATA),
        .axi_m_TLAST (m_axis_TLAST),
        .axi_m_TREADY(m_axis_TREADY)
    );

    //==================================================
    // Control: arm the engine cores on start, report done
    //==================================================
    // Same FSM as nptt_top.v: ST_IDLE -> ST_ARM -> ST_RUN -> ST_IDLE
    // done is LATCHED so it stays asserted until the next start pulse.
    //==================================================
    reg  [1:0] state_r;
    reg  [NUM_ENGINES-1:0] eng_start_r;
    reg  done_r;
    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_ARM  = 2'd1;
    localparam [1:0] ST_RUN  = 2'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r     <= ST_IDLE;
            eng_start_r <= {NUM_ENGINES{1'b0}};
            done_r      <= 1'b0;
        end else begin
            case (state_r)
                ST_IDLE: begin
                    if (start) begin
                        eng_start_r <= {NUM_ENGINES{1'b1}};
                        done_r      <= 1'b0;
                        state_r     <= ST_ARM;
                    end
                end
                ST_ARM: begin
                    // Engines consume eng_start on this edge; busy asserts
                    // from the next cycle.  Do NOT evaluate completion here.
                    eng_start_r <= {NUM_ENGINES{1'b0}};
                    state_r     <= ST_RUN;
                end
                ST_RUN: begin
                    if (~(|(eng_busy | eng_err))) begin
                        done_r  <= 1'b1;
                        state_r <= ST_IDLE;
                    end
                end
                default: state_r <= ST_IDLE;
            endcase
        end
    end

    assign eng_start = eng_start_r;
    assign done      = done_r;

endmodule

`default_nettype wire
