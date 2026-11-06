//=============================================================================
// File    : nptt_intt_wrapper.v
// Project : 玄算（HEDGE） - INTT wrapper for ac_adapter integration
// Level   : Subsystem Level
// Status  : v3.3 - Verilog-2001 clean (SV static-cast removed;
//                  was only ever parsed by iverilog -g2012)
//
// Input:  natural-order (1639 beats x 5 coefs) from POINTWISE
// Output: scatter-order (1664 beats x 5 coefs) to nptt_intt_top
//
// Transpose: natural g -> scatter (sb, sl) where
//   e=g%N, k2=g/N, ib=k2/CPB, sl=k2%CPB, sb=e+N*ib
//
// The 125 scatter-padding slots (ib=25, sl=3,4 for all engines) that are
// not covered by any natural coefficient are zeroed by the initial block.
//
// FSM: W_FILL -> W_ARM -> W_REPLAY -> W_DRAIN
//=============================================================================
`timescale 1ns / 1ps

module nptt_intt_wrapper #(
    parameter AXI_DATA_W       = 256,
    parameter COEF_WIDTH       = 48,
    parameter NUM_ENGINES      = 32,
    parameter COEFS_PER_ENGINE = 256,
    parameter PIPE_STALL_LIMIT = 200,
    parameter ROM_HEX          = "ntt32_twiddles.hex"
) (
    input  wire                     clk_main,
    input  wire                     rst_n,

    input  wire                     axi_s_axis_A_TVALID,
    input  wire [AXI_DATA_W-1:0]    axi_s_axis_A_TDATA,
    output wire                     axi_s_axis_A_TREADY,
    input  wire                     axi_s_axis_A_TLAST,

    input  wire                     axi_s_axis_B_TVALID,
    input  wire [AXI_DATA_W-1:0]    axi_s_axis_B_TDATA,
    output wire                     axi_s_axis_B_TREADY,
    input  wire                     axi_s_axis_B_TLAST,

    output wire                     axi_m_axis_TVALID,
    output wire [AXI_DATA_W-1:0]    axi_m_axis_TDATA,
    input  wire                     axi_m_axis_TREADY,
    output wire                     axi_m_axis_TLAST,

    input  wire [1:0]               mode,
    input  wire                     pipe_start,
    input  wire                     pipe_stall,

    output wire [7:0]               status,
    output wire                     busy,
    output wire                     err,
    output wire [31:0]              err_code
);

    localparam CW   = COEF_WIDTH;
    localparam N    = NUM_ENGINES;
    localparam N2   = COEFS_PER_ENGINE;
    localparam CPB  = AXI_DATA_W / CW;
    localparam TOTAL_COEFS  = N * N2;
    localparam BEATS_PER_ENG = (N2 + CPB - 1) / CPB;
    localparam TOTAL_BEATS  = N * BEATS_PER_ENG;
    localparam FRAME_IN     = (TOTAL_COEFS + CPB - 1) / CPB;

    localparam BW   = 12;

    // nptt_intt_top interface
    wire                 nptt_intt_s_TREADY;
    wire                 nptt_done;
    reg                  nptt_start_r;

    // FSM
    localparam [2:0] W_FILL   = 3'd0;
    localparam [2:0] W_ARM    = 3'd1;
    localparam [2:0] W_REPLAY = 3'd2;
    localparam [2:0] W_DRAIN  = 3'd3;

    reg [2:0]  wst_r;
    reg [BW-1:0] fill_cnt;
    reg [BW-1:0] replay_cnt;
    reg         frame_done;
    reg         armed;
    reg busy_r;

    // Registered beat presented to nptt_intt_top's s_axis
    reg [AXI_DATA_W-1:0] in_beat_data_r;
    reg                  in_beat_last_r;
    reg                  in_beat_vld_r;

    // Fill-side: accept natural beats
    wire fill_accept = (wst_r == W_FILL)
                       && (fill_cnt < FRAME_IN[BW-1:0])
                       && !frame_done
                       && axi_s_axis_A_TVALID;
    // Scatter buffer, ONE INDEPENDENT RAM PER NATURAL LANE (2026-10-25).
    //   bank bi holds the coefficients whose natural lane is bi, addressed
    //   by the scatter beat sb.  Each bank is written once per cycle from
    //   its own process, which is what lets Vivado infer real block RAMs.
    //   (Flat [0:TOTAL_BEATS*CPB-1] -> [Synth 8-3391]; banked 2-D array ->
    //   [Synth 8-11357] 3D-RAM, dissolved into 399360 registers.)
    // [2026-10-25] `reg` array, NOT a net array: Verilog-2001 only permits a
    // constant index into an array of nets, and the read side indexes this
    // with the computed lane (g2 % CPB).  It is a 5-entry read port mux in
    // front of the five real RAMs, never a memory itself.
    reg [CW-1:0] beat_rd [0:CPB-1];

    // scatter address of the coefficient (fill_cnt, lane):
    //   g = fill_cnt*CPB + lane; e = g%N; k2 = g/N; ib = k2/CPB; sb = e + N*ib
    function [31:0] scatter_sb;
        input [31:0] fc;
        input [31:0] lane;
        reg [31:0] gx, ex, k2x;
        begin
            gx   = fc * CPB + lane;
            ex   = gx % N;
            k2x  = gx / N;
            scatter_sb = ex + N * (k2x / CPB);
        end
    endfunction

    // Five independent block RAMs, one per natural lane.  Separate
    // one-dimensional arrays on purpose: a [CPB][TOTAL_BEATS] 2-D array is
    // treated by Vivado as a "3D-RAM" and dissolved into registers
    // ([Synth 8-11357] / [Synth 8-3391]), and a generate block that declares
    // its own RAM loses the module scope in iverilog.
    (* ram_style = "block" *) reg [CW-1:0] bmem0 [0:TOTAL_BEATS-1];
    (* ram_style = "block" *) reg [CW-1:0] bmem1 [0:TOTAL_BEATS-1];
    (* ram_style = "block" *) reg [CW-1:0] bmem2 [0:TOTAL_BEATS-1];
    (* ram_style = "block" *) reg [CW-1:0] bmem3 [0:TOTAL_BEATS-1];
    (* ram_style = "block" *) reg [CW-1:0] bmem4 [0:TOTAL_BEATS-1];

    always @(*) begin
        beat_rd[0] = bmem0[replay_cnt];
        beat_rd[1] = bmem1[replay_cnt];
        beat_rd[2] = bmem2[replay_cnt];
        beat_rd[3] = bmem3[replay_cnt];
        beat_rd[4] = bmem4[replay_cnt];
    end

    // The natural tail beat (FRAME_IN-1) carries only (TOTAL_COEFS % CPB) = 2
    // real coefficients; lanes 2..4 of that beat are zero-filled.
    wire last_nat_beat = (fill_cnt == FRAME_IN[BW-1:0] - 1'b1);

    // Reset-free write process: one write per RAM per cycle -> 5 block RAMs.
    always @(posedge clk_main) begin
        if (wst_r == W_FILL && fill_accept) begin
            bmem0[scatter_sb(fill_cnt, 32'd0)]
                <= axi_s_axis_A_TDATA[(0)*CW +: CW];
            bmem1[scatter_sb(fill_cnt, 32'd1)]
                <= axi_s_axis_A_TDATA[(1)*CW +: CW];
            bmem2[scatter_sb(fill_cnt, 32'd2)]
                <= last_nat_beat ? {CW{1'b0}}
                                 : axi_s_axis_A_TDATA[(2)*CW +: CW];
            bmem3[scatter_sb(fill_cnt, 32'd3)]
                <= last_nat_beat ? {CW{1'b0}}
                                 : axi_s_axis_A_TDATA[(3)*CW +: CW];
            bmem4[scatter_sb(fill_cnt, 32'd4)]
                <= last_nat_beat ? {CW{1'b0}}
                                 : axi_s_axis_A_TDATA[(4)*CW +: CW];
        end
    end

    assign axi_s_axis_A_TREADY = (wst_r == W_FILL)
                                 && (fill_cnt < FRAME_IN[BW-1:0])
                                 && !frame_done;

    // Output comes directly from nptt_intt_top's m_axis (scatter module).
    // The wrapper only transposes the input and fires the start pulse;
    // it does NOT drive the output data/valid/last lines.

    integer l;
    integer g;
    integer e;
    integer k2;
    integer ib;
    integer sl;
    integer sb;
    integer g2;      // [2026-10-25] banked read: natural index of a slot
    reg [AXI_DATA_W-1:0] tmp_beat;
    reg [AXI_DATA_W-1:0] acc_beat;

    always @(posedge clk_main or negedge rst_n) begin
        if (!rst_n) begin
            wst_r         <= W_FILL;
            fill_cnt      <= {BW{1'b0}};
            replay_cnt    <= {BW{1'b0}};
            frame_done    <= 1'b0;
            armed         <= 1'b0;
            nptt_start_r  <= 1'b0;
            busy_r        <= 1'b0;
            in_beat_data_r <= {AXI_DATA_W{1'b0}};
            in_beat_last_r <= 1'b0;
            in_beat_vld_r  <= 1'b0;
        end else begin
            nptt_start_r  <= 1'b0;

            case (wst_r)
                //----------------------------------------------------
                // W_FILL: accept natural beats, transpose to scatter
                W_FILL: begin
                    if (fill_accept) begin
                        // [2026-10-25] The transpose WRITE lives in the
                        // reset-free block below (RAM inference); only the
                        // fill counters stay in this FSM process.
                        if (fill_cnt + 1'b1 == FRAME_IN[BW-1:0]) begin
                            fill_cnt   <= {BW{1'b0}};
                            frame_done <= 1'b1;
                        end else begin
                            fill_cnt <= fill_cnt + 1'b1;
                        end
                    end
                    if (pipe_start) armed <= 1'b1;
                    if (frame_done && armed) begin
                        wst_r <= W_ARM;
                    end else if (frame_done && !armed) begin
                    end
                end

                //----------------------------------------------------
                // W_ARM: fire 1-cycle start pulse
                W_ARM: begin
                    nptt_start_r <= 1'b1;
                    wst_r        <= W_REPLAY;
                    replay_cnt   <= {BW{1'b0}};
                    busy_r       <= 1'b1;
                end

               //----------------------------------------------------
                // W_REPLAY: assemble and present scatter beats to nptt_intt_top
                W_REPLAY: begin
                    // Pre-assemble next beat if not currently sending
                    if (!in_beat_vld_r) begin
                        acc_beat     = {AXI_DATA_W{1'b0}};
                        in_beat_last_r <= (replay_cnt + 1'b1
                                           == TOTAL_BEATS[BW-1:0]);
                        for (l = 0; l < CPB; l = l + 1) begin
                            // [2026-10-25] banked layout: scatter beat
                            // replay_cnt, lane l  <-  g = ((sb/N)*CPB + l)*N
                            // + (sb % N) with sb = replay_cnt; the natural
                            // lane that produced it is g % CPB.
                            g2 = ((replay_cnt / N) * CPB + l) * N
                                 + (replay_cnt % N);
                            // Slots with k2 >= N2 are scatter padding that no
                            // natural coefficient maps to; emit 0 instead of
                            // relying on an initial-block clear.
                            if (((replay_cnt / N) * CPB + l) < N2)
                                acc_beat = acc_beat |
                                    ({AXI_DATA_W{1'b0}}
                                     | (beat_rd[g2 % CPB]
                                        << (l*CW)));
                        end
                        in_beat_data_r <= acc_beat;
                        in_beat_vld_r  <= 1'b1;
                    end
                    // Send beat: clear valid, increment counter
                    if (nptt_intt_s_TREADY && in_beat_vld_r) begin
                        in_beat_vld_r <= 1'b0;
                        replay_cnt    <= replay_cnt + 1'b1;
                        if (replay_cnt + 1'b1 >= TOTAL_BEATS[BW-1:0]) begin
                            wst_r <= W_DRAIN;
                        end
                    end
                end

                //----------------------------------------------------
                // W_DRAIN: wait for nptt_done
                W_DRAIN: begin
                    if (nptt_done) begin
                        wst_r      <= W_FILL;
                        fill_cnt   <= {BW{1'b0}};
                        frame_done <= 1'b0;
                        armed      <= 1'b0;
                        busy_r     <= 1'b0;
                    end
                end

                default: wst_r <= W_FILL;
            endcase
        end
    end

    //==================================================
    // nptt_intt_top instance
    //==================================================
    nptt_intt_top #(
        .AXI_DATA_W       (AXI_DATA_W),
        .COEF_WIDTH       (COEF_WIDTH),
        .NUM_ENGINES      (NUM_ENGINES),
        .COEFS_PER_ENGINE (COEFS_PER_ENGINE)
    ) u_nptt_intt (
        .clk              (clk_main),
        .rst_n            (rst_n),
        .s_axis_TVALID    (in_beat_vld_r),
        .s_axis_TDATA     (in_beat_data_r),
        .s_axis_TREADY    (nptt_intt_s_TREADY),
        .s_axis_TLAST     (in_beat_last_r),
        .m_axis_TVALID    (axi_m_axis_TVALID),
        .m_axis_TDATA     (axi_m_axis_TDATA),
        .m_axis_TLAST     (axi_m_axis_TLAST),
        .m_axis_TREADY    (axi_m_axis_TREADY),
        .start            (nptt_start_r),
        .done             (nptt_done)
    );

    //==================================================
    // B stream ignored
    //==================================================
    assign axi_s_axis_B_TREADY = 1'b1;
    assign busy       = busy_r;
    assign err        = 1'b0;
    assign err_code   = 32'd0;
    assign status     = {5'b0, armed, frame_done, nptt_start_r};

endmodule

`default_nettype wire
