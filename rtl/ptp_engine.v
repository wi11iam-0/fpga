/*

PTP engine: a PTPv2 slave that synchronises a ptp_clock to a PTP master

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * An IEEE 1588-2008 (PTPv2) ordinary clock in the slave state, using the end
 * to end delay mechanism over Layer 2 (Ethertype 0x88F7). It sits between
 * eth_mac_10g_fifo (with PTP_TS_ENABLE, TX_PTP_TS_CTRL_IN_TUSER and
 * TX_PTP_TAG_ENABLE set) and ptp_clock, all on the same clock as ptp_clock.
 *
 * Exchange, for each Sync from the master:
 *
 *     master                                slave (this module)
 *     t1 |------ Sync (seq S) ------------->| t2  RX timestamp of the Sync
 *        |------ Follow_Up {t1} ----------->|     (two-step masters only)
 *     t4 |<----- Delay_Req (seq N) ---------| t3  TX timestamp, tag N
 *        |------ Delay_Resp {t4, N} ------->|
 *
 *     mean path delay = ((t2 - t3) + (t4 - t1)) / 2
 *     offset          = t2 - t1 - mean path delay
 *
 * with the correctionField of the Sync and Follow_Up added to t1 and that of
 * the Delay_Resp taken from t4. The delay is written with t2 - t3 and t4 - t1
 * because each of those differences is between two readings of the same
 * clock, so it stays small even before the slave's time has been set.
 *
 * Step by step:
 * 1. The master is the source of the first Sync seen in domain cfg_domain.
 *    There is no best master clock algorithm. The master is dropped after
 *    MASTER_TIMEOUT cycles with no Sync or Announce from it.
 * 2. Sync: t2 is the RX timestamp, less cfg_rx_latency. A one-step Sync
 *    carries t1; a two-step Sync (twoStepFlag set) waits for the Follow_Up
 *    with the same sequenceId, which carries t1.
 * 3. With t1 and t2 in hand, if a path delay is known, the sample
 *    (t2, t1 + corrections + delay) goes to ptp_servo, and the engine waits
 *    for the servo to finish with it.
 * 4. A Delay_Req follows every cfg_delay_req_div-th Sync, if none is still
 *    waiting for its answer. It is sent with the MAC's timestamp enable set
 *    and its sequenceId as the tag, and is paired with the t1 and t2 of the
 *    Sync just before it, so that the two exchanges are close in time.
 * 5. t3 arrives on s_axis_tx_ts_* with the matching tag (cfg_tx_latency is
 *    added), and t4 in the Delay_Resp whose sequenceId and
 *    requestingPortIdentity match. With both, the path delay is computed and
 *    filtered: delay += (new - delay) / 2^DELAY_FILTER_SHIFT. A Delay_Req with
 *    no answer after DELAY_REQ_TIMEOUT cycles is abandoned.
 * 6. When the servo steps the clock, timestamps taken before the step are on
 *    the old timescale, so a waiting Follow_Up or Delay_Resp is abandoned and
 *    the Sync that caused the step is not used for a Delay_Req. The path
 *    delay estimate is kept, as it does not depend on the clock's offset.
 *
 * No timestamps are used while tx_ptp_locked or rx_ptp_locked is low, as the
 * MAC's copies of the time are not valid then.
 *
 * cfg_rx_latency and cfg_tx_latency correct for the MAC taking timestamps at
 * the XGMII start character instead of at IEEE 1588's reference point (the
 * first bit after the start of frame delimiter, on the wire). They have to be
 * measured; an error common to both shifts the offset by the same amount.
 *
 * Not supported: master role, peer to peer delay, PTP over UDP (that goes to
 * the UDP stack, not here), VLAN tagged frames, management and signalling
 * messages. Delay_Req is not randomised in time as the standard suggests.
 *
 * The frames the engine receives must already be PTP frames (see
 * axis_eth_type_demux). Frames are accepted for the PTP multicast address
 * 01:1B:19:00:00:00 or cfg_local_mac. Delay_Req goes to 01:1B:19:00:00:00
 * from cfg_local_mac, with clockIdentity made from cfg_local_mac (EUI-48 to
 * EUI-64, FF:FE in the middle) and portNumber 1.
 *
 * Time values are in ptp_clock's 96 bit time of day format,
 * {seconds[47:0], 2'b00, ns[29:0], fns[15:0]}. Delays and offsets are signed
 * ns * 2^16, rates signed ppb * 2^16.
 */
module ptp_engine #
(
    // ptp_clock settings; these must match the ptp_clock instance
    parameter PERIOD_NS_WIDTH = 4,
    parameter FNS_WIDTH = 32,
    parameter PERIOD_NS = 4'h6,
    parameter PERIOD_FNS = 32'h66666666,
    parameter PTP_CLOCK_PIPELINE = 0,
    // TX timestamp tag width; must match the MAC's PTP_TAG_WIDTH
    parameter PTP_TAG_WIDTH = 16,
    // servo settings, see ptp_servo
    parameter SERVO_KP = 45875,
    parameter SERVO_KI = 19661,
    parameter SERVO_MAX_PPB = 500000,
    parameter SERVO_FIRST_STEP_THRESHOLD_NS = 20000,
    parameter SERVO_STEP_THRESHOLD_NS = 1000000,
    parameter SERVO_LOCK_THRESHOLD_NS = 100,
    parameter SERVO_LOCK_COUNT = 8,
    parameter SERVO_STEP_SKIP_SAMPLES = 1,
    // path delay filter: each measurement moves the estimate 1/2^DELAY_FILTER_SHIFT of the way
    parameter DELAY_FILTER_SHIFT = 3,
    // path delay measurements beyond +/- this are discarded, ns
    parameter MAX_DELAY_NS = 100000,
    // clk cycles to wait for a Delay_Req's TX timestamp and Delay_Resp (1 s at 156.25 MHz)
    parameter DELAY_REQ_TIMEOUT = 156250000,
    // clk cycles without a Sync or Announce before the master is dropped (4 s at 156.25 MHz)
    parameter MASTER_TIMEOUT = 625000000
)
(
    input  wire                        clk,
    input  wire                        rst,

    /*
     * Received PTP frames with their RX timestamps, tuser = {timestamp, bad frame}
     */
    input  wire [63:0]                 s_axis_rx_tdata,
    input  wire [7:0]                  s_axis_rx_tkeep,
    input  wire                        s_axis_rx_tvalid,
    output wire                        s_axis_rx_tready,
    input  wire                        s_axis_rx_tlast,
    input  wire [96:0]                 s_axis_rx_tuser,

    /*
     * PTP frames to send, tuser = {tag, timestamp enable, bad frame}
     */
    output wire [63:0]                 m_axis_tx_tdata,
    output wire [7:0]                  m_axis_tx_tkeep,
    output wire                        m_axis_tx_tvalid,
    input  wire                        m_axis_tx_tready,
    output wire                        m_axis_tx_tlast,
    output wire [PTP_TAG_WIDTH+1:0]    m_axis_tx_tuser,

    /*
     * TX timestamps from the MAC
     */
    input  wire [95:0]                 s_axis_tx_ts_96,
    input  wire [PTP_TAG_WIDTH-1:0]    s_axis_tx_ts_tag,
    input  wire                        s_axis_tx_ts_valid,
    output wire                        s_axis_tx_ts_ready,

    /*
     * PTP clock
     */
    input  wire [95:0]                 ptp_ts_96,
    input  wire                        tx_ptp_locked,
    input  wire                        rx_ptp_locked,
    output wire [95:0]                 ptp_set_ts_96,
    output wire                        ptp_set_ts_96_valid,
    output wire [PERIOD_NS_WIDTH-1:0]  ptp_set_period_ns,
    output wire [FNS_WIDTH-1:0]        ptp_set_period_fns,
    output wire                        ptp_set_period_valid,

    /*
     * Configuration
     */
    input  wire                        cfg_enable,
    input  wire [47:0]                 cfg_local_mac,
    input  wire [7:0]                  cfg_domain,
    input  wire [7:0]                  cfg_delay_req_div,  // Delay_Req after every Nth Sync (0 and 1: every Sync)
    input  wire [31:0]                 cfg_rx_latency,     // subtracted from RX timestamps, ns * 2^16, signed
    input  wire [31:0]                 cfg_tx_latency,     // added to TX timestamps, ns * 2^16, signed

    /*
     * Status
     */
    output wire                        status_master_valid,
    output wire [79:0]                 status_master_port_id,
    output wire                        status_delay_valid,
    output wire [63:0]                 status_delay,         // mean path delay, ns * 2^16
    output wire [63:0]                 status_offset,        // local - master, ns * 2^16
    output wire                        status_offset_valid,  // pulses when status_offset changes
    output wire [63:0]                 status_adj,           // rate correction, ppb * 2^16, positive slows the clock
    output wire [1:0]                  status_servo_state,   // 0: first sample, 1: measuring rate, 2: tracking
    output wire                        status_servo_locked,
    output wire [15:0]                 status_sync_count,    // Syncs with t1 and t2 complete
    output wire [15:0]                 status_delay_count    // path delay measurements accepted
);

/*
 * Time of day arithmetic, as in ptp_servo
 */

// 10^9 ns in units of 2^-16 ns
localparam [63:0] NS_PER_S_Q16 = 64'd65536000000000;

// a signed number of seconds, -8 to 7, in ns * 2^16
function [63:0] sec_to_q16;
    input [3:0] s;
    begin
        case (s)
            4'd0:  sec_to_q16 = 64'd0;
            4'd1:  sec_to_q16 = NS_PER_S_Q16;
            4'd2:  sec_to_q16 = NS_PER_S_Q16*2;
            4'd3:  sec_to_q16 = NS_PER_S_Q16*3;
            4'd4:  sec_to_q16 = NS_PER_S_Q16*4;
            4'd5:  sec_to_q16 = NS_PER_S_Q16*5;
            4'd6:  sec_to_q16 = NS_PER_S_Q16*6;
            4'd7:  sec_to_q16 = NS_PER_S_Q16*7;
            4'd8:  sec_to_q16 = -(NS_PER_S_Q16*8);
            4'd9:  sec_to_q16 = -(NS_PER_S_Q16*7);
            4'd10: sec_to_q16 = -(NS_PER_S_Q16*6);
            4'd11: sec_to_q16 = -(NS_PER_S_Q16*5);
            4'd12: sec_to_q16 = -(NS_PER_S_Q16*4);
            4'd13: sec_to_q16 = -(NS_PER_S_Q16*3);
            4'd14: sec_to_q16 = -(NS_PER_S_Q16*2);
            default: sec_to_q16 = -NS_PER_S_Q16;
        endcase
    end
endfunction

// a - b; bit 64 is set if the seconds parts are more than 7 apart
function [64:0] tod_diff;
    input [95:0] a;
    input [95:0] b;
    reg [48:0] ds;
    reg [47:0] dn;
    begin
        ds = {1'b0, a[95:48]} - {1'b0, b[95:48]};
        dn = {2'b00, a[45:0]} - {2'b00, b[45:0]};
        tod_diff[63:0] = sec_to_q16(ds[3:0]) + {{16{dn[47]}}, dn};
        tod_diff[64] = (ds[48:3] != {46{1'b0}}) && (ds[48:3] != {46{1'b1}});
    end
endfunction

// a + d; bit 96 is set if |d| is 1 s or more
function [96:0] tod_add;
    input [95:0] a;
    input [63:0] d;
    reg [63:0] sum;
    reg [47:0] s;
    begin
        sum = {18'd0, a[45:0]} + d;
        s = a[95:48];
        if (sum[63]) begin
            sum = sum + NS_PER_S_Q16;
            s = s - 48'd1;
        end else if (sum >= NS_PER_S_Q16) begin
            sum = sum - NS_PER_S_Q16;
            s = s + 48'd1;
        end
        tod_add[95:0] = {s, 2'b00, sum[45:0]};
        tod_add[96] = d[63] ? ((-d) >= NS_PER_S_Q16) : (d >= NS_PER_S_Q16);
    end
endfunction

// |x| for a signed value
function [63:0] abs64;
    input [63:0] x;
    begin
        abs64 = x[63] ? -x : x;
    end
endfunction

/*
 * Constants
 */
localparam [47:0] PTP_MCAST_MAC = 48'h01_1B_19_00_00_00;

localparam [3:0]
    MSG_SYNC       = 4'h0,
    MSG_DELAY_REQ  = 4'h1,
    MSG_FOLLOW_UP  = 4'h8,
    MSG_DELAY_RESP = 4'h9,
    MSG_ANNOUNCE   = 4'hB;

localparam [7:0] CONTROL_DELAY_REQ = 8'h01;
localparam [7:0] LOG_INTERVAL_NONE = 8'h7F;

localparam [63:0] MAX_DELAY_Q16 = 64'd65536 * MAX_DELAY_NS;

localparam [3:0]
    ST_IDLE        = 4'd0,
    ST_RX          = 4'd1,
    ST_T1C         = 4'd2,
    ST_SAMPLE      = 4'd3,
    ST_SAMPLE_WAIT = 4'd4,
    ST_DREQ        = 4'd5,
    ST_DELAY_0     = 4'd6,
    ST_DELAY_1     = 4'd7,
    ST_DELAY_2     = 4'd8;

reg [3:0] state_reg = ST_IDLE;

// master
reg        master_valid_reg = 1'b0;
reg [79:0] master_port_id_reg = 80'd0;
reg [31:0] master_timer_reg = 32'd0;

// Sync in progress
reg        sync_pending_reg = 1'b0;  // two-step Sync waiting for its Follow_Up
reg [15:0] sync_seq_reg = 16'd0;
reg [95:0] sync_t2_reg = 96'd0;
reg [63:0] sync_corr_reg = 64'd0;
reg [95:0] t1_reg = 96'd0;
reg [63:0] corr_reg = 64'd0;
reg [95:0] t1c_reg = 96'd0;          // t1 + corrections

// Delay_Req in progress
reg        dreq_active_reg = 1'b0;
reg [15:0] dreq_seq_reg = 16'd0;
reg [15:0] dreq_seq_next_reg = 16'd0;
reg [95:0] dreq_t1c_reg = 96'd0;
reg [95:0] dreq_t2_reg = 96'd0;
reg [95:0] dreq_t3_reg = 96'd0;
reg [95:0] dreq_t4_reg = 96'd0;      // t4 - Delay_Resp correction
reg        dreq_got_t3_reg = 1'b0;
reg        dreq_got_t4_reg = 1'b0;
reg [31:0] dreq_timer_reg = 32'd0;
reg [7:0]  dreq_div_cnt_reg = 8'd0;
reg [95:0] dreq_origin_ts_reg = 96'd0;
reg        dreq_tx_valid_reg = 1'b0;

// path delay
reg        delay_valid_reg = 1'b0;
reg [63:0] delay_reg = 64'd0;
reg [63:0] delay_a_reg = 64'd0;      // t2 - t3
reg [63:0] delay_b_reg = 64'd0;      // t4 - t1
reg        delay_ovf_reg = 1'b0;
reg [63:0] delay_raw_reg = 64'd0;

// servo
reg [95:0] servo_local_reg = 96'd0;
reg [95:0] servo_master_reg = 96'd0;
reg        servo_sample_valid_reg = 1'b0;
reg        servo_restart_reg = 1'b0;
reg        step_seen_reg = 1'b0;

reg [15:0] sync_count_reg = 16'd0;
reg [15:0] delay_count_reg = 16'd0;

/*
 * Received messages
 */
wire        msg_valid;
wire        msg_ready;
wire [47:0] msg_dst_mac;
wire [3:0]  msg_type;
wire [3:0]  msg_version;
wire [7:0]  msg_domain;
wire [15:0] msg_flags;
wire [63:0] msg_correction;
wire [79:0] msg_src_port_id;
wire [15:0] msg_seq_id;
wire [95:0] msg_ts;
wire [79:0] msg_req_port_id;
wire [95:0] msg_rx_ts;

ptp_msg_rx
ptp_msg_rx_inst (
    .clk(clk),
    .rst(rst),

    .s_axis_tdata(s_axis_rx_tdata),
    .s_axis_tkeep(s_axis_rx_tkeep),
    .s_axis_tvalid(s_axis_rx_tvalid),
    .s_axis_tready(s_axis_rx_tready),
    .s_axis_tlast(s_axis_rx_tlast),
    .s_axis_tuser(s_axis_rx_tuser),

    .m_msg_valid(msg_valid),
    .m_msg_ready(msg_ready),
    .m_msg_dst_mac(msg_dst_mac),
    .m_msg_type(msg_type),
    .m_msg_version(msg_version),
    .m_msg_domain(msg_domain),
    .m_msg_flags(msg_flags),
    .m_msg_correction(msg_correction),
    .m_msg_src_port_id(msg_src_port_id),
    .m_msg_seq_id(msg_seq_id),
    .m_msg_log_interval(),
    .m_msg_ts(msg_ts),
    .m_msg_req_port_id(msg_req_port_id),
    .m_msg_rx_ts(msg_rx_ts)
);

/*
 * Delay_Req transmission
 */

// clockIdentity from the MAC address, port 1
wire [79:0] local_port_id = {cfg_local_mac[47:24], 16'hFFFE, cfg_local_mac[23:0], 16'd1};

wire [PTP_TAG_WIDTH-1:0] dreq_tag = dreq_seq_reg;
wire dreq_tx_ready;

ptp_msg_tx #(
    .TAG_WIDTH(PTP_TAG_WIDTH)
)
ptp_msg_tx_inst (
    .clk(clk),
    .rst(rst),

    .s_msg_valid(dreq_tx_valid_reg),
    .s_msg_ready(dreq_tx_ready),
    .s_msg_dst_mac(PTP_MCAST_MAC),
    .s_msg_src_mac(cfg_local_mac),
    .s_msg_type(MSG_DELAY_REQ),
    .s_msg_domain(cfg_domain),
    .s_msg_flags(16'd0),
    .s_msg_correction(64'd0),
    .s_msg_src_port_id(local_port_id),
    .s_msg_seq_id(dreq_seq_reg),
    .s_msg_control(CONTROL_DELAY_REQ),
    .s_msg_log_interval(LOG_INTERVAL_NONE),
    .s_msg_ts(dreq_origin_ts_reg),
    .s_msg_req_port_id_en(1'b0),
    .s_msg_req_port_id(80'd0),
    .s_msg_ts_en(1'b1),
    .s_msg_tag(dreq_tag),

    .m_axis_tdata(m_axis_tx_tdata),
    .m_axis_tkeep(m_axis_tx_tkeep),
    .m_axis_tvalid(m_axis_tx_tvalid),
    .m_axis_tready(m_axis_tx_tready),
    .m_axis_tlast(m_axis_tx_tlast),
    .m_axis_tuser(m_axis_tx_tuser)
);

/*
 * Servo
 */
wire servo_sample_ready;
wire servo_step;

ptp_servo #(
    .PERIOD_NS_WIDTH(PERIOD_NS_WIDTH),
    .FNS_WIDTH(FNS_WIDTH),
    .PERIOD_NS(PERIOD_NS),
    .PERIOD_FNS(PERIOD_FNS),
    .PTP_CLOCK_PIPELINE(PTP_CLOCK_PIPELINE),
    .KP(SERVO_KP),
    .KI(SERVO_KI),
    .MAX_PPB(SERVO_MAX_PPB),
    .FIRST_STEP_THRESHOLD_NS(SERVO_FIRST_STEP_THRESHOLD_NS),
    .STEP_THRESHOLD_NS(SERVO_STEP_THRESHOLD_NS),
    .LOCK_THRESHOLD_NS(SERVO_LOCK_THRESHOLD_NS),
    .LOCK_COUNT(SERVO_LOCK_COUNT),
    .STEP_SKIP_SAMPLES(SERVO_STEP_SKIP_SAMPLES)
)
ptp_servo_inst (
    .clk(clk),
    .rst(rst),
    .restart(servo_restart_reg),

    .s_sample_local(servo_local_reg),
    .s_sample_master(servo_master_reg),
    .s_sample_valid(servo_sample_valid_reg),
    .s_sample_ready(servo_sample_ready),

    .ptp_ts_96(ptp_ts_96),
    .ptp_set_ts_96(ptp_set_ts_96),
    .ptp_set_ts_96_valid(ptp_set_ts_96_valid),
    .ptp_set_period_ns(ptp_set_period_ns),
    .ptp_set_period_fns(ptp_set_period_fns),
    .ptp_set_period_valid(ptp_set_period_valid),

    .step(servo_step),
    .status_state(status_servo_state),
    .status_locked(status_servo_locked),
    .status_offset(status_offset),
    .status_offset_valid(status_offset_valid),
    .status_adj(status_adj)
);

/*
 * Protocol
 */
wire ts_locked = tx_ptp_locked && rx_ptp_locked;

wire msg_common_ok = cfg_enable && ts_locked &&
    msg_version == 4'd2 &&
    msg_domain == cfg_domain &&
    (msg_dst_mac == PTP_MCAST_MAC || msg_dst_mac == cfg_local_mac) &&
    msg_src_port_id != local_port_id;

wire msg_from_master = master_valid_reg && msg_src_port_id == master_port_id_reg;
wire msg_two_step = msg_flags[9];

wire [63:0] rx_latency = {{32{cfg_rx_latency[31]}}, cfg_rx_latency};
wire [63:0] tx_latency = {{32{cfg_tx_latency[31]}}, cfg_tx_latency};

wire [96:0] t2_calc = tod_add(msg_rx_ts, -rx_latency);
wire [96:0] t3_calc = tod_add(s_axis_tx_ts_96, tx_latency);
wire [96:0] t4_calc = tod_add(msg_ts, -msg_correction);
wire [96:0] t1c_calc = tod_add(t1_reg, corr_reg);
wire [96:0] master_calc = tod_add(t1c_reg, delay_reg);

wire [64:0] delay_a_calc = tod_diff(dreq_t2_reg, dreq_t3_reg);
wire [64:0] delay_b_calc = tod_diff(dreq_t4_reg, dreq_t1c_reg);
wire [63:0] delay_sum = delay_a_reg + delay_b_reg;
wire [63:0] delay_raw_calc = $signed(delay_sum) >>> 1;
wire [63:0] delay_err = delay_raw_reg - delay_reg;
wire [63:0] delay_filter_step = $signed(delay_err) >>> DELAY_FILTER_SHIFT;

assign msg_ready = state_reg == ST_RX;
assign s_axis_tx_ts_ready = state_reg == ST_IDLE && !msg_valid;

assign status_master_valid = master_valid_reg;
assign status_master_port_id = master_port_id_reg;
assign status_delay_valid = delay_valid_reg;
assign status_delay = delay_reg;
assign status_sync_count = sync_count_reg;
assign status_delay_count = delay_count_reg;

always @(posedge clk) begin
    servo_restart_reg <= 1'b0;

    if (dreq_tx_valid_reg && dreq_tx_ready) begin
        dreq_tx_valid_reg <= 1'b0;
    end

    if (servo_sample_valid_reg && servo_sample_ready) begin
        servo_sample_valid_reg <= 1'b0;
    end

    // Delay_Req timeout
    if (dreq_active_reg) begin
        dreq_timer_reg <= dreq_timer_reg + 32'd1;
        if (dreq_timer_reg >= DELAY_REQ_TIMEOUT) begin
            dreq_active_reg <= 1'b0;
        end
    end

    // master timeout
    if (master_valid_reg) begin
        master_timer_reg <= master_timer_reg + 32'd1;
        if (master_timer_reg >= MASTER_TIMEOUT) begin
            master_valid_reg <= 1'b0;
            sync_pending_reg <= 1'b0;
            dreq_active_reg <= 1'b0;
            delay_valid_reg <= 1'b0;
            servo_restart_reg <= 1'b1;
        end
    end

    // clock stepped: timestamps taken before now are on the old timescale
    if (servo_step) begin
        sync_pending_reg <= 1'b0;
        dreq_active_reg <= 1'b0;
        step_seen_reg <= 1'b1;
    end

    // timestamps not valid
    if (!ts_locked || !cfg_enable) begin
        sync_pending_reg <= 1'b0;
        dreq_active_reg <= 1'b0;
    end

    if (!cfg_enable) begin
        master_valid_reg <= 1'b0;
        delay_valid_reg <= 1'b0;
        servo_restart_reg <= 1'b1;
    end

    case (state_reg)
        ST_IDLE: begin
            if (msg_valid) begin
                state_reg <= ST_RX;
            end else if (s_axis_tx_ts_valid) begin
                // TX timestamp, taken this cycle (s_axis_tx_ts_ready is high)
                if (dreq_active_reg && !dreq_got_t3_reg && s_axis_tx_ts_tag == dreq_tag && !t3_calc[96]) begin
                    dreq_t3_reg <= t3_calc[95:0];
                    dreq_got_t3_reg <= 1'b1;
                    if (dreq_got_t4_reg) begin
                        state_reg <= ST_DELAY_0;
                    end
                end
            end
        end
        ST_RX: begin
            // message taken this cycle (msg_ready is high)
            state_reg <= ST_IDLE;

            if (msg_common_ok) begin
                case (msg_type)
                    MSG_SYNC: begin
                        if ((!master_valid_reg || msg_from_master) && !t2_calc[96]) begin
                            master_valid_reg <= 1'b1;
                            master_port_id_reg <= msg_src_port_id;
                            master_timer_reg <= 32'd0;

                            sync_seq_reg <= msg_seq_id;
                            sync_t2_reg <= t2_calc[95:0];

                            if (msg_two_step) begin
                                // t1 follows in the Follow_Up
                                sync_pending_reg <= 1'b1;
                                sync_corr_reg <= msg_correction;
                            end else begin
                                sync_pending_reg <= 1'b0;
                                t1_reg <= msg_ts;
                                corr_reg <= msg_correction;
                                state_reg <= ST_T1C;
                            end
                        end
                    end
                    MSG_FOLLOW_UP: begin
                        if (msg_from_master && sync_pending_reg && msg_seq_id == sync_seq_reg) begin
                            sync_pending_reg <= 1'b0;
                            t1_reg <= msg_ts;
                            corr_reg <= sync_corr_reg + msg_correction;
                            state_reg <= ST_T1C;
                        end
                    end
                    MSG_DELAY_RESP: begin
                        if (msg_from_master && dreq_active_reg && !dreq_got_t4_reg &&
                                msg_seq_id == dreq_seq_reg && msg_req_port_id == local_port_id &&
                                !t4_calc[96]) begin
                            dreq_t4_reg <= t4_calc[95:0];
                            dreq_got_t4_reg <= 1'b1;
                            if (dreq_got_t3_reg) begin
                                state_reg <= ST_DELAY_0;
                            end
                        end
                    end
                    MSG_ANNOUNCE: begin
                        if (msg_from_master) begin
                            master_timer_reg <= 32'd0;
                        end
                    end
                    default: begin
                        // not used by a slave
                    end
                endcase
            end
        end
        ST_T1C: begin
            // t1 + correctionField(s)
            if (t1c_calc[96]) begin
                state_reg <= ST_IDLE;
            end else begin
                t1c_reg <= t1c_calc[95:0];
                sync_count_reg <= sync_count_reg + 16'd1;
                state_reg <= ST_SAMPLE;
            end
        end
        ST_SAMPLE: begin
            // at local time t2, the master's time was t1 + corrections + delay
            state_reg <= ST_DREQ;
            if (delay_valid_reg && !master_calc[96]) begin
                servo_local_reg <= sync_t2_reg;
                servo_master_reg <= master_calc[95:0];
                servo_sample_valid_reg <= 1'b1;
                state_reg <= ST_SAMPLE_WAIT;
            end
        end
        ST_SAMPLE_WAIT: begin
            // wait for the servo to take the sample and finish with it, so
            // that any step it makes is seen before the next Delay_Req
            if (!servo_sample_valid_reg && servo_sample_ready) begin
                state_reg <= ST_DREQ;
            end
        end
        ST_DREQ: begin
            state_reg <= ST_IDLE;
            if (step_seen_reg) begin
                // this Sync's t1 and t2 are from before the step
                step_seen_reg <= 1'b0;
            end else if (!dreq_active_reg && !dreq_tx_valid_reg && cfg_enable && ts_locked) begin
                if ({1'b0, dreq_div_cnt_reg} + 9'd1 >= {1'b0, cfg_delay_req_div}) begin
                    dreq_div_cnt_reg <= 8'd0;

                    dreq_seq_reg <= dreq_seq_next_reg;
                    dreq_seq_next_reg <= dreq_seq_next_reg + 16'd1;
                    dreq_origin_ts_reg <= ptp_ts_96;  // approximate; the real t3 comes from the MAC
                    dreq_tx_valid_reg <= 1'b1;

                    dreq_active_reg <= 1'b1;
                    dreq_got_t3_reg <= 1'b0;
                    dreq_got_t4_reg <= 1'b0;
                    dreq_timer_reg <= 32'd0;
                    dreq_t1c_reg <= t1c_reg;
                    dreq_t2_reg <= sync_t2_reg;
                end else begin
                    dreq_div_cnt_reg <= dreq_div_cnt_reg + 8'd1;
                end
            end
        end
        ST_DELAY_0: begin
            delay_a_reg <= delay_a_calc[63:0];
            delay_b_reg <= delay_b_calc[63:0];
            delay_ovf_reg <= delay_a_calc[64] || delay_b_calc[64];
            state_reg <= ST_DELAY_1;
        end
        ST_DELAY_1: begin
            // mean path delay = ((t2 - t3) + (t4 - t1)) / 2
            dreq_active_reg <= 1'b0;
            delay_raw_reg <= delay_raw_calc;
            if (delay_ovf_reg || abs64(delay_raw_calc) > MAX_DELAY_Q16) begin
                state_reg <= ST_IDLE;
            end else begin
                state_reg <= ST_DELAY_2;
            end
        end
        ST_DELAY_2: begin
            if (cfg_enable && master_valid_reg) begin
                if (delay_valid_reg) begin
                    delay_reg <= delay_reg + delay_filter_step;
                end else begin
                    delay_reg <= delay_raw_reg;
                end
                delay_valid_reg <= 1'b1;
                delay_count_reg <= delay_count_reg + 16'd1;
            end
            state_reg <= ST_IDLE;
        end
        default: begin
            state_reg <= ST_IDLE;
        end
    endcase

    if (rst) begin
        state_reg <= ST_IDLE;
        master_valid_reg <= 1'b0;
        master_timer_reg <= 32'd0;
        sync_pending_reg <= 1'b0;
        dreq_active_reg <= 1'b0;
        dreq_got_t3_reg <= 1'b0;
        dreq_got_t4_reg <= 1'b0;
        dreq_timer_reg <= 32'd0;
        dreq_div_cnt_reg <= 8'd0;
        dreq_seq_next_reg <= 16'd0;
        dreq_tx_valid_reg <= 1'b0;
        delay_valid_reg <= 1'b0;
        delay_reg <= 64'd0;
        servo_sample_valid_reg <= 1'b0;
        servo_restart_reg <= 1'b0;
        step_seen_reg <= 1'b0;
        sync_count_reg <= 16'd0;
        delay_count_reg <= 16'd0;
    end
end

endmodule

`resetall
