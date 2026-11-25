/*

PTP message receiver: reads the fields of PTPv2 messages from Ethernet frames

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Reads IEEE 1588-2008 (PTPv2) messages carried directly in Ethernet frames
 * (Layer 2 transport, Ethertype 0x88F7, no VLAN tag) from a 64 bit AXI stream,
 * and presents the fields a PTP slave needs on m_msg_*, one message at a time.
 *
 * Input frames are whole Ethernet frames without FCS, as eth_mac_10g_fifo
 * delivers them with PTP_TS_ENABLE set: tuser = {RX timestamp, bad frame},
 * with the same timestamp on every word. The Ethertype is not checked here;
 * the frames are expected to have been picked out by Ethertype already (see
 * axis_eth_type_demux).
 *
 * The first 72 bytes of each frame are buffered, which covers every field up
 * to the end of a Delay_Resp. Frame byte n arrives in tdata[8*(n%8) +: 8] of
 * word n/8. After the last word the frame is checked, and if it is usable the
 * message is held on m_msg_* until m_msg_ready. s_axis_tready is low from the
 * last word of a frame until its message has been taken.
 *
 * A frame is dropped, without producing a message, when:
 * - the bad frame bit (tuser[0]) is set on its last word
 * - it is too short for its message type
 * - its message type carries a timestamp and the nanoseconds field is 10^9 or
 *   more
 *
 * Multi-byte fields are big endian on the wire and are returned as ordinary
 * numbers. Timestamps are returned in ptp_clock's 96 bit time of day format:
 * {seconds[47:0], 2'b00, nanoseconds[29:0], fractional nanoseconds[15:0]}.
 *
 * Byte offsets in the frame of the fields read here:
 *    0- 5  destination MAC address
 *   14     transportSpecific (high nibble), messageType (low nibble)
 *   15     versionPTP (low nibble)
 *   18     domainNumber
 *   20-21  flagField (twoStepFlag is bit 1 of byte 20, flagField[9])
 *   22-29  correctionField, ns * 2^16, signed
 *   34-43  sourcePortIdentity: clockIdentity (8 bytes), portNumber (2 bytes)
 *   44-45  sequenceId
 *   47     logMessageInterval
 *   48-57  originTimestamp (Sync, Delay_Req), preciseOriginTimestamp
 *          (Follow_Up) or receiveTimestamp (Delay_Resp): seconds (6 bytes),
 *          nanoseconds (4 bytes)
 *   58-67  requestingPortIdentity (Delay_Resp)
 */
module ptp_msg_rx
(
    input  wire         clk,
    input  wire         rst,

    /*
     * PTP frames in
     */
    input  wire [63:0]  s_axis_tdata,
    input  wire [7:0]   s_axis_tkeep,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,
    input  wire         s_axis_tlast,
    input  wire [96:0]  s_axis_tuser,

    /*
     * Message out
     */
    output wire         m_msg_valid,
    input  wire         m_msg_ready,
    output wire [47:0]  m_msg_dst_mac,
    output wire [3:0]   m_msg_type,
    output wire [3:0]   m_msg_version,
    output wire [7:0]   m_msg_domain,
    output wire [15:0]  m_msg_flags,
    output wire [63:0]  m_msg_correction,
    output wire [79:0]  m_msg_src_port_id,   // {clockIdentity[63:0], portNumber[15:0]}
    output wire [15:0]  m_msg_seq_id,
    output wire [7:0]   m_msg_log_interval,
    output wire [95:0]  m_msg_ts,            // timestamp in the message body
    output wire [79:0]  m_msg_req_port_id,   // requestingPortIdentity (Delay_Resp only)
    output wire [95:0]  m_msg_rx_ts          // when the frame was received, from tuser
);

localparam BUF_WORDS = 9;  // 72 bytes

localparam [3:0]
    MSG_SYNC       = 4'h0,
    MSG_DELAY_REQ  = 4'h1,
    MSG_FOLLOW_UP  = 4'h8,
    MSG_DELAY_RESP = 4'h9;

localparam [31:0] NS_PER_S = 32'd1000000000;

reg [BUF_WORDS*64-1:0] buf_reg = {BUF_WORDS*64{1'b0}};
reg [3:0] word_cnt_reg = 4'd0;  // words of the frame received so far, saturating at 15
reg [7:0] len_reg = 8'd0;       // bytes of the frame received so far, saturating at 255
reg [95:0] rx_ts_reg = 96'd0;
reg bad_reg = 1'b0;
reg done_reg = 1'b0;            // a whole frame is in buf_reg, waiting to be checked
reg msg_valid_reg = 1'b0;

// number of valid bytes in a word
function [3:0] keep_count;
    input [7:0] keep;
    integer k;
    begin
        keep_count = 4'd0;
        for (k = 0; k < 8; k = k + 1) begin
            keep_count = keep_count + {3'd0, keep[k]};
        end
    end
endfunction

wire [8:0] len_sum = {1'b0, word_cnt_reg == 4'd0 ? 8'd0 : len_reg} + {5'd0, keep_count(s_axis_tkeep)};

// fields of the buffered frame
reg [47:0] f_dst_mac;
reg [3:0]  f_type;
reg [3:0]  f_version;
reg [7:0]  f_domain;
reg [15:0] f_flags;
reg [63:0] f_correction;
reg [79:0] f_src_port_id;
reg [15:0] f_seq_id;
reg [7:0]  f_log_interval;
reg [47:0] f_ts_s;
reg [31:0] f_ts_ns;
reg [79:0] f_req_port_id;

reg [7:0]  f_min_len;
reg        f_has_ts;

integer i;

always @* begin
    for (i = 0; i < 6; i = i + 1) begin
        f_dst_mac[8*(5-i) +: 8] = buf_reg[8*(0+i) +: 8];
    end

    f_type = buf_reg[8*14 +: 4];
    f_version = buf_reg[8*15 +: 4];
    f_domain = buf_reg[8*18 +: 8];
    f_flags = {buf_reg[8*20 +: 8], buf_reg[8*21 +: 8]};

    for (i = 0; i < 8; i = i + 1) begin
        f_correction[8*(7-i) +: 8] = buf_reg[8*(22+i) +: 8];
    end

    for (i = 0; i < 10; i = i + 1) begin
        f_src_port_id[8*(9-i) +: 8] = buf_reg[8*(34+i) +: 8];
    end

    f_seq_id = {buf_reg[8*44 +: 8], buf_reg[8*45 +: 8]};
    f_log_interval = buf_reg[8*47 +: 8];

    for (i = 0; i < 6; i = i + 1) begin
        f_ts_s[8*(5-i) +: 8] = buf_reg[8*(48+i) +: 8];
    end

    for (i = 0; i < 4; i = i + 1) begin
        f_ts_ns[8*(3-i) +: 8] = buf_reg[8*(54+i) +: 8];
    end

    for (i = 0; i < 10; i = i + 1) begin
        f_req_port_id[8*(9-i) +: 8] = buf_reg[8*(58+i) +: 8];
    end

    // shortest frame that holds every field of this message type
    case (f_type)
        MSG_SYNC, MSG_DELAY_REQ, MSG_FOLLOW_UP: begin
            f_min_len = 8'd58;  // 14 byte Ethernet header + 44 byte message
            f_has_ts = 1'b1;
        end
        MSG_DELAY_RESP: begin
            f_min_len = 8'd68;  // 14 + 54
            f_has_ts = 1'b1;
        end
        default: begin
            f_min_len = 8'd48;  // 14 + 34 byte PTP header
            f_has_ts = 1'b0;
        end
    endcase
end

wire frame_ok = !bad_reg && (len_reg >= f_min_len) && (!f_has_ts || f_ts_ns < NS_PER_S);

assign s_axis_tready = !done_reg && !msg_valid_reg;

// buf_reg does not change while a message is waiting, so the fields are
// taken straight from it
assign m_msg_valid = msg_valid_reg;
assign m_msg_dst_mac = f_dst_mac;
assign m_msg_type = f_type;
assign m_msg_version = f_version;
assign m_msg_domain = f_domain;
assign m_msg_flags = f_flags;
assign m_msg_correction = f_correction;
assign m_msg_src_port_id = f_src_port_id;
assign m_msg_seq_id = f_seq_id;
assign m_msg_log_interval = f_log_interval;
assign m_msg_ts = {f_ts_s, 2'b00, f_ts_ns[29:0], 16'd0};
assign m_msg_req_port_id = f_req_port_id;
assign m_msg_rx_ts = rx_ts_reg;

always @(posedge clk) begin
    if (msg_valid_reg && m_msg_ready) begin
        msg_valid_reg <= 1'b0;
    end

    if (done_reg) begin
        done_reg <= 1'b0;
        msg_valid_reg <= frame_ok;
    end

    if (s_axis_tvalid && s_axis_tready) begin
        if (word_cnt_reg < BUF_WORDS) begin
            buf_reg[word_cnt_reg*64 +: 64] <= s_axis_tdata;
        end

        if (word_cnt_reg == 4'd0) begin
            rx_ts_reg <= s_axis_tuser[96:1];
        end

        if (word_cnt_reg != 4'd15) begin
            word_cnt_reg <= word_cnt_reg + 4'd1;
        end

        len_reg <= len_sum[8] ? 8'd255 : len_sum[7:0];

        if (s_axis_tlast) begin
            word_cnt_reg <= 4'd0;
            bad_reg <= s_axis_tuser[0];
            done_reg <= 1'b1;
        end
    end

    if (rst) begin
        word_cnt_reg <= 4'd0;
        len_reg <= 8'd0;
        bad_reg <= 1'b0;
        done_reg <= 1'b0;
        msg_valid_reg <= 1'b0;
    end
end

endmodule

`resetall
