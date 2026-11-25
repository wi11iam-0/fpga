/*

PTP message transmitter: builds PTPv2 messages as Ethernet frames

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Builds an IEEE 1588-2008 (PTPv2) message as an Ethernet frame (Layer 2
 * transport, Ethertype 0x88F7, no VLAN tag) and sends it on a 64 bit AXI
 * stream, in the form eth_mac_10g_fifo takes with TX_PTP_TS_CTRL_IN_TUSER and
 * TX_PTP_TAG_ENABLE set: tuser = {tag, timestamp enable, bad frame}, held for
 * the whole frame. With s_msg_ts_en set, the MAC returns the frame's TX
 * timestamp together with s_msg_tag.
 *
 * Messages are 44 bytes (Sync, Delay_Req, Follow_Up), or 54 bytes with
 * s_msg_req_port_id_en set, which appends requestingPortIdentity (Delay_Resp).
 * The frame (58 or 68 bytes) carries no FCS and is not padded; the MAC adds
 * both.
 *
 * s_msg_ts is in ptp_clock's 96 bit time of day format, {seconds[47:0], 2'b00,
 * nanoseconds[29:0], fractional nanoseconds[15:0]}; the fractional part is
 * dropped. The other fields are written big endian as the standard requires.
 * Reserved fields, transportSpecific and minorVersionPTP are 0, versionPTP is 2.
 *
 * A new message is taken (s_msg_ready high) only when the previous frame has
 * been sent.
 */
module ptp_msg_tx #
(
    // TX timestamp tag width; must match the MAC's PTP_TAG_WIDTH
    parameter TAG_WIDTH = 16
)
(
    input  wire                  clk,
    input  wire                  rst,

    /*
     * Message in
     */
    input  wire                  s_msg_valid,
    output wire                  s_msg_ready,
    input  wire [47:0]           s_msg_dst_mac,
    input  wire [47:0]           s_msg_src_mac,
    input  wire [3:0]            s_msg_type,
    input  wire [7:0]            s_msg_domain,
    input  wire [15:0]           s_msg_flags,
    input  wire [63:0]           s_msg_correction,
    input  wire [79:0]           s_msg_src_port_id,   // {clockIdentity[63:0], portNumber[15:0]}
    input  wire [15:0]           s_msg_seq_id,
    input  wire [7:0]            s_msg_control,
    input  wire [7:0]            s_msg_log_interval,
    input  wire [95:0]           s_msg_ts,
    input  wire                  s_msg_req_port_id_en,
    input  wire [79:0]           s_msg_req_port_id,
    input  wire                  s_msg_ts_en,         // ask the MAC for a TX timestamp
    input  wire [TAG_WIDTH-1:0]  s_msg_tag,

    /*
     * PTP frames out
     */
    output wire [63:0]           m_axis_tdata,
    output wire [7:0]            m_axis_tkeep,
    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire                  m_axis_tlast,
    output wire [TAG_WIDTH+1:0]  m_axis_tuser
);

localparam BUF_WORDS = 9;  // 72 bytes, enough for the 68 byte Delay_Resp frame

wire [31:0] ts_ns = {2'b00, s_msg_ts[45:16]};

// the frame for the message on s_msg_*
reg [BUF_WORDS*64-1:0] frame;

integer i;

always @* begin
    frame = {BUF_WORDS*64{1'b0}};

    // Ethernet header
    for (i = 0; i < 6; i = i + 1) begin
        frame[8*(0+i) +: 8] = s_msg_dst_mac[8*(5-i) +: 8];
        frame[8*(6+i) +: 8] = s_msg_src_mac[8*(5-i) +: 8];
    end
    frame[8*12 +: 8] = 8'h88;
    frame[8*13 +: 8] = 8'hF7;

    // PTP header
    frame[8*14 +: 8] = {4'h0, s_msg_type};
    frame[8*15 +: 8] = 8'h02;
    frame[8*16 +: 8] = 8'h00;
    frame[8*17 +: 8] = s_msg_req_port_id_en ? 8'd54 : 8'd44;
    frame[8*18 +: 8] = s_msg_domain;
    frame[8*20 +: 8] = s_msg_flags[15:8];
    frame[8*21 +: 8] = s_msg_flags[7:0];
    for (i = 0; i < 8; i = i + 1) begin
        frame[8*(22+i) +: 8] = s_msg_correction[8*(7-i) +: 8];
    end
    for (i = 0; i < 10; i = i + 1) begin
        frame[8*(34+i) +: 8] = s_msg_src_port_id[8*(9-i) +: 8];
    end
    frame[8*44 +: 8] = s_msg_seq_id[15:8];
    frame[8*45 +: 8] = s_msg_seq_id[7:0];
    frame[8*46 +: 8] = s_msg_control;
    frame[8*47 +: 8] = s_msg_log_interval;

    // timestamp: seconds (6 bytes), nanoseconds (4 bytes)
    for (i = 0; i < 6; i = i + 1) begin
        frame[8*(48+i) +: 8] = s_msg_ts[48+8*(5-i) +: 8];
    end
    for (i = 0; i < 4; i = i + 1) begin
        frame[8*(54+i) +: 8] = ts_ns[8*(3-i) +: 8];
    end

    // requestingPortIdentity
    if (s_msg_req_port_id_en) begin
        for (i = 0; i < 10; i = i + 1) begin
            frame[8*(58+i) +: 8] = s_msg_req_port_id[8*(9-i) +: 8];
        end
    end
end

reg [BUF_WORDS*64-1:0] frame_reg = {BUF_WORDS*64{1'b0}};
reg [3:0] word_reg = 4'd0;
reg [3:0] last_word_reg = 4'd0;
reg [7:0] last_keep_reg = 8'd0;
reg [TAG_WIDTH+1:0] tuser_reg = {TAG_WIDTH+2{1'b0}};
reg active_reg = 1'b0;

wire last_word = word_reg == last_word_reg;

assign s_msg_ready = !active_reg;

assign m_axis_tdata = frame_reg[word_reg*64 +: 64];
assign m_axis_tkeep = last_word ? last_keep_reg : 8'hff;
assign m_axis_tvalid = active_reg;
assign m_axis_tlast = last_word;
assign m_axis_tuser = tuser_reg;

always @(posedge clk) begin
    if (m_axis_tvalid && m_axis_tready) begin
        if (last_word) begin
            active_reg <= 1'b0;
        end else begin
            word_reg <= word_reg + 4'd1;
        end
    end

    if (s_msg_valid && s_msg_ready) begin
        frame_reg <= frame;
        word_reg <= 4'd0;
        if (s_msg_req_port_id_en) begin
            // 68 bytes: 8 full words and 4 bytes
            last_word_reg <= 4'd8;
            last_keep_reg <= 8'h0f;
        end else begin
            // 58 bytes: 7 full words and 2 bytes
            last_word_reg <= 4'd7;
            last_keep_reg <= 8'h03;
        end
        tuser_reg <= {s_msg_tag, s_msg_ts_en, 1'b0};
        active_reg <= 1'b1;
    end

    if (rst) begin
        word_reg <= 4'd0;
        active_reg <= 1'b0;
    end
end

endmodule

`resetall
