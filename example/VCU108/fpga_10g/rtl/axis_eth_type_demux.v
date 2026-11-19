/*

AXI4-Stream Ethernet frame demultiplexer, selecting on Ethertype

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Sends each Ethernet frame to one of two outputs: frames whose Ethertype is
 * ETH_TYPE go to m_match_axis, all other frames go to m_other_axis. Frames are
 * passed through unchanged, including tuser.
 *
 * The Ethertype is in bytes 12 and 13 of the frame, which on a 64 bit bus
 * arrive in the second word. The first word of each frame is therefore held
 * in a one word buffer until the second word is on the input and the
 * destination is known. This adds one cycle of latency; back to back frames
 * still pass at full rate.
 *
 * Only untagged frames are recognised. A frame with an 802.1Q VLAN tag has
 * Ethertype 0x8100 in this position and goes to m_other_axis.
 */
module axis_eth_type_demux #
(
    // Width of AXI stream interfaces in bits (only 64 is supported)
    parameter DATA_WIDTH = 64,
    // tkeep signal width (words per cycle)
    parameter KEEP_WIDTH = (DATA_WIDTH/8),
    // tuser signal width
    parameter USER_WIDTH = 1,
    // Ethertype of the frames sent to m_match_axis
    parameter ETH_TYPE = 16'h88F7
)
(
    input  wire                   clk,
    input  wire                   rst,

    /*
     * AXI Stream input
     */
    input  wire [DATA_WIDTH-1:0]  s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]  s_axis_tkeep,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,
    input  wire                   s_axis_tlast,
    input  wire [USER_WIDTH-1:0]  s_axis_tuser,

    /*
     * AXI Stream output: frames with Ethertype ETH_TYPE
     */
    output wire [DATA_WIDTH-1:0]  m_match_axis_tdata,
    output wire [KEEP_WIDTH-1:0]  m_match_axis_tkeep,
    output wire                   m_match_axis_tvalid,
    input  wire                   m_match_axis_tready,
    output wire                   m_match_axis_tlast,
    output wire [USER_WIDTH-1:0]  m_match_axis_tuser,

    /*
     * AXI Stream output: all other frames
     */
    output wire [DATA_WIDTH-1:0]  m_other_axis_tdata,
    output wire [KEEP_WIDTH-1:0]  m_other_axis_tkeep,
    output wire                   m_other_axis_tvalid,
    input  wire                   m_other_axis_tready,
    output wire                   m_other_axis_tlast,
    output wire [USER_WIDTH-1:0]  m_other_axis_tuser
);

// check configuration
initial begin
    if (DATA_WIDTH != 64) begin
        $error("Error: AXI stream interface width must be 64 (instance %m)");
        $finish;
    end

    if (KEEP_WIDTH * 8 != DATA_WIDTH) begin
        $error("Error: AXI stream interface requires byte (8-bit) granularity (instance %m)");
        $finish;
    end
end

// Ethertype, read from the input while it holds the second word of a frame:
// bytes 12 and 13 of the frame (big endian) are at tdata[39:32] and tdata[47:40]
wire [15:0] s_eth_type = {s_axis_tdata[39:32], s_axis_tdata[47:40]};
wire s_eth_type_match = (s_axis_tkeep[5:4] == 2'b11) && (s_eth_type == ETH_TYPE);

// one word buffer
reg [DATA_WIDTH-1:0] buf_tdata_reg = {DATA_WIDTH{1'b0}};
reg [KEEP_WIDTH-1:0] buf_tkeep_reg = {KEEP_WIDTH{1'b0}};
reg                  buf_tvalid_reg = 1'b0;
reg                  buf_tlast_reg = 1'b0;
reg [USER_WIDTH-1:0] buf_tuser_reg = {USER_WIDTH{1'b0}};
reg                  buf_first_reg = 1'b0;  // buffered word is the first word of a frame

reg s_first_reg = 1'b1;  // next input word is the first word of a frame
reg match_reg = 1'b0;    // destination of the frame in progress, after its first word

// Destination of the buffered word:
// - first word of a frame that is also its last word: too short to carry an
//   Ethertype, so it goes to m_other_axis
// - first word of a longer frame: decided by the second word, so it waits
//   until that word is on the input
// - any later word: the same destination as the rest of its frame
wire route_known = buf_tvalid_reg && (!buf_first_reg || buf_tlast_reg || s_axis_tvalid);
wire route_match = buf_first_reg ? (!buf_tlast_reg && s_eth_type_match) : match_reg;

wire buf_out = (m_match_axis_tvalid && m_match_axis_tready) ||
               (m_other_axis_tvalid && m_other_axis_tready);

assign s_axis_tready = !buf_tvalid_reg || buf_out;

assign m_match_axis_tdata  = buf_tdata_reg;
assign m_match_axis_tkeep  = buf_tkeep_reg;
assign m_match_axis_tvalid = route_known && route_match;
assign m_match_axis_tlast  = buf_tlast_reg;
assign m_match_axis_tuser  = buf_tuser_reg;

assign m_other_axis_tdata  = buf_tdata_reg;
assign m_other_axis_tkeep  = buf_tkeep_reg;
assign m_other_axis_tvalid = route_known && !route_match;
assign m_other_axis_tlast  = buf_tlast_reg;
assign m_other_axis_tuser  = buf_tuser_reg;

always @(posedge clk) begin
    if (buf_out) begin
        buf_tvalid_reg <= 1'b0;

        if (buf_first_reg) begin
            // remember where the rest of this frame goes
            match_reg <= route_match;
        end
    end

    if (s_axis_tvalid && s_axis_tready) begin
        buf_tdata_reg <= s_axis_tdata;
        buf_tkeep_reg <= s_axis_tkeep;
        buf_tvalid_reg <= 1'b1;
        buf_tlast_reg <= s_axis_tlast;
        buf_tuser_reg <= s_axis_tuser;
        buf_first_reg <= s_first_reg;

        s_first_reg <= s_axis_tlast;
    end

    if (rst) begin
        buf_tvalid_reg <= 1'b0;
        s_first_reg <= 1'b1;
        match_reg <= 1'b0;
    end
end

endmodule

`resetall
