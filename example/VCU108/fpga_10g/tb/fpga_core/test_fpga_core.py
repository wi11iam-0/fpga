"""

Copyright (c) 2020 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

"""

import logging
import os

from scapy.layers.l2 import Ether, ARP
from scapy.layers.inet import IP, UDP
from scapy.packet import Raw

import cocotb_test.simulator

import cocotb
from cocotb.log import SimLog
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotb.utils import get_sim_time, get_time_from_sim_steps

from cocotbext.eth import GmiiFrame, GmiiSource, GmiiSink
from cocotbext.eth import XgmiiFrame, XgmiiSource, XgmiiSink
from cocotbext.axi import AxiStreamBus, AxiStreamSink


CLK_PERIOD_NS = 6.4  # dut.clk and, on the board, the XGMII clocks (156.25 MHz)


def ptp_ts_96_to_ns(ts):
    """Convert a 96 bit PTP timestamp {seconds[47:0], 2'b00, ns[29:0], fns[15:0]} to ns."""
    seconds = ts >> 48
    ns = (ts >> 16) & 0x3fffffff
    fns = ts & 0xffff
    return seconds * 1e9 + ns + fns / 2**16


def start_time_ns(frame):
    """Simulation time at which the start character of an XGMII frame was on the interface, in ns.

    A frame starts in byte lane 0 or 4 of a 64 bit XGMII word. A start in
    lane 4 is half a word, so half a clock period, later than the word itself.
    """
    word_time_ns = get_time_from_sim_steps(frame.sim_time_start, "ns")
    return word_time_ns + CLK_PERIOD_NS * frame.start_lane / 8


def build_udp_frame(payload):
    eth = Ether(src='5a:51:52:53:54:55', dst='02:00:00:00:00:00')
    ip = IP(src='192.168.1.100', dst='192.168.1.128')
    udp = UDP(sport=5678, dport=1234)
    return eth / ip / udp / payload


PTP_ETH_TYPE = 0x88F7
PTP_MULTICAST_MAC = '01:1b:19:00:00:00'


def build_ptp_frame(sequence_id, length=44, src='5a:51:52:53:54:55'):
    """Ethernet frame carrying a PTP message (IEEE 1588 over Ethernet).

    Only the fields that identify the message are filled in (messageType =
    Sync, versionPTP, messageLength, sequenceId); the rest is a counting
    pattern. 44 bytes is the length of a Sync, Follow_Up or Delay_Req
    message; other lengths are used to move where the frame ends on the bus.
    """
    msg = bytearray(x % 256 for x in range(length))
    msg[0] = 0x00                               # transportSpecific 0, messageType 0 (Sync)
    msg[1] = 0x02                               # versionPTP 2
    msg[2:4] = length.to_bytes(2, 'big')        # messageLength
    msg[30:32] = sequence_id.to_bytes(2, 'big') # sequenceId
    return Ether(src=src, dst=PTP_MULTICAST_MAC, type=PTP_ETH_TYPE) / Raw(bytes(msg))


def eth_type(frame):
    """Ethertype of an Ethernet frame given as bytes."""
    return int.from_bytes(frame[12:14], 'big')


def frame_bytes_without_fcs(frame):
    """Bytes the MAC passes on for a frame sent with XgmiiFrame.from_payload:
    the payload, padded to the 60 byte minimum."""
    return bytes(frame).ljust(60, b'\x00')


def axis_tuser_first(frame):
    """tuser of the first word of an AXI stream frame (cocotbext-axi collapses
    tuser to one value when it is the same on every word)."""
    return frame.tuser[0] if isinstance(frame.tuser, list) else frame.tuser


class TB:
    def __init__(self, dut, xgmii_clk_period_ns=2.56):
        self.dut = dut

        self.log = SimLog("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        cocotb.start_soon(Clock(dut.clk, 6.4, units="ns").start())

        # PTP sample clock: only has to be unrelated to the other clocks
        # (the board uses its 125 MHz clock)
        cocotb.start_soon(Clock(dut.ptp_sample_clk, 9.9, units="ns").start())

        # Ethernet
        cocotb.start_soon(Clock(dut.phy_gmii_clk, 8, units="ns").start())

        self.gmii_source = GmiiSource(dut.phy_gmii_rxd, dut.phy_gmii_rx_er, dut.phy_gmii_rx_dv,
            dut.phy_gmii_clk, dut.phy_gmii_rst, dut.phy_gmii_clk_en)
        self.gmii_sink = GmiiSink(dut.phy_gmii_txd, dut.phy_gmii_tx_er, dut.phy_gmii_tx_en,
            dut.phy_gmii_clk, dut.phy_gmii_rst, dut.phy_gmii_clk_en)

        dut.phy_gmii_clk_en.setimmediatevalue(1)

        # PTP engine port: received PTP frames with their RX timestamps
        self.ptp_rx_sink = AxiStreamSink(AxiStreamBus.from_prefix(dut, "ptp_rx_axis"), dut.clk, dut.rst)

        self.qsfp_source = []
        self.qsfp_sink = []

        for y in range(1, 5):
            cocotb.start_soon(Clock(getattr(dut, f"qsfp_rx_clk_{y}"), xgmii_clk_period_ns, units="ns").start())
            source = XgmiiSource(getattr(dut, f"qsfp_rxd_{y}"), getattr(dut, f"qsfp_rxc_{y}"), getattr(dut, f"qsfp_rx_clk_{y}"), getattr(dut, f"qsfp_rx_rst_{y}"))
            self.qsfp_source.append(source)
            cocotb.start_soon(Clock(getattr(dut, f"qsfp_tx_clk_{y}"), xgmii_clk_period_ns, units="ns").start())
            sink = XgmiiSink(getattr(dut, f"qsfp_txd_{y}"), getattr(dut, f"qsfp_txc_{y}"), getattr(dut, f"qsfp_tx_clk_{y}"), getattr(dut, f"qsfp_tx_rst_{y}"))
            self.qsfp_sink.append(sink)

        dut.btnu.setimmediatevalue(0)
        dut.btnl.setimmediatevalue(0)
        dut.btnd.setimmediatevalue(0)
        dut.btnr.setimmediatevalue(0)
        dut.btnc.setimmediatevalue(0)
        dut.sw.setimmediatevalue(0)
        dut.uart_rxd.setimmediatevalue(0)
        dut.uart_cts.setimmediatevalue(0)

    async def init(self):

        self.dut.rst.setimmediatevalue(0)
        self.dut.phy_gmii_rst.setimmediatevalue(0)
        for y in range(1, 5):
            getattr(self.dut, f"qsfp_rx_rst_{y}").setimmediatevalue(0)
            getattr(self.dut, f"qsfp_tx_rst_{y}").setimmediatevalue(0)

        for k in range(10):
            await RisingEdge(self.dut.clk)

        self.dut.rst.value = 1
        self.dut.phy_gmii_rst.value = 1
        for y in range(1, 5):
            getattr(self.dut, f"qsfp_rx_rst_{y}").value = 1
            getattr(self.dut, f"qsfp_tx_rst_{y}").value = 1

        for k in range(10):
            await RisingEdge(self.dut.clk)

        self.dut.rst.value = 0
        self.dut.phy_gmii_rst.value = 0
        for y in range(1, 5):
            getattr(self.dut, f"qsfp_rx_rst_{y}").value = 0
            getattr(self.dut, f"qsfp_tx_rst_{y}").value = 0

    async def wait_ptp_locked(self):
        """Wait until the PTP time has been transferred into the MAC RX and TX clock domains.

        Timestamps taken before the two ptp_clock_cdc instances lock are not valid.
        """
        while not (self.dut.rx_ptp_locked.value.integer and self.dut.tx_ptp_locked.value.integer):
            await RisingEdge(self.dut.clk)

    async def ptp_time_offset_ns(self):
        """Return the offset between PTP time and simulation time, in ns.

        The PTP clock free-runs from 0 at reset, so it differs from simulation
        time by a constant. ptp_ts_96 changes on each clk edge, and the value
        read at an edge is the one that was presented since the previous edge.
        """
        await RisingEdge(self.dut.clk)
        ptp_ns = ptp_ts_96_to_ns(self.dut.ptp_ts_96.value.integer)
        return ptp_ns - (get_sim_time("ns") - CLK_PERIOD_NS)

    async def count_stack_rx_frames(self, counter):
        """Count frames passed on from the MAC towards the switch and UDP stack.

        counter is a one element list, so the count can be read while this runs.
        """
        dut = self.dut
        while True:
            await RisingEdge(dut.clk)
            if dut.mac_rx_axis_tvalid.value.integer and dut.mac_rx_axis_tready.value.integer and dut.mac_rx_axis_tlast.value.integer:
                counter[0] += 1

    async def monitor_tx_ptp_ts(self, timestamps):
        """Append every TX timestamp to the list as a (tag, ns) tuple."""
        while True:
            await RisingEdge(self.dut.clk)
            if self.dut.tx_ptp_ts_valid.value.integer:
                timestamps.append((self.dut.tx_ptp_ts_tag.value.integer,
                    ptp_ts_96_to_ns(self.dut.tx_ptp_ts_96.value.integer)))


@cocotb.test()
async def run_test(dut):

    tb = TB(dut)

    await tb.init()

    tb.log.info("test UDP RX packet")

    payload = bytes([x % 256 for x in range(256)])
    eth = Ether(src='5a:51:52:53:54:55', dst='02:00:00:00:00:00')
    ip = IP(src='192.168.1.100', dst='192.168.1.128')
    udp = UDP(sport=5678, dport=1234)
    test_pkt = eth / ip / udp / payload

    test_frame = XgmiiFrame.from_payload(test_pkt.build())

    await tb.qsfp_source[0].send(test_frame)

    tb.log.info("receive ARP request")

    rx_frame = await tb.qsfp_sink[0].recv()

    rx_pkt = Ether(bytes(rx_frame.get_payload()))

    tb.log.info("RX packet: %s", repr(rx_pkt))

    assert rx_pkt.dst == 'ff:ff:ff:ff:ff:ff'
    assert rx_pkt.src == test_pkt.dst
    assert rx_pkt[ARP].hwtype == 1
    assert rx_pkt[ARP].ptype == 0x0800
    assert rx_pkt[ARP].hwlen == 6
    assert rx_pkt[ARP].plen == 4
    assert rx_pkt[ARP].op == 1
    assert rx_pkt[ARP].hwsrc == test_pkt.dst
    assert rx_pkt[ARP].psrc == test_pkt[IP].dst
    assert rx_pkt[ARP].hwdst == '00:00:00:00:00:00'
    assert rx_pkt[ARP].pdst == test_pkt[IP].src

    tb.log.info("send ARP response")

    eth = Ether(src=test_pkt.src, dst=test_pkt.dst)
    arp = ARP(hwtype=1, ptype=0x0800, hwlen=6, plen=4, op=2,
        hwsrc=test_pkt.src, psrc=test_pkt[IP].src,
        hwdst=test_pkt.dst, pdst=test_pkt[IP].dst)
    resp_pkt = eth / arp

    resp_frame = XgmiiFrame.from_payload(resp_pkt.build())

    await tb.qsfp_source[0].send(resp_frame)

    tb.log.info("receive UDP packet")

    rx_frame = await tb.qsfp_sink[0].recv()

    rx_pkt = Ether(bytes(rx_frame.get_payload()))

    tb.log.info("RX packet: %s", repr(rx_pkt))

    assert rx_pkt.dst == test_pkt.src
    assert rx_pkt.src == test_pkt.dst
    assert rx_pkt[IP].dst == test_pkt[IP].src
    assert rx_pkt[IP].src == test_pkt[IP].dst
    assert rx_pkt[UDP].dport == test_pkt[UDP].sport
    assert rx_pkt[UDP].sport == test_pkt[UDP].dport
    assert rx_pkt[UDP].payload == test_pkt[UDP].payload

    tb.log.info("test gigabit tap, RX side")

    # insert tap
    await RisingEdge(dut.clk)
    dut.sw.value = 0x8
    await RisingEdge(dut.clk)

    payload = bytes([x % 256 for x in range(256)])
    eth = Ether(src='5a:51:52:53:54:55', dst='02:00:00:00:00:00')
    ip = IP(src='192.168.1.100', dst='192.168.1.128')
    udp = UDP(sport=5678, dport=1234)
    test_pkt = eth / ip / udp / payload

    test_frame = GmiiFrame.from_payload(test_pkt.build())

    await tb.gmii_source.send(test_frame)

    tb.log.info("loop back packet on XGMII interface")

    rx_frame = await tb.qsfp_sink[0].recv()

    rx_pkt = Ether(bytes(rx_frame.get_payload()))

    tb.log.info("RX packet: %s", repr(rx_pkt))

    await tb.qsfp_source[0].send(rx_frame)

    tb.log.info("receive UDP packet")

    rx_frame = await tb.gmii_sink.recv()

    rx_pkt = Ether(bytes(rx_frame.get_payload()))

    tb.log.info("RX packet: %s", repr(rx_pkt))

    assert rx_pkt.dst == test_pkt.src
    assert rx_pkt.src == test_pkt.dst
    assert rx_pkt[IP].dst == test_pkt[IP].src
    assert rx_pkt[IP].src == test_pkt[IP].dst
    assert rx_pkt[UDP].dport == test_pkt[UDP].sport
    assert rx_pkt[UDP].sport == test_pkt[UDP].dport
    assert rx_pkt[UDP].payload == test_pkt[UDP].payload

    tb.log.info("test gigabit tap, TX side")

    # insert tap
    await RisingEdge(dut.clk)
    dut.sw.value = 0xC
    await RisingEdge(dut.clk)

    payload = bytes([x % 256 for x in range(256)])
    eth = Ether(src='5a:51:52:53:54:55', dst='02:00:00:00:00:00')
    ip = IP(src='192.168.1.100', dst='192.168.1.128')
    udp = UDP(sport=5678, dport=1234)
    test_pkt = eth / ip / udp / payload

    test_frame = GmiiFrame.from_payload(test_pkt.build())

    await tb.gmii_source.send(test_frame)

    tb.log.info("loop back packet on XGMII interface")

    rx_frame = await tb.qsfp_sink[0].recv()

    rx_pkt = Ether(bytes(rx_frame.get_payload()))

    tb.log.info("RX packet: %s", repr(rx_pkt))

    await tb.qsfp_source[0].send(rx_frame)

    tb.log.info("receive UDP packet")

    rx_frame = await tb.gmii_sink.recv()

    rx_pkt = Ether(bytes(rx_frame.get_payload()))

    tb.log.info("RX packet: %s", repr(rx_pkt))

    assert rx_pkt.dst == test_pkt.src
    assert rx_pkt.src == test_pkt.dst
    assert rx_pkt[IP].dst == test_pkt[IP].src
    assert rx_pkt[IP].src == test_pkt[IP].dst
    assert rx_pkt[UDP].dport == test_pkt[UDP].sport
    assert rx_pkt[UDP].sport == test_pkt[UDP].dport
    assert rx_pkt[UDP].payload == test_pkt[UDP].payload

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


@cocotb.test()
async def run_test_ptp(dut):
    """Check the PTP hardware clock and the timestamping of the 10G MAC.

    RX: PTP frames (Ethertype 0x88F7) mixed with UDP frames are sent into QSFP
    lane 1. Only the PTP frames must appear on the PTP engine port, unchanged,
    each with an RX timestamp matching the time the testbench saw it start on
    the XGMII interface. Everything else must still reach the UDP stack.

    TX: every frame sent gets a TX timestamp, compared the same way.
    """

    # Run the XGMII clocks at 156.25 MHz as on the board, rather than at the
    # faster rate used by run_test: the accuracy of a timestamp is set by the
    # period of the clock it is captured on.
    tb = TB(dut, xgmii_clk_period_ns=CLK_PERIOD_NS)

    await tb.init()

    tb.log.info("check PTP clock rate")

    # the PTP clock is held at 0 until the reset released by tb.init() has taken effect
    for k in range(10):
        await RisingEdge(dut.clk)

    cycles = 1000
    start_ns = ptp_ts_96_to_ns(dut.ptp_ts_96.value.integer)
    for k in range(cycles):
        await RisingEdge(dut.clk)
    elapsed_ns = ptp_ts_96_to_ns(dut.ptp_ts_96.value.integer) - start_ns

    tb.log.info("PTP clock advanced %f ns in %d cycles", elapsed_ns, cycles)

    # ptp_clock evens out its rounding error every 5 cycles, leaving less than 1e-4 ns
    assert abs(elapsed_ns - cycles * CLK_PERIOD_NS) < 1e-3

    tb.log.info("wait for PTP CDC lock")

    await tb.wait_ptp_locked()

    # "locked" is reported while the transferred time is still a couple of ns
    # out; it takes about another 10000 cycles to settle to its final value
    for k in range(12000):
        await RisingEdge(dut.clk)

    ptp_offset_ns = await tb.ptp_time_offset_ns()

    tb.log.info("PTP time is simulation time %+f ns", ptp_offset_ns)

    tx_ts_list = []
    cocotb.start_soon(tb.monitor_tx_ptp_ts(tx_ts_list))

    stack_rx_count = [0]
    cocotb.start_soon(tb.count_stack_rx_frames(stack_rx_count))

    # Frames in the order they cross the XGMII interface, in each direction.
    # The XGMII source appends a copy of each frame to frames_in once it has
    # been sent; that copy records when the frame was on the interface.
    frames_in = []
    frames_out = []

    tb.log.info("resolve ARP with a first UDP packet")

    first_pkt = build_udp_frame(bytes(range(32)))
    first_frame = XgmiiFrame.from_payload(first_pkt.build(), tx_complete=frames_in.append)
    await tb.qsfp_source[0].send(first_frame)

    arp_request_frame = await tb.qsfp_sink[0].recv()
    frames_out.append(arp_request_frame)
    assert Ether(bytes(arp_request_frame.get_payload()))[ARP].op == 1

    arp_reply_pkt = Ether(src=first_pkt.src, dst=first_pkt.dst) / ARP(
        hwtype=1, ptype=0x0800, hwlen=6, plen=4, op=2,
        hwsrc=first_pkt.src, psrc=first_pkt[IP].src,
        hwdst=first_pkt.dst, pdst=first_pkt[IP].dst)
    arp_reply_frame = XgmiiFrame.from_payload(arp_reply_pkt.build(), tx_complete=frames_in.append)
    await tb.qsfp_source[0].send(arp_reply_frame)

    frames_out.append(await tb.qsfp_sink[0].recv())

    tb.log.info("send a burst of UDP and PTP frames")

    # Back to back, alternating UDP and PTP frames of different lengths, so
    # that frames end in every byte lane and start in both lane 0 and lane 4
    udp_payloads = [bytes(x % 256 for x in range(n)) for n in (19, 20, 21, 22, 23, 24, 25, 26, 60, 128, 256, 512)]
    ptp_pkts = [build_ptp_frame(sequence_id=k, length=44 + k) for k in range(len(udp_payloads))]

    for udp_payload, ptp_pkt in zip(udp_payloads, ptp_pkts):
        for pkt in (build_udp_frame(udp_payload), ptp_pkt):
            await tb.qsfp_source[0].send(XgmiiFrame.from_payload(pkt.build(), tx_complete=frames_in.append))

    tb.log.info("check that the UDP frames are echoed")

    for payload in udp_payloads:
        frame = await tb.qsfp_sink[0].recv()
        frames_out.append(frame)
        assert Ether(bytes(frame.get_payload()))[UDP].payload.load == payload

    tb.log.info("check that the PTP frames arrive on the PTP engine port")

    ptp_frames_rx = [await tb.ptp_rx_sink.recv() for k in range(len(ptp_pkts))]

    # let the last TX timestamp cross from the TX clock domain
    for k in range(20):
        await RisingEdge(dut.clk)

    assert tb.ptp_rx_sink.empty(), "a frame that is not PTP reached the PTP engine port"

    # every frame that is not PTP still goes to the UDP stack
    ptp_frames_in = [f for f in frames_in if eth_type(f.get_payload()) == PTP_ETH_TYPE]
    assert stack_rx_count[0] == len(frames_in) - len(ptp_frames_in)

    # The XGMII models report the clock edge at which a word is driven
    # (source) or sampled (sink). The MAC timestamps a frame
    # - on RX at the edge where it samples the start word, one cycle after
    #   the source drives it
    # - on TX at the edge where it drives the start word, one cycle before
    #   the sink samples it
    rx_capture_delay_ns = CLK_PERIOD_NS
    tx_capture_delay_ns = -CLK_PERIOD_NS

    # Timestamps should be accurate to well under a clock cycle. The error
    # that remains is in the transfer of the PTP time into the MAC clock
    # domains, which settles to within a fraction of a ns.
    max_error_ns = CLK_PERIOD_NS / 4

    tb.log.info("check the PTP frames and their RX timestamps")

    assert len(ptp_frames_in) == len(ptp_pkts)

    for pkt, frame_in, frame_rx in zip(ptp_pkts, ptp_frames_in, ptp_frames_rx):
        # passed on unchanged and not marked bad
        assert bytes(frame_rx.tdata) == frame_bytes_without_fcs(pkt.build())
        tuser = axis_tuser_first(frame_rx)
        assert tuser & 1 == 0

        ts_sim_ns = ptp_ts_96_to_ns(tuser >> 1) - ptp_offset_ns
        error_ns = ts_sim_ns - (start_time_ns(frame_in) + rx_capture_delay_ns)
        tb.log.info("RX PTP frame: start lane %d at %.2f ns, timestamp %.2f ns, error %+.3f ns",
            frame_in.start_lane, start_time_ns(frame_in), ts_sim_ns, error_ns)
        assert abs(error_ns) < max_error_ns

    assert {frame.start_lane for frame in ptp_frames_in} == {0, 4}

    tb.log.info("check TX timestamps")

    assert len(tx_ts_list) == len(frames_out)

    # the tag is a count of transmitted frames, starting from 0 at reset
    assert [tag for tag, ts_ns in tx_ts_list] == list(range(len(frames_out)))

    for frame, (tag, ts_ns) in zip(frames_out, tx_ts_list):
        ts_sim_ns = ts_ns - ptp_offset_ns
        error_ns = ts_sim_ns - (start_time_ns(frame) + tx_capture_delay_ns)
        tb.log.info("TX frame %d: start lane %d at %.2f ns, timestamp %.2f ns, error %+.3f ns",
            tag, frame.start_lane, start_time_ns(frame), ts_sim_ns, error_ns)
        assert abs(error_ns) < max_error_ns

    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


# cocotb-test

tests_dir = os.path.abspath(os.path.dirname(__file__))
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', '..', 'rtl'))
lib_dir = os.path.abspath(os.path.join(rtl_dir, '..', 'lib'))
axis_rtl_dir = os.path.abspath(os.path.join(lib_dir, 'eth', 'lib', 'axis', 'rtl'))
eth_rtl_dir = os.path.abspath(os.path.join(lib_dir, 'eth', 'rtl'))


def test_fpga_core(request):
    dut = "fpga_core"
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = dut

    verilog_sources = [
        os.path.join(rtl_dir, f"{dut}.v"),
        os.path.join(rtl_dir, "axis_eth_type_demux.v"),
        os.path.join(eth_rtl_dir, "eth_mac_1g_fifo.v"),
        os.path.join(eth_rtl_dir, "eth_mac_1g.v"),
        os.path.join(eth_rtl_dir, "axis_gmii_rx.v"),
        os.path.join(eth_rtl_dir, "axis_gmii_tx.v"),
        os.path.join(eth_rtl_dir, "eth_mac_10g_fifo.v"),
        os.path.join(eth_rtl_dir, "eth_mac_10g.v"),
        os.path.join(eth_rtl_dir, "axis_xgmii_rx_64.v"),
        os.path.join(eth_rtl_dir, "axis_xgmii_tx_64.v"),
        os.path.join(eth_rtl_dir, "lfsr.v"),
        os.path.join(eth_rtl_dir, "ptp_clock.v"),
        os.path.join(eth_rtl_dir, "ptp_clock_cdc.v"),
        os.path.join(eth_rtl_dir, "eth_axis_rx.v"),
        os.path.join(eth_rtl_dir, "eth_axis_tx.v"),
        os.path.join(eth_rtl_dir, "udp_complete_64.v"),
        os.path.join(eth_rtl_dir, "udp_checksum_gen_64.v"),
        os.path.join(eth_rtl_dir, "udp_64.v"),
        os.path.join(eth_rtl_dir, "udp_ip_rx_64.v"),
        os.path.join(eth_rtl_dir, "udp_ip_tx_64.v"),
        os.path.join(eth_rtl_dir, "ip_complete_64.v"),
        os.path.join(eth_rtl_dir, "ip_64.v"),
        os.path.join(eth_rtl_dir, "ip_eth_rx_64.v"),
        os.path.join(eth_rtl_dir, "ip_eth_tx_64.v"),
        os.path.join(eth_rtl_dir, "ip_arb_mux.v"),
        os.path.join(eth_rtl_dir, "arp.v"),
        os.path.join(eth_rtl_dir, "arp_cache.v"),
        os.path.join(eth_rtl_dir, "arp_eth_rx.v"),
        os.path.join(eth_rtl_dir, "arp_eth_tx.v"),
        os.path.join(eth_rtl_dir, "eth_arb_mux.v"),
        os.path.join(axis_rtl_dir, "arbiter.v"),
        os.path.join(axis_rtl_dir, "priority_encoder.v"),
        os.path.join(axis_rtl_dir, "axis_adapter.v"),
        os.path.join(axis_rtl_dir, "axis_fifo.v"),
        os.path.join(axis_rtl_dir, "axis_switch.v"),
        os.path.join(axis_rtl_dir, "axis_register.v"),
        os.path.join(axis_rtl_dir, "axis_async_fifo.v"),
        os.path.join(axis_rtl_dir, "axis_async_fifo_adapter.v"),
    ]

    parameters = {}

    # parameters['A'] = val

    extra_env = {f'PARAM_{k}': str(v) for k, v in parameters.items()}

    sim_build = os.path.join(tests_dir, "sim_build",
        request.node.name.replace('[', '-').replace(']', ''))

    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=verilog_sources,
        toplevel=toplevel,
        module=module,
        parameters=parameters,
        sim_build=sim_build,
        extra_env=extra_env,
    )
