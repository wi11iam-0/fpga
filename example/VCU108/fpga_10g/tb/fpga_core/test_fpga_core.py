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

import cocotb_test.simulator

import cocotb
from cocotb.log import SimLog
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotb.utils import get_sim_time, get_time_from_sim_steps

from cocotbext.eth import GmiiFrame, GmiiSource, GmiiSink
from cocotbext.eth import XgmiiFrame, XgmiiSource, XgmiiSink


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

    async def monitor_rx_ptp_ts(self, timestamps):
        """Append every RX timestamp (in ns) to the list."""
        while True:
            await RisingEdge(self.dut.clk)
            if self.dut.rx_ptp_ts_valid.value.integer:
                timestamps.append(ptp_ts_96_to_ns(self.dut.rx_ptp_ts_96.value.integer))

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
    """Check the PTP hardware clock and the RX/TX timestamps of the 10G MAC.

    UDP frames are sent to the echo port on QSFP lane 1. For every frame in
    and every frame out, the timestamp reported by the DUT is compared with
    the time the testbench saw the start of the frame on the XGMII interface.
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

    rx_ts_list = []
    tx_ts_list = []
    cocotb.start_soon(tb.monitor_rx_ptp_ts(rx_ts_list))
    cocotb.start_soon(tb.monitor_tx_ptp_ts(tx_ts_list))

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

    tb.log.info("send a burst of UDP packets")

    # Back to back frames of different lengths, so that frames end in every
    # byte lane and start in both lane 0 and lane 4
    payloads = [bytes(x % 256 for x in range(n)) for n in (19, 20, 21, 22, 23, 24, 25, 26, 60, 128, 256, 512)]

    for payload in payloads:
        frame = XgmiiFrame.from_payload(build_udp_frame(payload).build(), tx_complete=frames_in.append)
        await tb.qsfp_source[0].send(frame)

    for payload in payloads:
        frame = await tb.qsfp_sink[0].recv()
        frames_out.append(frame)
        assert Ether(bytes(frame.get_payload()))[UDP].payload.load == payload

    # let the last TX timestamp cross from the TX clock domain
    for k in range(20):
        await RisingEdge(dut.clk)

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

    tb.log.info("check RX timestamps")

    assert len(rx_ts_list) == len(frames_in)

    for frame, ts_ns in zip(frames_in, rx_ts_list):
        ts_sim_ns = ts_ns - ptp_offset_ns
        error_ns = ts_sim_ns - (start_time_ns(frame) + rx_capture_delay_ns)
        tb.log.info("RX frame: start lane %d at %.2f ns, timestamp %.2f ns, error %+.3f ns",
            frame.start_lane, start_time_ns(frame), ts_sim_ns, error_ns)
        assert abs(error_ns) < max_error_ns

    assert {frame.start_lane for frame in frames_in} == {0, 4}

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

    tb.log.info("check latency through the FPGA from the timestamps")

    # frames_in[0] is echoed as frames_out[1] (after the ARP exchange), and
    # each frame of the burst as the frame at the same index
    echo_pairs = [(0, 1)] + [(k, k) for k in range(2, len(frames_in))]

    for in_index, out_index in echo_pairs:
        # start of frame in to start of frame out, as reported by the XGMII models
        measured_ns = (tx_ts_list[out_index][1] - tx_capture_delay_ns) - (rx_ts_list[in_index] - rx_capture_delay_ns)
        actual_ns = start_time_ns(frames_out[out_index]) - start_time_ns(frames_in[in_index])
        tb.log.info("echo of frame %d: %.2f ns by timestamps, %.2f ns actual", in_index, measured_ns, actual_ns)
        assert abs(measured_ns - actual_ns) < 2 * max_error_ns

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
        os.path.join(eth_rtl_dir, "ptp_ts_extract.v"),
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
