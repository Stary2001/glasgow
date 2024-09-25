import struct
import logging
import asyncio
import os
import pty
import time
from amaranth import *
from amaranth.lib import io
from amaranth.lib.cdc import FFSynchronizer

from ....support.bits import *
from ....support.logging import *
from ....support.endpoint import *
from ... import *

from ....protocol.blackmagic_debug import BlackmagicRemote

class SWDProbeBus(Elaboratable):
    def __init__(self, ports):
        self.ports = ports
        self.swclk = Signal(init=1)
        self.swdio_i = Signal()
        self.swdio_o = Signal()
        self.swdio_z = Signal()

    def elaborate(self, platform):
        m = Module()
        m.submodules.swclk = swclk_buffer = io.Buffer("o", self.ports.swclk)
        m.submodules.swdio = swdio_buffer = io.Buffer("io", self.ports.swdio)

        m.d.comb += [
            swclk_buffer.o.eq(self.swclk),
            swdio_buffer.oe.eq(~self.swdio_z),
            swdio_buffer.o.eq(self.swdio_o)
        ]
        m.submodules += [
            FFSynchronizer(swdio_buffer.i, self.swdio_i),
        ]
        return m


class BlackmagicSubtarget(Elaboratable):
    def __init__(self, ports, out_fifo, in_fifo, period_cyc, us_cyc):
        self.ports      = ports
        self.out_fifo   = out_fifo
        self.in_fifo    = in_fifo
        self.period_cyc = period_cyc
        self.us_cyc     = us_cyc
        self.srst_z     = Signal(init=0)
        self.srst_o     = Signal(init=0)

    def elaborate(self, platform):
        m = Module()

        out_fifo = self.out_fifo
        in_fifo  = self.in_fifo

        m.submodules.bus = bus = SWDProbeBus(self.ports)
        m.d.comb += [
            self.srst_z.eq(0),
        ]
        if self.ports.srst is not None:
            m.submodules.srst_buffer = srst_buffer = io.Buffer("o")
            m.d.sync += [
                srst_buffer.oe.eq(~self.srst_z),
                srst_buffer.o.eq(~self.srst_o)
            ]

        blink = Signal()
        try:
            m.submodules.io_blink = io_blink = io.Buffer("o", platform.request("led", dir="-"))
            m.d.comb += io_blink.o.eq(blink)

            m.submodules.off = off = io.Buffer("o", platform.request("led", 1, dir="-"))
            m.submodules.off2 = off2 = io.Buffer("o", platform.request("led", 2, dir="-"))
            m.submodules.off3 = off3 = io.Buffer("o", platform.request("led", 3, dir="-"))
            m.submodules.off4 = off4 = io.Buffer("o", platform.request("led", 4, dir="-"))

            m.d.comb += off.o.eq(0)
            m.d.comb += off2.o.eq(0)
            m.d.comb += off3.o.eq(0)
            m.d.comb += off4.o.eq(0)
        except:
            pass

        timer = Signal(range(max(self.period_cyc, 1000 * self.us_cyc)))
        with m.If(timer != 0):
            m.d.sync += timer.eq(timer - 1)
        with m.Else():
            with m.If(out_fifo.r_rdy):
                m.d.comb += out_fifo.r_en.eq(1)
                with m.Switch(out_fifo.r_data):
                    # remote_bitbang_swdio_drive(int is_output)
                    with m.Case(*b"Oo"):
                        m.d.sync += bus.swdio_z.eq(out_fifo.r_data[5])
                    # remote_bitbang_swdio_read()
                    with m.Case(*b"c"):
                        m.d.comb += out_fifo.r_en.eq(in_fifo.w_rdy)
                        m.d.comb += in_fifo.w_en.eq(1)
                        m.d.comb += in_fifo.w_data.eq(b"0"[0] | Cat(bus.swdio_i))
                    # write swclk
                    with m.Case(*b"de"):
                        m.d.sync += bus.swclk.eq(out_fifo.r_data[0])
                        #m.d.sync += timer.eq(self.period_cyc - 1)
                    # write swdio
                    with m.Case(*b"fg"):
                        m.d.sync += bus.swdio_o.eq(out_fifo.r_data[0])
                    # remote_bitbang_reset(int trst, int srst)
                    with m.Case(*b"rs"):
                        m.d.sync += self.srst_o.eq(out_fifo.r_data - ord(b"r"))
                    # remote_bitbang_blink(int on)
                    with m.Case(*b"Bb"):
                        m.d.sync += blink.eq(~out_fifo.r_data[5])
                    # new extension: wait a cycle
                    with m.Case(*b"w"):
                        m.d.sync += timer.eq(self.period_cyc - 1)
                    with m.Default():
                        # Hang if an unknown command is received.
                        m.d.comb += out_fifo.r_en.eq(0)

        return m

SWDIO_READ = b"c"
SWCLK_LOW = b"d"
SWCLK_HIGH = b"e"
SWDIO_LOW = b"f"
SWDIO_HIGH = b"g"
WAIT = b"w"
SWDIO_FLOAT = b"o"
SWDIO_DRIVE = b"O"


class SWDInterface:
    def __init__(self, lower):
        self.lower = lower
        self.line_dir = False
        self.nrst = True

    async def turnaround(self, newdir):
        if self.line_dir == newdir:
            return
        self.line_dir = newdir

        if not newdir:
            await self.lower.write(b"".join([SWDIO_FLOAT, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))
        else:
            await self.lower.write(b"".join([WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW, SWDIO_DRIVE]))

    async def get_nrst(self):
        return self.nrst

    # Invert this here. state = True means is in reset.
    async def set_nrst(self, state):
        if state:
            self.nrst = True
            await self.lower.write(b"r")
        else:
            self.nrst = False
            await self.lower.write(b"s")

    async def set_led(self, state):
        if state:
            await self.lower.write(b"B")
        else:
            await self.lower.write(b"b")
    
    async def swd_out(self, num_clocks, data, parity=False):
        await self.turnaround(True)

        for i in range(0, num_clocks):
            if data & (1<<i):
                await self.lower.write(b"".join([SWDIO_HIGH, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))
            else:
                await self.lower.write(b"".join([SWDIO_LOW, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))

        # Write parity bit
        if parity:
            if (data.bit_count() & 1) == 1:
                await self.lower.write(b"".join([SWDIO_HIGH, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))
            else:
                await self.lower.write(b"".join([SWDIO_LOW, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))

        await self.lower.flush()

    async def swd_in(self, num_clocks, parity=False):
        # SWD: in %02x clocks
        await self.turnaround(False)

        num_clocks_with_parity = num_clocks
        if parity:
            num_clocks_with_parity = num_clocks + 1

        await self.lower.write(b"".join([WAIT, SWDIO_READ, SWCLK_HIGH, WAIT, SWCLK_LOW]) * num_clocks_with_parity)

        data_u32 = 0
        data = await self.lower.read(num_clocks_with_parity)

        # TODO: i am pretty sure this is just a bit reverse
        for i in range(0, num_clocks):
            data_u32 >>= 1
            if data[i] == b"1"[0]:
                data_u32 |= (1<<31)
        data_u32 >>= (32-num_clocks)

        if parity:
            await self.turnaround(True)
            if (data_u32.bit_count() & 1) != (data[num_clocks] & 1):
                # Parity error
                return (True, data_u32)
        return (False, data_u32)

class SWDBlackmagicRemote(BlackmagicRemote):
    def __init__(self, iface: SWDInterface):
        self.iface = iface
        super().__init__()

    def get_current_frequency(self):
        pass

    def set_current_frequency(self, freq):
        pass

    async def get_nrst(self):
        return await self.iface.get_nrst()

    async def set_nrst(self, state):
        await self.iface.set_nrst(state)

    async def set_led(self, state):
        await self.iface.set_led(state)
    
    async def swd_turnaround(self, direction):
        await self.iface.turnaround(direction)
    
    async def swd_out(self, num_clocks, data, use_parity):
        return await self.iface.swd_out(num_clocks, data, use_parity)

    async def swd_in(self, num_clocks, use_parity):
        return await self.iface.swd_in(num_clocks, use_parity)

    async def jtag_reset(self):
        raise NotImplementedError()
    
    async def jtag_shift_tms(self, tms_states, clock_cycles):
        raise NotImplementedError()
    
    async def jtag_tdi_tdo_seq(self, clock_cycles, data, last):
        raise NotImplementedError()
    
    async def jtag_next_bit(self, tms_state, tdi_state):
        raise NotImplementedError()

class SWDBlackmagicApplet(GlasgowApplet):
    logger = logging.getLogger(__name__)
    help = "expose bitbang interface for BMP"
    description = """
    Exposes a PTY for use with Black Magic Debug
    """

    def __init__(self):
        self.frequency = None

    @classmethod
    def add_build_arguments(cls, parser, access):
        super().add_build_arguments(parser, access)

        access.add_pin_argument(parser, "swdio", default=True)
        access.add_pin_argument(parser, "swclk", default=True)

        access.add_pin_argument(parser, "srst")

        parser.add_argument(
            "-f", "--frequency", metavar="FREQ", type=int, default=100,
            help="set SWCLK frequency to FREQ kHz (default: %(default)s)")

    def build(self, target, args):
        self.frequency = args.frequency * 1000

        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        iface.add_subtarget(BlackmagicSubtarget(
            ports=iface.get_port_group(
                swclk = args.pin_swclk,
                swdio = args.pin_swdio,
                srst  = args.pin_srst
            ),
            out_fifo=iface.get_out_fifo(),
            in_fifo=iface.get_in_fifo(),
            period_cyc=int(target.sys_clk_freq // (args.frequency * 1000)),
            us_cyc=int(target.sys_clk_freq // 1_000_000),
        ))

    async def run(self, device, args):
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
        swd_iface = SWDInterface(iface)
        return swd_iface

    @classmethod
    def add_interact_arguments(cls, parser):
        pass

    async def interact(self, device, args, iface: SWDInterface):
        master, slave = pty.openpty()
        print(os.ttyname(slave))

        remote = SWDBlackmagicRemote(iface)
        await remote.run(pty=master)

    @classmethod
    def tests(cls):
        from . import test
        return test.BlackmagicAppletTestCase