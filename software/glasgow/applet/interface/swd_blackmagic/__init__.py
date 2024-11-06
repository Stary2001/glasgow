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

CMD_MASK       = 0b0000_1111
CMD_TURNAROUND = 0
FLAGS_MASK       = 0b1111_0000
TURNAROUND_IN = 0 << 4
TURNAROUND_OUT = 1 << 4

CMD_SWD_IN = 1
CMD_SWD_OUT  = 2
CMD_SWD_GPIO  = 3

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

        cmd_plus_flags = Signal(8)
        cmd_only = Signal(4)
        flags_only = Signal(8)
        m.d.comb += cmd_only.eq(cmd_plus_flags & CMD_MASK)
        m.d.comb += flags_only.eq(cmd_plus_flags & FLAGS_MASK)

        in_bit = Signal()
        seq_length = Signal(8)
        seq_length_counter = Signal(8)
        seq_data = Signal(32)

        seq_counter = Signal(range(5))

        timer = Signal(range(max(self.period_cyc, 1000 * self.us_cyc)))

        with m.FSM() as fsm:
            with m.State("RECV-COMMAND"):
                m.d.comb += in_fifo.flush.eq(1)
                with m.If(out_fifo.r_rdy):
                    m.d.comb += out_fifo.r_en.eq(1)
                    m.d.sync += cmd_plus_flags.eq(out_fifo.r_data)
                    m.d.sync += seq_counter.eq(0)

                    m.next = "COMMAND"

            with m.State("COMMAND"):
                with m.If(cmd_only == CMD_TURNAROUND):
                    m.next = "TURNAROUND"
                with m.Elif((cmd_only == CMD_SWD_IN) | (cmd_only == CMD_SWD_OUT)):
                    m.next = "RECV-LENGTH"
                with m.Elif(cmd_only == CMD_SWD_GPIO):
                    m.next = "SET-IO"
                with m.Else():
                    m.next = "RECV-COMMAND"

            with m.State("RECV-LENGTH"):
                with m.If(out_fifo.r_rdy):
                    m.d.comb += out_fifo.r_en.eq(1)

                    with m.If(cmd_only == CMD_SWD_OUT):
                        m.d.sync += bus.swdio_o.eq(0)

                        m.d.sync += seq_length.eq(out_fifo.r_data)
                        m.d.sync += seq_length_counter.eq(out_fifo.r_data)
                        m.next = "RECV-DATA"
                    with m.Elif(cmd_only == CMD_SWD_IN):
                        m.d.sync += seq_length.eq(out_fifo.r_data)
                        m.d.sync += seq_length_counter.eq(out_fifo.r_data)

                        m.d.sync += timer.eq(self.period_cyc)
                        m.next = "CLOCK-RISE"

            with m.State("RECV-DATA"):
                with m.If(out_fifo.r_rdy):
                    m.d.comb += out_fifo.r_en.eq(1)
                    m.d.sync += seq_data.bit_select(seq_counter * 8,8).eq(out_fifo.r_data)

                    m.d.sync += seq_counter.eq(seq_counter+1)
                    with m.If(seq_counter == 3):
                        m.next = "OUT-SEQ"

            with m.State("TURNAROUND"):
                with m.If(cmd_plus_flags == (CMD_TURNAROUND | TURNAROUND_IN)):
                    m.d.sync += bus.swdio_z.eq(1)

                m.d.sync += timer.eq(self.period_cyc)
                m.next = "CLOCK-RISE"

            with m.State("IN-SEQ"):
                m.d.sync += timer.eq(self.period_cyc)
                with m.If(seq_length_counter == 0):
                    # todo: fold adjust into this
                    m.d.sync += seq_data.eq((seq_data >> 1) | (in_bit << 31))
                    m.next = "IN-SEQ-ADJUST"
                with m.Else():
                    m.d.sync += seq_data.eq((seq_data >> 1) | (in_bit << 31))
                    m.next = "CLOCK-RISE"

            with m.State("IN-SEQ-ADJUST"):
                m.d.sync += seq_data.eq(seq_data >> (32-seq_length).as_unsigned())
                m.next = "IN-SEQ-UPLOAD"

            with m.State("IN-SEQ-UPLOAD"):
                with m.If(in_fifo.w_rdy):
                    m.d.comb += in_fifo.w_en.eq(1)
                    m.d.comb += in_fifo.w_data.eq(seq_data.bit_select(seq_counter*8, 8))
                    m.d.sync += seq_counter.eq(seq_counter+1)

                    with m.If(seq_counter == 3):
                        m.next = "RECV-COMMAND"

            with m.State("OUT-SEQ"):
                with m.If(seq_length_counter == 0):
                    m.next = "RECV-COMMAND"
                with m.Else():
                    m.d.sync += seq_data.eq(seq_data >> 1)
                    m.d.sync += bus.swdio_o.eq(seq_data & 1)
                    m.d.sync += seq_length_counter.eq(seq_length_counter - 1)

                    m.d.sync += timer.eq(self.period_cyc)
                    m.next = "CLOCK-RISE"

            with m.State("CLOCK-RISE"):
                with m.If(timer != 0):
                    m.d.sync += timer.eq(timer-1)
                with m.Else():
                    with m.If(cmd_only == CMD_SWD_IN):
                        m.d.sync += in_bit.eq(bus.swdio_i)
                    m.d.sync += bus.swclk.eq(1)
                    m.d.sync += timer.eq(self.period_cyc)
                    m.next = "CLOCK-FALL"

            with m.State("CLOCK-FALL"):
                with m.If(timer != 0):
                    m.d.sync += timer.eq(timer-1)
                with m.Else():
                    m.d.sync += bus.swclk.eq(0)
                    with m.If(cmd_plus_flags == (CMD_TURNAROUND | TURNAROUND_OUT)):
                        m.d.sync += bus.swdio_z.eq(0)
                        m.next = "RECV-COMMAND"
                    with m.Elif(cmd_only == CMD_SWD_IN):
                        m.d.sync += seq_length_counter.eq(seq_length_counter-1)
                        m.next = "IN-SEQ"
                    with m.Elif(cmd_only == CMD_SWD_OUT):
                        m.next = "OUT-SEQ"
                    with m.Else():
                        m.next = "RECV-COMMAND"

            with m.State("SET-IO"):
                # nrst, led?
                pass


        return m


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
            await self.lower.write([CMD_TURNAROUND | TURNAROUND_IN])
        else:
            await self.lower.write([CMD_TURNAROUND | TURNAROUND_OUT])

    async def get_nrst(self):
        return self.nrst

    # Invert this here. state = True means is in reset.
    async def set_nrst(self, state):
        if state:
            self.nrst = True
            #await self.lower.write(b"r")
        else:
            self.nrst = False
            #await self.lower.write(b"s")

    async def set_led(self, state):
        if state:
            pass
            #await self.lower.write(b"B")
        else:
            pass
            #await self.lower.write(b"b")
    
    async def swd_out(self, num_clocks, data, parity=False):
        await self.turnaround(True)
        await self.lower.write(struct.pack("<BBI", CMD_SWD_OUT, num_clocks, data))

        # parity isnt real, it can't hurt me
        # Write parity bit
        #if parity:
        #    if (data.bit_count() & 1) == 1:
        #        await self.lower.write(b"".join([SWDIO_HIGH, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))
        #    else:
        #        await self.lower.write(b"".join([SWDIO_LOW, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))

        #await self.lower.flush()

    async def swd_in(self, num_clocks, parity=False):
        # SWD: in %02x clocks
        await self.turnaround(False)
        await self.lower.write(struct.pack("<BB", CMD_SWD_IN, num_clocks))
        data_u32 = struct.unpack("<I", await self.lower.read(4))[0]

        if parity:
            await self.turnaround(True)
            #if (data_u32.bit_count() & 1) != (data[num_clocks] & 1):
            #    # Parity error
            #    return (True, data_u32)
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