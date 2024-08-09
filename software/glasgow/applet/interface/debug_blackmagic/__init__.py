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

def consume_commands(s):
    start = None
    for i in range(0, len(s)):
        if s[i:i+1] == b"!":
            start = i+1
        elif s[i:i+1] == b"#" and start != None:
            yield s[start:i]
            start = None

# Will eventually live in `..swd_probe`.
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
                    with m.Case(*b"rstu"):
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

REMOTE_ERROR_UNRECOGNISED = b"1"
REMOTE_ERROR_WRONGLEN     = b"2"
REMOTE_ERROR_FAULT        = b"3"
REMOTE_ERROR_EXCEPTION    = b"4"

REMOTE_RESP_OK     = b'K'
REMOTE_RESP_PARERR = b'P'
REMOTE_RESP_ERR    = b'E'
REMOTE_RESP_NOTSUP = b'N'

"""

/* generic soft reset: 1, 1, 1, 1, 1, 0 */
#define jtagtap_soft_reset() jtag_proc.jtagtap_tms_seq(0x1fU, 6)

/* Goto Shift-IR: 1, 1, 0, 0 */
#define jtagtap_shift_ir() jtag_proc.jtagtap_tms_seq(0x03U, 4)

/* Goto Shift-DR: 1, 0, 0 */
#define jtagtap_shift_dr() jtag_proc.jtagtap_tms_seq(0x01U, 3)

/* Goto Run-test/Idle: 1, 1, 0 */
#define jtagtap_return_idle(cycles) jtag_proc.jtagtap_tms_seq(0x01, (cycles) + 1U)
# """

class BlackmagicApplet(GlasgowApplet):
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

        access.add_pin_argument(parser, "swclk", default=True)
        access.add_pin_argument(parser, "swdio", default=True)

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
        return await device.demultiplexer.claim_interface(self, self.mux_interface, args)

    @classmethod
    def add_interact_arguments(cls, parser):
        pass

    async def interact(self, device, args, iface):
        master, slave = pty.openpty()
        print(os.ttyname(slave))

        line_dir = False
        async def turnaround(newdir):
            nonlocal line_dir
            if line_dir == newdir:
                return
            line_dir = newdir

            if not newdir:
                await iface.write(b"".join([SWDIO_FLOAT, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))
            else:
                await iface.write(b"".join([WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW, SWDIO_DRIVE]))

        def reply(resp, *args):
            os.write(master, b"&" + resp + b"".join(args) + b"#")

        def reply_int(resp, code: int):
            reply(resp, f"{code:x}".encode("ascii"))

        while True:
            chunk = await asyncio.get_event_loop().run_in_executor(None, lambda: os.read(master, 1024))

            for cmd in consume_commands(chunk):
                code = cmd[0:2]
                # General: start
                if code == b"GA":
                    await iface.write(b"B")
                    # Return probe name
                    reply(REMOTE_RESP_OK, b"Glasgow")
                elif code == b"Gf":
                    # General: get clock frequency
                    # This is in little endian, because the firmware does
                    # remote_respond_buf(REMOTE_RESP_OK, (uint8_t *)&freq, 4);
                    reply(REMOTE_RESP_OK, self.frequency.to_bytes(4, 'little').hex().encode('ascii'))
                elif code == b"GF":
                    # General: set clock frequency
                    clock_freq = int(cmd[2:], 16)
                    print("TODO: Set frequency to ", clock_freq, "Hz")
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"GE":
                    # General: set clock OE
                    # TODO: always on for now
                    # either 0/1
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"Gp" or code == b"GP":
                    # General: set/get power switch
                    # Report not supported for power switch for now
                    reply(REMOTE_RESP_NOTSUP)
                elif code == b"GV":
                    # Return target voltage (as string)
                    reply(REMOTE_RESP_OK, b"at least 2")
                elif code == b"Gz":
                    # Return value of nRST
                    reply_int(REMOTE_RESP_OK, 1)
                elif code == b"GZ":
                    # Set nRST to next byte
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"HC":
                    # Highlevel: check
                    # return protocol version v4
                    os.write(master, b"&K4#")
                elif code == b"HA":
                    # Highlevel: what accelerations are available?
                    # Return no acceleration.
                    os.write(master, b"&K0#")
                elif code == b"JS":
                    # JTAG: Start
                    # jtagtap_init()
                    """	/* Ensure we're in JTAG mode */
                    for (size_t i = 0; i <= 50U; ++i)
                        jtagtap_next(true, false); /* 50 + 1 idle cycles for SWD reset */
                    jtagtap_tms_seq(0xe73cU, 16U); /* SWD to JTAG sequence */"""
                    pass
                elif code == b"JR":
                    # JTAG: Reset
                    # jtagtap_reset()
                    jtagtap_tms_seq(0x1f, 6)
                elif code == b"JT":
                    # JTAG: tms sequence
                    # obvious: jtagprobe CMD_SHIFT_TMS
                    clock_cycles = int(cmd[2:4], 16)
                    tms_states = int(cmd[4:6], 16)
                    # jtagtap_tms_seq(tms_states, clock_cycles)
                    pass
                elif code == b"JC":
                    # JTAG: clock
                    # this seems to be unused?
                    tms_state = cmd[2:2] != b"0"
                    tdi_state = cmd[3:3] != b"0"
                    clock_cycles = int(cmd[4:6], 16)
                    raise RuntimeError("we hope jtagtap_cycle is never called")
                    pass
                elif code == b"JD" or code == b"Jd":
                    # JTAG: REMOTE_TDITDO_TMS or REMOTE_TDITDO_NOTMS
                    # jtag applet only has tms version?
                    # oh. no shit. just last=False
                    clock_cycles = int(cmd[2:4], 16)
                    data = int(cmd[4:], 16)
                    # jtagtap_tdi_tdo_seq()
                elif code == b"JN":
                    # JTAG: Next bit
                    # This seems to not exist, we need to hack it
                    # set tdi, tms, set clk high, read tdo, set clk low
                    # tdio(0,) or tdio(1,) with last=False/True (tms state)

                    # jtagtap_next(packet[2] == '1', packet[3] == '1');
                    pass
                elif code == b"SS":
                    # SWD: init
                    await turnaround(False)
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"So" or code == b"SO":
                    # SWD: out %02x clocks + %x data
                    await turnaround(True)

                    num_clocks = int(cmd[2:4], 16)
                    data = int(cmd[4:], 16)
                    for i in range(0, num_clocks):
                        if data & (1<<i):
                            await iface.write(b"".join([SWDIO_HIGH, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))
                        else:
                            await iface.write(b"".join([SWDIO_LOW, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))

                    # Write parity bit
                    if code == b"SO":
                        if (data.bit_count() & 1) == 1:
                            await iface.write(b"".join([SWDIO_HIGH, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))
                        else:
                            await iface.write(b"".join([SWDIO_LOW, WAIT, SWCLK_HIGH, WAIT, SWCLK_LOW]))

                    await iface.flush()
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"Si" or code == b"SI":
                    # SWD: in %02x clocks
                    await turnaround(False)

                    num_clocks = int(cmd[2:4], 16)
                    if code == b"Si":
                        num_clocks_with_parity = num_clocks
                    else:
                        num_clocks_with_parity = num_clocks + 1

                    await iface.write(b"".join([WAIT, SWDIO_READ, SWCLK_HIGH, WAIT, SWCLK_LOW]) * num_clocks_with_parity)

                    data_u32 = 0
                    data = await iface.read(num_clocks_with_parity)

                    # TODO: i am pretty sure this is just a bit reverse
                    for i in range(0, num_clocks):
                        data_u32 >>= 1
                        if data[i] == b"1"[0]:
                            data_u32 |= (1<<31)
                    data_u32 >>= (32-num_clocks)

                    if code == b"SI":
                        await turnaround(True)
                        if (data_u32.bit_count() & 1) != (data[num_clocks] & 1):
                            # Parity error
                            os.write(master, b"&"+REMOTE_RESP_PARERR+b"#")
                            continue

                    reply(REMOTE_RESP_OK, hex(data_u32)[2:].encode("utf-8"))
                else:
                    print("Unknown", cmd)
                    reply(REMOTE_RESP_ERR, REMOTE_ERROR_UNRECOGNISED)

    @classmethod
    def tests(cls):
        from . import test
        return test.BlackmagicAppletTestCase