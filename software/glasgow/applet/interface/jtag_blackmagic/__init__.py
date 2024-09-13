import struct
import logging
import asyncio
import os
import pty
import time
from amaranth import *
from amaranth.lib import io
from amaranth.lib.cdc import FFSynchronizer
from amaranth.lib.fifo import SyncFIFOBuffered

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

from ..jtag_probe import JTAGProbeAdapter, JTAGProbeDriver, JTAGProbeInterface

class BlackmagicSubtarget(Elaboratable):
    def __init__(self, ports, out_fifo, in_fifo, period_cyc, us_cyc):
        self.ports = ports
        self.out_fifo   = out_fifo
        self.in_fifo    = in_fifo
        self.period_cyc = period_cyc
        self.us_cyc     = us_cyc

        self.srst_z     = Signal(init=0)
        self.srst_o     = Signal(init=0)
        self.swd_select = Signal(init=0)
    
    def elaborate(self, platform):
        m = Module()
        
        m.submodules.swd_in_fifo = swd_in_fifo = InFifoReplacement(width=8,depth=16)
        swd_in_read_stream = swd_in_fifo.r_stream
        # swd_in_fifo is written by subtarget

        m.submodules.swd_out_fifo = swd_out_fifo = SyncFIFOBuffered(width=8,depth=16)
        swd_out_write_stream = swd_out_fifo.w_stream
        # swd_out_fifo is read by subtarget

        m.submodules.jtag_in_fifo = jtag_in_fifo = InFifoReplacement(width=8,depth=16)
        jtag_in_read_stream = jtag_in_fifo.r_stream
        m.submodules.jtag_out_fifo = jtag_out_fifo = SyncFIFOBuffered(width=8,depth=16)
        jtag_out_write_stream = jtag_out_fifo.w_stream

        m.submodules.bus = bus = JTAGSWDProbeBus(self.ports)
        
        m.submodules.swd = SWDSubtarget(bus, swd_out_fifo, swd_in_fifo, self.period_cyc, self.us_cyc)
        m.submodules.jtag_adapter = JTAGProbeAdapter(m.submodules.bus, self.period_cyc)
        m.submodules.jtag_driver  = JTAGProbeDriver(m.submodules.jtag_adapter, jtag_out_fifo, jtag_in_fifo)

        m.d.comb += [
            bus.swd_select.eq(self.swd_select)
        ]

        # In fifo: FROM applet TO host
        # Out fifo: FROM host TO glasgow
            # Read from out fifo
            # Write to in fifo
        
        usb_in_write_stream = self.in_fifo.stream
        usb_out_read_stream = self.out_fifo.stream

        with m.If(self.swd_select):
            m.d.comb += [
                swd_in_read_stream.ready.eq(usb_in_write_stream.ready),
                usb_in_write_stream.payload.eq(swd_in_read_stream.payload),
                usb_in_write_stream.valid.eq(swd_in_read_stream.valid),
                
                self.in_fifo.flush.eq(swd_in_fifo.flush),

                usb_out_read_stream.ready.eq(swd_out_write_stream.ready),
                swd_out_write_stream.payload.eq(usb_out_read_stream.payload),
                swd_out_write_stream.valid.eq(usb_out_read_stream.valid),
            ]
        with m.Else():
             m.d.comb += [
                swd_in_read_stream.ready.eq(usb_in_write_stream.ready),
                usb_in_write_stream.payload.eq(swd_in_read_stream.payload),
                usb_in_write_stream.valid.eq(swd_in_read_stream.valid),
                
                self.in_fifo.flush.eq(jtag_in_fifo.flush),

                usb_out_read_stream.ready.eq(swd_out_write_stream.ready),
                swd_out_write_stream.payload.eq(usb_out_read_stream.payload),
                swd_out_write_stream.valid.eq(usb_out_read_stream.valid),
            ]
        
        return m

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


class BlackmagicInterface:
    # todo
    def __init__(self, swd_iface, jtag_iface):
        self.swd = swd_iface
        self.jtag = jtag_iface

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

        access.add_pin_argument(parser, "swdio_tms", default=True)
        access.add_pin_argument(parser, "swclk", default=True)
        access.add_pin_argument(parser, "tdi", default=True)
        access.add_pin_argument(parser, "tdo", default=True)

        access.add_pin_argument(parser, "trst")
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
                swdio_tms = args.pin_swdio_tms,
                tdi = args.pin_tdi,
                tdo = args.pin_tdo,
                srst  = args.pin_srst
            ),
            out_fifo=iface.get_out_fifo(),
            in_fifo=iface.get_in_fifo(),
            period_cyc=int(target.sys_clk_freq // (args.frequency * 1000)),
            us_cyc=int(target.sys_clk_freq // 1_000_000),
        ))

    async def run(self, device, args):
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
        jtag_iface = JTAGProbeInterface(iface, self.logger, has_trst=args.pin_trst is not None)
        swd_iface = SWDInterface(iface)
        bmp_iface = BlackmagicInterface(swd_iface, jtag_iface)
        return bmp_iface

    @classmethod
    def add_interact_arguments(cls, parser):
        pass

    async def interact(self, device, args, iface: BlackmagicInterface):
        master, slave = pty.openpty()
        print(os.ttyname(slave))

        def reply(resp, *args):
            os.write(master, b"&" + resp + b"".join(args) + b"#")

        def reply_int(resp, code: int):
            reply(resp, f"{code:x}".encode("ascii"))

        """while True:
            print("True")
            await iface.swd.led(True)
            time.sleep(1)
            print("False")
            await iface.swd.led(False)
            time.sleep(1)"""

        while True:
            chunk = await asyncio.get_event_loop().run_in_executor(None, lambda: os.read(master, 1024))

            for cmd in consume_commands(chunk):
                code = cmd[0:2]
                # General: start
                if code == b"GA":
                    await iface.swd.led(True)
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
                    os.write(master, b"&K0#")
                elif code == b"JR":
                    # JTAG: Reset
                    # jtagtap_reset()
                    #self.jtag.jtagtap_tms_seq(0x1f, 6)
                    pass
                    os.write(master, b"&K0#")
                elif code == b"JT":
                    # JTAG: tms sequence
                    # obvious: jtagprobe CMD_SHIFT_TMS
                    clock_cycles = int(cmd[2:4], 16)
                    tms_states = int(cmd[4:6], 16)
                    # lol shift_tms wants bits. how do
                    print(bin(tms_states), clock_cycles)
                    iface.jtag.shift_tms(tms_states, clock_cycles)
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
                    await iface.swd.turnaround(False)
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"So" or code == b"SO":
                    num_clocks = int(cmd[2:4], 16)
                    data = int(cmd[4:], 16)

                    await iface.swd.swd_out(num_clocks, data, parity=(code == b"SO"))
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"Si" or code == b"SI":
                    num_clocks = int(cmd[2:4], 16)
                    parity_error, data = await iface.swd.swd_in(num_clocks, parity=(code == b"SI"))
                    if parity_error:
                        reply(REMOTE_RESP_PARERR, hex(data)[2:].encode("utf-8"))
                    else:
                        reply(REMOTE_RESP_OK, hex(data)[2:].encode("utf-8"))

                else:
                    print("Unknown", cmd)
                    reply(REMOTE_RESP_ERR, REMOTE_ERROR_UNRECOGNISED)

    @classmethod
    def tests(cls):
        from . import test
        return test.BlackmagicAppletTestCase