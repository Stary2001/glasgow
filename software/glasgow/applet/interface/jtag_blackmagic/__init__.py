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

from ....protocol.blackmagic_debug import BlackmagicRemote
from ..jtag_probe import JTAGProbeInterface, JTAGProbeSubtarget, JTAGState

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

class JTAGBlackmagicRemote(BlackmagicRemote):
    def __init__(self, iface: JTAGProbeInterface):
        self.iface = iface
        super().__init__()

    def get_current_frequency(self):
        pass

    def set_current_frequency(self, freq):
        pass

    async def set_led(self, state):
        pass
    
    async def swd_turnaround(self, direction):
        raise NotImplementedError()
    
    async def swd_out(self, num_clocks, data, use_parity):
        raise NotImplementedError()

    async def swd_in(self, num_clocks, use_parity):
        raise NotImplementedError()

    async def jtag_reset(self):
        await self.iface.enter_test_logic_reset()
    
    async def jtag_shift_tms(self, tms_states, clock_cycles):
        # This is silly, but the JTAG applet checks its state...

        # Going into IDLE normally.
        if clock_cycles == 2 and tms_states == 0x01:
            await self.iface.enter_run_test_idle()
        elif clock_cycles >= 3 and tms_states == 0x01:
            # RISC-V support requires waiting in run-test/idle, but the bits are the same
            # for exiting EXIT-DR as entering SHIFT-DR from IDLE. Use the state to disambiguate these.
            if self.iface._state == JTAGState.DREXIT1:
                await self.iface.run_test_idle(clock_cycles - 2)
            else:
                await self.iface.enter_shift_dr()
        elif clock_cycles == 4 and tms_states == 0x03:
            await self.iface.enter_shift_ir()
        else:
            raise NotImplementedError("Unimplemented TMS transition!")
    
    async def jtag_tdi_tdo_seq(self, clock_cycles, data, last):
        tdi_bits = []
        for i in range(clock_cycles):
            if data & (1<<i):
                tdi_bits.append(1)
            else:
                tdi_bits.append(0)
        return await self.iface.shift_tdio(tdi_bits, last=last)

    async def jtag_next_bit(self, tms_state, tdi_state):
        return await self.iface.shift_tdio((int(tdi_state),), last=tms_state)

class JTAGBlackmagicApplet(GlasgowApplet):
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

        access.add_pin_argument(parser, "tms", default=True)
        access.add_pin_argument(parser, "tck", default=True)
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
        iface.add_subtarget(JTAGProbeSubtarget(
            ports=iface.get_port_group(
                tck = args.pin_tck,
                tms = args.pin_tms,
                tdi = args.pin_tdi,
                tdo = args.pin_tdo,
                trst  = args.pin_trst,
                srst  = args.pin_srst
            ),
            out_fifo=iface.get_out_fifo(),
            in_fifo=iface.get_in_fifo(),
            period_cyc=int(target.sys_clk_freq // (args.frequency * 1000)),
        ))

    async def run(self, device, args):
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
        jtag_iface = JTAGProbeInterface(iface, self.logger, has_trst=args.pin_trst is not None)
        return jtag_iface

    @classmethod
    def add_interact_arguments(cls, parser):
        pass

    async def interact(self, device, args, iface: JTAGProbeInterface):
        master, slave = pty.openpty()
        print(os.ttyname(slave))

        remote = JTAGBlackmagicRemote(iface)
        await remote.run(pty=master)

    @classmethod
    def tests(cls):
        from . import test
        return test.BlackmagicAppletTestCase