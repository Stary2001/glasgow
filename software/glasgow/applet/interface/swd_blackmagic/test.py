from ... import *
from . import SWDBlackmagicApplet
import asyncio

class BlackmagicAppletTestCase(GlasgowAppletTestCase, applet=SWDBlackmagicApplet):
    @synthesis_test
    def test_build(self):
        self.assertBuilds()

    """@applet_simulation_test("aaaa")
    async def test_loopback(self):
        iface = await self.run_simulated_applet()
        await iface.swd_out(32, 0xaa55aa55, False)

        await iface.swd_in(3)
        self.assertTrue(False)

    def aaaa(self):
        self.build_simulated_applet()"""

    async def setup_samd21_scan(self, mode):
        iface = await self.run_hardware_applet(mode)
        if mode == "record":
            # etc
            pass
        return iface

    @applet_hardware_test(args=["-M", "--port", "A"], setup="setup_samd21_scan")
    async def do_samd21_scan(self, iface):
        print(iface)

        # Line reset
        await iface.swd_out(32, 0xffffffff, False)
        await iface.swd_out(32, 0x0fffffff, False)
        await iface.lower.flush()

        # Read DPIDR
        await iface.swd_out(8, 0xa5)
        await iface.lower.flush()

        await asyncio.sleep(1)
        _, ack_value = await iface.swd_in(3)
        await iface.lower.flush()
        print("ack: ", ack_value)

        parity_error, dpidr_value = await iface.swd_in(32, True)
        print("dpidr: ", hex(dpidr_value))

        await iface.swd_out(8, 0, False)
        await iface.lower.flush()

        self.assertFalse(parity_error)

        self.assertTrue(dpidr_value == 0x0bc11477)