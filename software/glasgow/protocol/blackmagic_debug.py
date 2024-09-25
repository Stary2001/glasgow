import os
import asyncio
from abc import ABCMeta, abstractmethod

from ..applet import GlasgowAppletError

__all__ = ["BlackmagicRemote"]

REMOTE_ERROR_UNRECOGNISED = b"1"
REMOTE_ERROR_WRONGLEN     = b"2"
REMOTE_ERROR_FAULT        = b"3"
REMOTE_ERROR_EXCEPTION    = b"4"

REMOTE_RESP_OK     = b'K'
REMOTE_RESP_PARERR = b'P'
REMOTE_RESP_ERR    = b'E'
REMOTE_RESP_NOTSUP = b'N'

REMOTE_ACCEL_ADIV5     = 1 << 0
REMOTE_ACCEL_CORTEX_AR = 1 << 1
REMOTE_ACCEL_RISCV     = 1 << 2
REMOTE_ACCEL_ADIV6     = 1 << 3

def consume_commands(s):
    start = None
    for i in range(0, len(s)):
        if s[i:i+1] == b"!":
            start = i+1
        elif s[i:i+1] == b"#" and start != None:
            yield s[start:i]
            start = None

class BlackmagicRemote(metaclass=ABCMeta):
    def __init__(self):
        self.current_frequency = 0

    @abstractmethod
    def get_current_frequency(self):
        pass

    @abstractmethod
    def set_current_frequency(self, freq):
        pass

    @abstractmethod
    async def get_nrst(self):
        pass

    @abstractmethod
    async def set_nrst(self, state):
        pass

    @abstractmethod
    async def set_led(self, state):
        pass
    
    @abstractmethod
    async def swd_turnaround(self, direction):
        pass
    
    @abstractmethod
    async def swd_out(self, num_clocks, data, parity):
        pass

    @abstractmethod
    async def swd_in(self, num_clocks, parity):
        pass

    @abstractmethod
    async def jtag_reset(self):
        pass
    
    @abstractmethod
    async def jtag_shift_tms(self, tms_states, clock_cycles):
        pass
    
    @abstractmethod
    async def jtag_tdi_tdo_seq(self, clock_cycles, data, last):
        pass
    
    @abstractmethod
    async def jtag_next_bit(self, tms_state, tdi_state):
        pass

    async def run(self, pty):
        def reply(resp, *args):
            os.write(pty, b"&" + resp + b"".join(args) + b"#")

        def reply_int(resp, code: int):
            reply(resp, f"{code:x}".encode("ascii"))

        while True:
            chunk = await asyncio.get_event_loop().run_in_executor(None, lambda: os.read(pty, 1024))

            for cmd in consume_commands(chunk):
                code = cmd[0:2]
                # General: start
                if code == b"GA":
                    await self.set_led(True)
                    # Return probe name
                    reply(REMOTE_RESP_OK, b"Glasgow")
                elif code == b"Gf":
                    # General: get clock frequency
                    # This is in little endian, because the firmware does
                    # remote_respond_buf(REMOTE_RESP_OK, (uint8_t *)&freq, 4);
                    reply(REMOTE_RESP_OK, self.current_frequency.to_bytes(4, 'little').hex().encode('ascii'))
                elif code == b"GF":
                    # General: set clock frequency
                    clock_freq = int(cmd[2:], 16)
                    print("TODO: Set frequency to ", clock_freq, "Hz")
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"GE":
                    # General: set clock OE
                    # TODO: always on for now
                    # either 0/1
                    print("TODO: Set clock OE")
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"Gp" or code == b"GP":
                    print("TODO: Set target power")
                    # General: set/get power switch
                    # Report not supported for power switch for now
                    reply(REMOTE_RESP_NOTSUP)
                elif code == b"GV":
                    # Return target voltage (as string)
                    reply(REMOTE_RESP_OK, b"at least 2")
                elif code == b"Gz":
                    # Return value of nRST
                    nrst_value = await self.get_nrst()
                    reply_int(REMOTE_RESP_OK, 1 if nrst_value else 0)
                elif code == b"GZ":
                    # Set nRST to next byte
                    nrst_value = int(cmd[2:3])
                    await self.set_nrst(nrst_value == 1)
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"HC":
                    # Highlevel: check
                    # return protocol version v4
                    reply_int(REMOTE_RESP_OK, 4)
                elif code == b"HA":
                    # Highlevel: what accelerations are available?
                    # Return no acceleration.
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"SS":
                    # SWD: init
                    await self.swd_turnaround(False)
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"So" or code == b"SO":
                    num_clocks = int(cmd[2:4], 16)
                    data = int(cmd[4:], 16)

                    await self.swd_out(num_clocks, data, use_parity=(code == b"SO"))
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"Si" or code == b"SI":
                    num_clocks = int(cmd[2:4], 16)
                    parity_error, data = await self.swd_in(num_clocks, use_parity=(code == b"SI"))
                    if parity_error:
                        reply_int(REMOTE_RESP_PARERR, data)
                    else:
                        reply_int(REMOTE_RESP_OK, data)
                elif code == b"JS":
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"JR":
                    # JTAG: Reset

                    await self.jtag_reset()
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"JT":
                    # JTAG: tms sequence

                    clock_cycles = int(cmd[2:4], 16)
                    tms_states = int(cmd[4:6], 16)
                    await self.jtag_shift_tms(tms_states, clock_cycles)
                    reply_int(REMOTE_RESP_OK, 0)
                elif code == b"JC":
                    # JTAG: clock
                    # this seems to be unused?
                    tms_state = cmd[2:2] != b"0"
                    tdi_state = cmd[3:3] != b"0"
                    clock_cycles = int(cmd[4:6], 16)
                    raise RuntimeError("we hope jtagtap_cycle is never called")
                elif code == b"JD" or code == b"Jd":
                    clock_cycles = int(cmd[2:4], 16)
                    data = int(cmd[4:], 16)

                    # JD = tms set
                    # Jd = tms not set
                    bits = await self.jtag_tdi_tdo_seq(clock_cycles, data, last=(code == b"JD"))
                    data = 0
                    for i, x in enumerate(bits):
                        data |= x << i
                    reply_int(REMOTE_RESP_OK, data)
                elif code == b"JN":
                    # JTAG: Next bit

                    result = await self.jtag_next_bit(cmd[2:3] == b'1', cmd[3:4] == b'1')
                    reply_int(REMOTE_RESP_OK, result[0])
                elif code == b"HJ":
                    # Ignore
                    reply_int(REMOTE_RESP_OK, 0)
                else:
                    print("Unknown", cmd)
                    reply(REMOTE_RESP_ERR, REMOTE_ERROR_UNRECOGNISED)
