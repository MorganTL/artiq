from numpy import int32, int64

from artiq.language.core import *
from artiq.language.types import *
from artiq.coredevice.rtio import rtio_output, rtio_input_data
from artiq.coredevice import spi2 as spi
from artiq.language.units import ms, us, ns

from artiq.coredevice.dac34h84 import DAC34H84
from artiq.coredevice.trf372017 import TRF372017

PHASER_BOARD_ID = 19
PHASER_HW_REV_VARIANT = 1 << 4

PHASER_GW_DRTIO = 3


# TODO: add docs
# TODO: add get/set in pairs
class Phaser:
    kernel_invariants = {"core", "channel", "target_read"}

    def __init__(
        self, dmgr, channel, clk_sel=0, gw_rev=PHASER_GW_DRTIO, core_device="core"
    ):
        # TODO: deal with clk_sel
        self.channel = channel
        self.core = dmgr.get(core_device)
        self.target_read = 1 << 7

    @staticmethod
    def get_rtio_channels(channel_base, gw_rev=PHASER_GW_DRTIO, **kwargs):
        if gw_rev == PHASER_GW_DRTIO:
            return [(channel_base, "base")]
        raise ValueError("invalid gw_rev `{}` for Phaser DRTIO".format(gw_rev))

    @kernel
    def write(self, address, data):
        rtio_output((self.channel << 8) | address, data)

    @kernel
    def read(self, address):
        rtio_output((self.channel << 8) | address | self.target_read, 0)
        return rtio_input_data(self.channel)


class PhaserChannel:
    kernel_invariants = {"core", "channel", "target_read"}

    def __init__(self, dmgr, channel, core_device="core"):
        self.channel = channel
        self.core = dmgr.get(core_device)
        self.target_read = 1 << 7

    @staticmethod
    def get_rtio_channels(channel_base, gw_rev=PHASER_GW_DRTIO, **kwargs):
        return [(channel_base, "channel")]

    @kernel
    def write(self, address, data):
        rtio_output((self.channel << 8) | address, data)

    @kernel
    def read(self, address):
        rtio_output((self.channel << 8) | address | self.target_read, 0)
        return rtio_input_data(self.channel)

    @kernel
    def set_test_mode_en(self, en):
        data = 1 if en else 0
        self.write(0x00, data)

    @kernel
    def set_test_iq_word(self, i, q):
        c = int64(self.core.ref_multiplier)
        self.write(0x01, i)
        delay_mu(c)
        self.write(0x02, q)
        delay_mu(c)
