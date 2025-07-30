from migen import *

from artiq.gateware.phaser.dds import PipelinedAdder
from artiq.gateware.phaser.register import *
from artiq.gateware.phaser.dac_phy import *

def repeat_interleave(a, b, out):
    assert len(out) % len(Cat(a, b)) == 0
    # try to fit interleaved din_0, din_1 into out by repeating each element if necessary
    repeats = len(out) // len(Cat(a, b))

    interleaved = []
    for first, second in zip(a, b):
        interleaved.extend([first for _ in range(repeats)])
        interleaved.extend([second for _ in range(repeats)])
    return out.eq(Cat(*interleaved))

class DUT(Module):
    def __init__(self, n_regs):
        self.regs = [Signal(16) for _ in range(n_regs)]
        self.submodules += PipelinedAdder(self.regs)

        n = 2
        self.din = Signal(8)

        dacclk = False
        if dacclk:
            self.comb += repeat_interleave(C((1 << n) - 1, n), C(0, n), self.din)
        else:
            self.comb += repeat_interleave(C(0b01, n), C(0, n), self.din)

        # from operator import add
        # freq = [Signal(10) for _ in range(10)]
        # self.submodules.adder = adder = PipelinedAdder(freq)
        # print(len(reduce(add , freq)))
        # print(len(adder.o))


class TestBench:
    def __init__(self):
        self.n_regs = 10
        self.dut = DUT(self.n_regs)
        self.fragment = self.dut.get_fragment()

    def setup(self):
        for i, r in enumerate(self.dut.regs):
            yield r.eq(i + 1)
        yield
        for i, r in enumerate(self.dut.regs):
            yield r.eq(i + 2)

        for _ in range(self.n_regs + 1):
            yield

        din = yield self.dut.din
        print(f"din = {bin(din)}")

    def run(self, gen):
        run_simulation(self.fragment, gen, vcd_name="testbench.vcd")


if __name__ == "__main__":
    tb = TestBench()
    tb.run(tb.setup())



from numpy import int32, int64

from artiq.coredevice.dac34h84 import DAC34H84
from artiq.coredevice import spi2 as spi
from artiq.coredevice.rtio import rtio_output, rtio_input_data
from artiq.coredevice.trf372017 import TRF372017
from artiq.language.core import *
from artiq.language.types import *
from artiq.language.units import us, GHz


# TODO:
# - add docs
# - add get/set in pairs
# - check all kernel_invariants
# - unify data/value
# - ensure read ops has no slack (let user do it themselves)
# - feature parity with og phaser
class Phaser:
    kernel_invariants = {"core", "channel", "target_read"}

    def __init__(self, dmgr, channel, core_device="core"):
        # TODO: deal with clk_sel
        self.channel = channel
        self.core = dmgr.get(core_device)
        # TODO: decide whether I should hard code this?
        self.target_read = 1 << 7

    @staticmethod
    def get_rtio_channels(channel_base, **kwargs):
        return [(channel_base, "base")]

    @kernel
    def init(self):
        pass
        # TODO: add dac iotest (need change dac sources)

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
        # TODO: decide whether I should hard code this?
        self.target_read = 1 << 7

    @staticmethod
    def get_rtio_channels(channel_base, **kwargs):
        return [(channel_base, "channel")]

    @kernel
    def write(self, address, data):
        rtio_output((self.channel << 8) | address, data)

    @kernel
    def read(self, address):
        rtio_output((self.channel << 8) | address | self.target_read, 0)
        return rtio_input_data(self.channel)


DAC_SPI_ADDR_WIDTH = 7
DAC_SPI_CMD_WIDTH = DAC_SPI_ADDR_WIDTH + 1
DAC_SPI_DATA_WIDTH = 16

DAC_SPI_DIV = 20  # min 100 ns for DAC SPI
DAC_SPI_DIV_TEMP = 200  # min 1 us when reading DAC temperature register
DAC_SPI_CONFIG = (
    0 * spi.SPI_OFFLINE
    | 0 * spi.SPI_END
    | 0 * spi.SPI_INPUT
    | 0 * spi.SPI_CS_POLARITY
    | 0 * spi.SPI_CLK_POLARITY
    | 0 * spi.SPI_CLK_PHASE
    | 0 * spi.SPI_LSB_FIRST
    | 0 * spi.SPI_HALF_DUPLEX
)


class DAC:
    """DAC DAC34H84 driver

    :param spi_device: SPI bus device name.
    :param fifo_offset: The DAC FIFO offset settings
    :param input_data_rate: Input data rate in sample per second (default: 250MSPS)
    :param core_device: Core device name (default: "core").
    """

    kernel_invariants = {
        "core",
        "bus",
        "fifo_offset",
        "f_dac",
        "init_mmap",
    }

    def __init__(
        self,
        dmgr,
        spi_device,
        fifo_offset=3,  # NOTE: 3 when 250MSPS
        input_data_rate=250e6,
        core_device="core",
    ):
        self.core = dmgr.get(core_device)
        self.bus = dmgr.get(spi_device)

        self.fifo_offset = fifo_offset
        self.f_dac = 1 * GHz
        settings = {
            # Target 1 GSPS for all channels, set VCO = 4 GHz and per-scaler = 4
            "pll_p": 0b100,
            "pll_vco": 0x3F,
            "syncsel_mixerab": 0b1000,  # register sync only via sif_sync write
            "syncsel_mixercd": 0b1000,  # register sync only via sif_sync write
        }
        # f_ostr must be f_daclk/(k*8*interpolation) where k is integer - SLAS751D Table 6.8
        # f_pdf = f_ostr when PLL is enable - SLAA584 Figure 28
        if input_data_rate == 500e6:
            # f_data = 500 MSPS (non-interleaved), 2x to reach 1 GSPS
            settings["interpolation"] = 1

            # f_ostr = f_pdf = 62.5 MHz when n divider is 2
            settings["pll_n"] = 0b0001
            # VCO @ 4 GHz when m divider is 16 and no need for m doubling
            settings["pll_m"] = 16
            settings["pll_m2"] = 0

        elif input_data_rate == 250e6:
            # f_data = 250 MSPS (non-interleaved), 4x to reach 1 GSPS
            settings["interpolation"] = 2

            # f_ostr = f_pdf = 31.25 MHz when n divider is 4
            settings["pll_n"] = 0b0011
            # VCO @ 4 GHz when m divider is 32 and no need for m doubling
            settings["pll_m"] = 32
            settings["pll_m2"] = 0

        else:
            raise ValueError("Invalid input data rate")

        self.init_mmap = DAC34H84(settings).get_mmap()

    @kernel
    def init(self, tune_fifo_offset_en=True):
        """Initialize the DAC

        Sets up SPI mode, confirms chip presence, configures the PLL, and sets up FIFO offset.

        .. note:: To establish deterministic latency between RTIO time base and DAC
            output, the DAC FIFO offset ``self.fifo_offset`` must be fixed between
            initalization. If ``tune_fifo_offset`` = ``True``, a value with maximum
            margin is determined automatically by `tune_fifo_offset`.

        :param tune_fifo_offset_en: Enable the DAC FIFO offset tuning (default: False)
        """
        # set sif4_enable to enter 4-wire SPI mode
        self.write(0x02, 0x0080)
        if self.read(0x7F) != 0x5409:
            raise ValueError("DAC34H84 version mismatch")
        delay(10 * us)
        if self.read(0x00) != 0x049C:
            raise ValueError("DAC34H84 resets fail")
        delay(10 * us)
        if (self.read(0x06, DAC_SPI_DIV_TEMP) >> 8) > 85:
            raise ValueError("DAC34H84 overheats")
        delay(10 * us)

        for data in self.init_mmap:
            self.write(data >> 16, data & 0xFFFF)

        cfg_0x18 = self.read(0x18)
        delay(10 * us)
        # toggle PLL reset
        self.write(0x18, cfg_0x18 & 0x400)
        delay(10 * us)
        self.write(0x18, cfg_0x18 & ~0x400)

        cfg_0x18 = self.read(0x18)
        delay(10 * us)
        # Use PLL loop filter voltage to check lock status - Table 10, Step 34 SLAS751D section 7.5.2.4
        if not (0x2 <= cfg_0x18 & 0b111 <= 0x5):
            raise ValueError("DAC34H84 PLL fail to lock")

        # Disable PLL N-dividers sync - Table 10, Step 41 SLAS751D section 7.5.2.4
        self.write(0x18, cfg_0x18 & ~0x0800)

        self.tune_fifo_offset()

        # TODO: impl https://github.com/m-labs/artiq/pull/1657
        # expose the necessary settings instead of "workaround" for broken init
        # avoid malformed output for: mixer_ena=1, nco_ena=0 after power up
        self.write(self.init_mmap[2] >> 16, self.init_mmap[2] | (1 << 4))
        delay(40 * us)
        self.sync()
        delay(100 * us)
        self.write(self.init_mmap[2] >> 16, self.init_mmap[2])
        delay(40 * us)
        self.sync()
        delay(100 * us)

