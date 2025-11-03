from artiq.language.core import kernel
from artiq.language.units import MHz, kHz, dB, GHz
from artiq.experiment import *
from numpy import int32, int64

[
    HW_VARIANT,
    GW_VARIANT,
    SAMPLE_PER_CYCLE,
    AVAILABLE_TONES,
    DAC_CTRL_ADDR,
    DAC_STATUS_ADDR,
    DAC_SOURCE_SEL_ADDR,
    DAC_TEST_WORD_0_I_ADDR,
    DAC_TEST_WORD_0_Q_ADDR,
    DAC_TEST_WORD_1_I_ADDR,
    DAC_TEST_WORD_1_Q_ADDR,
    ATT_RESET_N,
    TRF_PS,
    TRF_LOCK_DETECT,
] = range(14)


class Phaser_Kernel(EnvExperiment):
    kernel_invariants = {"write_delay", "use_external_lo"}

    def build(self):
        self.setattr_device("core")
        self.setattr_device("led0")
        self.setattr_device("led1")
        phaser_name = "phaser_drtio_mtdds0"
        for i in range(5):
            self.setattr_device(f"{phaser_name}_led{i}")
            setattr(self, f"phaser_led{i}", getattr(self, f"{phaser_name}_led{i}"))

        self.setattr_device(f"{phaser_name}_fpga")
        setattr(self, "phaser", getattr(self, f"{phaser_name}_fpga"))

        for ch in range(2):
            ch_name = f"{phaser_name}_channel{ch}"
            self.setattr_device(ch_name)
            setattr(self, f"phaser_ch{ch}", getattr(self, ch_name))

        self.write_delay = 40 * us

        self.use_external_lo = False
        self.phaser_channel = self.phaser_ch0


        self.freqs = [ (10 + i) * MHz for i in range(20)]

    @kernel
    def setup_phaser_mtdds_channel(self, channel, frequencies):
        self.core.break_realtime()
        channel.attenuator.set_att(6.0 * dB)

        f_len = len(frequencies)
        assert f_len <= channel.tones
        for n in range(f_len):
            # delay to prevent RTIO collision
            channel.ddss[n].set_frequency(frequencies[n])
            delay(200 * us)
            channel.ddss[n].set_amplitude(1.0 / f_len)
            delay(200 * us)
            channel.ddss[n].enable_phase_accumulator(True)
            delay(200 * us)

    
    @kernel
    def run(self):
        self.core.reset()

        self.phaser.init()
        self.phaser_ch0.init()
        self.phaser_ch1.init()

        self.setup_phaser_mtdds_channel(self.phaser_ch0, self.freqs)
        self.setup_phaser_mtdds_channel(self.phaser_ch1, self.freqs)
