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

        for i in range(8):
            self.setattr_device(f"ttl{i}")

        mirny_name = "mirny0"
        self.setattr_device(f"{mirny_name}_cpld")
        setattr(self, "mirny", getattr(self, f"{mirny_name}_cpld"))
        for ch in range(4):
            ch_name = f"{mirny_name}_ch{ch}"
            self.setattr_device(ch_name)
            setattr(self, f"mirny_ch{ch}", getattr(self, ch_name))

        self.write_delay = 100 * ms


        self.use_external_lo = False
        self.phaser_channel = self.phaser_ch0

    @kernel
    def run(self):
        self.core.reset()

        self.phaser.init()
        self.phaser_channel.init()

        dds_n = 1
        self.phaser_channel.ddss[dds_n].enable_phase_accumulator(False)
        delay(self.write_delay)
        self.phaser_channel.ddss[dds_n].set_frequency(-(10 + dds_n) * MHz)
        delay(self.write_delay)
        self.phaser_channel.ddss[dds_n].set_phase_offset(0.0)
        delay(self.write_delay)
        self.phaser_channel.ddss[dds_n].set_amplitude(1.0 / dds_n)
        delay(self.write_delay)
        self.phaser_channel.attenuator.set_att(0.0 * dB)
        delay(self.write_delay)
        self.phaser_channel.ddss[dds_n].enable_phase_accumulator(True)
        delay(self.write_delay)


        self.mirny.init()
        self.mirny_ch0.init()
        self.mirny_ch0.set_att(0.0 * dB)
        self.mirny_ch0.set_frequency(1000 * MHz)
        self.mirny_ch0.sw.on()
        self.phaser_channel.upconverter.enable_mixer_rf_output(True)
        # for i in range(1000, 1010, 10):
        #     self.phaser_channel.upconverter.enable_mixer_rf_output(False)
        #     delay(self.write_delay)

        #     self.phaser_channel.upconverter.set_mixer_frequency(i * MHz)
        #     delay(self.write_delay)
        #     self.phaser_channel.upconverter.calibrate_vco()
        #     delay(2000 * us)  # TODO: cal delay
        #     if not self.phaser_channel.upconverter_pll_locked():
        #         raise ValueError("TRF372017 PLL fails to lock")
        #     delay(10.0 * us)


        #     self.phaser_channel.upconverter.enable_mixer_rf_output(True)
        #     delay(self.write_delay)
        #     delay(10 * ms)
