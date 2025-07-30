from artiq.language.core import kernel
from artiq.language.units import MHz, kHz, dB, GHz
from artiq.experiment import *
from numpy import int32, int64
from artiq.coredevice.dac34h84 import DAC34H84


class Phaser_Kernel(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("led0")
        self.setattr_device("led1")
        self.setattr_device("phaser0")
        self.phaser = self.phaser0

        assert DAC34H84.interpolation == 1 # 2x

    @kernel
    def run(self):
        self.core.reset()
        duc = 20 * MHz
        osc = [i * 1 * MHz for i in range(5)]

        self.phaser.init()
        self.phaser.channel[0].set_duc_frequency(duc)
        self.phaser.channel[0].set_duc_cfg()
        self.phaser.channel[0].set_att(0 * dB)
        self.phaser.channel[1].set_duc_frequency(-duc)
        self.phaser.channel[1].set_duc_cfg()
        self.phaser.channel[1].set_att(0 * dB)
        self.phaser.duc_stb()
        delay(1 * ms)
        for i in range(len(osc)):
            self.phaser.channel[0].oscillator[i].set_frequency(osc[i])
            self.phaser.channel[0].oscillator[i].set_amplitude_phase(1.0 / len(osc))
            self.phaser.channel[1].oscillator[i].set_frequency(-osc[i])
            self.phaser.channel[1].oscillator[i].set_amplitude_phase(1.0 / len(osc))
            delay(1 * ms)
