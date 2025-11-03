from artiq.language.core import kernel
from artiq.language.units import MHz, kHz, dB, GHz
from artiq.experiment import *
from numpy import int32, int64

from artiq.coredevice.phaser_drtio import *


@portable
def twos_comp(val):
    if val & (1 << 15) != 0:
        val = val - (1 << 16)
    return val


@portable
def mu_to_volt(gain, mu):
    return (4.096 * twos_comp(mu) / 0x7FFF) * (5 / 2) / (10 ** (gain))


@portable
def mu_to_volt_no_gain(mu):
    return 4.096 * twos_comp(mu) / 0x7FFF


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

        # Zotino voltage source
        zotino_name = "zotino0"
        self.setattr_device(zotino_name)
        setattr(self, "zotino", getattr(self, zotino_name))

        self.write_delay = 40 * us

        self.use_external_lo = False
        self.phaser_channel = self.phaser_ch0

        self.freqs = [(10 + i) * MHz for i in range(20)]

    @kernel
    def setup_phaser_mtdds_channel(self, channel, frequencies):
        channel.attenuator.set_att(6.0 * dB)
        delay(200 * us)

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
    def init(self, volts):
        self.core.reset()
        self.phaser.init()

        self.zotino.init()
        delay(200 * us)
        self.zotino.set_dac(volts)

    @kernel
    def get_adc_mu(self):
        self.core.break_realtime() # because of the RPC fn after this
        adc0_mu = self.phaser.get_adc_mu(0)
        delay(40.0 * us)
        adc1_mu = self.phaser.get_adc_mu(1)
        delay(40.0 * us)
        return adc0_mu, adc1_mu

    @kernel
    def get_adc_terms(self):
        self.core.break_realtime()
        return self.phaser.read(ADC_TERMS)

    @kernel
    def set_adc_gains(self, gain0, gain1):
        self.phaser.set_pgia(0, gain0)
        delay_mu(int64(self.core.ref_multiplier))
        self.phaser.set_pgia(1, gain1)
        delay_mu(int64(self.core.ref_multiplier))

    @rpc
    def print_adc_values(self, adc_mus, gain_mus):
        s = ""
        for i, (adc_mu, gain_mu) in enumerate(zip(adc_mus, gain_mus)):
            s += f"adc{i} {adc_mu} {twos_comp(adc_mu)} {mu_to_volt_no_gain(adc_mu):.4f}V {mu_to_volt(gain_mu, adc_mu):.4f}V |"
        print(s)

    @kernel
    def set_iir_coeff(self, servo, profile, b0, a1, b1):
        b0_mu, a1_mu, b1_mu = servo.cal_iir_coeff(b0, a1, b1)

        #non profile regs
        addr_offset = 4 + profile * 6
        servo.write(addr_offset + 0, b0_mu)
        delay_mu(int64(self.core.ref_multiplier))
        servo.write(addr_offset + 1, a1_mu)
        delay_mu(int64(self.core.ref_multiplier))
        servo.write(addr_offset + 2, b1_mu)
        delay_mu(int64(self.core.ref_multiplier))
        # servo.write(addr_offset + 3, offset_mu)
        # delay_mu(int64(self.core.ref_multiplier))

    @kernel
    def get_iir_x1y1(self, servo, profile):
        addr_offset = 4 + profile * 6

        x1 = servo.read(addr_offset + 4)
        delay(40.0 * us)
        y1 = servo.get_y1_mu(profile)
        delay(40.0 * us)
        return x1, y1

    @kernel
    def run(self):
        self.init([0.5, 0.5]) # NOTE: Zotino has 470 Ohm output impedance
        self.set_adc_gains(1, 10)

        ch = self.phaser_ch0
        ch.init()
        self.setup_phaser_mtdds_channel(ch, [10 * MHz])

        servo = ch.servo

        profile = 0
        active_profile = 1
        if False:
            offset, kp, ki = -0.015259022, 1.0, 0.0
            servo.set_iir(profile, offset, kp, ki)
        else:
            # in machien unit
            offset_mu, b0_mu, a1_mu, b1_mu = 3000, 1 << 11, 0, 0
            servo.set_iir_mu(profile, offset_mu, b0_mu, a1_mu, b1_mu)

            offset_mu, b0_mu, a1_mu, b1_mu = -1560*2, 1 << 11, 0, 0
            servo.set_iir_mu(profile + 1, offset_mu, b0_mu, a1_mu, b1_mu)


        servo.set_active_profile(active_profile)
        servo.enable_iir(True)

        self.phaser.select_dac_source(0, 2) # servo -> DAC
        
        while True:
            x1, y1 = self.get_iir_x1y1(servo, active_profile)
            adc0, adc1 = self.get_adc_mu()
            print(y1, adc0)
            delay(5000 * us)
         
        # while True:
        #     self.print_adc_values(self.get_adc_mu(), self.phaser.gain_mus)
        #     terms = self.get_adc_terms()
        #     print(f"terminations = 0b{terms:02b}")

