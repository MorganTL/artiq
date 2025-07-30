from artiq.language.core import kernel
from artiq.language.units import MHz, kHz, dB, GHz
from artiq.experiment import *
from artiq.coredevice.core import rtio_get_counter
from artiq.coredevice.rtio import (
    rtio_output,
    rtio_input_data,
    rtio_input_timestamped_data,
)
from artiq.gateware.phaser.dac_phy import DAC_FIFO_DEPTH
from artiq.coredevice import spi2 as spi
from artiq.coredevice.dac34h84 import DAC34H84
from numpy import int32, int64

ATT_SPI_DIV = 5  # min 33 ns for attenuator SPI
DAC_SPI_DIV = 20  # min 100 ns for DAC SPI
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

DAC_SPI_ADDR_WIDTH = 7
DAC_SPI_CMD_WIDTH = DAC_SPI_ADDR_WIDTH + 1
DAC_SPI_DATA_WIDTH = 16

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
    ATT_CTRL,
] = range(12)


class Phaser_Kernel(EnvExperiment):
    kernel_invariants = {"dds_cfg"}

    def build(self):
        self.setattr_device("core")
        self.setattr_device("led0")
        self.setattr_device("led1")
        phaser_name = "phaser_mtdds0"
        for i in range(5):
            self.setattr_device(f"{phaser_name}_led{i}")
            setattr(self, f"phaser_led{i}", getattr(self, f"{phaser_name}_led{i}"))

        self.setattr_device(f"{phaser_name}")
        setattr(self, "phaser", getattr(self, f"{phaser_name}"))

        for ch in range(2):
            self.setattr_device(f"{phaser_name}_channel{ch}")
            setattr(self, f"phaser_ch{ch}", getattr(self, f"{phaser_name}_channel{ch}"))

        self.setattr_device(f"{phaser_name}_dac")
        setattr(self, "dac", getattr(self, f"{phaser_name}_dac"))

        self.setattr_device(f"{phaser_name}_att0")
        self.setattr_device(f"{phaser_name}_att1")
        setattr(self, "att0_spi", getattr(self, f"{phaser_name}_att0"))
        setattr(self, "att1_spi", getattr(self, f"{phaser_name}_att1"))

        self.dac_init_mem_map = DAC34H84().get_mmap()

        self.rtlink_slack = int64(400_000)
        # self.rtlink_slack = int64(self.core.ref_multiplier)
        self.sysclk = 125 * MHz

        self.sample_per_cycle = self.phaser.samples_per_cycle

        # for optimization (2D array is slow...)
        tones = 10
        self.dds_cfg = [
            # RF0
            [
                # frequency, turns, amplitudes
                # [(20 + 1 * i) * MHz, 0.0, (1.0 / tones)]
                # for i in range(tones)
                # ----------
                [20.0 * MHz, 0.0, 0.5],
                # [50.0 * MHz, 0.0, 0.5],
            ],
            # RF1
            [
                # ----------
                [20.0 * MHz, 0.0, 0.5],
                [21.0 * MHz, 0.0, 0.5],
                # ----------
                # [10 * MHz, 0.0, 1.0],
                # ----------
                # [100 * MHz, 0.0, 0.2],
                # [25 * MHz, 0.0, 0.2],
                # [12.5 * MHz, 0.0, 0.2],
            ],
        ]

    # NOTE: this works fine
    @subkernel(destination=4)
    def foo(self) -> TInt32:
        # self.phaser_led0.on()
        # self.phaser_led0.off()
        self.phaser.write(0x03, 0x03)
        delay_mu(8 * 3)
        return self.phaser.read(0x03)

    @kernel
    def run(self):
        # TODO: detect baseband variant
        use_subkernel = False
        # NOTE: SoC is having rtlink.input issue :<, it delays for one cycle for some reason
        # if use_subkernel:
        #     subkernel_preload(self.foo)
        #     self.core.reset()
        #     delay(100 * ms)
        #     self.foo()
        #     data = subkernel_await(self.foo)
        #     print(data)
        # NOTE: fifo offset change not related to rtio_init() syscall...
        self.core.reset()
         
        # at_mu(rtio_get_counter() + 125000)
        # self.phaser.init()

        self.dac_init()
        self.dac_output_test()

        # self.debug_print()

    @kernel
    def dac_output_test(self):

        # TODO: rewrite all of this
        #
        # 1) DAC NCO mixer frequency sweep mode
        # 2) DAC DDS freqeuncy sweep mode
        # 3) Normal DAC operation

        # Base mode
        atts_cfg = [
            0 * dB,  # RF0
            0 * dB,  # RF1
        ]
        # DDS frequency sweep config
        dds_freq_sweep = False
        start, stop, step = (0 * MHz, (125 - 1) * MHz, 0.1 * MHz)

        # lf cfg will be used whenever nco mixer is enable
        use_dac_nco_mixer = False
        lf_cfg = [
            # RF0
            [0.0 * MHz, 0.0, 1.0],
            # RF1
            [0.0 * MHz, 0.0, 1.0],
        ]
        dac_nco_mixer_cfg = [
            [10 * MHz, 0.0],  # RF0
            [10 * MHz, 0.0],  # RF1
        ]

        # NCO frequency sweep config
        nco_mixer_freq_sweep = False
        lo_start, lo_stop, lo_step = (0 * MHz, 500 * MHz, 0.1 * MHz)

        if dds_freq_sweep and use_dac_nco_mixer:
            raise ValueError(
                "DAC NCO mixer should be off when doing DDS frequency sweep"
            )

        if dds_freq_sweep and nco_mixer_freq_sweep:
            raise ValueError("Only one frequency sweep can be active at the same time")

        # apply settings
        channels = [self.phaser_ch0, self.phaser_ch1]

        channels[0].attenuator.set_att(atts_cfg[0])
        channels[1].attenuator.set_att(atts_cfg[1])
        delay(10 * us)

        write_delay = 5 * us
        if nco_mixer_freq_sweep or use_dac_nco_mixer:
            for i in range(len(channels)):
                channels[i].ddss[0].enable_phase_accumulator(False)
                delay(write_delay)
                channels[i].ddss[0].set_frequency(lf_cfg[i][0])
                delay(write_delay)
                channels[i].ddss[0].set_phase_offset(lf_cfg[i][1])
                delay(write_delay)
                channels[i].ddss[0].set_amplitude(lf_cfg[i][2])
                delay(write_delay)
                channels[i].ddss[0].enable_phase_accumulator(True)
                delay(write_delay)
        elif dds_freq_sweep:
            # prepare freq sweep
            for i in range(len(channels)):
                channels[i].ddss[0].enable_phase_accumulator(False)
                delay(write_delay)
                channels[i].ddss[0].set_phase_offset(0.0)
                delay(write_delay)
                channels[i].ddss[0].set_amplitude(1.0)
                delay(write_delay)
            freq = start
            while freq < stop:
                for i in range(len(channels)):
                    channels[i].ddss[0].set_frequency(freq)
                    delay(write_delay)
                    channels[i].ddss[0].enable_phase_accumulator(True)
                    delay(5 * ms)
                freq += step
            # turn off after sweep
            for i in range(len(channels)):
                channels[i].ddss[0].enable_phase_accumulator(False)
                delay(write_delay)
        else:
            for i in range(len(channels)):
                for j in range(len(self.dds_cfg[i])):
                    channels[i].ddss[j].enable_phase_accumulator(False)
                    delay(write_delay)
                    channels[i].ddss[j].set_frequency(self.dds_cfg[i][j][0])
                    delay(write_delay)
                    channels[i].ddss[j].set_phase_offset(self.dds_cfg[i][j][1])
                    delay(write_delay)
                    channels[i].ddss[j].set_amplitude(self.dds_cfg[i][j][2])
                    delay(write_delay)
                    channels[i].ddss[j].enable_phase_accumulator(True)
                    delay(write_delay)

        self.phaser.dac.set_mixer_enable(False)
        if use_dac_nco_mixer:
            self.phaser.dac.set_mixer_enable(True)
            for i in range(len(channels)):
                channels[i].stage_dac_nco_mixer_frequency(dac_nco_mixer_cfg[i][0])
                delay_mu(int64(self.core.ref_multiplier))
                channels[i].stage_nco_mixer_phase_offset(dac_nco_mixer_cfg[i][1])
                delay_mu(int64(self.core.ref_multiplier))
                self.dac.sync()

        if nco_mixer_freq_sweep:
            freq = lo_start
            self.phaser.dac.set_mixer_enable(True)
            while freq < lo_stop:
                for i in range(len(channels)):
                    channels[i].stage_dac_nco_mixer_frequency(freq)
                    delay_mu(int64(self.core.ref_multiplier))
                self.dac.sync()
                freq += lo_step
                delay(5 * ms)
            self.phaser.dac.set_mixer_enable(False)

    @kernel
    def debug_print(self):
        self.core.break_realtime()
        hw_variant = self.phaser.read(HW_VARIANT)
        delay_mu(self.rtlink_slack)
        samples = self.phaser.read(SAMPLE_PER_CYCLE)
        delay_mu(self.rtlink_slack)
        tones = self.phaser.read(AVAILABLE_TONES)
        delay_mu(self.rtlink_slack)
        dac_ctrl = self.phaser.read(DAC_CTRL_ADDR)
        delay_mu(self.rtlink_slack)
        dac_alarm = self.phaser.read(DAC_STATUS_ADDR)
        delay_mu(self.rtlink_slack)
        dac0 = self.phaser.read(DAC_TEST_WORD_0_I_ADDR)
        delay_mu(self.rtlink_slack)
        dac1 = self.phaser.read(DAC_TEST_WORD_0_Q_ADDR)
        delay_mu(self.rtlink_slack)
        dac2 = self.phaser.read(DAC_TEST_WORD_1_I_ADDR)
        delay_mu(self.rtlink_slack)
        dac3 = self.phaser.read(DAC_TEST_WORD_1_Q_ADDR)
        delay_mu(self.rtlink_slack)

        att0_mu = self.att0_spi.get_att_mu()
        delay_mu(self.rtlink_slack)
        att0 = self.att0_spi.get_att()
        delay_mu(self.rtlink_slack)

        # SLAS751D section 6.8 timing requirement when reading...
        # - Temperature register: SCLK period >1 us
        # - Other register: SCLK period > 100 ns
        # Take x10 of DAC_SPI_DIV to be safe
        temperature = self.dac_read(0x06, 10 * DAC_SPI_DIV) >> 8
        if temperature > 85:
            raise ValueError("DAC34H84 overheat")
        print("dac temperature read = ", temperature)
        print("hw_variant =", hw_variant)
        print("samples =", samples, "tones =", tones)
        print("dac ctrl =", dac_ctrl, "dac alarm =", dac_alarm)
        print("dac0 =", dac0, "dac1 =", dac1, "dac2 =", dac2, "dac3 =", dac3)
        print("att0_mu =", att0_mu, "att0 =", att0, "dB")
        delay_mu(1_000_000)

    # TODO: put this in a separate class
    @kernel
    def dac_init(self):
        self.phaser.init()

        delay(10 * ms)
        # NOTE: no need in production (assume all init steps are correct (: )
        # check interpolation rate is correct
        interpolation_mu = (self.dac.read(0x00) >> 8) & 0xF
        delay(10 * us)  # slack for read
        if self.sysclk * self.sample_per_cycle * (1 << interpolation_mu) != 1 * GHz:
            raise ValueError("Interpolation rate mismatch")
        self.debug_check_dac_alarm()

    # TODO: tune the read/write slack
    @kernel
    def dac_read(self, addr, div=DAC_SPI_DIV) -> TInt32:
        data = self.dac.read(addr)
        delay(10 * us)  # slack
        return data

    @kernel
    def dac_write(self, addr, value, div=DAC_SPI_DIV):
        self.dac.write(addr, value)

    @kernel
    def clear_alarms(self):
        self.dac.write(0x05, 0x0000)

    @kernel
    def get_alarms(self):
        alarm = self.dac.read(0x05)
        delay(10 * us)
        return alarm

    @kernel
    def debug_check_dac_alarm(self):
        self.clear_alarms()
        delay(10 * us)
        alarm = self.get_alarms() & ~0x40  # masked out the revered bit
        if alarm != 0:
            print("alarm = ", alarm)
            raise ValueError("DAC34H84 alarm")
