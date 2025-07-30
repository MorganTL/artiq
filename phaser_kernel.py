from artiq.language.core import kernel
from artiq.language.units import MHz, kHz, dB
from artiq.experiment import *
from artiq.coredevice.rtio import (
    rtio_output,
    rtio_input_data,
    rtio_input_timestamped_data,
)
from artiq.gateware.phaser.dac import DAC_FIFO_DEPTH, SAMPLE_PER_CYCLE
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

GW_REV = 0x01
DAC_CTRL_ADDR = 0x02
DAC_STATUS_ADDR = 0x03
ATT_CTRL = 0x04


CH_TEST_EN_ADDR = 0x00
CH_TEST_WORD_I_ADDR = 0x01
CH_TEST_WORD_Q_ADDR = 0x02
CH_NCO_FREQ_WORD_ADDR = 0x03
CH_NCO_PHASE_WORD_ADDR = 0x04
CH_NCO_CLR_ADDR = 0x05

class Phaser_Kernel(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("led0")
        self.setattr_device("led1")
        for i in range(5):
            self.setattr_device(f"phaser_drtio0_led{i}")
            self.setattr_device(f"phaser_drtio0_spi{i}")
            setattr(self, f"phaser_led{i}", getattr(self, f"phaser_drtio0_led{i}"))

        self.setattr_device("phaser_drtio0_base")
        self.setattr_device("phaser_drtio0_channel0")
        self.setattr_device("phaser_drtio0_channel1")
        self.phaser = self.phaser_drtio0_base
        self.phaser_ch0 = self.phaser_drtio0_channel0
        self.phaser_ch1 = self.phaser_drtio0_channel1
        self.dac_spi = self.phaser_drtio0_spi0

        self.att0_spi = self.phaser_drtio0_spi3
        self.att1_spi = self.phaser_drtio0_spi4

        self.dac_init_mem_map = DAC34H84().get_mmap()

        self.rtlink_slack = int64(10_000)
        self.sysclk = 125 * MHz

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
        self.core.reset()

        self.dac_init()
        delay_mu(self.rtlink_slack)

        self.dac_att_test()

        self.debug_print()

    @kernel
    def dac_att_test(self):
        att_spis = [self.att0_spi, self.att1_spi]
        # dac config
        freqs = [10 * MHz, 15 * MHz]  # baseband: RF0, RF1
        phases = [0.0, 0.0] # in turns baseband: RF0, RF1
        interfaces = [self.phaser_ch0, self.phaser_ch1]
        for i in range(len(freqs)):
            # TODO: which formula is better?
            # ftw = int32(round(frequency*((1 << 30)/(125*MHz))))
            ftw = int32(
                round(
                    ((int64(1) << 32) - 1)
                    * (freqs[i] / (SAMPLE_PER_CYCLE * self.sysclk))
                )
            )
            turn = int32(round(phases[i]*(1 << 16)))
            interfaces[i].write(CH_NCO_CLR_ADDR, 1)
            delay_mu(int64(self.core.ref_multiplier))
            interfaces[i].write(CH_NCO_FREQ_WORD_ADDR, ftw)
            delay_mu(int64(self.core.ref_multiplier))
            interfaces[i].write(CH_NCO_PHASE_WORD_ADDR, turn)
            delay_mu(int64(self.core.ref_multiplier))

        for i in range(2):
            interfaces[i].write(CH_NCO_CLR_ADDR, 0)
            delay_mu(int64(self.core.ref_multiplier))

        # digital attenuator config

        # reset atts
        self.phaser.write(ATT_CTRL, 0b00)
        delay(10 * ms)
        self.phaser.write(ATT_CTRL, 0b11)
        delay(10 * ms)

        atts = [0 * dB, 0 * dB]  # baseband: RF0, RF1
        atts_mu = [0, 0]
        for i in range(len(atts)):
            # 2 lsb are inactive, resulting in 8 LSB per dB
            data = 0xFF - int32(round(atts[i] * 8))
            if data < 0 or data > 0xFF:
                raise ValueError("attenuation out of bounds")
            atts_mu[i] = data

        assert len(att_spis) == len(atts_mu)
        for i in range(len(att_spis)):
            self.set_att_mu(atts_mu[i], att_spis[i])

    # TODO: check whether mu should check/gate length out of bound
    @kernel
    def set_att_mu(self, data, interface, div=ATT_SPI_DIV):
        self.spi_cfg(div, False, True, 8, interface)
        interface.write(data << 24)

    @kernel
    def get_att_mu(self, interface, div=ATT_SPI_DIV):
        self.spi_cfg(div, True, True, 8, interface)
        interface.write(0)  # shift in zeros, shift out current value
        delay(10 * us)  # slack
        # shift it back
        data = interface.read() & 0xFF
        self.spi_cfg(div, False, True, 8, self.att1_spi)
        interface.write(data << 24)
        delay(10 * us)  # slack
        return data

    @kernel
    def dac_nco_mu(self, addr, ftw):
        self.phaser.write(addr, ftw)
        delay_mu(self.rtlink_slack)

    @kernel
    def debug_print(self):
        self.core.break_realtime()
        board_id = self.phaser.read(0x00)
        delay_mu(self.rtlink_slack)
        dac_ctrl = self.phaser.read(DAC_CTRL_ADDR)
        delay_mu(self.rtlink_slack)
        dac_alarm = self.phaser.read(DAC_STATUS_ADDR)
        delay_mu(self.rtlink_slack)
        dac0 = self.phaser_ch0.read(CH_TEST_WORD_I_ADDR)
        delay_mu(self.rtlink_slack)
        dac1 = self.phaser_ch0.read(CH_TEST_WORD_Q_ADDR)
        delay_mu(self.rtlink_slack)
        dac2 = self.phaser_ch1.read(CH_TEST_WORD_I_ADDR)
        delay_mu(self.rtlink_slack)
        dac3 = self.phaser_ch1.read(CH_TEST_WORD_Q_ADDR)
        delay_mu(self.rtlink_slack)

        ch0_nco = self.phaser_ch0.read(CH_NCO_FREQ_WORD_ADDR)
        delay_mu(self.rtlink_slack)
        ch1_nco = self.phaser_ch1.read(CH_NCO_FREQ_WORD_ADDR)
        delay_mu(self.rtlink_slack)

        att0_mu = self.get_att_mu(self.att0_spi)

        # SLAS751D section 6.8 timing requirement when reading...
        # - Temperature register: SCLK period >1 us
        # - Other register: SCLK period > 100 ns
        # Take x10 of DAC_SPI_DIV to be safe
        temperature = self.dac_read(0x06, 10 * DAC_SPI_DIV) >> 8
        if temperature > 85:
            raise ValueError("DAC34H84 overheat")
        print("dac temperature read = ", temperature)
        print("board_id =", board_id)
        print("dac ctrl =", dac_ctrl, "dac alarm =", dac_alarm)
        print("dac0 =", dac0, "dac1 =", dac1, "dac2 =", dac2, "dac3 =", dac3)
        print("NCO ch0=", ch0_nco, "ch1 =", ch1_nco)
        print("attr0 =", att0_mu)
        delay_mu(1_000_000)

    # TODO: put this in a separate class
    @kernel
    def dac_init(self):
        # Power-Up Sequence - SLAS751D section 7.5.1

        # Toggle reset but keep tx off
        self.set_dac_ctrl(dac_txena=0, dac_resetb=0, dac_sleep=0)
        delay_mu(self.rtlink_slack)
        self.set_dac_ctrl(dac_txena=0, dac_resetb=1, dac_sleep=0)
        delay_mu(self.rtlink_slack)

        # enable 4-wire SPI mode, sif4_enable
        self.dac_write(0x02, 0x0080)
        if self.dac_read(0x7F) != 0x5409:
            raise ValueError("DAC34H84 version mismatch")
        if self.dac_read(0x00) != 0x049C:
            raise ValueError("DAC34H84 reset fail")

        for data in self.dac_init_mem_map:
            self.dac_write(data >> 16, data & 0xFFFF)

        self.dac_sync()

        # pll_ndivsync_ena disable
        cfg_0x18 = self.dac_read(0x18)
        self.dac_write(0x18, cfg_0x18 & ~0x0800)

        # TODO: remove custom patterns?
        test_patterns = [
            [0xF05A, 0x05AF, 0x5AF0, 0xAF05],  # test channel/iq/byte/nibble
            [0x7A7A, 0xB6B6, 0xEAEA, 0x4545],  # datasheet pattern a
            [0x1A1A, 0x1616, 0xAAAA, 0xC6C6],  # datasheet pattern b
        ]
        for p in test_patterns:
            self.dac_iotest(p)

        lf_volt = self.dac_read(0x18) & 7
        # Use PLL loop filter voltage to check lock status - Table 10, Step 34 SLAS751D section 7.5.2.4
        if not (0x2 <= lf_volt <= 0x5):
            print("lf_volt =", lf_volt)
            raise ValueError("DAC34H84 PLL lock failed")

        self.dac_tune_fifo_offset()

        self.debug_check_dac_alarm()

        # avoid malformed output for: mixer_ena=1, nco_ena=0 after power up
        self.dac_write(
            self.dac_init_mem_map[2] >> 16, self.dac_init_mem_map[2] | (1 << 4)
        )
        delay(40 * us)
        self.dac_sync()
        delay(100 * us)
        self.dac_write(self.dac_init_mem_map[2] >> 16, self.dac_init_mem_map[2])
        delay(40 * us)
        self.dac_sync()
        delay(100 * us)

        self.set_dac_ctrl(dac_txena=1, dac_resetb=1, dac_sleep=0)
        delay_mu(self.rtlink_slack)

    @kernel
    def spi_cfg(self, div, read, end, length, interface):
        """
        Set the SPI machine configuration

        :param div: SPI clock divider relative to rtio clock
        :param end: Whether to end the SPI transaction and deassert chip select
        :param length: SPI transfer length
        """
        spi_cfg = DAC_SPI_CONFIG
        if end:
            spi_cfg |= spi.SPI_END
        if read:
            spi_cfg |= spi.SPI_INPUT
        interface.set_config_mu(spi_cfg, length, div, 1)

    # TODO: tune the read/write slack
    @kernel
    def dac_read(self, addr, div=DAC_SPI_DIV):
        cmd = 1 << 7 | addr & ((1 << DAC_SPI_ADDR_WIDTH) - 1)
        self.spi_cfg(div, False, False, DAC_SPI_CMD_WIDTH, self.dac_spi)
        self.dac_spi.write(cmd << 24)
        self.spi_cfg(div, True, True, DAC_SPI_DATA_WIDTH, self.dac_spi)
        self.dac_spi.write(0)
        delay(10 * us)  # slack
        return self.dac_spi.read()

    @kernel
    def dac_write(self, addr, value, div=DAC_SPI_DIV):
        self.spi_cfg(
            div, False, True, DAC_SPI_CMD_WIDTH + DAC_SPI_DATA_WIDTH, self.dac_spi
        )
        cmd = addr & ((1 << DAC_SPI_ADDR_WIDTH) - 1)
        value = value & ((1 << DAC_SPI_DATA_WIDTH) - 1)
        self.dac_spi.write(cmd << 24 | value << 8)

    @kernel
    def dac_sync(self):
        config1f = self.dac_read(0x1F)
        self.dac_write(0x1F, config1f & ~0x2)
        self.dac_write(0x1F, config1f | 0x2)

    # TODO: cleanup/rewrite fifo offset
    @kernel
    def dac_tune_fifo_offset(self):
        """Scan through ``fifo_offset`` and configure midpoint setting.

        :return: Optimal ``fifo_offset`` setting with maximum margin to write
            pointer.
        """
        # expect two or three error free offsets:
        #
        # read offset 01234567
        # write pointer  w
        # distance    32101234
        # error free  x     xx
        cfg_09 = self.dac_read(0x09)
        good = 0
        for o in range(DAC_FIFO_DEPTH):
            self.dac_write(0x09, (cfg_09 & 0x1FFF) | (o << 13))  # set new fifo_offset
            self.clear_dac_alarms()
            delay(0.1 * ms)  # let it run for a bit
            if (self.get_dac_alarms() >> 11) & 0x7 == 0:  # check for fifo alarm
                good |= 1 << o
        # if there are good offsets across the wrap around
        # offset for computations
        if good & 0x81 == 0x81:
            good = ((good << 4) & 0xF0) | (good >> 4)
            offset = 4
        else:
            offset = 0
        # calculate mean
        sum = 0
        count = 0
        for o in range(8):
            if good & (1 << o):
                sum += o
                count += 1
        if count == 0:
            raise ValueError("no good fifo offset")
        best = ((sum // count) + offset) % 8
        self.dac_write(0x09, (cfg_09 & 0x1FFF) | (best << 13))
        return best

    @kernel
    def dac_iotest(self, pattern):
        if len(pattern) != 4:
            raise ValueError("pattern length mismatch")

        self.phaser_ch0.set_test_mode_en(True)
        self.phaser_ch1.set_test_mode_en(True)

        for i in range(len(pattern)):
            # repeat the pattern twice
            self.dac_write(0x25 + i, pattern[i])
            self.dac_write(0x29 + i, pattern[i])

        self.phaser_ch0.set_test_iq_word(pattern[0], pattern[1])
        self.phaser_ch1.set_test_iq_word(pattern[2], pattern[3])

        cfg_01 = self.dac_read(0x01)
        self.dac_write(0x01, cfg_01 | 0x8000)  # enable iotest
        self.dac_write(0x04, 0x0000)  # clear iotest_result

        error = self.dac_read(0x04)
        self.dac_write(0x01, cfg_01)  # restore config
        if error != 0:
            print("error reg =", error)
            self.debug_print()
            raise ValueError("DAC iotest failure")

        self.phaser_ch0.set_test_mode_en(False)
        self.phaser_ch1.set_test_mode_en(False)
        delay_mu(self.rtlink_slack)

    @kernel
    def clear_dac_alarms(self):
        self.dac_write(0x05, 0x0000)

    @kernel
    def get_dac_alarms(self):
        return self.dac_read(0x05)

    @kernel
    def debug_check_dac_alarm(self):
        self.clear_dac_alarms()
        alarm = self.get_dac_alarms() & ~0x40  # masked out the revered bit
        if alarm != 0:
            print("alarm = ", alarm)
            raise ValueError("DAC34H84 alarm")

    @kernel
    def set_dac_ctrl(self, dac_txena, dac_resetb, dac_sleep):
        self.phaser.write(
            DAC_CTRL_ADDR,
            (dac_txena & 1) | (dac_resetb & 1) << 1 | (dac_sleep & 1) << 2,
        )
