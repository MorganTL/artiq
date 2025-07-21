# TODO: add COPYRIGHT

from collections import namedtuple

from migen import *
# TODO remove this
from migen.build.generic_platform import *
from misoc.cores.duc import PhasedDUC

from artiq.gateware.phaser.adc import ADCControl
from artiq.gateware.phaser.register import RO, RW, RegisterDecoder
from artiq.gateware.phaser.dac import DAC34H84PHY, DAC_DATA_WIDTH, SAMPLE_PER_CYCLE
from artiq.gateware.phaser.iir import Iir, Dsp
from artiq.gateware.phaser.multi_tone import IQPair
from artiq.gateware.rtio import rtlink


PHASER_BOARD_ID = 19
PHASER_GW_DRTIO = 3
SERVO_PROFILES = 4  # number iir coefficient profiles per servo channel
SERVO_CHANNELS = 2  # number servochannels

def Register(width=0, read=True, write=True):
    pass

phaser_registers = [
    (0x00,),
    # Sinara board id (19) as assigned in the Sinara EEPROM layout
    ("board_id", Register(write=False)),
    # hardware revision and variant
    ("hw_rev", Register(write=False)),
    # gateware revision
    ("gw_rev", Register(write=False)),
    # configuration (clk_sel, dac_resetb, dac_sleep,
    # dac_txena, trf0_ps, trf1_ps, att0_rstn, att1_rstn)
    ("cfg", Register()),
    # status (dac_alarm, trf0_ld, trf1_ld, term0_stat,
    # term1_stat, spi_idle)
    ("sta", Register(write=False)),
    # frame crc error counter
    ("crc_err", Register(write=False)),
    # led configuration
    ("led", Register(width=6)),
    # fan pwm duty cycle
    ("fan", Register()),
    # DUC settings update strobe
    ("duc_stb", Register(write=False, read=False)),
    # ADC gain configuration (pgia0_gain, pgia1_gain)
    ("adc_cfg", Register(width=4)),
    # spi configuration (offline, end, clk_phase, clk_polarity,
    # half_duplex, lsb_first)
    ("spi_cfg", Register()),
    # spi divider and transaction length (div(5), len(3))
    ("spi_divlen", Register()),
    # spi chip select (dac, trf0, trf1, att0, att1)
    ("spi_sel", Register()),
    # spi mosi data and transaction start/continue
    ("spi_datw", Register(read=False)),
    # spi readback data, available after each transaction
    ("spi_datr", Register(write=False)),
    # dac data interface sync delay (for sync-dac_clk alignment and
    # n-div/pll/ostr fifo output synchronization)
    ("sync_dly", Register(width=3)),
    (0x10,),
    # digital upconverter (duc) configuration
    # (accu_clr, accu_clr_once, data_select (0: duc, 1: test))
    ("duc0_cfg", Register()),
    ("duc0_reserved", Register(read=False, write=False)),
    # duc frequency tuning word (msb first)
    ("duc0_f", Register(), Register(), Register(), Register()),
    # duc phase offset word
    ("duc0_p", Register(), Register()),
    # dac data
    (
        "dac0_data",
        Register(write=False),
        Register(write=False),
        Register(write=False),
        Register(write=False),
    ),
    # dac test data for duc_cfg:data_select == 1
    ("dac0_test", Register(), Register(), Register(), Register()),
    (0x20,),
    # digital upconverter (duc) configuration
    # (accu_clr, accu_clr_once, data_select (0: duc, 1: test))
    ("duc1_cfg", Register()),
    ("duc1_reserved", Register(read=False, write=False)),
    # duc frequency tuning word (msb first)
    ("duc1_f", Register(), Register(), Register(), Register()),
    # duc phase offset word
    ("duc1_p", Register(), Register()),
    # dac data
    (
        "dac1_data",
        Register(write=False),
        Register(write=False),
        Register(write=False),
        Register(write=False),
    ),
    # dac test data for duc_cfg:data_select == 1
    ("dac1_test", Register(), Register(), Register(), Register()),
    (0x30,),
    # (ch0_profile[2], en0)
    ("servo0_cfg", Register()),
    # (ch1_profile[2], en1)
    ("servo1_cfg", Register()),
]


# TODO: sync is 250MHz
# TODO: use this?
class PWM(Module):
    """Pulse width modulation"""

    def __init__(self, pin, width=10):
        cnt = Signal(width, reset_less=True)
        self.duty = Signal.like(cnt)
        self.sync += [
            cnt.eq(cnt + 1),
            If(
                cnt == 0,
                pin.eq(1),
            ),
            If(
                cnt == self.duty,
                pin.eq(0),
            ),
        ]


Phy = namedtuple("Phy", "rtlink probes overrides")

# TODO: accept pins instead of platform? checkout shuttler
class NeoPhaser(Module):
    def __init__(self, att_rstn_pins, dac_data_pins, dac_ctrl_pins, sys_clk_freq):


        # TODO: add gateware/rtio/phy later
        # Just add a rtlink with address and data similar to the register table
        
        # TODO: add tone generation 
        # IQ pair
        # cossine0 IQ ────┐
        #    ...          +──── PhasedDUC ──── DACPHY
        # cossineN IQ ────┘

        # TODO: keep in mind of clock domain (rio vs sys)
        # The DAC PHY is clocked in sys but data coming in should be rio
        self.submodules.dac = dac = DAC34H84PHY(dac_data_pins, dac_ctrl_pins)

        # Control and status
        dac_ctrls = Signal(3)
        dac_status = Signal(1)
        att_rstn = Signal(2)
        self.sync.rio += [
            Cat(dac.en, dac.reset_n, dac.sleep).eq(dac_ctrls),
            dac_status.eq(dac.alarm),
            att_rstn_pins[0].eq(att_rstn[0]),
            att_rstn_pins[1].eq(att_rstn[1]),
        ]
        
        # TODO: choose a better data & address width
        # TODO: add GW rev?
        reg = [
            (C(PHASER_BOARD_ID), RO),
            (C(PHASER_GW_DRTIO), RO),
            (dac_ctrls, RW),
            (dac_status, RO),
            (att_rstn, RW)
        ]
        self.submodules.reg_decoder = reg_decoder = RegisterDecoder(reg, 32, 8) 
        decoders = [reg_decoder]

        # TODO: choose better width
        f_width, p_width = 32, DAC_DATA_WIDTH
        for ch, iq_ch in enumerate([[dac.sink_4x_a, dac.sink_4x_b], [dac.sink_4x_c, dac.sink_4x_d]]):
            iq_pair = IQPair(SAMPLE_PER_CYCLE, f_width, p_width, DAC_DATA_WIDTH)
            self.submodules += iq_pair
            decoders.append(iq_pair.reg_decoder)

            for i, (sink_i, sink_q) in enumerate(zip(*iq_ch)):
                self.comb += [
                    sink_i.eq(iq_pair.source[i].i),
                    sink_q.eq(iq_pair.source[i].q),
                ]

        # connect register decoder to rtlink
        self.phys = []
        for decoder in decoders:
            rt_interface = rtlink.Interface(
                rtlink.OInterface(
                    data_width=len(decoder.sink.data),
                    address_width=len(decoder.sink.address),
                    enable_replace=False,
                ),
                rtlink.IInterface(data_width=len(decoder.source.data)),
            )
            self.comb += [
                decoder.sink.stb.eq(rt_interface.o.stb),
                decoder.sink.address.eq(rt_interface.o.address),
                decoder.sink.data.eq(rt_interface.o.data),
                rt_interface.i.stb.eq(decoder.source.stb),
                rt_interface.i.data.eq(decoder.source.data),
            ]
            self.phys.append(Phy(rt_interface, [], []))
