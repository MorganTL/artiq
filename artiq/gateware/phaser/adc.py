from migen.genlib.io import DifferentialInput, DifferentialOutput
from migen import *

from artiq.gateware.phaser.register import RO, RW

import math


# TODO: rename files to LTC230.py
# TODO: add adc test case
# TODO: this should handle the crazy pin polarity switching and LTC2320 Module should be generic 
class ADCControl(Module):
    def __init__(self, adc_pins, adc_ctrl_pins, adc_sck_divider, sys_clk_freq):
        # TODO: verify the params is indeed correct

        width = 16
        self.submodules.adc = adc = PhaserLTC2320(
            adc_pins, width, adc_sck_divider, sys_clk_freq
        )
        self.comb += adc.start.eq(1)

        self.regs = []
        for i in range(2):
            setattr(self, f"ch{i}_data", Signal(width))
            setattr(self, f"ch{i}_term", Signal(1))
            setattr(self, f"ch{i}_gain", Signal(2))

            data = getattr(self, f"ch{i}_data")
            term = getattr(self, f"ch{i}_term")
            gain = getattr(self, f"ch{i}_gain")

            self.sync.rio += [
                data.eq(adc.data[i]),
                term.eq(adc_ctrl_pins.term_stat[i]),
                adc_ctrl_pins.gain0[i].eq(gain[0]),
                adc_ctrl_pins.gain1[i].eq(gain[1]),
            ]

            self.regs.extend([(data, RO), (term, RO), (gain, RW)])


# TODO: rename this to LTC2320Interface?
# TODO: put adcparam as arguments
# TODO: put a warning on polarity and port switch
class PhaserLTC2320(Module):
    """
    LTC2320 interface
    """

    def __init__(self, pins, width, sck_divider, sys_clk_freq):
        self.data = [Signal(width) for _ in range(2)]
        self.data_ready = Signal()
        self.start = Signal()
        self.idle = Signal()

        # # #

        # TODO: rewrite this crap
        # 1) ADC INPUT polarity is swapped, need to flip the value after reading
        # 2) ADC timinig issue
        #    - handle ADC convertion timing
        #    - deal with sck frequency limit (max 105 MHz)
        #    - sck need to be hold high before conv

        cnvn = Signal()
        self.submodules.sck_gen = sck_gen = SCKGen(width, sck_divider, sys_clk_freq)
        self.specials += [
            # the polarity was swapped on PCB
            DifferentialOutput(~sck_gen.out, pins.sck_n, pins.sck_p),
            DifferentialOutput(~cnvn, pins.cnvn_n, pins.cnvn_p),
        ]

        # From 232316fc - ADC TIMING CHARACTERISTICS table
        min_t_cnvn_h = 25  # ns, CNVN high time
        min_t_cnvn2sck = (
            9.5  # ns, time between CNVN filing edge and sck begin (dcnvsckl)
        )
        min_t_sck2next_cnvn_h = (
            19.1  # ns, time between last sck failing edge and next CNVN
        )

        # TODO: update this
        # Assuming sys_clk_freq = 125 MHz (8 ns)
        # 32 ns t_cnvh, 12 ns t_conv/t_DCNVSCKL, 192 ns data transfer, 24 ns t_rtt/tDSCKLCNVH
        # Note that there is one extra cycle (4 ns) at the end of a transaction.
        # Total: 264 ns -> 3.788 MSps

        sys_clk_period = 1e9 / sys_clk_freq
        cnvn_high_cnts = math.ceil(min_t_cnvn_h / sys_clk_period)
        wait_before_sck_cnts = math.ceil(min_t_cnvn2sck / sys_clk_period)
        next_cnvn_cnts = math.ceil(min_t_sck2next_cnvn_h / sys_clk_period)

        count = Signal(max=max(cnvn_high_cnts, next_cnvn_cnts), reset_less=True)

        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            self.idle.eq(1),
            If(self.start,
                NextValue(count, cnvn_high_cnts),
                NextState("CNVN_HIGH")
           ),
        )

        fsm.act("CNVN_HIGH",
            cnvn.eq(1),
            If(count == 0,
                NextState("WAIT_DATA_READY"),
            ).Else(
                NextValue(count, count - 1),
            ),
        )
        fsm.delayed_enter("WAIT_DATA_READY", "SCK_BEGIN", wait_before_sck_cnts)

        fsm.act("SCK_BEGIN",
            sck_gen.en.eq(1),
            If(sck_gen.done,
                NextValue(count, next_cnvn_cnts),
                NextState("WAIT_DATA")
            ),
        )

        update = Signal()
        fsm.act("WAIT_DATA",
            # account for sck->clkout round trip time
            If(count == 0,
                update.eq(1),
                NextState("IDLE"),
            ).Else(
                NextValue(count, count - 1),
            ),
        )

        # TODO: put this in another modules?

        clk_out_n = Signal()
        self.clock_domains.cd_clkout_n = ClockDomain()
        self.comb += self.cd_clkout_n.clk.eq(clk_out_n)

        sdos = [Signal(), Signal()]
        self.specials += [
            DifferentialInput(pins.clkout_p, pins.clkout_n, ~clk_out_n),
            DifferentialInput(pins.sdo_p[0], pins.sdo_n[0], sdos[0]),
            # the polarity was swapped on PCB
            DifferentialInput(pins.sdo_n[1], pins.sdo_p[1], ~sdos[1]),
        ]

        # ports are swapped (SMA_IN_0 = ADC_SDO2 and SMA_IN_1 = ADC_SDO1)
        data_mapping = {0: 1, 1: 0}
        for i, sdo in enumerate(sdos):
            sdo_sr = Signal(width)
            self.sync.clkout_n += [
                sdo_sr[1:].eq(sdo_sr),
                sdo_sr[0].eq(sdo),
            ]
            # TODO: add cdc
            self.sync += [
                self.data_ready.eq(0),
                If(update,
                    self.data[data_mapping[i]].eq(sdo_sr),
                    self.data_ready.eq(1),
                ),
            ]


class SCKGen(Module):
    def __init__(self, width, divider, sys_clk_freq):
        # output sys/divider and minimum SCK Period is 9.4 ns
        sys_clk_period = 1e9 / sys_clk_freq
        assert sys_clk_period * divider >= 9.4

        self.out = Signal()
        self.en = Signal()
        self.done = Signal()

        # # #

        max_edge_count = width * divider
        edge_count = Signal(max=max_edge_count, reset=max_edge_count - 1)
        self.sync += [
            If(self.en,
                If(edge_count == 0,
                    self.done.eq(1),
                ).Else(
                    edge_count.eq(edge_count - 1),
                    self.out.eq(~self.out),
                ),
            ).Else(
                # SCK needs to be idle high
                self.out.eq(1),
                edge_count.eq(edge_count.reset),
            ),
        ]
