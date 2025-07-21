# Copyright 2013-2017 Robert Jordens <jordens@gmail.com>
#
# shuttler is developed based on pdq.
#
# pdq is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# pdq is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with pdq.  If not, see <http://www.gnu.org/licenses/>.

from collections import namedtuple
from operator import add

from migen import *
from migen.genlib.cdc import MultiReg
from misoc.cores.spi2 import SPIMachine, SPIInterface
from misoc.cores.duc import complex
from misoc.cores.duc import PhasedDUC, MultiDDS
from misoc.interconnect.stream import Endpoint
from misoc.interconnect.csr import *
from misoc.cores.cordic import Cordic
from artiq.gateware.rtio import rtlink

from artiq.gateware.phaser.dac_data import DacData
from artiq.gateware.phaser.adc import Adc, AdcParams
from artiq.gateware.phaser.iir import Iir, Dsp
from artiq.gateware.phaser.interpolate import SampleMux, InterpolateChannel

from artiq.gateware import rtio

class RTPhy(Module):
    def __init__(self, regs):
        self.rtlink = rtlink.Interface(
            rtlink.OInterface(data_width=32, address_width=4,
                              enable_replace=True))
        self.sync += [
            If(self.rtlink.o.stb,
                Array(regs)[self.rtlink.o.address].eq(self.rtlink.o.data)
            )
        ]


### Can be run in 250MHz Domain
class DDSChannel(Module):
    def __init__(self, share_lut=None):

        # DDS Input
        # f: Frequency
        # p: Phaser
        # a: amplitude
        # clr: clear bit
        # o: Complex i and q

        self.submodules.dds = MultiDDS(
            n=20, fwidth=32, xwidth=16, z=19, zl=10, share_lut=share_lut)

        ####### THIS ONLY ASSIGN FOR 5 OUT OF 10 tones. This should generate sinwaves in 25MS/S
        for i in range(5):
            setattr(self, f"frequency{i}", self.dds.i[i].f)
            setattr(self, f"phase_amplitude{i}", Cat(self.dds.i[i].a, self.dds.i[i].clr, self.dds.i[i].p))
        # 5 



        # self.frequency = [i.f for i in self.dds.i]
        # self.phase_amplitude = [Cat(i.a, i.clr, i.p) for i in self.dds.i]


        # self.submodules.frequency = RTPhy([i.f for i in self.dds.i])
        # self.submodules.phase_amplitude = RTPhy(
        #     [Cat(i.a, i.clr, i.p) for i in self.dds.i])

### SHOULD BE in 125MHz Clock Domain
### 



# t_frame should be 80ns


# Can be run in 250MHz. Should be downsampled
class Base(Module):
    def __init__(self, ds_factor=1):
        assert ds_factor > 0

        self.submodules.ch0 = DDSChannel()
        self.submodules.ch1 = DDSChannel(share_lut=self.ch0.dds.cs.lut)

        n_channels = 2
        n_samples = 8
        n_bits = 14
        body = Signal(n_samples*n_channels*2*n_bits, reset_less=True)
        self.output = Signal(n_samples*n_channels*2*n_bits, reset_less=True)
        self.sync += [
            If(self.ch0.dds.valid,  # & self.ch1.dds.valid,
                # recent:ch0:i as low order in body
                Cat(body).eq(Cat(self.ch0.dds.o.i[2:], self.ch0.dds.o.q[2:],
                                 self.ch1.dds.o.i[2:], self.ch1.dds.o.q[2:],
                                 body)),
            ),
        ]

        counter = Signal(max=n_samples, reset=n_samples-1)
        self.sync += [
            If(counter == 0,
                counter.eq(n_samples - 1)
            ).Elif(self.ch0.dds.valid,
                counter.eq(counter - 1)
            )
        ]

        self.comb += [
            self.output.eq(body),
            self.ch0.dds.stb.eq(1),
            self.ch1.dds.stb.eq(1), 
        ]

        # self.output_valid = Signal()
        # self.comb += [
        #     self.output_valid.eq(1)
        # ]

        self.output_valid = Signal()
        self.comb += [
            If(counter == 0,
                self.output_valid.eq(1)
            ).Else(
                self.output_valid.eq(0)
            )
        ]


        # counter = Signal(max=n_samples)
        # self.sync += [
        #     If(self.ch0.dds.valid,
        #         counter.eq(counter - 1),
        #     ),

        #     If(counter == 0,
        #         counter.eq(n_samples - 1),
        #     ).Else(
        #         counter.eq(counter - 1),
        #         self.ch0.dds.stb.eq(0),
        #         self.ch1.dds.stb.eq(0),
        #     )
        # ]


        # self.comb += [
        #     self.output.eq(body),
        #     self.ch0.dds.stb.eq(1),
        #     self.ch1.dds.stb.eq(1), 
        # ]



        # if ds_factor == 1:
        #     self.ch0.dds.stb.eq(1),
        #     self.ch1.dds.stb.eq(1), 
        # else:
        #     counter = Signal(max=ds_factor)
        #     self.sync += [
        #         If(counter == 0,
        #             counter.eq(ds_factor - 1),
        #             self.ch0.dds.stb.eq(1),
        #             self.ch1.dds.stb.eq(1),
        #         ).Else(
        #             counter.eq(counter - 1),
        #             self.ch0.dds.stb.eq(0),
        #             self.ch1.dds.stb.eq(0),
        #         )
        #     ]


# runs in 250MHz
class Interpolation(Module):
    def __init__(self, b_sample, n_channel, n_mux, t_frame):
        n_samples = n_mux * n_channel * 2
        # header = Record(header_layout)
        body = Signal(n_samples * b_sample)
        self.frame = Signal(len(body))
        # self.frame = Signal(len(body) + len(header))
        self.stb = Signal()
        self.comb += [
            body.eq(self.frame)
        ]
        # self.comb += [
        #     Cat(header.raw_bits(), body).eq(self.frame),
        # ]

        
        # ORG SAMPLEMUX
        self.submodules.zoh = SampleMux(
            b_sample=b_sample, n_channel=n_channel, n_mux=n_mux, t_frame=t_frame
        )
        self.comb += [
            self.zoh.body.eq(body),
            self.zoh.body_stb.eq(self.stb),
            # self.zoh.body_stb.eq(self.stb & (header.type == 1)),
        ]
        

        self.interpolate = []

        # 2 data [ 2 Channel OMPLEX ]
        self.data = [[Record(complex(16)) for _ in range(n_channel)] for _ in range(2)]
        for ch in range(n_channel):
            for iq in "iq":
                inter = InterpolateChannel()
                self.submodules += inter
                self.interpolate.append(inter)
                self.comb += [
                    inter.input.data.eq(getattr(self.zoh.sample[ch], iq)),
                    inter.input.stb.eq(self.zoh.sample_stb),
                    getattr(self.data[0][ch], iq).eq(inter.output.data0),
                    getattr(self.data[1][ch], iq).eq(inter.output.data1),
                    inter.output.ack.eq(1),
                ]

        # self.submodules.bus = Bus()
        # self.comb += [
        #     self.bus.bus.dat_w.eq(header.data),
        #     self.bus.bus.adr.eq(header.addr),
        #     self.bus.bus.we.eq(self.stb & header.we),
        #     self.bus.bus.re.eq(self.stb & ~header.we),
        #     self.response.eq(self.bus.bus.dat_r),
        # ]

Phy = namedtuple("Phy", "rtlink probes overrides")
SERVO_PROFILES = 4  # number iir coefficient profiles per servo channel
SERVO_CHANNELS = 2  # number servochannels
UPCONV_CHANNELS = 2

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

class Config(Module):
    def __init__(self, platform, regs):
        # One interface for all registers
        self.i = Endpoint([
            ("data", 32),
            ("addr",  7),
        ])
        self.o = Endpoint([
            ("data", 32),
        ])


        self.sync += [
        # self.sync.rio += [
            self.o.stb.eq(0),
            # regs["strobe"][self.i.addr[:6]].eq(0),
            If(self.i.stb,
                If(~self.i.addr[6], # READ bit is not true then assign
                    # Registers storage are in sys2x domain
                    # regs is in sys2x domain. No additional cdc is needed.
                    # sys -> sys2x
                    regs["writable"][self.i.addr[:6]].eq(self.i.data),
                    # regs["strobe"][self.i.addr[:6]].eq(1),
                ).Else(
                    # sys2x -> sys
                    # Registers read are static. No CDC is needed.
                    self.o.data.eq(regs["readable"][self.i.addr[:6]]),
                    self.o.stb.eq(1),
                )
            )
        ]

        self.comb += [
            regs["strobe"][self.i.addr[:6]].eq(self.i.stb)
        ]

class Logic(Module):
    def __init__(self, platform):
        self.phys = []
        # name, width, writable, readable, is_Strobe
        reg_map = [
            ("board_id", 8, False, True, False),
            # # hardware revision and variant
            ("hw_rev", 8, False, True, False),
            # # gateware revision
            ("gw_rev", 8, False, True, False),
            # configuration (clk_sel, dac_resetb, dac_sleep,
            # dac_txena, trf0_ps, trf1_ps, att0_rstn, att1_rstn)
            ("cfg", 8, True, True, False),
            # status (dac_alarm, trf0_ld, trf1_ld, term0_stat,
            # term1_stat, spi_idle)
            ("sta", 8, False, True, False),
            # frame crc error counter
            ("crc_err", 8, False, True, False),
            # led configuration
            # ("led", 6, True, True, False),
            # fan pwm duty cycle
            ("fan_pwm", 8, True, True, False),
            # DUC settings update strobe # ?????????? Register(write=False, read=False))
            ("duc_stb", 1, False, False, True),
            # ADC gain configuration (pgia0_gain, pgia1_gain)
            ("adc_cfg", 4, True, True, False),

            ##### SPI Replaced RTIO_SPI2

            # # spi configuration (offline, end, clk_phase, clk_polarity,
            # # half_duplex, lsb_first)
            # ("spi_cfg", 8, True, True, False),
            # # spi divider and transaction length (div(5), len(3))
            # ("spi_divlen", 8, True, True, False),
            # # spi chip select (dac, trf0, trf1, att0, att1)
            # ("spi_sel", 8, True, True, False),
            # # spi mosi data and transaction start/continue
            # ("spi_datw", 8, True, False, False),
            # # spi readback data, available after each transaction
            # ("spi_datr", 8, False, True, False),


            # dac data interface sync delay (for sync-dac_clk alignment and
            # n-div/pll/ostr fifo output synchronization)
            ("sync_dly", 3, True, True, False),
        ]

        ##### (0x10,), ##### (0x20,),
        for i in range(UPCONV_CHANNELS):
            reg_map += [
                    (f"duc{i}_cfg", 8, True, True, False),
                    # duc frequency tuning word (msb first)
                    (f"duc{i}_f", 32, True, True, False),
                    # duc phase offset word
                    (f"duc{i}_p", 16, True, True, False),
                    # dac data #### separate read only logic out <<<<
                    (f"dac{i}_data", 32 , False, True, False),
                    # dac test data for duc_cfg:data_select == 1
                    (f"dac{i}_test", 32, True, True, False),
                ]

        ##### (0x30,),
        for i in range(SERVO_CHANNELS):
            reg_map += [
                (f"servo{i}_cfg", 8, True, True, False),
                
            ]
            for j in range(5):
                reg_map += [
                    (f"ch{i}_frequency{j}", 32, True, True, False),
                    (f"ch{i}_phase_amplitude{j}", 32, True, True, False),
                ]

        # add servo data registers
        for i in range(SERVO_CHANNELS):
            for j in range(SERVO_PROFILES):
                for k in range(4):  # 3 coefficients + offset
                    reg_map.append(
                        (f"ch{i}_profile{j}_data{k}", 16, True, False, False)
                    )

        regs_writable, regs_readable, regs_strobe = [], [], []

        count = 0
        for name, width, writable, readable, is_strobe in reg_map:
            print(count, name)
            count += 1
            setattr(self, name, Signal(width))
            reg, dummy = getattr(self, name), Signal(width)

            regs_writable = regs_writable + [reg] if writable else regs_writable + [dummy]
            regs_strobe = regs_strobe + [reg] if is_strobe else regs_strobe + [dummy]
            regs_readable = regs_readable + [reg] if readable else regs_readable + [0]

        self.regs = {
            "writable": Array(regs_writable),
            "writable_list": regs_writable,
            "readable": Array(regs_readable),
            "readable_list": regs_readable,
            "strobe": Array(regs_strobe),
            "strobe_list": regs_strobe,
        }

        dac_ctrl = platform.request("dac_ctrl")
        trf_ctrl = [platform.request("trf_ctrl") for _ in range(2)]
        # TEMP self.
        self.att_rstn = [platform.request("att_rstn") for _ in range(2)]
        adc_ctrl = platform.request("adc_ctrl")
        self.comb += [
            # Sinara.boards.index("Phaser") == 19

            # This is known in compile time
            # self.readonly_regs.board_id.eq(19),
            # self.readonly_regs.hw_rev.eq(Cat(platform.request("hw_rev"), platform.request("hw_variant"))),
            # self.readonly_regs.gw_rev.eq(0x01),

            self.board_id.eq(19),
            self.hw_rev.eq(Cat(platform.request("hw_rev"), platform.request("hw_variant"))),
            self.gw_rev.eq(0x01),


            # LED is declared outside of this thing
            Cat(
                platform.request("clk_sel"),
                dac_ctrl.resetb,
                dac_ctrl.sleep,
                dac_ctrl.txena,
                trf_ctrl[0].ps,
                trf_ctrl[1].ps,
                self.att_rstn[0],
                self.att_rstn[1],
            ).eq(self.cfg),
            Cat(adc_ctrl.gain0, adc_ctrl.gain1).eq(self.adc_cfg),
            # NO CRC Checker
        ]


        #FIXME t_frame has to be 80ns
        self.submodules.interpolation = Interpolation(b_sample=14, n_channel=2, n_mux=8, t_frame=8 * 10)
        # self.submodules.interpolation = Interpolation(b_sample=14, n_channel=2, n_mux=8, t_frame=8 * 10)

        # pulse = Signal(reset=1)
        # self.sync += pulse.eq(~pulse)
        # self.comb += [
        #     # Avoid data to be clock in two times into sys2x domain
        #     self.interpolation.stb.eq(pulse & self.base.ch0.dds.valid),
        #     self.interpolation.frame.eq(self.base.output)
        # ]
        
        fan = platform.request("fan_pwm")
        fan.reset_less = True
        self.submodules.fan = PWM(fan)
        self.comb += self.fan.duty[-8:].eq(self.fan_pwm)

        ##### Replace-ED with RTIO SPI2 on Phaser
        # platform.request("dac_spi"),
        # platform.request("trf_spi", 0),
        # platform.request("trf_spi", 1),
        # platform.request("att_spi", 0),
        # platform.request("att_spi", 1),
        ####################

        # 32 ns t_cnvh, 16 ns t_conv/t_DCNVSCKL, 192 ns data transfer, 20 ns t_rtt/tDSCKLCNVH
        # Note that there is one extra cycle (4 ns) at the end of a transaction.
        # Total: 264 ns -> 3.788 MSps
        adc_parameters = AdcParams(
            width=16, channels=2, lanes=2, t_cnvh=8, t_conv=4, t_rtt=5
        )

        self.submodules.adc = adc = Adc(platform.request("adc"), adc_parameters)
        self.comb += adc.start.eq(1)

        # potential IIR improvements:
        # - Use a0=-1: invert y0 before or after clipping and flip the other coefficient signs
        # - a1 = (1 - epsilon) and pass epsilon as a 16 bit. Then use wider a1 in computation.
        # - unsigned output to gain the sign bit for data

        # log2_a0 = 14 bit for an effective fixedpoint a0 of 0.5
        self.submodules.iir = iir = Iir(
            w_coeff=16,
            w_data=16,
            log2_a0=14,
            n_profiles=SERVO_PROFILES,
            n_channels=SERVO_CHANNELS,
        )
        self.comb += [
            [inp.eq(data) for inp, data in zip(iir.inp, adc.data)],
            iir.stb_in.eq(adc.done),
        ]

        # connect iir to servo data registers
        for i in range(SERVO_CHANNELS):
            for j in range(SERVO_PROFILES):
                for k in range(3):  # 3 coefficients
                    self.comb += iir.coeff[k][j][i].eq(
                        getattr(self, f"ch{i}_profile{j}_data{k}")
                        # self.decoder.get(f"ch{i}_profile{j}_data{k}", "write")
                    )
                self.comb += iir.offset[j][i].eq(
                    getattr(self, f"ch{i}_profile{j}_data3")
                    # self.decoder.get(f"ch{i}_profile{j}_data3", "write")
                )

        # connect hold and profile select so that they update after a filter update is done
        self.sync += [
            If(
                iir.stb_out,
                # bit 0 is the ch enable bit, bit 1 the hold bit
                iir.ch_profile[0].eq((self.servo0_cfg)[2:]),
                iir.ch_profile[1].eq((self.servo1_cfg[2:])),
                iir.hold[0].eq(self.servo0_cfg[1]),
                iir.hold[1].eq(self.servo1_cfg[1]),
                
            #     iir.ch_profile[0].eq(self.decoder.get(f"servo0_cfg", "write")[2:]),
            #     iir.ch_profile[1].eq(self.decoder.get(f"servo1_cfg", "write")[2:]),
            #     iir.hold[0].eq(self.decoder.get(f"servo0_cfg", "write")[1]),
            #     iir.hold[1].eq(self.decoder.get(f"servo1_cfg", "write")[1]),
            )
        ]

        self.submodules.dac = DacData(platform.request("dac_data"))
        self.comb += [
            # sync istr counter every frame
            # this is correct since dac samples per frame is 8*20 and
            # thus divisible by the EB depth of 8.


            ####?????
            self.dac.data_sync.eq(self.interpolation.stb),
            ####?????


            self.dac.sync_dly.eq(self.sync_dly),
        ]

        for ch in range(2):
            duc = PhasedDUC(n=2, pwidth=19, fwidth=32, zl=10)
            self.submodules += duc
            cfg = getattr(self, "duc{}_cfg".format(ch))
            servo_enable = getattr(self, "servo{}_cfg".format(ch))[0]
            self.sync += [
                # keep accu cleared
                duc.clr.eq(cfg[0]),
                If(
                    # Write 1 to trigger it
                    self.duc_stb,   # OG: self.decoder.registers["duc_stb"][0].bus.we,
                    # clear accu once
                    If(
                        cfg[1],
                        duc.clr.eq(1),
                    ),
                    duc.f.eq(getattr(self, "duc{}_f".format(ch))),
                    # msb align to 19 bit duc.p
                    duc.p[3:].eq(getattr(self, "duc{}_p".format(ch))),
                ),
            ]
            for t, (ti, to) in enumerate(zip(duc.i, duc.o)):
                servo_dsp_i = Dsp()
                servo_dsp_q = Dsp()
                self.submodules += [servo_dsp_i, servo_dsp_q]

                self.comb += [

                    #### IQ LUT CONNECTION POINT # Correct Frequency
                    ti.i.eq(self.interpolation.data[t][ch].i),
                    ti.q.eq(self.interpolation.data[t][ch].q),
                    #### IQ LUT CONNECTION POINT

                    servo_dsp_i.c.eq(
                        (1 << len(self.dac.data[2 * t][ch]) - 2) - 1
                    ),  # rounding offset
                    servo_dsp_q.c.eq((1 << len(self.dac.data[2 * t][ch]) - 2) - 1),
                ]
                self.sync += [
                    If(
                        cfg[2:4] == 0,  # ducx_cfg_sel
                        self.dac.data[2 * t][ch].eq(to.i),
                        self.dac.data[2 * t + 1][ch].eq(to.q),
                    ),
                    servo_dsp_i.a.eq(to.i),
                    servo_dsp_q.a.eq(to.q),
                    servo_dsp_i.b.eq(iir.outp[ch]),
                    servo_dsp_q.b.eq(iir.outp[ch]),
                    If(
                        servo_enable,
                        self.dac.data[2 * t][ch].eq(
                            servo_dsp_i.p >> len(self.dac.data[2 * t][ch] - 1)
                        ),
                        self.dac.data[2 * t + 1][ch].eq(
                            servo_dsp_q.p >> len(self.dac.data[2 * t][ch] - 1)
                        ),
                    ),
                ]

            self.sync += [
                If(
                    cfg[2:4] == 1,  # ducx_cfg_sel
                    # i is lsb, q is msb
                    # repeat the test data to fill the oserdes
                    Cat([d[ch] for d in self.dac.data]).eq(
                        Replicate(getattr(self, "dac{}_test".format(ch)), 2)
                    ),
                ),
            ]

            # used for reading test data only
            self.comb += [
                # even sample just before the oserdes
                getattr(self, "dac{}_data".format(ch)).eq(
                    Cat(d[ch] for d in self.dac.data)
                ),
            ]



class Phaser(Module):
    """Phaser module.

    #######

    Attributes:
        phys (list): List of Endpoints.
    """
    def __init__(self, platform):
        self.phys = []


        cdr = ClockDomainsRenamer({"sys1x": "sys", "sys" : "sys2x", "sys2": "sys4x", "sys2q": "sys4x_dqs"})

        self.submodules.logic = cdr(Logic(platform))
        self.phys.extend(self.logic.phys)

        # Generate samples in 250 MHz / 10 = 25 MHz rate
        self.submodules.base = cdr(Base(ds_factor=10))
        for i in range(5):
            self.comb += [
                getattr(self.base.ch0, f"frequency{i}").eq(getattr(self.logic, f"ch0_frequency{i}")),
                getattr(self.base.ch0, f"phase_amplitude{i}").eq(getattr(self.logic, f"ch0_phase_amplitude{i}")),
                getattr(self.base.ch1, f"frequency{i}").eq(getattr(self.logic, f"ch1_frequency{i}")),
                getattr(self.base.ch1, f"phase_amplitude{i}").eq(getattr(self.logic, f"ch1_phase_amplitude{i}")),
            ]



        #####\

        # Make sure data is only sampled once in sys2x domain
        # Data is now generated in the 250MHz domain No
        # pulse = Signal(reset=1)
        # self.sync.sys2x += pulse.eq(~pulse)
        # self.comb += [
        #     self.logic.interpolation.stb.eq(self.base.ch0.dds.valid),
        #     self.logic.interpolation.frame.eq(self.base.output)
        # ]



        # connect zoh under the hood
        self.comb += [
            self.logic.interpolation.stb.eq(self.base.output_valid),
            self.logic.interpolation.frame.eq(self.base.output)
        ]


        # RUN IN RIO Domain 125MHZ
        self.submodules.cfg = Config(platform, self.logic.regs)
        cfg_rtl_iface = rtlink.Interface(
            rtlink.OInterface(
                data_width=len(self.cfg.i.data),
                address_width=len(self.cfg.i.addr),
                enable_replace=False,
            ),
            rtlink.IInterface(
                data_width=len(self.cfg.o.data),
            ),
        )
        self.comb += [
            self.cfg.i.stb.eq(cfg_rtl_iface.o.stb),
            self.cfg.i.addr.eq(cfg_rtl_iface.o.address),
            self.cfg.i.data.eq(cfg_rtl_iface.o.data),
            cfg_rtl_iface.i.stb.eq(self.cfg.o.stb),
            cfg_rtl_iface.i.data.eq(self.cfg.o.data),
        ]
        self.phys.append(Phy(cfg_rtl_iface, [], []))
