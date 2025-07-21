#!/usr/bin/env python3

import argparse

from migen import *
from migen.build.generic_platform import *

from misoc.cores import gpio, spi2
from misoc.targets.phaser import BaseSoC
from misoc.integration.builder import builder_args, builder_argdict

from artiq.gateware.amp import AMPSoC
from artiq.gateware import rtio
from artiq.gateware.rtio.xilinx_clocking import fix_serdes_timing_path
from artiq.gateware.rtio.phy import ttl_simple
from artiq.gateware.rtio.phy import spi2 as rtio_spi
from artiq.gateware.drtio.transceiver import eem_serdes
from artiq.gateware.drtio.rx_synchronizer import NoRXSynchronizer
from artiq.gateware.drtio import *
from artiq.gateware.phaser.phaser import Phaser
from artiq.build_soc import *

# TEMP
from migen.genlib.io import DifferentialOutput


class Satellite(BaseSoC, AMPSoC):
    mem_map = {
        "rtio":          0x20000000,
        "drtioaux":      0x50000000,
        "mailbox":       0x70000000
    }
    mem_map.update(BaseSoC.mem_map)

    def __init__(self, gateware_identifier_str=None, **kwargs):
        BaseSoC.__init__(self,
            cpu_type="vexriscv",
            cpu_bus_width=64,
            sdram_controller_type="minicon",
            l2_size=128*1024,
            l2_line_size=64,
            clk_freq=125e6,
            **kwargs)
        AMPSoC.__init__(self)
        add_identifier(self, gateware_identifier_str=gateware_identifier_str)

        platform = self.platform

        drtio_eem_io = [
            ("drtio_tx", 0,
                Subsignal("p", Pins("eem0:d0_cc_p eem0:d1_p eem0:d2_p eem0:d3_p")),
                Subsignal("n", Pins("eem0:d0_cc_n eem0:d1_n eem0:d2_n eem0:d3_n")),
                IOStandard("LVDS_25"),
            ),
            ("drtio_rx", 0,
                Subsignal("p", Pins("eem0:d4_p eem0:d5_p eem0:d6_p eem0:d7_p")),
                Subsignal("n", Pins("eem0:d4_n eem0:d5_n eem0:d6_n eem0:d7_n")),
                IOStandard("LVDS_25"), Misc("DIFF_TERM=TRUE"),
            ),
        ]

        platform.add_extension(drtio_eem_io)
        data_pads = [
            (platform.request("drtio_rx"), platform.request("drtio_tx"))
        ]

        test_eem_output = [
            ("test_eem_out", 0,
                Subsignal("p", Pins("eem1:d0_cc_p eem1:d1_p eem1:d2_p eem1:d3_p eem1:d4_p eem1:d5_p eem1:d6_p eem1:d7_p")),
                Subsignal("n", Pins("eem1:d0_cc_n eem1:d1_n eem1:d2_n eem1:d3_n eem1:d4_n eem1:d5_n eem1:d6_n eem1:d7_n")),
                IOStandard("LVDS_25"),
            ),
        ]
        platform.add_extension(test_eem_output)
        test_eem_pads = platform.request("test_eem_out")


        self.submodules.eem_transceiver = eem_serdes.EEMSerdes(self.platform, data_pads)
        self.csr_devices.append("eem_transceiver")
        self.config["HAS_DRTIO_EEM"] = None
        self.config["EEM_DRTIO_COUNT"] = 1

        self.submodules.rtio_tsc = rtio.TSC(glbl_fine_ts_width=3)

        cdr = ClockDomainsRenamer({"rtio_rx": "sys"})
        core = cdr(DRTIOSatellite(
            self.rtio_tsc, self.eem_transceiver.channels[0],
            NoRXSynchronizer()))
        self.submodules.drtiosat = core
        self.csr_devices.append("drtiosat")

        self.submodules.drtioaux0 = cdr(DRTIOAuxController(
            core.link_layer, self.cpu_dw))
        self.csr_devices.append("drtioaux0")

        drtio_aux_mem_size = 1024 * 16 # max_packet * 8 buffers * 2 (tx, rx halves)
        memory_address = self.mem_map["drtioaux"]
        self.add_wb_slave(memory_address, drtio_aux_mem_size, self.drtioaux0.bus)
        self.add_memory_region("drtioaux0_mem", memory_address | self.shadow_base, drtio_aux_mem_size)

        self.config["HAS_DRTIO"] = None
        self.add_csr_group("drtioaux", ["drtioaux0"])
        self.add_memory_group("drtioaux_mem", ["drtioaux0_mem"])

        # Async reset gateware if data lane is idle
        self.comb += self.crg.reset.eq(self.eem_transceiver.rst)

        fix_serdes_timing_path(platform)

        self.config["DRTIO_ROLE"] = "satellite"
        self.config["RTIO_FREQUENCY"] = "125.0"
        
        self.rtio_channels = []
        print("User LED at RTIO channel 0x{:06x}".format(len(self.rtio_channels)))
        for i in range(5):
            user_led = self.platform.request("user_led", i)
            phy = ttl_simple.Output(user_led)
            self.submodules += phy
            self.rtio_channels.append(rtio.Channel.from_phy(phy))
 
        # FIXME All registers in sys2x are static. There is no CDC problem
        platform.add_false_path_constraints(self.crg.cd_sys.clk, self.crg.cd_sys2x.clk)
        platform.add_false_path_constraints(self.crg.cd_sys2x.clk, self.crg.cd_sys.clk)

        self.submodules.phaser = Phaser(self.platform)
        print("PHASER Config at RTIO channel 0x{:06x}".format(len(self.rtio_channels)))
        self.rtio_channels.extend(rtio.Channel.from_phy(phy) for phy in self.phaser.phys)

        dac_spi = rtio_spi.SPIMaster(self.platform.request("dac_spi"))
        self.submodules += dac_spi
        print("PHASER DAC_SPI at RTIO channel 0x{:06x}".format(len(self.rtio_channels)))
        self.rtio_channels.append(rtio.Channel.from_phy(dac_spi))

        att_spi_pads = []

        trf0_spi = rtio_spi.SPIMaster(self.platform.request("trf_spi", 0))
        self.submodules += trf0_spi
        print("PHASER TRF0 at RTIO channel 0x{:06x}".format(len(self.rtio_channels)))
        self.rtio_channels.append(rtio.Channel.from_phy(trf0_spi))

        att_spi_pads.append(self.platform.request("att_spi", 0))
        att_spi_pads.append(self.platform.request("att_spi", 1))

        att0_spi = rtio_spi.SPIMaster(att_spi_pads[0])
        self.submodules += att0_spi
        print("PHASER ATT0 at RTIO channel 0x{:06x}".format(len(self.rtio_channels)))
        self.rtio_channels.append(rtio.Channel.from_phy(att0_spi))

        trf1_spi = rtio_spi.SPIMaster(self.platform.request("trf_spi", 1))
        self.submodules += trf1_spi
        print("PHASER TRF1 at RTIO channel 0x{:06x}".format(len(self.rtio_channels)))
        self.rtio_channels.append(rtio.Channel.from_phy(trf1_spi))

        att1_spi = rtio_spi.SPIMaster(att_spi_pads[1])
        self.submodules += att1_spi
        print("PHASER ATT1 at RTIO channel 0x{:06x}".format(len(self.rtio_channels)))
        self.rtio_channels.append(rtio.Channel.from_phy(att1_spi))


        # for i in range(2):
        #     setattr(self, f"trf{i}_spi", rtio_spi.SPIMaster(
        #         platform.request("trf_spi", i)
        #     ))
        #     self.submodules += getattr(self, f"trf{i}_spi")
        #     print("PHASER TRF{:01x} at RTIO channel 0x{:06x}".format(i, len(self.rtio_channels)))
        #     self.rtio_channels.append(rtio.Channel.from_phy(getattr(self, f"trf{i}_spi")))

        #     att_spi_pads.append(platform.request("att_spi", i))

        #     setattr(self, f"att{i}_spi", rtio_spi.SPIMaster(
        #         att_spi_pads[i]
        #     ))
        #     self.submodules += getattr(self, f"att{i}_spi")
        #     print("PHASER ATT{:01x} at RTIO channel 0x{:06x}".format(i, len(self.rtio_channels)))
        #     self.rtio_channels.append(rtio.Channel.from_phy(getattr(self, f"att{i}_spi")))

        att0_spi_pads = att_spi_pads[0]

        self.specials += [
            DifferentialOutput(att0_spi_pads.clk, test_eem_pads.p[0], test_eem_pads.n[0]),
            DifferentialOutput(att0_spi_pads.miso, test_eem_pads.p[1], test_eem_pads.n[1]),
            DifferentialOutput(att0_spi_pads.mosi, test_eem_pads.p[2], test_eem_pads.n[2]),
            DifferentialOutput(att0_spi_pads.cs_n, test_eem_pads.p[3], test_eem_pads.n[3]),
            DifferentialOutput(self.phaser.logic.att_rstn[0], test_eem_pads.p[4], test_eem_pads.n[4]),
            DifferentialOutput(self.phaser.base.ch0.dds.valid, test_eem_pads.p[5], test_eem_pads.n[5]),
            DifferentialOutput(self.phaser.logic.interpolation.stb, test_eem_pads.p[6], test_eem_pads.n[6]),
            DifferentialOutput(self.phaser.logic.interpolation.zoh.sample_stb, test_eem_pads.p[7], test_eem_pads.n[7]),
        ]

        # self.comb += [
        #     att0_spi_pads.clk.eq(test_eem_pads[0]),
        #     att0_spi_pads.miso.eq(test_eem_pads[1]),
        #     att0_spi_pads.mosi.eq(test_eem_pads[2]),
        #     att0_spi_pads.cs_n.eq(test_eem_pads[2]),
        # ]


        error_led = self.platform.request("user_led", 5)
        self.submodules.error_led = gpio.GPIOOut(Cat(error_led))
        self.csr_devices.append("error_led")
        self.config["HAS_ERROR_LED"] = None

        self.config["HAS_RTIO_LOG"] = None
        self.config["RTIO_LOG_CHANNEL"] = len(self.rtio_channels)
        self.rtio_channels.append(rtio.LogChannel())

        self.add_rtio(self.rtio_channels)

    def add_rtio(self, rtio_channels, sed_lanes=8):
        # Only add MonInj core if there is anything to monitor
        if any([len(c.probes) for c in rtio_channels]):
            self.submodules.rtio_moninj = rtio.MonInj(rtio_channels)
            self.csr_devices.append("rtio_moninj")

        # satellite (master-controlled) RTIO
        self.submodules.local_io = SyncRTIO(self.rtio_tsc, rtio_channels, lane_count=sed_lanes)
        self.comb += [ 
            self.drtiosat.async_errors.eq(self.local_io.async_errors),
            self.local_io.sed_spread_enable.eq(self.drtiosat.sed_spread_enable.storage)
        ]

        # subkernel RTIO
        self.submodules.rtio = rtio.KernelInitiator(self.rtio_tsc)
        self.register_kernel_cpu_csrdevice("rtio")

        self.submodules.rtio_dma = rtio.DMA(self.get_native_sdram_if(), self.cpu_dw)
        self.csr_devices.append("rtio_dma")
        self.submodules.cri_con = rtio.CRIInterconnectShared(
            [self.drtiosat.cri, self.rtio_dma.cri],
            [self.local_io.cri],
            enable_routing=True)
        self.csr_devices.append("cri_con")
        self.submodules.routing_table = rtio.RoutingTableAccess(self.cri_con)
        self.csr_devices.append("routing_table")

        self.submodules.rtio_analyzer = rtio.Analyzer(self.rtio_tsc, self.local_io.cri,
                                                self.get_native_sdram_if(), cpu_dw=self.cpu_dw)
        self.csr_devices.append("rtio_analyzer")


def main():
    parser = argparse.ArgumentParser(
        description="ARTIQ device binary builder for Phaser")
    builder_args(parser)
    parser.set_defaults(output_dir="artiq_phaser")
    parser.add_argument("-V", "--variant", default="phaser")
    parser.add_argument("--gateware-identifier-str", default=None,
                        help="Override ROM identifier")
    args = parser.parse_args()

    argdict = dict()
    soc = Satellite(**argdict)
    build_artiq_soc(soc, builder_argdict(args))


if __name__ == "__main__":
    main()
