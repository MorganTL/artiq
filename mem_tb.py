from migen import *
from migen.genlib.coding import Decoder

from artiq.gateware.phaser.adc_phy import *


class DUT(Module):
    def __init__(self, dw, addr_width):
        cfg = [
            ("b0", (dw, True)),
            ("a1", (dw, True)),
            ("b1", (dw, True)),
            ("offset", (dw, True)),
        ]  # a0 is determined by fractional_width

        self.addr_r = Signal(addr_width)
        self.config_r = Record(cfg)
        self.specials.cfg_mem = cfg_mem = Memory(len(self.config_r), addr_width)
        # readback
        self.specials.wide_mem_port = wide_mem_port = cfg_mem.get_port()
        self.comb += [
            self.config_r.raw_bits().eq(wide_mem_port.dat_r),
            wide_mem_port.adr.eq(self.addr_r),
        ]
        # write
        self.addr_w = Signal(addr_width + len(cfg))
        self.config_w = Signal(dw)
        self.specials.smol_mem_port = smol_mem_port = cfg_mem.get_port(
            write_capable=True, we_granularity=dw
        )
        # TODO: finish the decoder properly (esp the 0, 1, 2 etc...)
        # need a priority decoder?
        #

        self.submodules.addr_decoder = addr_decoder = Decoder(len(cfg))
        
        bit_mask = log2_int(len(cfg))
        self.comb += [
            addr_decoder.i.eq(self.addr_w[:bit_mask]),
            # TODO: improve the bit masking
            smol_mem_port.adr.eq(self.addr_w[bit_mask:]),
            smol_mem_port.we.eq(addr_decoder.o),
            smol_mem_port.dat_w.eq(Replicate(self.config_w, len(cfg))),
        ]


class TestBench:
    def __init__(self):
        self.dut = DUT(14, 8)
        self.fragment = self.dut.get_fragment()

    # TODO: follow RTServoMem
    # TODO: fix only b0 is writen to
    def setup(self):
        base_addr = 0b0100
        yield self.dut.addr_r.eq(base_addr >> 4)
        for i, cfg in enumerate(["b0", "a1", "b1", "offset"]):
            yield self.dut.config_w.eq(10 << i)
            yield self.dut.addr_w.eq(base_addr | i)
            yield self.dut.addr_r.eq(base_addr >> 2)
            yield
        yield # make sure data are stb into mem 
        yield from self.print_config_r()

    def print_config_r(self):
        s = "reading config"
        for c in ["b0", "a1", "b1", "offset"]:
            s += f" {c} = {(yield getattr(self.dut.config_r, c))} |"
        print(s)

    def run(self, gen):
        run_simulation(self.fragment, gen, clocks={"sys": 8}, vcd_name="testbench.vcd")


if __name__ == "__main__":
    tb = TestBench()
    tb.run(tb.setup())
