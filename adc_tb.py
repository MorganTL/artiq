from migen import *

from artiq.gateware.phaser.adc_phy import *


class DUT(Module):
    def __init__(self):
        self.submodules.clk = LTC2323PHY(None, None, 125e6)


class TestBench:
    def __init__(self):
        self.dut = DUT()
        self.fragment = self.dut.get_fragment()

    def setup(self):
        for _ in range(100):
            yield

    def run(self, gen):
        run_simulation(self.fragment, gen, clocks={"sys": 8}, vcd_name="testbench.vcd")


if __name__ == "__main__":
    tb = TestBench()
    tb.run(tb.setup())

    # period = 10
    # sys_clk_freq = 125e6
    # max_count = (ADC_DATA_WIDTH * period) / (1e9 / sys_clk_freq)

    # sck = 0
    # for _ in range(ADC_DATA_WIDTH):
    #     sck = sck << period | 0b11111_00000

    # sck_case = {}
    # for i in range(int(max_count)):
    #     sck_case[i] = sck & ((1 << 8) - 1)
    #     sck >>= 8

    # for key in sck_case.keys():
    #     print(f"{key:02}: 0b{sck_case[key]:08b}")
