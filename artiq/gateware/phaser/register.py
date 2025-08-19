from migen import *

(RW, RO, WO) = range(3)


class RegisterDecoder(Module):
    def __init__(self, register_table, data_width, address_width):
        self.sink = Record(
            [("stb", 1), ("data", data_width), ("address", address_width)]
        )
        self.source = Record([("stb", 1), ("data", data_width)])

        # # #

        assert 0 <= address_width <= 8
        # MSB is read bit
        read_bit = 1 << (address_width - 1)

        cases = {}
        addr = 0x00
        for ch, (reg, type) in enumerate(register_table):
            assert len(reg) <= data_width
            assert type in [RW, RO]
            if type in [RW, RO]:
                cases[addr | read_bit] = self.source.data.eq(reg)
            if type in [RW, WO]:
                cases[addr] = reg.eq(self.sink.data)
            addr += 1
        assert addr <= 1 << (address_width - 1)

        self.sync.rio += [
            self.source.stb.eq(0),
            If(
                self.sink.stb,
                Case(self.sink.address, cases),
                self.source.stb.eq(self.sink.address[-1]),
            ),
        ]
