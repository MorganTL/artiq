import unittest

from migen import *
from misoc.cores.coaxpress.common import word_width
from artiq.gateware.cxp_grabber.pixel import Pixel_Parser
from artiq.gateware.cxp_grabber.core import ROI

from math import ceil

_pixel_code = {
    8: 0x0101,
    10: 0x0102,
    12: 0x0103,
    14: 0x0104,
    16: 0x0105,
}


def mono_pixelword_generator(
    x_size,
    y_size,
    pixel_width,
    white_pixel=False,
    with_line_marker=False,
    with_eol_marked=False,
):
    words_per_image_line = ceil(x_size * pixel_width / word_width)
    gray = 0
    packet = []
    for _ in range(y_size):
        packed = 0
        for x in range(x_size):
            if white_pixel:
                gray = (2**pixel_width) - 1
            else:
                gray += 1
            packed += gray << x * pixel_width

        # new line indicator
        # TODO: change this to record??? reference packet interface from test/drtio
        if with_line_marker:
            packet += [
                {"data": C(0x7C7C7C7C, word_width), "k": Replicate(1, 4)},
                {"data": C(0x02020202, word_width), "k": Replicate(0, 4)},
            ]
        for i in range(words_per_image_line):
            serialized = (packed & (0xFFFF_FFFF << i * word_width)) >> i * word_width
            print(f"{serialized:#010X}")
            if (i == words_per_image_line - 1) and with_eol_marked:
                packet.append(
                    {"data": C(serialized, word_width), "k": Replicate(0, 4), "eop": 1}
                )
            else:
                packet.append({"data": C(serialized, word_width), "k": Replicate(0, 4)})

    return packet


class DUT(Module):
    def __init__(self, res_width, count_width):
        self.parser = Pixel_Parser(res_width)
        self.roi = ROI(self.parser.source_pixel4x, count_width)
        self.submodules += self.parser, self.roi


class Testbench:
    def __init__(self, res_width, count_width):
        self.dut = DUT(res_width, count_width)

    def write_frame_data(self, x_size, y_size, pixel_code):
        yield self.dut.parser.x_size.eq(x_size)
        yield self.dut.parser.y_size.eq(y_size)
        yield self.dut.parser.pixel_code.eq(pixel_code)
        yield

    def write_line(self, packet):
        for i, word in enumerate(packet):
            if i in [0, 1]:
                # simulate line marker between line
                yield
            yield self.dut.parser.sink.data.eq(word["data"])
            yield self.dut.parser.sink.stb.eq(1)
            # TODO: update this when changing to record or something better
            if "eop" in word:
                yield self.dut.parser.sink.eop.eq(1)
            else:
                yield self.dut.parser.sink.eop.eq(0)

            yield
        yield self.parser.stb.eq(0)  # present accidental stb

    def write_frame(self, packet):
        for i, word in enumerate(packet):
            yield self.dut.parser.sink.data.eq(word["data"])
            yield self.dut.parser.sink.stb.eq(1)
            # TODO: update this when changing to record or something better
            if "eop" in word:
                yield self.dut.parser.sink.eop.eq(1)
                yield
                # simulated the 2 cycle delay after linebreak
                yield self.dut.parser.sink.stb.eq(0)
                yield self.dut.parser.sink.eop.eq(0)
                yield
                yield
            else:
                yield self.dut.parser.sink.eop.eq(0)
                yield

        yield self.dut.parser.sink.stb.eq(0)  # present accidental stb
        yield
        yield
        yield
        yield

    def write_roi_cofig(self, x0, y0, x1, y1):
        yield self.dut.roi.cfg.x0.eq(x0)
        yield self.dut.roi.cfg.y0.eq(y0)
        yield self.dut.roi.cfg.x1.eq(x1)
        yield self.dut.roi.cfg.y1.eq(y1)
        yield

    def fetch_roi_output(self):
        if (yield self.dut.roi.out.update) == 1:
            return (yield self.dut.roi.out.gray)
        else:
            return -1

    def test_roi(self, x_size, y_size, pixel_width, x0, y0, x1, y1):
        yield from self.write_roi_cofig(x0, y0, x1, y1)
        packet = mono_pixelword_generator(
            x_size, y_size, pixel_width, white_pixel=True, with_eol_marked=True
        )
        yield from self.write_frame_data(x_size, y_size, _pixel_code[pixel_width])
        yield from self.write_frame(packet)

    def run(self, gen):
        run_simulation(self.dut, gen, vcd_name="sim-cxp.vcd")


if __name__ == "__main__":
    # mono_pixelword_generator(14, 5, 4)
    tb = Testbench(16, 31)

    def gen():
        yield from tb.test_roi(10, 10, 8, 1, 1, 4, 4)

    tb.run(gen())
    pass
