import unittest

from migen import *
from misoc.cores.coaxpress.common import word_width
from artiq.gateware.cxp_grabber.pixel import Pixel_Parser
from artiq.gateware.cxp_grabber.core import ROI

from math import ceil
from collections import namedtuple


WordLayout = namedtuple("WordLayout", ["data", "k", "stb", "eop"])


def mono_pixelword_generator(
    x_size,
    y_size,
    pixel_width,
    white_pixel=False,
    with_eol_marked=False,
    stb_line_marker=False,
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

        # Line marker
        packet += [
            WordLayout(
                data=C(0x7C7C7C7C, word_width),
                k=Replicate(1, 4),
                stb=1 if stb_line_marker else 0,
                eop=0,
            ),
            WordLayout(
                data=C(0x02020202, word_width),
                k=Replicate(0, 4),
                stb=1 if stb_line_marker else 0,
                eop=0,
            ),
        ]

        for i in range(words_per_image_line):
            serialized = (packed & (0xFFFF_FFFF << i * word_width)) >> i * word_width
            eop = 1 if ((i == words_per_image_line - 1) and with_eol_marked) else 0
            packet.append(
                WordLayout(
                    data=C(serialized, word_width), k=Replicate(0, 4), stb=1, eop=eop
                ),
            )

    return packet


class DUT(Module):
    def __init__(self, res_width, count_width):
        self.parser = Pixel_Parser(res_width)
        self.roi = ROI(self.parser.source_pixel4x, count_width)
        self.submodules += self.parser, self.roi


class Testbench:
    def __init__(self, res_width, count_width):
        self.dut = DUT(res_width, count_width)
        self.fragment = self.dut.get_fragment()

    def write_frame_info(self, x_size, y_size, pixel_code):
        yield self.dut.parser.x_size.eq(x_size)
        yield self.dut.parser.y_size.eq(y_size)
        yield self.dut.parser.pixel_format_code.eq(pixel_code)
        yield

    def write_frame(self, packet):
        for i, word in enumerate(packet):
            yield self.dut.parser.sink.data.eq(word.data)
            yield self.dut.parser.sink.stb.eq(word.stb)
            yield self.dut.parser.sink.eop.eq(word.eop)
            yield

        yield self.dut.parser.sink.stb.eq(0)  # prevent accidental stb

    def write_roi_cofig(self, x0, y0, x1, y1):
        yield self.dut.roi.cfg.x0.eq(x0)
        yield self.dut.roi.cfg.y0.eq(y0)
        yield self.dut.roi.cfg.x1.eq(x1)
        yield self.dut.roi.cfg.y1.eq(y1)
        yield

    def fetch_roi_output(self):
        if (yield self.dut.roi.out.update) == 1:
            return (yield self.dut.roi.out.count)
        else:
            return -1

    def delay(self, cycle):
        for _ in range(cycle):
            yield

    def run(self, gen):
        run_simulation(self.fragment, gen, vcd_name="sim-cxp.vcd")


class TestROI(unittest.TestCase):
    def test_cxp_roi(self):
        tb = Testbench(16, 31)

        pixel_code = {
            8: 0x0101,
            10: 0x0102,
            12: 0x0103,
            14: 0x0104,
            16: 0x0105,
        }

        def gen(x_size, y_size, pixel_width, x0, y0, x1, y1):
            expected_count = (x1 - x0) * (y1 - y0) * ((2**pixel_width) - 1)

            yield from tb.write_roi_cofig(x0, y0, x1, y1)

            packet = mono_pixelword_generator(
                x_size, y_size, pixel_width, white_pixel=True, with_eol_marked=True
            )
            yield from tb.write_frame_info(x_size, y_size, pixel_code[pixel_width])
            yield from tb.write_frame(packet)

            # there is a 5 cycle between stbing the last pixel word and roi update is ready
            for _ in range(5):
                yield
            self.assertEqual((yield from tb.fetch_roi_output()), expected_count)

        tb.run(gen(10, 10, 8, 0, 0, 8, 8))
        tb.run(gen(10, 10, 10, 1, 1, 8, 8))
        tb.run(gen(10, 10, 12, 2, 2, 8, 8))
        tb.run(gen(10, 10, 14, 3, 3, 8, 8))
        tb.run(gen(10, 10, 16, 4, 4, 8, 8))


if __name__ == "__main__":
    unittest.main()
