import unittest

from migen import *
from misoc.cores.coaxpress.common import word_width
from artiq.gateware.cxp_grabber.pixel import Pixel_Parser

from math import ceil

_pixel_code = {
    8: 0x0101,
    10: 0x0102,
    12: 0x0103,
    14: 0x0104,
    16: 0x0105,
}


def mono_pixelword_generator(pixel_width, x_size, y_size, white_pixel=False):
    words_per_image_line = ceil(x_size * pixel_width / word_width)
    gray = 0
    for _ in range(y_size):
        packed = 0
        for x in range(x_size):
            if white_pixel:
                gray = (2 ^ pixel_width) - 1
            else:
                gray += 1
            packed += gray << x * pixel_width

        # new line indicator
        packet = [
            {"data": C(0x7C7C7C7C, word_width), "k": Replicate(1, 4)},
            {"data": C(0x02020202, word_width), "k": Replicate(0, 4)},
        ]
        for i in range(words_per_image_line):
            serialized = (packed & (0xFFFF_FFFF << i * word_width)) >> i * word_width
            print(f"{serialized:08X}")
            packet.append({"data": C(serialized, word_width), "k": Replicate(0, 4)})

    return packet


class Testbench:
    def __init__(self, res_width):
        self.input = []
        self.dut = Pixel_Parser(res_width)

    def write_config(self, x_size, y_size, pixel_format):
        yield self.dut.x_size.eq(x_size)
        yield self.dut.y_size.eq(y_size)
        yield self.dut.pixel_format.eq(pixel_format)
        yield

    def write_word(self, word):
        yield self.dut.sink.data.eq(word)
        yield

    def fetch_pixels(self):
        yield self.dut.source_pixel4x


if __name__ == "__main__":
    mono_pixelword_generator(14, 5, 4)
    pass
