from migen import *
from misoc.cores.coaxpress.common import char_width, KCode, word_width


from math import ceil
from collections import namedtuple


WordLayout = namedtuple("WordLayout", ["data", "k", "stb", "eop"])


def frame_header_generator(**kwargs):
    header = [
        WordLayout(
            data=Replicate(KCode["stream_marker"], 4),
            k=Replicate(1, 4),
            stb=1,
            eop=0,
        ),
        WordLayout(
            data=Replicate(C(0x01, char_width), 4),
            k=Replicate(0, 4),
            stb=1,
            eop=0,
        ),
    ]

    header_layout = [
        ("StreamID", char_width),
        ("SourceTag", 2 * char_width),
        ("Xsize", 3 * char_width),
        ("Xoffs", 3 * char_width),  # horizontal offset in pixels
        ("Ysize", 3 * char_width),
        ("Yoffs", 3 * char_width),  # vertical offset in pixels
        ("DsizeL", 3 * char_width), # number of data words per image line
        ("PixelF", 2 * char_width),
        ("TapG", 2 * char_width),   # tap geometry
        ("Flags", char_width),
    ]
    for f in header_layout:
        name = f[0]
        char_len = f[1] // char_width
        if name in kwargs:
            # CXP use MSB when sending duplicate chars in sequence
            bytes = kwargs.get(name).to_bytes(char_len, "big")
            chars = [C(b, char_width) for b in bytes]
        else:
            chars = [C(0x00, char_width) for _ in range(char_len)]

        for c in chars:
            print(f"{c.value:#010X}")
            header.append(
                WordLayout(
                    data=Replicate(c, 4),
                    k=Replicate(0, 4),
                    stb=1,
                    eop=0,
                ),
            )

    return header


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
                data=Replicate(KCode["stream_marker"], 4),
                k=Replicate(1, 4),
                stb=1 if stb_line_marker else 0,
                eop=0,
            ),
            WordLayout(
                data=Replicate(C(0x02, char_width), 4),
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


if __name__ == "__main__":
    frame_header_generator(
        StreamID=1,
        SourceTag=2,
        Xsize=3,
        # Xoffs=0,
        # Ysize=0,
        # Yoffs=0,
        # DsizeL=0,
        # PixelF=0,
        # TapG=0,
        # Flags=0,
    )
