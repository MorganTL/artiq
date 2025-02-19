from migen import *
from misoc.interconnect.csr import *
from misoc.interconnect.stream import Endpoint
from misoc.cores.coaxpress.common import char_width, word_width

from math import lcm

max_pixel_width = 16
# the pixel data don't include any K code nor duplicate char
pixelword_layout = [("data", word_width)]


class Pixel_Unpacker(Module):
    """
    Unpack 32 bits words into 4x pixel

    Assume:
    - x_size arrive at least one cycle before any pixel data
    - the last pixel word is marked with eop 

    Only support:
    - Pixel format: mono8, mono10, mono12, mono14, mono16
    """
    def __init__(self, size):
        assert size <= max_pixel_width
        assert size in [8, 10, 12, 14, 16]

        self.x_size = Signal(3*char_width)

        self.sink = Endpoint(pixelword_layout)
        self.source = Endpoint(
            [
                ("data", max_pixel_width * 4),
                ("valid", 4),
            ]
        )

        # # #

        sink_dw, source_dw = layout_len(pixelword_layout), size*4
        ring_buf_size = lcm(sink_dw, source_dw)
        # ensure the shift register is at least twice the size of sink/source dw
        if (ring_buf_size//sink_dw) < 2:
            ring_buf_size = ring_buf_size * 2
        if (ring_buf_size//source_dw) < 2:
            ring_buf_size = ring_buf_size * 2

        # Control interface

        reset_reg = Signal()
        we = Signal()
        re = Signal()
        level = Signal(max=ring_buf_size)
        w_cnt = Signal(max=ring_buf_size//sink_dw)
        r_cnt = Signal(max=ring_buf_size//source_dw)

        self.sync += [
            If(reset_reg,
                level.eq(level.reset),
            ).Else(
                If(we & ~re, level.eq(level + sink_dw)),
                If(~we & re, level.eq(level - source_dw)),
                If(we & re, level.eq(level + sink_dw - source_dw)),
            ),

            If(reset_reg,
                w_cnt.eq(w_cnt.reset),
                r_cnt.eq(r_cnt.reset),
            ).Else(
                If(we, 
                    If(w_cnt == ((ring_buf_size//sink_dw) - 1),
                        w_cnt.eq(w_cnt.reset),
                    ).Else(
                        w_cnt.eq(w_cnt + 1),
                    )
                ),
                If(re, 
                    If(r_cnt == ((ring_buf_size//source_dw) - 1),
                        r_cnt.eq(r_cnt.reset),
                    ).Else(
                        r_cnt.eq(r_cnt + 1),
                    )
                ),
            )
        ]
         
        extra_eol_handling = size in [10, 12, 14]
        if extra_eol_handling:
            # the source need to be stb twice
            # (one for level >= source_dw and the other for the remaining pixels)
            # when last word of each line packet satisfied the following condition:
            # 
            # if there exist an integers j such that
            # sink_dw * i > size * j > source_dw * k  
            # where i,k are postive integers and source_dw * k - sink_dw * (i-1) > 0
            # 
            stb_aligned = Signal()
            match size: 
                case 10:
                    # For example size == 10
                    # 32 * 2 > 10 * (5) > 40 * 1
                    # 32 * 2 > 10 * (6) > 40 * 1
                    # 32 * 3 > 10 * (9) > 40 * 2
                    # ...
                    # 
                    # the packing pattern for size == 10 repeat every 16 pixels
                    # the remaining special case can be taken care off using modulo operation
                    stb_cases = {
                        5: stb_aligned.eq(1),
                        6: stb_aligned.eq(1),
                        9: stb_aligned.eq(1),
                    }
                    self.sync += Case(self.x_size[:4], stb_cases) # mod 16
                case 12:
                    stb_cases = {
                        5: stb_aligned.eq(1),
                    }
                    self.sync += Case(self.x_size[:3], stb_cases) # mod 8
                case 14:
                    stb_cases = {
                        9: stb_aligned.eq(1),
                        13: stb_aligned.eq(1),
                    }
                    self.sync += Case(self.x_size[:4], stb_cases) # mod 16



        self.submodules.fsm = fsm = FSM(reset_state="SHIFTING")
        fsm.act(
            "SHIFTING",
            self.sink.ack.eq(1),
            self.source.stb.eq(level >= source_dw),
            we.eq(self.sink.stb),
            re.eq((self.source.stb & self.source.ack)),
            If(self.sink.stb & self.sink.eop,
                (If(stb_aligned,
                    NextState("MOVE_ALIGNED_PIX"),
                ).Else(
                    NextState("MOVE_REMAINING_PIX"),
                ) if extra_eol_handling else
                    NextState("MOVE_REMAINING_PIX"),
                )
            ),
        )

        if extra_eol_handling:
            fsm.act(
                "MOVE_ALIGNED_PIX",
                self.source.stb.eq(1),
                re.eq((self.source.stb & self.source.ack)),
                NextState("MOVE_REMAINING_PIX"),
            )

        stb_remaining_pix = Signal()
        fsm.act(
            "MOVE_REMAINING_PIX",
            reset_reg.eq(1),
            self.source.stb.eq(1),
            stb_remaining_pix.eq(1),
            NextState("SHIFTING"),
        )

        # Data path
         
        ring_buf = Signal(ring_buf_size, reset_less=True)

        sink_cases = {}
        for i in range(ring_buf_size//sink_dw):
            sink_cases[i] = [
                ring_buf[sink_dw*i:sink_dw*(i+1)].eq(self.sink.data),
            ]
        self.sync += If(self.sink.stb, Case(w_cnt, sink_cases))

        source_cases = {}
        for i in range(ring_buf_size//source_dw):
            source_cases[i] = []
            for j in range(4):
                source_cases[i].append(
                    self.source.data[max_pixel_width * j : max_pixel_width * (j + 1)].eq(
                        ring_buf[(source_dw * i) + (size * j) : (source_dw * i) + (size * (j + 1))]
                    )
                )

        # calcule which last pixels are valid
        valid = Signal(4)
        bit_cases = {
            0: valid.eq(0b1111),
            1: valid.eq(0b0001),
            2: valid.eq(0b0011),
            3: valid.eq(0b0111),
        }
        self.sync += Case(self.x_size[:2], bit_cases)

        self.comb += [
            Case(r_cnt, source_cases),
            If(stb_remaining_pix,
                self.source.valid.eq(valid),
                self.source.eop.eq(1),
            ).Else(
                self.source.valid.eq(0b1111),
            ),
        ]


class Pixel_Coordinate_Tracker(Module):
    """
    Track and append 4x pixel with xy coordinates

    Assume:
    - y_size arrive at least one cycle before any pixel data
    - camera is in area scan mode
    - 1X-1Y Tap geometry
    """
    def __init__(self, res_width):
        # largest x/y pixel size supported by frame header are 24 bits
        assert res_width <= 3*char_width

        # line scaning frame will have y_size = 0 and won't trigger the end of frame bit
        self.y_size = Signal(3*char_width)
        self.sink = Endpoint(
            [
                ("data", max_pixel_width * 4),
                ("valid", 4),
            ]
        )

        # # #
        
        self.pixel4x = []
        for _ in range(4):
            self.pixel4x.append(Record([
                ("x", res_width),
                ("y", res_width),
                ("gray", max_pixel_width),
                ("stb", 1),
                ("eof", 1), # end of frame
            ]))

        x_4x = [Signal(len(self.pixel4x[0].x), reset=i) for i in range(4)]
        y_r = Signal(len(self.pixel4x[0].y))

        y_max = Signal.like(self.y_size)
        self.sync += [
            self.sink.ack.eq(1),
            y_max.eq(self.y_size - 1),
        ]
        for i, (x_r, pix) in enumerate(zip(x_4x, self.pixel4x)):
            self.sync += [
                pix.stb.eq(0),
                pix.eof.eq(0),
                If(self.sink.stb,
                    If(self.sink.eop,
                        # new line
                        x_r.eq(x_r.reset),

                        If(y_r == y_max,
                            pix.eof.eq(1),
                            y_r.eq(y_r.reset),
                        ).Else(
                            y_r.eq(y_r + 1),
                        )
                    ).Else(
                        x_r.eq(x_r + 4),
                    ),
                    pix.stb.eq(self.sink.valid[i]),
                    pix.x.eq(x_r),
                    pix.y.eq(y_r),
                    pix.gray.eq(self.sink.data[max_pixel_width*i:max_pixel_width*(i+1)]),
                )
            ]


class Pixel_Parser(Module):
    """
    Prase 32 bit pixel word into 4x pixel with xy coordinate

    Only support:
    - Pixel format: mono8, mono10, mono12, mono14, mono16
    - Tap geometry: 1X-1Y
    - Scaning mode: area scanning

    """
    def __init__(self, res_width):
        self.x_size = Signal(3 * char_width)
        self.y_size = Signal(3 * char_width)
        self.pixel_format_code = Signal(2 * char_width)

        self.sink = Endpoint(pixelword_layout)

        # # #
         
        # 
        #          32                  4x pixel
        # sink  ───/───┬──> 8  bits ──┬───/───> pixel coordinate ─────> 4x pixel with
        #              ├──> 10 bits ──┤         tracker                 xy coordinate
        #              ├──> 12 bits ──┤
        #              ├──> 14 bits ──┤
        #              └──> 16 bits ──┘
        #                pixel unpacker
        # 


        # From Table 34 (CXP-001-2021)
        pixel_formats = {
            "mono8": 0x0101,
            "mono10": 0x0102,
            "mono12": 0x0103,
            "mono14": 0x0104,
            "mono16": 0x0105,
        }

        unpackers = {}
        for s in [8, 10, 12, 14, 16]:
            unpacker = Pixel_Unpacker(s)
            unpackers["mono"+str(s)] = unpacker
            self.submodules += unpacker
            self.sync += unpacker.x_size.eq(self.x_size),


        self.submodules.tracker = tracker = Pixel_Coordinate_Tracker(res_width)
        self.sync += tracker.y_size.eq(self.y_size)

        # discard unknown pixel format
        mux_cases = {"default": [self.sink.ack.eq(1)]}
        for fmt, code in pixel_formats.items():
            mux_cases[code] = [
                self.sink.connect(unpackers[fmt].sink),
                unpackers[fmt].source.connect(tracker.sink),
            ]

        self.comb += Case(self.pixel_format_code, mux_cases)

        self.source_pixel4x = tracker.pixel4x


