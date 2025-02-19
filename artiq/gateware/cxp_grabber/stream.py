from migen import *
from misoc.interconnect.csr import *
from misoc.interconnect.stream import Endpoint
from misoc.cores.liteeth_mini.mac.crc import LiteEthMACCRCEngine
from misoc.cores.coaxpress.common import (
    char_width,
    KCode,
    word_layout_dchar,
    word_width,
    switch_endianness,
)

from artiq.gateware.cxp_grabber.pixel import pixelword_layout

from types import SimpleNamespace


class Stream_Packet_Arbiter(Module):
    def __init__(self, routing_ids=[0]):
        n_id = len(routing_ids)
        assert n_id > 0

        self.sources = [Endpoint(word_layout_dchar) for _ in range(n_id)]
        self.sink = Endpoint(word_layout_dchar)

        # # #

        
        stream_id = Signal(char_width)
        pak_tag = Signal(char_width)
        stream_pak_size = Signal(char_width * 2)

        self.submodules.fsm = fsm = FSM(reset_state="WAIT_HEADER")

        fsm.act("WAIT_HEADER",
            self.sink.ack.eq(1),
            If(
                self.sink.stb,
                NextValue(stream_id, self.sink.dchar),
                NextState("GET_PAK_TAG"),
            ),
        )

        fsm.act("GET_PAK_TAG",
            self.sink.ack.eq(1),
            If(
                self.sink.stb,
                NextValue(pak_tag, self.sink.dchar),
                NextState("GET_PAK_SIZE_0"),
            ),
        )

        fsm.act("GET_PAK_SIZE_0",
            self.sink.ack.eq(1),
            If(
                self.sink.stb,
                NextValue(stream_pak_size[8:], self.sink.dchar),
                NextState("GET_PAK_SIZE_1"),
            ),
        )

        routing_case = {"default": NextState("DISCARD")}
        for id in (routing_ids):
            routing_case[id] = [NextState(f"COPY_TO_{id}")]

        fsm.act("GET_PAK_SIZE_1",
            self.sink.ack.eq(1),
            If(
                self.sink.stb,
                NextValue(stream_pak_size[:8], self.sink.dchar),
                Case(stream_id, routing_case),
            ),
        )

        for key in routing_case:
            if key == "default":
                fsm.act("DISCARD",
                    self.sink.ack.eq(1),
                    If(self.sink.stb,
                        NextValue(stream_pak_size, stream_pak_size - 1),
                        If(stream_pak_size == 0,
                            NextValue(stream_id, stream_id.reset),
                            NextValue(pak_tag, pak_tag.reset),
                            NextValue(stream_pak_size, stream_pak_size.reset),
                            NextState("DISCARD_K29.7"),
                        )
                    ),
                )
            else:
                fsm.act(f"COPY_TO_{key}",
                    self.sink.connect(self.sources[key], omit={"eop"}),

                    If(self.sink.stb & self.sources[key].ack,
                        NextValue(stream_pak_size, stream_pak_size - 1),
                        If(stream_pak_size == 0,
                            # mark the crc word with eop
                            self.sources[key].eop.eq(1),
                            NextValue(stream_id, stream_id.reset),
                            NextValue(pak_tag, pak_tag.reset),
                            NextValue(stream_pak_size, stream_pak_size.reset),
                            NextState("DISCARD_K29.7"),
                        )
                    ),
                
                )

        fsm.act("DISCARD_K29.7",
            self.sink.ack.eq(1),
            If(self.sink.stb, NextState("WAIT_HEADER")),
        )


@ResetInserter()
@CEInserter()
class CXPCRC32(Module):
    # Section 9.2.2.2 (CXP-001-2021)
    width = 32
    polynom = 0x04C11DB7
    seed = 2**width - 1
    check = 0x00000000

    def __init__(self, data_width):
        self.data = Signal(data_width)
        self.value = Signal(self.width)
        self.error = Signal()

        # # #

        self.submodules.engine = LiteEthMACCRCEngine(
            data_width, self.width, self.polynom
        )
        reg = Signal(self.width, reset=self.seed)
        self.sync += reg.eq(self.engine.next)
        self.comb += [
            self.engine.data.eq(self.data),
            self.engine.last.eq(reg),
            self.value.eq(reg[::-1]),
            self.error.eq(reg != self.check),
        ]


class CXPCRC32_Checker(Module):
    def __init__(self):
        self.error = Signal()

        self.sink = Endpoint(word_layout_dchar)
        self.source = Endpoint(word_layout_dchar)

        # # #

        self.submodules.crc = crc = CXPCRC32(word_width)
        self.comb += crc.data.eq(self.sink.data),

        self.submodules.fsm = fsm = FSM(reset_state="INIT")
        fsm.act("INIT",
            crc.reset.eq(1),
            NextState("CHECKING"),
        )

        fsm.act("RESET",
            crc.reset.eq(1),
            self.error.eq(crc.error),
            NextState("CHECKING"),
        )

        fsm.act("CHECKING",
            If(self.sink.stb & self.sink.eop,
                # discard the crc
                self.sink.ack.eq(1),
                NextState("RESET"),
            ).Else(
                self.sink.connect(self.source),
            ),
            crc.ce.eq(self.sink.stb & self.source.ack),
        )
    
        
class Frame_Header_Reader(Module):
    def __init__(self):
        self.decode_err = Signal()

        self.new_frame = Signal()

        # # #

        # Table 47 (CXP-001-2021)
        n_header_chars = 23
        img_header_layout = [
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
        assert layout_len(img_header_layout) == n_header_chars * char_width

        self.sink = Endpoint(word_layout_dchar)
        self.source = Endpoint(pixelword_layout)

        # # #


        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            self.sink.ack.eq(1),
            If((self.sink.stb & (self.sink.dchar == KCode["stream_marker"]) & (self.sink.dchar_k == 1)),
                NextState("DECODE"),
            )
        )

        fsm.act("COPY",
            # until for new line or new frame
            If((self.sink.stb & (self.sink.dchar == KCode["stream_marker"]) & (self.sink.dchar_k == 1)),
                self.sink.ack.eq(1),
                NextState("DECODE"),
            ).Else(
                self.sink.connect(self.source, omit={"k", "dchar", "dchar_k"}),
            )
        )

        type = {
            "new_frame": 0x01,
            "line_break": 0x02,
        }

        cnt = Signal(max=n_header_chars)
        fsm.act("DECODE",
            self.sink.ack.eq(1),
            If(self.sink.stb,
                Case(self.sink.dchar, {
                    type["new_frame"]: [
                        NextValue(cnt, cnt.reset),
                        NextState("GET_FRAME_DATA"),
                    ],
                    type["line_break"]: [
                        NextState("COPY"),
                    ],
                    "default": [
                         self.decode_err.eq(1),
                         # discard all data until valid frame header
                         NextState("IDLE"),
                    ],
                }),
            )
        )

        packet_buffer = Signal(layout_len(img_header_layout))
        case = dict(
            (i, NextValue(packet_buffer[8*i:8*(i+1)], self.sink.dchar))
            for i in range(n_header_chars)
        )
        fsm.act("GET_FRAME_DATA",
            self.sink.ack.eq(1),
            If(self.sink.stb,
                Case(cnt, case),
                If(cnt == n_header_chars - 1,
                    self.new_frame.eq(1),
                    NextState("COPY"),
                    NextValue(cnt, cnt.reset),
                ).Else(
                    NextValue(cnt, cnt + 1),
                ),
            ),
        )

        # dissect packet 
        self.header = SimpleNamespace()
        idx = 0
        for name, size in img_header_layout:
            # CXP also use MSB when sending duplicate chars in sequence
            setattr(self.header, name, switch_endianness(packet_buffer[idx:idx+size]))
            idx += size

class End_of_line_Marker(Module):
    def __init__(self):
        # Assume words_per_img_line arrive at least one cycle before pixel data
        self.words_per_img_line = Signal(3*char_width)

        self.sink = Endpoint(pixelword_layout)
        self.source = Endpoint(pixelword_layout) 
        
        # # #

        cnt = Signal.like(self.words_per_img_line, reset=1)
        self.sync += [
            If(self.source.ack,
                self.sink.connect(self.source, omit={"ack", "eop"}),
                If(self.sink.stb,
                    If(cnt == 1,
                        cnt.eq(self.words_per_img_line)
                    ).Else(
                        cnt.eq(cnt - 1),
                    )
                ),
            ),
        ]
        self.comb += [
            self.sink.ack.eq(self.source.ack),
            # repurpose eop as end of line
            self.source.eop.eq(cnt == 1),
        ]
