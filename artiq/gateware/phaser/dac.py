from migen import *

from operator import xor

DAC_DATA_WIDTH = 16
DAC_FIFO_DEPTH = 8
SAMPLE_PER_CYCLE = 4


class DAC34H84PHY(Module):
    """
    A 1 GSPS DAC34H84 PHY designed for 125 MHz sys clock

    Supports:
    - A 500 MHz data clock output
    - Two 1 GSPS time interleaved data outputs
    - An even parity bit output
    - FIFO pointer synchronization
    """

    def __init__(self, data_pins, ctrl_pins):
        self.alarm = Signal()
        self.en = Signal()
        self.reset_n = Signal()
        self.sleep = Signal()

        self.sink_4x_a = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]
        self.sink_4x_b = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]
        self.sink_4x_c = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]
        self.sink_4x_d = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]

        # # #

        # Control / status pins
        self.comb += [
            self.alarm.eq(ctrl_pins.alarm),
            ctrl_pins.txena.eq(self.en),
            ctrl_pins.resetb.eq(self.reset_n),
            ctrl_pins.sleep.eq(self.sleep),
        ]


        # Data clock generation
        # Using a 90 degree clock offset to sample all the data and control signals to avoid metastability
        self.submodules.data_clk_phy = data_clk_phy = TxSerializer(
            data_pins.data_clk_p, data_pins.data_clk_n, cd_4x="sys4x_dqs"
        )
        self.comb += data_clk_phy.din.eq(0b0101_0101)

        # Time interleaved data and even parity bit
        self.submodules.data_phy = data_phy = DAC34H84DataPHY(data_pins)
        self.submodules.even_parity_phy = even_parity_phy = DAC34H84EvenParityPHY(
            data_pins
        )
        for i in range(SAMPLE_PER_CYCLE):
            self.sync += [
                data_phy.sink_4x_a[i].eq(self.sink_4x_a[i]),
                data_phy.sink_4x_b[i].eq(self.sink_4x_b[i]),
                data_phy.sink_4x_c[i].eq(self.sink_4x_c[i]),
                data_phy.sink_4x_d[i].eq(self.sink_4x_d[i]),

                even_parity_phy.sink_4x_a[i].eq(self.sink_4x_a[i]),
                even_parity_phy.sink_4x_b[i].eq(self.sink_4x_b[i]),
                even_parity_phy.sink_4x_c[i].eq(self.sink_4x_c[i]),
                even_parity_phy.sink_4x_d[i].eq(self.sink_4x_d[i]),
            ]

        # FIFO pointer synchronization
        self.submodules.istr_phy = istr_phy = TxSerializer(
            data_pins.istr_parityab_p, data_pins.istr_parityab_n
        )
        # DAC PLL is synchronized by the rising edge of LVDS SYNC signal going to the N-divider circuit - SLAA584 Section 2.4
        # Which also provide an internal OSTR signal to synchronize the FIFO write pointer
        self.submodules.sync_phy = sync_phy = TxSerializer(
            data_pins.sync_p, data_pins.sync_n
        )

        # SLAS751D Section 7.3.4
        # - rising edge on the sync signal source (ISTR or SYNC) causes the pointer to return to its original position
        # - it is necessary to have the ISTR and SYNC signals to repeat at multiples of 8 FIFO samples
        # So we just raise the sync signals when the FIFO is full
         
        fifo_full_interval = DAC_FIFO_DEPTH // SAMPLE_PER_CYCLE
        write_read_counter = Signal(max=fifo_full_interval, reset=fifo_full_interval - 1)
        self.sync += [
            If(write_read_counter == 0,
                write_read_counter.eq(write_read_counter.reset),
            ).Else(write_read_counter.eq(write_read_counter - 1))
        ]
        self.comb +=[
            istr_phy.din[0].eq(write_read_counter == 0),
            sync_phy.din[0].eq(write_read_counter == 0)
        ]


class DAC34H84EvenParityPHY(Module):
    def __init__(self, pins):
        self.sink_4x_a = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]
        self.sink_4x_b = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]
        self.sink_4x_c = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]
        self.sink_4x_d = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]

        # # #

        parity_ac = Signal(SAMPLE_PER_CYCLE)
        parity_bd = Signal(SAMPLE_PER_CYCLE)

        for i in range(SAMPLE_PER_CYCLE):
            sample_a = self.sink_4x_a[i]
            sample_b = self.sink_4x_b[i]
            sample_c = self.sink_4x_c[i]
            sample_d = self.sink_4x_d[i]
            self.comb += [
                parity_ac[i].eq(
                    reduce(
                        xor, [sample_a[j] ^ sample_c[j] for j in range(DAC_DATA_WIDTH)]
                    )
                ),
                parity_bd[i].eq(
                    reduce(
                        xor, [sample_b[j] ^ sample_d[j] for j in range(DAC_DATA_WIDTH)]
                    )
                ),
            ]

        self.submodules.tx = tx = TxInterleavedSerializer(
            pins.paritycd_p, pins.paritycd_n
        )
        self.comb += [
            tx.din_0.eq(parity_ac),
            tx.din_1.eq(parity_bd),
        ]


class DAC34H84DataPHY(Module):
    def __init__(self, pins):
        assert DAC_DATA_WIDTH == len(pins.data_a_p) == len(pins.data_b_p)
        self.sink_4x_a = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]
        self.sink_4x_b = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]
        self.sink_4x_c = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]
        self.sink_4x_d = [Signal(DAC_DATA_WIDTH) for _ in range(SAMPLE_PER_CYCLE)]

        # # #

        def get_tx_bits(sink, n):
            return Cat(sink[i][n] for i in range(SAMPLE_PER_CYCLE))

        # A3, B8: p-n swapped on pcb
        inverted_pad = [(0, 3), (1, 8)]
        txpins_arg = []
        for port_i, port in enumerate(
            [(pins.data_a_p, pins.data_a_n), (pins.data_b_p, pins.data_b_n)]
        ):
            for pad_i, (pad_p, pad_n) in enumerate(zip(*port)):
                if port_i == 0:
                    sink_0, sink_1 = self.sink_4x_a, self.sink_4x_b
                else:
                    sink_0, sink_1 = self.sink_4x_c, self.sink_4x_d

                inverted = (port_i, pad_i) in inverted_pad
                p, n = (pad_n, pad_p) if inverted else (pad_p, pad_n)

                txpins_arg.append(
                    [
                        get_tx_bits(sink_0, pad_i),
                        get_tx_bits(sink_1, pad_i),
                        p,
                        n,
                        inverted,
                    ]
                )

        for sink_0_bits, sink_1_bits, pad_p, pad_n, inverted in txpins_arg:
            # From SLAS751D section 7.3.3
            # The data for channels A and B or (C and D) is interleaved in the form
            # A0[i], B0[i], A1[i], B1[i], A2[i]… into the DAB[i]P/N LVDS inputs.
            tx = TxInterleavedSerializer(pad_p, pad_n, inverted)
            self.submodules += tx
            self.comb += [
                tx.din_0.eq(sink_0_bits),
                tx.din_1.eq(sink_1_bits),
            ]


class TxSerializer(Module):
    """
    A 8-bit DDR TX serializer
    """

    def __init__(self, o_pad_p, o_pad_n, invert=False, cd_4x="sys4x"):
        self.din = Signal(8)

        # # #

        ser_out = Signal()
        t_out = Signal()
        self.specials += [
            # Serializer
            Instance(
                "OSERDESE2",
                p_DATA_RATE_OQ="DDR",
                p_DATA_RATE_TQ="BUF",
                p_DATA_WIDTH=8,
                p_TRISTATE_WIDTH=1,
                p_INIT_OQ=0x00,
                o_OQ=ser_out,
                o_TQ=t_out,
                i_RST=ResetSignal(),
                i_CLK=ClockSignal(cd_4x),
                i_CLKDIV=ClockSignal(),
                i_D1=self.din[0] ^ invert,
                i_D2=self.din[1] ^ invert,
                i_D3=self.din[2] ^ invert,
                i_D4=self.din[3] ^ invert,
                i_D5=self.din[4] ^ invert,
                i_D6=self.din[5] ^ invert,
                i_D7=self.din[6] ^ invert,
                i_D8=self.din[7] ^ invert,
                i_TCE=1,
                i_OCE=1,
                i_T1=0,
            ),
            # IOB
            Instance(
                "OBUFTDS",
                i_I=ser_out,
                i_T=t_out,
                o_O=o_pad_p,
                o_OB=o_pad_n,
            ),
        ]


class TxInterleavedSerializer(Module):
    """
    A 2x4-bit time interleaved TX serializer
    Each cycle, it sends the interleaved data as follows
    din_0[0], din_1[0], din_0[1], din_1[1], din_0[2], din_1[2], din_0[3], din_1[3]
    """

    def __init__(self, o_pad_p, o_pad_n, invert=False):
        self.submodules.serializer = serializer = TxSerializer(o_pad_p, o_pad_n, invert)
        self.din_0 = serializer.din[::2]
        self.din_1 = serializer.din[1::2]
