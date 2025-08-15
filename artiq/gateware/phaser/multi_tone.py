from migen import *

from misoc.cores.duc import PhasedAccu, complex
from misoc.cores.cossin import CosSinGen

from artiq.gateware.phaser.register import RW, RegisterDecoder


class NCO(Module):
    """
    n samples NCO
    """

    def __init__(self, n, f_width, p_width, iq_width):
        self.f = Signal(f_width)
        self.p = Signal(p_width)
        self.clr = Signal()

        self.source = [Record(complex(iq_width)) for _ in range(n)]

        # # #

        # Frequency word to frequency formula:
        # Freq (in Hz) =  (self.f / self.f.max) * sys clk freq (in Hz) * n
        #
        # For example:
        # - self.f = 0x00FF_FFFF, self.f.max = 0xFFFF_FFFF (32-bits)
        # - sys clk = 125 MHz, n = 4 Samples per cycle
        #
        # Freq = (0x00FF_FFFF/0xFFFF_FFFF) * 125 MHz * 4 = 1.953124884 MHz
        #
        # Details math:
        # Assume constant frequency word (self.f) and zero phase offset (self.p),
        # The PhasedAccu + CosSinGen generates:
        # ┌                                            ┐
        # │ φ1(n) = φ(n-1) + 1 * (self.f / self.f.max) │
        # │ φ2(n) = φ(n-1) + 2 * (self.f / self.f.max) │ per cycle
        # │       ⋮                                    │
        # │ φn(n) = φ(n-1) + n * (self.f / self.f.max) │
        # └                                            ┘
        # where n is the sample per cycle
        #
        # We can simply the vector as:
        # φ(n) = φ(n-1) + (self.f / self.f.max) per (cycle / n)
        #
        # Using the phase to frequency formula, we can get:
        # frequency = dφ/dt
        # => (φ(n) - φ(n-1)) / (period of sys clk / n)
        # => (self.f / self.f.max) * sys clk freq * n
        #
        self.submodules.accu = accu = PhasedAccu(n, f_width, p_width)
        self.comb += [
            accu.f.eq(self.f),
            accu.p.eq(self.p),
            accu.clr.eq(self.clr),
        ]

        generators = []
        for i in range(n):
            if i & 1:
                share_lut = generators[i - 1].lut
            else:
                share_lut = None

            # CosSinGen output width = x + 1
            csg = CosSinGen(z=len(self.accu.z[0]), x=iq_width - 1, share_lut=share_lut)
            assert len(self.source[i].i) == len(csg.x)

            generators.append(csg)
            self.sync += [
                csg.z.eq(self.accu.z[i]),
                self.source[i].i.eq(csg.x),
                self.source[i].q.eq(csg.y),
            ]
        self.submodules += generators


# TODO: move rtlink out of this module?
# TODO: improve docs
class IQPair(Module):
    """
    IQ pair with iotest
    """

    def __init__(self, n, f_width, p_width, iq_width):

        self.source = [Record(complex(iq_width)) for _ in range(n)]

        # # #

        self.submodules.nco = nco = NCO(n, f_width, p_width, iq_width)
        freq_word = Signal.like(nco.f)
        phase_word = Signal.like(nco.p)
        clr = Signal.like(nco.clr)
        self.sync.rio += [
            nco.f.eq(freq_word),
            nco.p.eq(phase_word),
            nco.clr.eq(clr),
        ]

        # Test mode
        # Expose source output directly for testing
        test_mod_en = Signal()
        test_word_i = Signal(iq_width)
        test_word_q = Signal(iq_width)
        for i, (src, src_nco) in enumerate(zip(self.source, nco.source)):
            self.sync.rio += [
                If(test_mod_en,
                    src.i.eq(test_word_i),
                    src.q.eq(test_word_q),
                ).Else(
                    src.i.eq(src_nco.i),
                    src.q.eq(src_nco.q),
                ),
            ]

        # TODO: choose a better data & address width
        reg = [
            (test_mod_en, RW),
            (test_word_i, RW),
            (test_word_q, RW),
            (freq_word, RW),
            (phase_word, RW),
            (clr, RW),
        ]
        self.submodules.reg_decoder = RegisterDecoder(reg, 32, 8) 
