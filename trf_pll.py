from numpy import int32, int64
from artiq.language.types import *
from artiq.language.units import us, GHz, MHz, ns


def main():
    f_ref = 125

    R = 4  # 1 - 8191
    Fractional = 40
    PLL_DIV = 2
    P = 4  # 4 or 8
    OUT_DIV = 1  # 1, 2, 4, 8

    f_pfd = f_ref / R

    f_pm = f_pfd * Fractional
    f_vco = f_pm * PLL_DIV
    f_lo = f_vco / OUT_DIV
    f_N = f_pm / P

    print(
        f"R = {R} | Fractional = {Fractional} | PLL_DIV = {PLL_DIV} | P = {P} | OUT_DIV = {OUT_DIV}"
    )
    print(
        f"f_pdf = {f_pfd} | f_vco = {f_vco} | f_pm = {f_pm} | f_N = {f_N} | f_lo = {f_lo}"
    )

    print(
        f"""
                                      {f_pfd:<10}                {f_vco}
                   ┌───────┐          MHz                       MHz
     refclk───────>│ div R │───────>┌──────┐     ┌──────┐     ┌───────┐          ┌──────┐
     {f_ref:<11}   └───────┘        │ PDF  │────>│ LPF  │────>│  VCO  │────┬────>│OUTDIV│───> {f_lo} MHz
     MHz                       ┌───>└──────┘     └──────┘     └───────┘    │     └──────┘
                               │                                           │
                               │    ┌───────┐                 ┌───────┐    │
                               └────│ NFRAC │<─────────┬──────│PLL DIV│<───┘
                                    └───────┘          │      └───────┘             
                                                       │{f_pm} MHz                         
                                                       │      ┌───────┐                        
                                                       └─────>│ div P │───> {f_N} MHz                  
                                                              └───────┘      
          """
    )

    if f_pfd > 100:
        print("WARNING: PDF freq too high MAX 100 MHz")
    if not (4800 > f_vco > 2400):
        print("WARNING: VCO out of range 4800 - 2400 MHz")

    if f_pm > 3000:
        print("WARNING: prescalar freq too high MAX 3000 MHz")

    if f_N > 375:
        print("WARNING: N freq too high MAX 375 MHz")


TRF_MAX_VCO_FREQ = 4.8 * GHz
TRF_MIN_VCO_FREQ = 2.4 * GHz
TRF_MAX_N_FREQ = 375 * MHz
TRF_MAX_PM_FREQ = 3 * GHz
TRF_MAX_PFD_FREQ = 100 * MHz


class testbench:
    def __init__(self):
        self.refclk = 125 * MHz

    def set_frequency(self, frequency):

        if frequency > TRF_MAX_VCO_FREQ:
            raise ValueError("Requested frequency too high")

        # select minimal output divider
        lo_div_sel = 0
        f_vco = frequency
        while f_vco < TRF_MIN_VCO_FREQ:
            f_vco *= 2
            lo_div_sel += 1

        if (1 << lo_div_sel) > 8:
            raise ValueError("Requested frequency too low")

        # assume prescaler is 4/5
        # n_min, n_max = 23, 75
        # prescaler = 4

        for n_min, n_max, prescaler in [(23, 75, 4), (75, 1 << 16, 8)]:
            # NOTE: should I cal f_pm first or f_pfd first...
            # => f_pm first, I will get the largest f_pm possible
            # As f_pdf * N = f_pm, f_pdf will be the largest
            #
            # f_n = f_vco / (pll_div * P) = f_pm / P
            # f_pfd = f_vco / (pll_div * N) = f_pm / N
            #
            # f_pdf * N = f_pm
            # f_n * P = f_pm = f_vco / pll_div

            pll_div_sel = 0
            f_pm = f_vco
            while (f_pm / prescaler) > TRF_MAX_N_FREQ or f_pm > TRF_MAX_PM_FREQ:
                f_pm /= 2
                pll_div_sel += 1

            r_div = 1
            f_pfd = self.refclk
            n_int, n_frac = self.calculate_n_divider(f_pm, f_pfd)
            while n_int < n_min or f_pfd > TRF_MAX_PFD_FREQ:
                r_div += 1
                f_pfd = self.refclk / r_div
                n_int, n_frac = self.calculate_n_divider(f_pm, f_pfd)

            if n_int < n_max:
                print(f"n_int < n_max {n_int} < {n_max}")
                break

        print(
            f"R = {r_div} | NINT = {n_int} NFRAC = {n_frac} | PLL_DIV_SEL = {pll_div_sel} | LO_DIV_SEL = {lo_div_sel} | Prescalar = {prescaler}"
        )

        print(
            f"f_pdf = {self.get_freq_str(f_pfd)} |\
 f_pm = {self.get_freq_str(f_pm)} |\
 f_N = {self.get_freq_str(f_pm / prescaler)} |\
 f_vco = {self.get_freq_str(f_vco)} |\
 f_lo = {self.get_freq_str(frequency)}"
        )

    def calculate_n_divider(self, f_pm, f_pfd):
        # print(
        #     f"f_pdf = {self.get_freq_str(f_pfd)} |\
        #  f_vco = {self.get_freq_str(f_vco)} |"
        # )
        return int32(f_pm // f_pfd), int32(((f_pm / f_pfd) % 1.0) * float(1 << 25))

    def get_freq_str(self, freq):
        return str(freq / (1 * MHz)) + " MHz"


tb = testbench()
for f in range(300, 4801, 100):
    print("---------------------------------")
    tb.set_frequency(f * MHz)

print( 1 / (1 * MHz))
