from collections import namedtuple
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as sg


Plot = namedtuple("Plot", ["x", "xlabel", "y", "ylabel"])
if __name__ == "__main__":
    sys = sg.TransferFunction([1], [1, 1])
    w, mag, phase = sg.bode(sys)
    freq = w / (2 * np.pi)

    plots = [
        Plot(freq, "Frequency (Hz)", mag, "Magnitude (dB)"),
        Plot(freq, "Frequency (Hz)", phase, "Phase (degree)"),
    ]

    plt.figure()
    for i, p in enumerate(plots):
        plt.subplot(len(plots), 1, i + 1)
        plt.semilogx(p.x, p.y)  # Bode magnitude plot
        plt.grid(which="both")
        plt.ylabel(p.ylabel)
    plt.savefig("./plot.png")
