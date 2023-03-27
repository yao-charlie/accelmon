import argparse
import numpy as np
import scipy as sp
import scipy.signal as signal
from matplotlib import pyplot as plt

if __name__ == "__main__":
     
    # parser = argparse.ArgumentParser(description='Test frequency analysis')
    # parser.add_argument('-s','--sample-rate',type=float, help='Sample rate (ksps)')
    # parser.add_argument('datafile', type=str, help='filename (csv) for raw data')
    # args = parser.parse_args()

    class TestArgs:
        def __init__(self, L, D):
            self.datafile = 'examples/test2khz_sin_0p1_l{}.csv'.format(L)
            self.SAMPLEN = L
            self.DIV = D

            T_sys = 1/48000000.
            T_gclk = T_sys * self.DIV
            N_conv = 7 + 0.5*self.SAMPLEN
            self.T_conv = N_conv*T_gclk*4.0

    args = TestArgs(10,50)

    csv = np.genfromtxt(args.datafile, delimiter=",")
    
    ndrop = int(0.1/args.T_conv)  # remove 100ms

    v = csv[ndrop:] * 3.3/4095.0
    v -= np.average(v)
 
    
    Ts = args.T_conv
    Twindow = 0.1
    nperwindow = int(Twindow/Ts)

    print("sample rate: {} ksps, Ts={} us".format(0.001/Ts, Ts*1e6))

    fv, tv, vv = signal.stft(v, fs=1.0/Ts, nperseg=nperwindow)

    # plt.pcolormesh(tv, fv, np.log10(np.abs(vv)), shading='gouraud')
    # plt.title('STFT Magnitude')
    # plt.ylabel('Frequency [Hz]')
    # plt.xlabel('Time [sec]')
    # plt.show()


    zz = sp.fft.fftshift(sp.fft.fft(v))
    ff = sp.fft.fftshift(sp.fft.fftfreq(v.size, d=Ts))
    plt.plot(ff, np.abs(zz))
    plt.title("SAMPLEN={}, DIV={}, T_conv={:.2f}us, R={:.2f}ksps".format(args.SAMPLEN, args.DIV, args.T_conv*1e6, 0.001/args.T_conv))
    plt.xlabel("frequency (Hz)")
    plt.ylabel("magnitude (a.u.)")
    plt.ylim(0, 60)
    plt.show()


    


