import argparse
import numpy as np
import scipy as sp
import scipy.signal as signal
from matplotlib import pyplot as plt
import time

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
    nshiftstep = nperwindow // 2
    nsteps = ((v.size - nperwindow) // nshiftstep) + 1
    Tstep = nshiftstep*Ts
    
    print("sample rate: {} ksps, Ts={} us".format(0.001/Ts, Ts*1e6))

    spectrogram = np.zeros((np.min([nsteps,int(1.0/Tstep)+1]), nperwindow ))
    spectrum = np.zeros((nperwindow, 1))

    zmaxz = 2
    ww = signal.windows.tukey(nperwindow)
    ff = sp.fft.fftshift(sp.fft.fftfreq(nperwindow, d=Ts))

    fig, axs = plt.subplots(2,1)
    line, = axs[0].plot([], lw=3)
    text = axs[0].text(0.1*ff.max(),0.9*zmaxz, "")
    img = axs[1].imshow(spectrogram, vmin=-1, vmax=1,
                        extent=[ff.min(), ff.max(), 1.0, 0],
                        origin='upper',
                        aspect='auto',
                        interpolation="None", cmap="inferno")


    axs[0].set_xlim(ff.min(), ff.max())
    axs[0].set_ylim([0, 1.1*zmaxz])
    #axs[0].set_title('STFT Magnitude')
    axs[0].set_ylabel('|S| (a.u.)')
    axs[1].set_xlabel('f (Hz)')
    #axs[1].set_xlim(ff.min(), ff.max())
    #axs[1].set_ylim([0, 1.1*zmaxz])

    fig.canvas.draw()   # note that the first draw comes before setting data 

    # cache the background
    blit = True
    if blit:
        ax0_background = fig.canvas.copy_from_bbox(axs[0].bbox)
        ax1_background = fig.canvas.copy_from_bbox(axs[1].bbox)
    plt.show(block=False)

    ti = 0.
    istart = 0
    istop = nperwindow
    for i in range(nsteps):
        zz = sp.fft.fftshift(sp.fft.fft(v[istart:istop]*ww))
        for k in range(spectrogram.shape[0]-1,0,-1):
            spectrogram[k,:] = spectrogram[k-1,:]
        spectrogram[0,:] = zz**2

        img.set_data(spectrogram)
        line.set_data(ff, np.abs(zz))
        tx = 't={}s'.format(ti) 
        text.set_text(tx)
        
        if blit:
            fig.canvas.restore_region(ax0_background)
            fig.canvas.restore_region(ax1_background)
            axs[0].draw_artist(line)
            axs[0].draw_artist(text)
            axs[1].draw_artist(img)

            fig.canvas.blit(axs[0].bbox)
            fig.canvas.blit(axs[1].bbox)
        else:
            fig.canvas.draw()

        fig.canvas.flush_events()
        
        ti += Tstep
        istart += nshiftstep
        istop += nshiftstep
        time.sleep(Tstep)

    # ww = signal.windows.boxcar(nperwindow)
    # zz = sp.fft.fftshift(sp.fft.fft(v[0:ww.size]))
    # ff = sp.fft.fftshift(sp.fft.fftfreq(ww.size, d=Ts))
    # plt.plot(ff, np.abs(zz))
    # plt.show()

    # fv, tv, vv = signal.stft(v, fs=1.0/Ts, nperseg=nperwindow)

    # print(fv.size)
    # print(tv.size)
    # print(vv.shape)

    # plt.plot(fv, np.abs(vv[:,1]))
    # plt.show()


    # fig, ax = plt.subplots()
    # line, = ax.plot([], lw=3)
    # text = ax.text(0.1*fv.max(),0.9*np.abs(vv).max(), "")
    
    # ax.set_xlim(fv.min(), fv.max())
    # ax.set_ylim([0, 1.1*np.abs(vv).max()])
    # ax.set_title('STFT Magnitude')
    # ax.set_ylabel('|S| (a.u.)')
    # ax.set_xlabel('f (Hz)')

    # fig.canvas.draw()   # note that the first draw comes before setting data 

    # # cache the background
    # ax_background = fig.canvas.copy_from_bbox(ax.bbox)
    # plt.show(block=False)

    # for i, ti in enumerate(tv):
    #     line.set_data(fv, np.abs(vv[:,i]))
    #     tx = 't={}s'.format(ti) 
    #     text.set_text(tx)
        
    #     fig.canvas.restore_region(ax_background)
    #     ax.draw_artist(line)
    #     ax.draw_artist(text)

    #     fig.canvas.blit(ax.bbox)
    #     fig.canvas.flush_events()

    #     time.sleep(0.1)

    # plt.pcolormesh(tv, fv, np.abs(vv), shading='gouraud')
    # plt.show()

    # zz = sp.fft.fftshift(sp.fft.fft(v))
    # ff = sp.fft.fftshift(sp.fft.fftfreq(v.size, d=Ts))
    # plt.plot(ff, np.abs(zz))
    # plt.title("SAMPLEN={}, DIV={}, T_conv={:.2f}us, R={:.2f}ksps".format(args.SAMPLEN, args.DIV, args.T_conv*1e6, 0.001/args.T_conv))
    # plt.xlabel("frequency (Hz)")
    # plt.ylabel("magnitude (a.u.)")
    # plt.ylim(0, 60)
    # plt.show()


    


