# -*- coding: utf-8 -*-
"""
Created on Fri Jun 14 20:36:01 2013
SineTools.py
auxiliary functions related to sine-approximation

@author: bruns01
"""

import scipy as sp
from scipy import linalg as la
import matplotlib.pyplot as mp

def sampletimes(Fs, T):  #
    """
    generate a t_i vector with \n
    sample rate Fs \n
    from 0 to T
    """
    num = sp.ceil(T * Fs)
    return sp.linspace(0, T, num, dtype=sp.float64)


# a = displacement, velocity or acceleration amplitude
def sinewave(f, a, phi, ti, offset=0, noise=0, absnoise=0, drift=0, ampdrift=0):
    """
    generate a sampled sine wave s_i = s(t_i) with \n
    amplitude a \n
    initial phase phi \n
    sample times t_i \n
    bias offset (default 0)\n
    noise as multiple of the amplitude in noise level \n
    absnoise as a additive noise component \n
    drift as multiples of amplitude per duration in drifting zero \n
    ampdrift as a drifting amplitude given as multiple of amplitude \n
    """
    Tau = ti[-1] - ti[0]
    n = 0
    n = 0
    if noise != 0:
        n = a * noise * sp.randn(len(ti))
    if absnoise != 0:
        n = n + absnoise * sp.randn(len(ti))

    d = drift * a / Tau

    s = (
        a * (1 + ampdrift / Tau * ti) * sp.sin(2 * sp.pi * f * ti - phi)
        + n
        + d * ti
        + offset
    )
    return s


def fm_counter_sine(
    fm, f, x, phi, ti, offset=0, noise=0, absnoise=0, drift=0, ampdrift=0, lamb=633.0e-9
):
    """
    calculate counter value of heterodyne signal at \n
    carrier freq. fm\n
    x = displacement amplitude \n
    initial phase phi \n
    sample times t_i \n
    bias or offset (default 0)\n
    noise as multiple of the amplitude in noise level \n
    absnoise as a additive noise component \n
    drift as multiples of amplitude per duration in drifting zero \n
    ampdrift as a drifting amplitude given as multiple of amplitude \n
    lamb as wavelength of Laser
    """
    Tau = ti[-1] - ti[0]
    n = 0
    if noise != 0:
        n = x * noise * sp.randn(len(ti))
    if absnoise != 0:
        n = n + absnoise * sp.randn(len(ti))

    d = drift * x / Tau

    s = (
        1.0
        / lamb
        * (
            x * (1 + ampdrift / Tau * ti) * sp.sin(2 * sp.pi * f * ti - phi)
            + n
            + d * ti
            + offset
        )
    )
    s = sp.floor(s + fm * ti)

    return s


# sine fit at known frequency
def threeparsinefit(y, t, f0):
    """
    sine-fit at a known frequency\n
    y vector of sample values \n
    t vector of sample times\n
    f0 known frequency\n
    \n
    returns a vector of coefficients [a,b,c]\n
    for y = a*sin(2*pi*f0*t) + b*cos(2*pi*f0*t) + c
    """
    w0 = 2 * sp.pi * f0

    a = sp.array([sp.cos(w0 * t), sp.sin(w0 * t), sp.ones(t.size)])

    abc = la.lstsq(a.transpose(), y)
    return abc[0][0:3]  ## fit vector a*sin+b*cos+c


# sine fit at known frequency and detrending
def threeparsinefit_lin(y, t, f0):
    """
    sine-fit with detrending at a known frequency\n
    y vector of sample values \n
    t vector of sample times\n
    f0 known frequency\n
    \n
    returns a vector of coefficients [a,b,c,d]\n
    for y = a*sin(2*pi*f0*t) + b*cos(2*pi*f0*t) + c*t + d
    """
    w0 = 2 * sp.pi * f0

    a = sp.array([sp.cos(w0 * t), sp.sin(w0 * t), sp.ones(t.size), t, sp.ones(t.size)])

    abc = la.lstsq(a.transpose(), y)
    return abc[0][0:4]  ## fit vector


def calc_threeparsine(abc, t, f0):
    """
    return y = abc[0]*sin(2*pi*f0*t) + abc[1]*cos(2*pi*f0*t) + abc[2]
    """
    w0 = 2 * sp.pi * f0
    return abc[0] * sp.cos(w0 * t) + abc[1] * sp.sin(w0 * t) + abc[2]


def amplitude(abc):
    """
    return the amplitude given the coefficients of\n
    y = a*sin(2*pi*f0*t) + b*cos(2*pi*f0*t) + c
    """
    return sp.absolute(abc[1] + 1j * abc[0])


def phase(abc, deg=False):
    """
    return the (sine-)phase given the coefficients of\n
    y = a*sin(2*pi*f0*t) + b*cos(2*pi*f0*t) + c \n
    returns angle in rad by default, in degree if deg=True
    """
    return sp.angle(abc[1] + 1j * abc[0], deg=deg)


def magnitude(A1, A2):
    """
    return the magnitude of the complex ratio of sines A2/A1\n
    given two sets of coefficients \n
    A1 = [a1,b1,c1]\n
    A2 = [a2,b2,c2]
    """
    return amplitude(A2) / amplitude(A1)


def phase_delay(A1, A2, deg=False):
    """
    return the phase difference of the complex ratio of sines A2/A1\n
    given two sets of coefficients \n
    A1 = [a1,b1,c1]\n
    A2 = [a2,b2,c2]\n
    returns angle in rad by default, in degree if deg=True
    """
    return phase(A2, deg=deg) - phase(A1, deg=deg)


# periodical sinefit at known frequency
def seq_threeparsinefit(y, t, f0,periods=1):
    """
    period-wise sine-fit at a known frequency\n
    y vector of sample values \n
    t vector of sample times\n
    f0 known frequency\n
    \n
    returns a (n,3)-matrix of coefficient-triplets [[a,b,c], ...]\n
    for y = a*sin(2*pi*f0*t) + b*cos(2*pi*f0*t) + c
    """
    Tau = 1.0 / f0
    dt = t[1] - t[0]
    N = int(Tau / dt) *periods  ## samples per section
    M = int(sp.floor(t.size / N))  ## number of sections or periods

    abc = sp.zeros((M, 3))

    for i in range(int(M)):
        ti = t[i * N : (i + 1) * N]
        yi = y[i * N : (i + 1) * N]
        abc[i, :] = threeparsinefit(yi, ti, f0)
    return abc  ## matrix of all fit vectors per period


# four parameter sine-fit (with frequency approximation)
def fourparsinefit(y, t, f0, tol=1.0e-7, nmax=1000):
    """
    y sampled data values \n
    t sample times of y \n
    f0 estimate of sine frequency \n
    tol rel. frequency correction where we stop \n
    nmax maximum number of iterations taken \n
    \n
    returns the vector [a, b, c, w] of  a*sin(w*t)+b*cos(w*t)+c
    """
    abcd = threeparsinefit(y, t, f0)
    w = 2 * sp.pi * f0
    err = 1
    i = 0
    while (err > tol) and (i < nmax):
        D = sp.array(
            [
                sp.cos(w * t),
                sp.sin(w * t),
                sp.ones(t.size),
                (-1.0) * abcd[0] * t * sp.sin(w * t) + abcd[1] * t * sp.cos(w * t),
            ]
        )

        abcd = (la.lstsq(D.transpose(), y))[0]
        dw = abcd[3]
        w = w + 0.9 * dw
        i += 1
        err = sp.absolute(dw / w)

    assert i < nmax, "iteration error"

    return sp.hstack((abcd[0:3], w / (2 * sp.pi)))


def calc_fourparsine(abcf, t):
    """
    return y = abc[0]*sin(2*pi*f0*t) + abc[1]*cos(2*pi*f0*t) + abc[2]
    """
    w0 = 2 * sp.pi * abcf[3]
    return abcf[0] * sp.cos(w0 * t) + abcf[1] * sp.sin(w0 * t) + abcf[2]


"""
from octave ...
function abcw = fourParSinefit(data,w0)
  abc = threeParSinefit(data,w0);
  a=abc(1);
  b=abc(2);
  c=abc(3);
  w = w0;
  
  do 
  D = [sin(w.*data(:,1)) , cos(w.*data(:,1)) , ones(rows(data),1) , a.*data(:,1).*cos(w.*data(:,1)) - b.*data(:,1).*sin(w.*data(:,1)) ];
  
  s = D \ data(:,2);
  dw = s(4);
  w = w+0.9*dw;
  err = abs(dw/w);

  until (err < 1.0e-8 );
  
  abcw = [s(1),s(2),s(3),w];
  
endfunction
"""

# periodical sinefit at known frequency
def seq_fourparsinefit(y, t, f0, tol=1.0e-7, nmax=1000, debug_plot=False,periods=1):
    """
    period-wise sine-fit at a known frequency\n
    y vector of sample values \n
    t vector of sample times\n
    f0 estimate of excitation frequency\n
    nmax maximum of iteration to improve f0 \n
    debug_plot Flag for plotting the sequential fit for dubugging \n
     \n
    returns a (n,3)-matrix of coefficient-triplets [[a,b,c], ...]\n
    for y = a*sin(2*pi*f0*t) + b*cos(2*pi*f0*t) + c
    """
    Tau = 1.0 / f0
    dt = t[1] - t[0]
    N = int(sp.floor(Tau / dt))*periods  ## samples per section
    M = int(sp.floor(t.size / N))  ## number of sections or periods

    abcd = sp.zeros((M, 4))

    for i in range(M):
        ti = t[i * N : (i + 1) * N]
        yi = y[i * N : (i + 1) * N]
        abcd[i, :] = fourparsinefit(yi, ti, f0, tol=tol, nmax=nmax)

    if debug_plot:
        mp.ioff()
        fig = mp.figure("seq_fourparsinefit")
        fig.clear()
        p1 = fig.add_subplot(211)
        p2 = fig.add_subplot(212, sharex=p1)

        for i in range(M):
            p1.plot(t[i * N : (i + 1) * N], y[i * N : (i + 1) * N], ".")
            s = calc_fourparsine(
                abcd[i, :], t[i * N : (i + 1) * N]
            )  # fitted data to plot
            p1.plot(t[i * N : (i + 1) * N], s, "-")
            r = y[i * N : (i + 1) * N] - s  # residuals to plot
            p2.plot(t[i * N : (i + 1) * N], r, ".")
            yi = y[i * N : (i + 1) * N]
        mp.show()

    return abcd  ## matrix of all fit vectors per period


# fitting a pseudo-random multi-sine signal with 2*Nf+1 parameters
def multi_threeparsinefit(y, t, f0):  # fo vector of frequencies
    """
    fit a time series of a sum of sine-waveforms with a given set of frequencies\n
    y vector of sample values \n
    t vector of sample times\n
    f0 vector of known frequencies\n
    \n
    returns a vector of coefficient-triplets [a,b,c] for the frequencies\n
    for y = sum_i (a_i*sin(2*pi*f0_i*t) + b_i*cos(2*pi*f0_i*t) + c_i
    """
    w0 = 2 * sp.pi * f0
    # set up design matrix
    a = sp.ones(len(t))
    for w in w0:
        a = sp.vstack((sp.vstack((sp.cos(w * t), sp.sin(w * t))), a))

    abc = sp.linalg.lstsq(a.transpose(), y)
    return abc[0]  ## fit vector a*sin+b*cos+c


def multi_amplitude(abc):  # abc = [a1,b1 , a2,b2, ...,bias]
    """
    return the amplitudes given the coefficients of a multi-sine\n
    abc = [a1,b1 , a2,b2, ...,bias] \n
    y = sum_i (a_i*sin(2*pi*f0_i*t) + b_i*cos(2*pi*f0_i*t) + c_i
    """
    x = abc[0::2] + 1j * abc[1::2]
    return sp.absolute(x)


def multi_phase(abc, deg=False):  # abc = [bias, a1,b1 , a2,b2, ...]
    """
    return the initial phases given the coefficients of a multi-sine\n
    abc = [a1,b1 , a2,b2, ...,bias] \n
    y = sum_i (a_i*sin(2*pi*f0_i*t) + b_i*cos(2*pi*f0_i*t) + c_i
    """
    x = abc[1::2] + 1j * abc[2::2]
    return sp.angle(x, deg=deg)


def multi_waveform_abc(f, abc, t):
    """
    generate a sample time series of a multi-sine from coefficients and frequencies\n
    f vector of given frequencies \n
    abc = [a1,ba, a2,b2, ..., bias]\n
    t vector of sample times t_i\n
    \n
    returns the vector \n
    y = sum_i (a_i*sin(2*pi*f0_i*t) + b_i*cos(2*pi*f0_i*t) + bias
    """
    ret = 0.0 * t + abc[-1]  # bias
    for fi, a, b in zip(f, abc[0::2], abc[1::2]):
        ret = ret + a * sp.cos(2 * sp.pi * fi * t) + b * sp.sin(2 * sp.pi * fi * t)
    return ret


##################################
# Counter based stuff

# periodical sinefit to the linearly increasing heterodyne counter
# version based on Blume
def seq_threeparcounterfit(y, t, f0, diff=False):
    """
    period-wise (single-)sinefit to the linearly increasing heterodyne counter
    version based on "Blume et al. "\n
    y vector of sampled counter values
    t vector of sample times
    f given frequency\n
    \n
    returns (n,3)-matrix of coefficient-triplets [a,b,c] per period \n
    
    if diff=True use differentiation to remove carrier (c.f. source)
    """
    Tau = 1.0 / f0
    dt = t[1] - t[0]
    N = int(sp.floor(Tau / dt))  ## samples per section
    M = int(sp.floor(t.size / N))  ## number of sections or periods

    remove_counter_carrier(y, diff=diff)

    abc = sp.zeros((M, 4))

    for i in range(int(M)):
        ti = t[i * N : (i + 1) * N]
        yi = y[i * N : (i + 1) * N]

        abc[i, :] = threeparsinefit_lin(yi, ti, f0)
    return abc  ## matrix of all fit vectors per period


def remove_counter_carrier(y, diff=False):
    """
    remove the linear increase in the counter signal
    generated by the carrier frequency of a heterodyne signal\n
    y vector of samples of the signal
    """
    if diff:
        d = sp.diff(y)
        d = d - sp.mean(d)
        y = sp.hstack((0, sp.cumsum(d)))
    else:
        slope = y[-1] - y[0]  # slope of linear increment
        y = y - slope * sp.linspace(
            0.0, 1.0, len(y), endpoint=False
        )  # removal of linear increment
    return y


# calculate displacement and acceleration to the same analytical s(t)
# Bsp: fm = 2e7, f=10, s0=0.15, phi0=sp.pi/3, ti, drift=0.03, ampdrift=0.03,thd=[0,0.02,0,0.004]
def disp_acc_distorted(fm, f, s0, phi0, ti, drift=0, ampdrift=0, thd=0):
    """
    calculate the respective (displacement-) counter and acceleration
    for a parmeterized distorted sine-wave motion in order to compare accelerometry with interferometry \n
    fm is heterodyne carrier frequency (after mixing)\n
    f is mechanical sine frequency (nominal) \n
    phi_0 accelerometer phase delay \n
    ti vector of sample times \n
    drift is displacement zero drift \n
    ampdrift is displacement amplitude druft\n
    thd is vector of higher harmonic amplitudes (c.f. source)
    """
    om = 2 * sp.pi * f
    om2 = om ** 2
    tau = ti[-1] - ti[0]
    disp = sp.sin(om * ti + phi0)
    if thd != 0:
        i = 2
        for h in thd:
            disp = disp + h * sp.sin(i * om * ti + phi0)
            i = i + 1
    if ampdrift != 0:
        disp = disp * (1 + ampdrift / tau * ti)
    if drift != 0:
        disp = disp + s0 * drift / tau * ti
    disp = disp * s0
    disp = sp.floor((disp * 2 / 633e-9) + fm * ti)

    acc = -s0 * om2 * (1 + ampdrift / tau * ti) * sp.sin(om * ti + phi0)
    if ampdrift != 0:
        acc = acc + (2 * ampdrift * s0 * om * sp.cos(om * ti + phi0)) / tau
    if thd != 0:
        i = 2
        for h in thd:
            acc = acc - s0 * h * om2 * (1 + ampdrift / tau * ti) * i ** 2 * sp.sin(
                i * om * ti + phi0
            )
            if ampdrift != 0:
                acc = (
                    acc
                    + (2 * ampdrift * s0 * om * i * h * sp.cos(om * ti + phi0)) / tau
                )
            i = i + 1

    return disp, acc


###################################
# Generation and adaptation of Parameters of the Multi-Sine considering hardware constraints
def PR_MultiSine_adapt(
    f1,
    Nperiods,
    Nsamples,
    Nf=8,
    fs_min=0,
    fs_max=1e9,
    frange=10,
    log=True,
    phases=None,
    sample_inkr=1,
):
    """
    Returns an additive normalized Multisine time series. \n
    f1 = start frequency (may be adapted) \n
    Nperiods = number of periods of f1 (may be increased) \n
    Nsamples = Minimum Number of samples  \n
    Nf = number of frequencies in multi frequency mix \n
    fs_min = minimum sample rate of used device (default 0) \n
    fs_max = maximum sample rate of used device (default 0) \n
    frange = range of frequency as a factor relative to f1 (default 10 = decade) \n
    log = boolean for logarithmic (True, default) or linear (False) frequency scale \n
    phases = float array of given phases for the frequencies (default=None=random) \n
    deg= boolean for return phases in deg (True) or rad (False) \n
    sample_inkr = minimum block of samples to add to a waveform
    \n
    returns: freq,phase,fs,ti,multi \n
    freq= array of frequencies \n
    phase=used phases in deg or rad \n
    fs=sample rate \n
    ti=timestamps \n
    multi=array of time series values \n
    """
    if (
        Nsamples // sample_inkr * sample_inkr != Nsamples
    ):  # check multiplicity of sample_inkr
        Nsamples = (
            Nsamples // sample_inkr + 1
        ) * sample_inkr  # round to next higher multiple

    T0 = Nperiods / f1  # given duration
    fs0 = Nsamples / T0  # (implicitly) given sample rate

    if False:
        print("0 Nperiods: " + str(Nperiods))
        print("0 Nsamples: " + str(Nsamples))
        print("0 fs: " + str(fs0))
        print("0 T0: " + str(T0))
        print("0 f1: " + str(f1))

    fs = fs0
    if fs0 < fs_min:  # sample rate too low, then set to minimum
        fs = fs_min
        print("sample rate increased")
    elif fs0 > fs_max:  # sample rate too high, set to max-allowed and
        fs = fs_max
        Nperiods = sp.ceil(
            Nperiods * fs0 / fs_max
        )  # increase number of periods to get at least Nsamples samples
        T0 = Nperiods / f1
        print("sample rate reduced, Nperiods=" + str(Nperiods))

    Nsamples = T0 * fs
    if (
        Nsamples // sample_inkr * sample_inkr != Nsamples
    ):  # check multiplicity of sample_inkr
        Nsamples = (
            Nsamples // sample_inkr + 1
        ) * sample_inkr  # round to next higher multiple

    T1 = Nsamples / fs  # adapt exact duration
    f1 = Nperiods / T1  # adapt f1 for complete cycles
    if False:
        print("Nperiods: " + str(Nperiods))
        print("Nsamples: " + str(Nsamples))
        print("fs: " + str(fs))
        print("T1: " + str(T1))
        print("f1: " + str(f1))

    f_res = 1 / T1  # frequency resolution
    # determine a series of frequencies (freq[])
    if log:
        fact = sp.power(frange, 1.0 / (Nf - 1))  # factor for logarithmic scale
        freq = f1 * sp.power(fact, sp.arange(Nf))
    else:
        step = (frange - 1) * f1 / (Nf - 1)
        freq = sp.arange(f1, frange * f1 + step, step)

    # auxiliary function to find the nearest available frequency
    def find_nearest(
        x, possible
    ):  # match the theoretical freqs to the possible periodic freqs
        idx = (sp.absolute(possible - x)).argmin()
        return possible[idx]

    fi_pos = sp.arange(f1, frange * f1 + f_res, f_res)  # possible periodic frequencies
    f_real = []
    for f in freq:
        f_real.append(find_nearest(f, fi_pos))
    freq = sp.hstack(f_real)
    if True:
        print("freq: " + str(freq))

    if phases is None:  # generate random phases
        phase = sp.randn(Nf) * 2 * sp.pi  # random phase
    else:  # use given phases
        phase = phases

    return freq, phase, T1, fs


###################################
# Pseudo-Random-MultiSine for "quick calibration"
def PR_MultiSine(
    f1,
    Nperiods,
    Nsamples,
    Nf=8,
    fs_min=0,
    fs_max=1e9,
    frange=10,
    log=True,
    phases=None,
    deg=False,
    sample_inkr=1,
):
    """
    Returns an additive normalized Multisine time series. \n
    f1 = start frequency (may be adapted) \n
    Nperiods = number of periods of f1 (may be increased) \n
    Nsamples = Minimum Number of samples  \n
    Nf = number of frequencies in multi frequency mix \n
    fs_min = minimum sample rate of used device (default 0) \n
    fs_max = maximum sample rate of used device (default 0) \n
    frange = range of frequency as a factor relative to f1 (default 10 = decade) \n
    log = boolean for logarithmic (True, default) or linear (False) frequency scale \n
    phases = float array of given phases for the frequencies (default=None=random) \n
    deg= boolean for return phases in deg (True) or rad (False) \n
    sample_inkr = minimum block of samples to add to a waveform
    \n
    returns: freq,phase,fs,ti,multi \n
    freq= array of frequencies \n
    phase=used phases in deg or rad \n
    fs=sample rate \n
    ti=timestamps \n
    multi=array of time series values \n
    """

    freq, phase, T1, fs = PR_MultiSine_adapt(
        f1,
        Nperiods,
        Nsamples,
        Nf=Nf,
        fs_min=fs_min,
        fs_max=fs_max,
        frange=frange,
        log=log,
        phases=phases,
        sample_inkr=sample_inkr,
    )

    if deg:  # rad -> deg
        phase = phase * sp.pi / 180.0

    ti = sp.arange(T1 * fs, dtype=sp.float32) / fs

    multi = sp.zeros(len(ti), dtype=sp.float64)
    for f, p in zip(freq, phase):
        multi = multi + sp.sin(2 * sp.pi * f * ti + p)

    multi = multi / sp.amax(sp.absolute(multi))  # normalize

    if False:
        import matplotlib.pyplot as mp

        fig = mp.figure(1)
        fig.clear()
        pl1 = fig.add_subplot(211)
        pl2 = fig.add_subplot(212)
        pl1.plot(ti, multi, "-o")
        pl2.plot(sp.hstack((ti, ti + ti[-1] + ti[1])), sp.hstack((multi, multi)), "-o")
        mp.show()

    return (
        freq,
        phase,
        fs,
        multi,
    )  # frequency series, sample rate, sample timestamps, waveform


# PR_MultiSine(1,10,1500,5,fs_max=101,sample_inkr=7)
