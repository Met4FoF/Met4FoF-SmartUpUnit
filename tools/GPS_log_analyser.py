# -*- coding: utf-8 -*-
"""
Created on Mon Sep 23 11:03:07 2019

@author: benes
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy import signal
from scipy.stats import norm

LANG = "DE"
#GPSLoggArray = np.load(r"C:\Users\benes\Downloads\2019-09-23_gpslog.npy")
GPSLoggArray = np.load(r"data/2019-09-23_gpslog.npy")
GPSLoggArray = np.reshape(GPSLoggArray, (int(GPSLoggArray.size / 4), 4))
# cut out first 5 values sice they are pointless
GPSLoggArray = np.copy(GPSLoggArray[5:, :])
# cut out data in witch coldspray was used
#GPSLoggArray = np.copy(GPSLoggArray[1000:, :])
# substract startvalues for index and secs to have handy values starting at zero for time series
GPSLoggArray[:, 0] = GPSLoggArray[:, 0] - GPSLoggArray[0, 0]
GPSLoggArray[:, 1] = GPSLoggArray[:, 1] - GPSLoggArray[0, 1]
DF = pd.DataFrame(
    data=GPSLoggArray[:, 1:],  # values
    index=GPSLoggArray[:, 0],  # 1st column as index
    columns=["Secs", "nSecs", "uncer"],
)  # 1st row as the column names
splipostion = np.array([96603, 104099])
#splipostion = np.array([])
for i in np.arange(splipostion.size):
    DF["nSecs"][splipostion[i] :] += 100
DF["nSecDev"] = DF["nSecs"] - DF["nSecs"][0]
DF["nSecDev"] = DF["nSecs"] - DF["nSecs"][0] - np.mean(DF["nSecDev"])
AK = signal.correlate(DF["nSecDev"], DF["nSecDev"], mode="same")
fig1, ax1 = plt.subplots()
if LANG == "EN":
    nSecDevLabel = r"time deviation $t_{(is)}-t_{(shold)}$"
    uncerLabel = "Measurement uncertainty $u$"
if LANG == "DE":
    nSecDevLabel = r"Zeitabweichung $t_{(ist)}-t_{(soll)}$"
    uncerLabel = "Messunsicherheit $u$"
ax1.plot(2 * DF["uncer"], color="C1", label=uncerLabel)
ax1.plot(-2 * DF["uncer"], color="C1")
ax1.plot(DF["nSecDev"], color="C0", label=nSecDevLabel)
if LANG == "EN":
    ax1.set_ylabel(r"time deviation $t_{(is)}-t_{(should)}$in ns")
    ax1.set_xlabel("Time in s")
    ax1.set_title("Timingdeviation PTB PPS signal stamped with Data aqusition unit")
if LANG == "DE":
    ax1.set_ylabel(r"Zeitabweichung $t_{(ist)}-t_{(soll)}$ in ns")
    ax1.set_xlabel("Zeit in s")
    ax1.set_title(
        "Abweichung der zeitgestempelten PTB PPS Signale nach Zeitstemplung durch DAQ Hardware"
    )
ax1.grid(True)
ax1.legend()
fig1.show()
DF["inRange"] = (abs(DF["nSecDev"]) <= 2 * (DF["uncer"] * 60 / 59)).astype(int)
coverage = np.mean(DF["inRange"])
print("uncertantiy coverage is " + str(coverage * 100) + " %")

# best fit of data
(mu, sigma) = norm.fit(DF["nSecDev"])

bincount = int(DF["nSecDev"].max() - DF["nSecDev"].min())
fig2, ax2 = plt.subplots()
# the histogram of the data
n, bins, patches = ax2.hist(
    DF["nSecDev"], bincount, density=1, facecolor="green", alpha=0.75
)

# add a 'best fit' line
y = norm.pdf(bins, mu, sigma)

ax2.plot(bins, y, "r--", linewidth=2)

# plot
if LANG == "EN":
    ax2.set_xlabel("time deviation $t_{(is)}-t_{(should)}$in ns")
    ax2.set_ylabel("Frequency ")
    ax2.set_title(
        r"$\mathrm{Histogram\ of\ time deviation:}\ 2\sigma=%.3f ~ns,\ \sigma = %.3f$ ~ns n = $%i$"
        % (2 * sigma, sigma, DF["nSecDev"].size)
    )
if LANG == "DE":
    ax2.set_xlabel("Zeitabweichung $t_{(ist)}-t_{(soll)}$ in ns")
    ax2.set_ylabel("Häufigkeit")
    ax2.set_title(
        r"$\mathrm{Histogram~der~Zeitabweichungen:} \sigma = %.3f$ ns ,n = $%i$"
        % (sigma, DF["nSecDev"].size)
    )
ax2.grid(True)

fig2.show()
