#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 19 16:31:46 2018

@author: seeger01
"""


import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt

sollFreq=512
SollDeltaT=1/sollFreq
raw_data = np.genfromtxt('log_files/k050HZTekAFG3101ext10MHzacc.csv', delimiter=';', unpack=True)
deltaTick=raw_data[4,1:]-raw_data[4,:-1]
TGPS=raw_data[1]-raw_data[1,0]+raw_data[2]*1e-9
deltaGPS=TGPS[1:]-TGPS[:-1]
TSOll=np.zeros(TGPS.size)
#genrate expected Timestamps
for x in range(0,TSOll.size):
    TSOll[x]=SollDeltaT*x
    

deltaGPS=deltaGPS-np.mean(deltaGPS)
fig, ax = plt.subplots(tight_layout=True)
ax.plot(TSOll[1:],deltaGPS)
ax.set_xlabel(r'\textbf{Time} $T$ /s', fontsize=16)
ax.set_ylabel(r'\textbf{Time difference} $\Delta T=T_x-T_{x-1}$ /s', fontsize=16)#
ax.set_title(r'Timing jitter', fontsize=16,)
plt.show()

for x in range(0, deltaGPS.size):
    if(deltaGPS[x]>0.005 or deltaGPS[x]<-0.005):
       deltaGPS[x]=np.nan
       
fig2, ax2 = plt.subplots(tight_layout=True)
ax2.plot(TSOll[1:],deltaGPS*1e9)
ax2.set_xlabel(r'\textbf{Time} $T$ /s', fontsize=16)
ax2.set_ylabel(r'\textbf{Time difference} $\Delta T=T_x-T_{x-1}$ /ns', fontsize=16)
ax2.set_title(r'Timing jitter spikes removed', fontsize=16,)
plt.show()

fig3, ax3 = plt.subplots(tight_layout=True)
ax3.plot(TSOll,TGPS)
ax3.set_xlabel(r'\textbf{Time} $T$ /s', fontsize=16)
ax3.set_ylabel(r'\textbf{GPS Time} $T$ /s', fontsize=16)
ax3.set_title(r'GPS Timing gliches', fontsize=16,)
plt.show()