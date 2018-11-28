#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 16:54:54 2018

@author: seeger01
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
GPSTime = np.genfromtxt('GPSTimeDtata7.csv',delimiter=';')
#GPSTime = GPSTime[1500:3600,:]
DeltaTicks=np.zeros(GPSTime.shape[0]-1)
Ticks=np.zeros(GPSTime.shape[0]-1)
Secs=range(0, GPSTime.shape[0]-1)
for i in range(0, GPSTime.shape[0]-1):
    # cast to uint32 is needed to acive overflow behaivor 
    DeltaTicks[i]=np.uint32(int(GPSTime[i+1,1]))-np.uint32(int(GPSTime[i,1]))
Ticks=np.cumsum(DeltaTicks)
Sigma=np.std(DeltaTicks);
Mu=np.mean(DeltaTicks);
RelSigma=Sigma/Mu
AKF=np.correlate(DeltaTicks,DeltaTicks, "same")
print("Mu= ",Mu)
print("Sigma=",Sigma)
print("Sigma/Mu=",RelSigma)
plt.title("µC Oscillator Frequency vs Time")
plt.xlabel('GPS PPS Event Count')
plt.ylabel('µC Clock Ticks since last GPS PPS Event')
plt.plot(Secs,DeltaTicks)
plt.show()

#x = np.random.normal(size = 1000)
#n,bins,patches=plt.hist(DeltaTicks,normed=True, bins=47)
#y = mlab.normpdf( bins, Mu, Sigma)
#l = plt.plot(bins, y, 'r--', linewidth=2)
#plt.ylabel('Probability');
#plt.title(r'Tickes between GPS PPS pulses')
#plt.xlabel(r'Tickes between GPS PPS pulses')
#plt.show()