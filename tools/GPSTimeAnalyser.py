#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 16:54:54 2018

@author: seeger01
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from matplotlib import colors
from matplotlib.ticker import PercentFormatter

GPSTime = np.genfromtxt('GPSTimeDtata2.csv',delimiter=';')
GPSTime = GPSTime[200:1500,:]
DeltaTicks=np.zeros(GPSTime.shape[0]-1)
Ticks=np.zeros(GPSTime.shape[0]-1)
Secs=range(0, GPSTime.shape[0]-1)
for i in range(0, GPSTime.shape[0]-1):
    # cast to uint32 is needed to acive overflow behaivor 
    DeltaTicks[i]=np.uint32(int(GPSTime[i+1,1]))-np.uint32(int(GPSTime[i,1]))
Ticks=np.cumsum(DeltaTicks)
Sigma=np.std(DeltaTicks);
Mu=np.mean(DeltaTicks);
print("Mu= ",Mu)
print("Sigma=",Sigma)
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