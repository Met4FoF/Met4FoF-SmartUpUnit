#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 19 16:31:46 2018

@author: seeger01
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
dataStart = 10
with open('GPSTimeDtata8.csv', newline='') as csvfile:
    GPSCOunt=0
    datareader = csv.reader(csvfile,delimiter=';')
    tempGPSTArray=[]
    tempREFTArray=[]
    for row in datareader:
        if(row[0]=="GPST"):
            tempGPSTArray.append(row[1:3])
            GPSCOunt=GPSCOunt+1
        if(row[0]=="REFT"):
            tempREFTArray.append(row[1:3])
            GPSCOunt=GPSCOunt+1
    GPST=np.asarray(tempGPSTArray,dtype='uint32')
    DeltaGPST=GPST[1:,1]-GPST[:-1,1]
    DeltaGPSTx=GPST[0:-1,0]
    REFT=np.asarray(tempREFTArray,dtype='uint32')
    DeltaREFT=REFT[1:,1]-REFT[:-1,1]
    DeltaREFTx=REFT[0:-1,0]
    fig, ax = plt.subplots()
    ax.set(xlabel='Time (s)', ylabel='ticks since last PPS (Ticks)',
       title='µC clockticks between per second syncsignal')
    ax.plot(DeltaREFTx[dataStart:],DeltaREFT[dataStart:],label='PTB Ref. Clock PPS')
    ax.plot(DeltaGPSTx[dataStart:],DeltaGPST[dataStart:],label='GPS PPS')
    ax.grid()
    ax.legend()
    plt.show()