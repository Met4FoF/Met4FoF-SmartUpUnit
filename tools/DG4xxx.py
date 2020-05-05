#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 23 09:27:10 2019

@author: seeger01
"""
#this script should be used to controal an RIGOL DG4162 FGen
# wire shark debung filter options (ip.src == 192.168.2.3 || ip.dst == 192.168.2.3 ) && !(tcp.flags.ack && tcp.len <= 1)
import pyvisa #pip install -U pyvisa-py
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter
import warnings
import math
rm = pyvisa.ResourceManager()


class DG4xxx:

    def __init__(self,IP):

        self.params={'IP':IP}
        self.rm=rm
        self.visa=self.rm.open_resource('TCPIP0::'+IP+'::INSTR')
        self.visa.timeout=10000
        retval=self.visa.query('*IDN?')
        self.params['IP']
        self.params['Vendor']=retval.split(",")[0]
        self.params['Type']=retval.split(",")[1]
        self.params['SN']=retval.split(",")[2]
        self.params['FWVersion']=retval.split(",")[3].replace('\n', '')
        print(self.params)

    def queryCommand(self,CMD):
        return self.visa.query(CMD)

    def sendCommand(self,CMD):
        return self.visa.write(CMD)

    def fiereBurst(self,freq,ampl,offset,phase,cycles,channel=1,delay=False):
        minAmpl=-ampl/2+offset
        maxAmpl=ampl/2+offset
        if(minAmpl<0):
            raise ValueError('Output voltage should never be negative but is at minium '+str(minAmpl))
        if(maxAmpl>5):
            raise ValueError('Output voltage should never be grater than 5 Volt but is at max '+str(maxAmpl))
        if(freq<0.001):
            raise ValueError('Output frequency is to low (<1 mHz) or negative')
        self.sendCommand(':OUTPut'+str(channel)+':STATe OFF')
        self.sendCommand(':SOURce'+str(channel)+':APPLy:SINusoid '+str(freq)+','+str(ampl)+','+str(offset)+','+str(phase))
        self.sendCommand(':SOURce'+str(channel)+':BURSt:NCYCles '+str(cycles))
        self.sendCommand(':SOURce'+str(channel)+':BURSt:TRIGger:SOURce MANual') #activate manual TRIGger
        self.sendCommand(':SOURce'+str(channel)+':BURSt ON')#activate burst mode
        self.sendCommand(':OUTPut'+str(channel)+':STATe ON')
        self.sendCommand(':SOURce'+str(channel)+':BURSt:TRIGger')
        if delay:
            Duration=1/freq*cycles
            print("Burst Duration="+str(Duration)+' seconds \nsleeping this time')
            time.sleep(Duration)
            self.sendCommand(':OUTPut OFF')
            return
        else:
            return

    def ArmExtTrigSyncburst(self,freq,ampl,offset,phase,cycles,CH2Freq,CH2Cycles):
        #this function configures the function generator to output a sin signal with configured phase amplitude and frequency on CH1.
        #CH2 generates a synchronous square wave signal with adjustable frequency and 180 phase shift.
        #At t=0 no square pulse is generated.
        #the display of the function generator couldbe misleading
        self.sendCommand(':OUTPut1:STATe ON') #
        self.sendCommand(':SOURce1:APPLy:SINusoid '+str(freq)+','+str(ampl)+','+str(offset)+','+str(phase))
        self.sendCommand(':SOURce2:APPLy:SQUare '+str(CH2Freq)+',3,1.5,0')#phase is ignored in burst mode and has to be set seperate
        self.sendCommand(':SOURce1:BURSt ON')#activate burst mode
        self.sendCommand(':SOURce2:BURSt ON')#activate burst mode
        self.sendCommand(':SOURce2:BURSt:PHASe 180')#activate burst mode
        self.sendCommand(':SOURce1:BURSt:MODE TRIGgered')
        self.sendCommand(':SOURce2:BURSt:MODE TRIGgered ')
        self.sendCommand(':SOURce1:BURSt:NCYCles '+str(cycles))
        self.sendCommand(':SOURce2:BURSt:NCYCles '+str(CH2Cycles))
        self.sendCommand(':SOURce1:BURSt:TRIGger:SOURce EXTernal')
        self.sendCommand(':SOURce2:BURSt:TRIGger:SOURce EXTernal')
        self.sendCommand(':OUTPut1:STATe ON') #
        self.sendCommand(':OUTPut2:STATe ON')
        return

def Fireloop(FGEN):
    for x in range(0, 20):
        FGEN.fiereDifferentialBurst(100.0,0.01,0.005,0,10,True)

def generateDIN266Freqs(Start,Stop,SigDigts=-1,highres=False):
    """Returns an array with the standard frequencies between Start and Stop according to DIN EN ISO 266, if Start is greater than Stop the array is sorted in descending order
    

    Parameters
    ----------
    Start : float
        Start frquency.

    Stop : float
        Stop frequency.

    SigDigts : int>1, optional
        Number of significant digits therfore allwas grater than 1. The default is -1.

    Raises
    ------
    ValueError
        Start or Stop frequency musst not be zero.

    Returns
    -------
    np.array
        Array with the norm frequencys.

    """
    if not (Start!=0 and Stop !=0):
        raise ValueError('Start or Stop frequency musst not be zero')
    startEXP=math.ceil(np.log10(Start)*10)/10#round up to next n.n 3.1454-->3.2
    stopEXP=math.floor(np.log10(Stop)*10)/10#round down to next n.n 3.1454-->3.1
    if(stopEXP>=startEXP):
        freqEXP=np.arange(startEXP,stopEXP+0.1,0.1)
    if(stopEXP<startEXP):
        freqEXP=np.arange(stopEXP,startEXP+0.1,0.1)
        freqEXP=freqEXP[::-1]#return freqcs in inverted order since input is from high to low
    if(stopEXP==startEXP):
        freqEXP=np.array([startEXP])
    freqs=pow(10,freqEXP)
    if(SigDigts<1):
        return freqs
    else:
        roundfreqs=np.empty_like(freqs)
        for i in range(0, freqs.size):
            roundfreqs[i]=round(freqs[i], SigDigts-int(math.floor(np.log10(abs(freqs[i]))))-1)
            #https://stackoverflow.com/questions/3410976/how-to-round-a-number-to-significant-figures-in-python
            #round to significant figure
        return roundfreqs
