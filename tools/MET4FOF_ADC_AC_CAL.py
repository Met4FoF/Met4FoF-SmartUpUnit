#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 17 10:26:02 2020

@author: seeger01
"""


import MET4FOFDataReceiver as Datareceiver
import MSO5xxx as MSO
import DG4xxx as FGEN
import time
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import SineTools as st
import json as json



class Met4FOFADCCall:
    def __init__(self, Scope,FGen,Datareceiver,SensorID,Filename=None):

        if Filename!=None:
            with open(Filename) as json_file:
                tmp= json.load(json_file)
            self.metadata=tmp['MeataData']
            self.fitResults=tmp['FitResults']

        else:
            self.fitResults = {}#will store lists with fit results for the according channel
            self.Scope = Scope
            self.FGen = FGen
            self.DR = Datareceiver
            self.SensorID = SensorID
            self.bufferSizeMax = 1e5
            self.buffer = [None] * int(self.bufferSizeMax)
            self.bufferCount = 0
            self.Scope.single()
            self.metadata={'BordID':SensorID&0xFFFF0000,
                           'Date':datetime.now().isoformat(' ', 'seconds'),
                           'ScopeParams':self.Scope.params,
                           'FGenParams':self.FGen.params}
        self.TransferFunctions= {}



    def BufferFlush(self):
        self.buffer = [None] * int(self.bufferSizeMax)
        self.bufferCount = 0

    def BufferPushCB(self, Message, Description):
        if Message.sample_number % 100 == 0:
            print(Message)
        i = self.bufferCount
        if i == 0:
            self.Description = Description
        if i < self.bufferSizeMax:
            self.buffer[i] = Message
            self.bufferCount = i + 1

    def CallFreq(self, Freq, Ampl, Channel, offset=0, phase=0, SamplingFreq=2002,duration=10):
        cycles = Freq * duration
        if(Freq* duration>1e6):#DG4000 can not generate more than 1e6 N-Cyclebursts
            duration=1e6/Freq
            cycles=1e6
            print("To Many cycles planend actual mesurement time in s will be :"+str(duration))
        Samplingcycles = int(SamplingFreq * duration) - 1
        # flush Buffer
        self.BufferFlush()
        # arm fgen for burst generation
        self.FGen.ArmExtTrigSyncburst(
            Freq, Ampl, offset, phase, cycles, SamplingFreq, Samplingcycles
        )
        # fire Scope
        self.Scope.single()
        # atach buffer to Sensor data callback
        DR.AllSensors[self.SensorID].SetCallback(self.BufferPushCB)
        time.sleep(2)
        self.Scope.forceTrigger()
        # disable callback
        time.sleep(15)
        self.DR.AllSensors[self.SensorID].UnSetCallback()
        self.ADCdata = np.zeros(self.bufferCount)
        self.DeltaTime = np.zeros(self.bufferCount)
        unix_time_start = self.buffer[0].unix_time
        unix_nsecs_start = self.buffer[0].unix_time_nsecs
        for i in range(self.bufferCount):
            self.DeltaTime[i] = (
                self.buffer[i].unix_time
                - unix_time_start
                + (0 - (unix_nsecs_start - self.buffer[i].unix_time_nsecs) * 1e-9)
            )
            # self.DeltaTime[i]=i*(1/SamplingFreq)+0.5
            if Channel == "ADC1":
                self.ADCdata[i] = self.buffer[i].Data_11
            if Channel == "ADC2":
                self.ADCdata[i] = self.buffer[i].Data_12
            if Channel == "ADC3":
                self.ADCdata[i] = self.buffer[i].Data_13
        # detect glitches due to message packing in stm32 board
        maxDeltaT = 1 / SamplingFreq * 1.25  # 25% more than set sampling clock
        deltaDeltaT = np.diff(ADCCall.DeltaTime)
        # find maximum
        DeltaDeltaTmax = deltaDeltaT.max()
        if DeltaDeltaTmax > maxDeltaT:
            print("oldPackets found")
            pos = deltaDeltaT.argmax()
            self.ADCdata = self.ADCdata[pos + 1 :]
            self.DeltaTime = self.DeltaTime[pos + 1 :]
        self.DeltaTime = self.DeltaTime - self.DeltaTime[0] + (0.5 * 1 / (SamplingFreq))
        FIT = st.threeparsinefit(self.ADCdata, self.DeltaTime, Freq)
        phaseFit = -st.phase(FIT)
        amplFit = st.amplitude(FIT)
        amplTrans = 2 * amplFit / Ampl
        retVal = {
            "Freq": Freq,
            "Amplitude": amplTrans,
            "Phase": phaseFit,
            "TestAmplVPP.": Ampl,
        }
        if Channel in self.fitResults:
            if Freq in self.fitResults[Channel]:
                self.fitResults[Channel][Freq].append(retVal)
            else:
                self.fitResults[Channel][Freq]=[retVal]
        else:
            self.fitResults[Channel]={}
            self.fitResults[Channel][Freq]=[retVal]
        return retVal

    def GetTransferFunction(self,Channel):
        FreqNum=len(self.fitResults[Channel].keys())
        Transferfunction={'Frequencys':np.zeros(FreqNum),
                          'AmplitudeCoefficent':np.zeros(FreqNum),
                          'AmplitudeCoefficentUncer':np.zeros(FreqNum),
                          'Phase':np.zeros(FreqNum),
                          'PhaseUncer':np.zeros(FreqNum),
                          'N':np.zeros(FreqNum)}
        i=0
        for freq in self.fitResults[Channel].keys():
            Transferfunction['Frequencys'][i]=freq
            Transferfunction['AmplitudeCoefficent'][i]=np.mean([d['Amplitude'] for d in self.fitResults[Channel][freq]])
            Transferfunction['AmplitudeCoefficentUncer'][i]=2*np.std([d['Amplitude'] for d in self.fitResults[Channel][freq]])
            Transferfunction['Phase'][i]=np.mean([d['Phase'] for d in self.fitResults[Channel][freq]])
            Transferfunction['PhaseUncer'][i]=2*np.std([d['Phase'] for d in self.fitResults[Channel][freq]])
            Transferfunction['N'][i]=len([d['Phase'] for d in self.fitResults[Channel][freq]])
            i=i+1
        self.TransferFunctions[Channel]=Transferfunction
        return Transferfunction

    def SaveFitresults(self,Filename):
        with open(Filename, 'w') as fp:
            tmp={'MeataData':self.metadata,'FitResults':self.fitResults}
            json.dump(tmp, fp)

    def PlotTransferfunction(self,Channel,PlotType='lin'):
        BoardID=self.metadata['BordID']
        tf=self.GetTransferFunction(Channel)
        fig, (ax1, ax2) = plt.subplots(2, 1)
        if PlotType=='log':
            ax1.set_xscale("log")
            ax2.set_xscale("log")
        ax1.errorbar(tf['Frequencys'], tf['AmplitudeCoefficent'],yerr=tf['AmplitudeCoefficentUncer']*2, marker=".", markersize=2)
        fig.suptitle("Transfer function of "+str(Channel)+" of Board with ID"+hex(BoardID))
        ax1.set_ylabel("Relative magnitude $|S|$")
        ax1.grid(True)
        ax2.errorbar(tf['Frequencys'], tf['Phase'] / np.pi * 180,yerr=tf['PhaseUncer']*2/ np.pi * 180,marker='.', markersize=2)
        ax2.set_xlabel(r"Frequency $f$ in Hz")
        ax2.set_ylabel(r"Phase $\Delta\varphi$ in °")
        ax2.grid(True)
        ax1.legend(numpoints=1, fontsize=8,ncol=3)
        ax2.legend(numpoints=1, fontsize=8,ncol=3)
        plt.show()


if __name__ == "__main__":
    # ADCCall = Met4FOFADCCall(None,None,None,None,Filename='cal_data/200310_ADC1_AC_cal.json')
    DR = Datareceiver.DataReceiver("", 7654)
    Fgen = FGEN.DG4xxx("192.168.0.62")
    Scope = MSO.MSO5xxx("192.168.0.72")
    time.sleep(5)
    testfreqs = FGEN.generateDIN266Freqs(100,1e6, SigDigts=2)
    loops=3
    nptestfreqs=np.array([])
    for i in np.arange(loops):
        nptestfreqs = np.append(nptestfreqs,np.array(testfreqs))
    ADCCall = Met4FOFADCCall(Scope, Fgen, DR, 0x1FE40000)
    ADC1FreqRespons = []
    ampls = np.zeros(nptestfreqs.size*4)
    phase = np.zeros(nptestfreqs.size*4)
    i = 0
    for freqs in nptestfreqs:
        ADC1FreqRespons.append(ADCCall.CallFreq(freqs, 19.5, "ADC1"))
        ampls[i] = ADC1FreqRespons[i]["Amplitude"]
        phase[i] = ADC1FreqRespons[i]["Phase"]
        ADC1FreqRespons.append(ADCCall.CallFreq(freqs, 1.95, "ADC1"))
        ampls[i+1] = ADC1FreqRespons[i+1]["Amplitude"]
        phase[i+1] = ADC1FreqRespons[i+1]["Phase"]
        ADC1FreqRespons.append(ADCCall.CallFreq(freqs, 0.195, "ADC1"))
        ampls[i+2] = ADC1FreqRespons[i+1]["Amplitude"]
        phase[i+2] = ADC1FreqRespons[i+1]["Phase"]
        ADC1FreqRespons.append(ADCCall.CallFreq(freqs, 5.0, "ADC1"))
        ampls[i+3] = ADC1FreqRespons[i+1]["Amplitude"]
        phase[i+3] = ADC1FreqRespons[i+1]["Phase"]
        i = i + 4

    ADCCall.PlotTransferfunction('ADC1',PlotType='lin')
    ADCCall.PlotTransferfunction('ADC1',PlotType='log')





