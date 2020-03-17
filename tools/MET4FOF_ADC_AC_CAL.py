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
from scipy import stats ## for Student T Coverragefactor
from scipy.optimize import curve_fit #for fiting of Groupdelay
from scipy import interpolate #for 1D amplitude estimation
import logging
def PhaseFunc(f, GD):
    return GD*f*2*np.pi

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
            Transferfunction['N'][i]=N=len([d['Phase'] for d in self.fitResults[Channel][freq]])
            StudentTCoverageFactor95=stats.t.ppf(1-0.025,N)
            Transferfunction['AmplitudeCoefficent'][i]=np.mean([d['Amplitude'] for d in self.fitResults[Channel][freq]])
            Transferfunction['AmplitudeCoefficentUncer'][i]=np.std([d['Amplitude'] for d in self.fitResults[Channel][freq]])*StudentTCoverageFactor95
            Transferfunction['Phase'][i]=np.mean([d['Phase'] for d in self.fitResults[Channel][freq]])
            Transferfunction['PhaseUncer'][i]=np.std([d['Phase'] for d in self.fitResults[Channel][freq]])*StudentTCoverageFactor95

            i=i+1
        self.TransferFunctions[Channel]=Transferfunction
        return Transferfunction

    def SaveFitresults(self,Filename):
        with open(Filename, 'w') as fp:
            tmp={'MeataData':self.metadata,'FitResults':self.fitResults}
            json.dump(tmp, fp)

    def PlotTransferfunction(self,Channel,PlotType='lin',interpolSteps=1000):
        BoardID=self.metadata['BordID']
        tf=self.GetTransferFunction(Channel)
        XInterPol=np.linspace(np.min(tf['Frequencys']),np.max(tf['Frequencys']),interpolSteps)
        interPolAmp=np.zeros(interpolSteps)
        interPolPhase=np.zeros(interpolSteps)
        for i in range(interpolSteps):
            tmp=self.getNearestTF(Channel,XInterPol[i])
            interPolAmp[i]=tmp['AmplitudeCoefficent']
            interPolPhase[i]=tmp['Phase']
        fig, (ax1, ax2) = plt.subplots(2, 1)
        if PlotType=='log':
            ax1.set_xscale("log")
            ax2.set_xscale("log")
        ax1.errorbar(tf['Frequencys'], tf['AmplitudeCoefficent'],yerr=tf['AmplitudeCoefficentUncer'], fmt='o', markersize=2,label='Mesured Values')
        ax1.plot(XInterPol,interPolAmp)
        fig.suptitle("Transfer function of "+str(Channel)+" of Board with ID"+hex(BoardID))
        ax1.set_ylabel("Relative magnitude $|S|$")
        ax1.grid(True)
        ax2.errorbar(tf['Frequencys'], tf['Phase'] / np.pi * 180,yerr=tf['PhaseUncer']/ np.pi * 180,fmt='o', markersize=2,label='Mesured Values')
        ax2.plot(XInterPol,interPolPhase/ np.pi * 180)
        ax2.set_xlabel(r"Frequency $f$ in Hz")
        ax2.set_ylabel(r"Phase $\Delta\varphi$ in °")
        ax2.grid(True)
        ax1.legend(numpoints=1, fontsize=8,ncol=3)
        ax2.legend(numpoints=1, fontsize=8,ncol=3)
        plt.show()

    def getNearestTF(self,Channel,freq):
        Freqs=self.TransferFunctions[Channel]['Frequencys']
        testFreqIDX=np.argmin(abs(Freqs-freq))
        if Freqs[testFreqIDX]-freq==0:#ok we hit an calibrated point no need to interpolate
            return {'Frequency':self.TransferFunctions[Channel]['Frequencys'][testFreqIDX],
                    'AmplitudeCoefficent':self.TransferFunctions[Channel]['AmplitudeCoefficent'][testFreqIDX],
                    'AmplitudeCoefficentUncer':self.TransferFunctions[Channel]['AmplitudeCoefficentUncer'][testFreqIDX],
                    'Phase':self.TransferFunctions[Channel]['Phase'][testFreqIDX],
                    'PhaseUncer':self.TransferFunctions[Channel]['PhaseUncer'][testFreqIDX],
                    'N':self.TransferFunctions[Channel]['N'][testFreqIDX]}
        else:
            #interpolate
            tmp=self.getInterPolatedAmplitude(Channel,freq)
            A=tmp[0]
            AErr=tmp[1]
            tmp=self.getGroupDelay(Channel)
            GD=tmp[0]
            GDErr=tmp[1]
            Phase=PhaseFunc(freq,GD)
            PhaseErr=PhaseFunc(freq,GDErr)
            return {'Frequency':freq,
                    'AmplitudeCoefficent':A,
                    'AmplitudeCoefficentUncer':AErr,
                    'Phase':Phase,
                    'PhaseUncer':PhaseErr,
                    'N':0}

    def __getitem__(self, key):
        if len(key)==4:
            return self.TransferFunctions[key]
        if len(key)==2:
            return self.getNearestTF(key[0],key[1])
        else:
            raise ValueError("Invalide Key:  > "+str(key)+" <Use either [Channel] eg ['ADC1] or [Channel,Frequency] eg ['ADC1',1000]  as key ")

    def getGroupDelay(self,Channel):
        freqs=self.TransferFunctions[Channel]['Frequencys']
        phases=self.TransferFunctions[Channel]['Phase']
        phaseUncer=self.TransferFunctions[Channel]['Phase']
        popt, pcov = curve_fit(PhaseFunc, freqs, phases,sigma=phaseUncer,absolute_sigma=True)
        return[popt[0],pcov[0]]

    def getInterPolatedAmplitude(self,Channel,freq):
        Freqs=self.TransferFunctions[Channel]['Frequencys']
        Ampls=self.TransferFunctions[Channel]['AmplitudeCoefficent']
        AmplErr=self.TransferFunctions[Channel]['AmplitudeCoefficentUncer']
        testFreqIDX=np.argmin(abs(Freqs-freq))
        DeltaInterpolIDX=0
        if freq-Freqs[testFreqIDX]<0:
            DeltaInterpolIDX=-1
        if freq-Freqs[testFreqIDX]>0:
            DeltaInterpolIDX=1
        if testFreqIDX+DeltaInterpolIDX<0:
            raise ValueError("Extrapolation not supported! minimal Frequency is"+Freqs[0])
        if testFreqIDX+DeltaInterpolIDX>=Freqs.size:
            raise ValueError("Extrapolation not supported! maximal Frequency is"+Freqs[-1])
        if DeltaInterpolIDX==0:
            return [self.TransferFunctions[Channel]['AmplitudeCoefficent'][testFreqIDX],self.TransferFunctions[Channel]['AmplitudeCoefficentUncer'][testFreqIDX]]
        elif DeltaInterpolIDX ==-1:
            IDX=[testFreqIDX-1,testFreqIDX]
            x=Freqs[IDX]
            A=Ampls[IDX]
            AErr=AmplErr[IDX]
        elif DeltaInterpolIDX==1:
            IDX=[testFreqIDX,testFreqIDX+1]
            x=Freqs[IDX]
            A=Ampls[IDX]
            AErr=AmplErr[IDX]
        fA = interpolate.interp1d(x, A)
        fAErr = interpolate.interp1d(x, AErr)
        logging.info('Interpolateded transferfunction for Channel '+str(Channel)+'at Freq '+str(freq))  # will not print anything
        return[fA(freq),fAErr(freq)]




if __name__ == "__main__":
    ADCCall = Met4FOFADCCall(None,None,None,None,Filename='cal_data/1FE4_AC_CAL/20200311_1FE4_ADC1_AC_CAL.json')
    # DR = Datareceiver.DataReceiver("", 7654)
    # Fgen = FGEN.DG4xxx("192.168.0.62")
    # Scope = MSO.MSO5xxx("192.168.0.72")
    # time.sleep(5)
    # testfreqs = FGEN.generateDIN266Freqs(100,1e6, SigDigts=2)
    # loops=3
    # nptestfreqs=np.array([])
    # for i in np.arange(loops):
    #     nptestfreqs = np.append(nptestfreqs,np.array(testfreqs))
    # ADCCall = Met4FOFADCCall(Scope, Fgen, DR, 0x1FE40000)
    # ADC1FreqRespons = []
    # ampls = np.zeros(nptestfreqs.size*4)
    # phase = np.zeros(nptestfreqs.size*4)
    # i = 0
    # for freqs in nptestfreqs:
    #     ADC1FreqRespons.append(ADCCall.CallFreq(freqs, 19.5, "ADC1"))
    #     ampls[i] = ADC1FreqRespons[i]["Amplitude"]
    #     phase[i] = ADC1FreqRespons[i]["Phase"]
    #     ADC1FreqRespons.append(ADCCall.CallFreq(freqs, 1.95, "ADC1"))
    #     ampls[i+1] = ADC1FreqRespons[i+1]["Amplitude"]
    #     phase[i+1] = ADC1FreqRespons[i+1]["Phase"]
    #     ADC1FreqRespons.append(ADCCall.CallFreq(freqs, 0.195, "ADC1"))
    #     ampls[i+2] = ADC1FreqRespons[i+1]["Amplitude"]
    #     phase[i+2] = ADC1FreqRespons[i+1]["Phase"]
    #     ADC1FreqRespons.append(ADCCall.CallFreq(freqs, 5.0, "ADC1"))
    #     ampls[i+3] = ADC1FreqRespons[i+1]["Amplitude"]
    #     phase[i+3] = ADC1FreqRespons[i+1]["Phase"]
    #     i = i + 4

    ADCCall.PlotTransferfunction('ADC1',PlotType='lin')
    ADCCall.PlotTransferfunction('ADC1',PlotType='log')





