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
import math



import logging
def PhaseFunc(f, GD):
    return GD*f*2*np.pi

class Met4FOFADCCall:
    def __init__(self, Scope,FGen,Datareceiver,SensorID,Filenames=[None]):

        if Filenames!=[None]:
            i=0
            for CalFile in Filenames:
                print(CalFile)
                if i==0:
                    with open(CalFile ) as json_file:
                        tmp= json.load(json_file)
                    self.metadata=tmp['MeataData']
                    self.fitResults=tmp['FitResults']
                    i=i+1
                    json_file.close()
                else:
                    with open(CalFile) as json_file:
                        tmp= json.load(json_file)
                    if(self.metadata['BordID']==tmp['MeataData']['BordID']):
                        for Channel in tmp['FitResults']:
                            for Freqs in tmp['FitResults'][Channel]:
                                self.fitResults[Channel][Freqs]+=(tmp['FitResults'][Channel][Freqs])
                    else:
                        raise RuntimeWarning("BoardIDs"+self.metadata['BordID']+'and'+tmp.metadata['BordID']+'DO Not Match ignoring File'+CalFile)
                    i=i+1
                    json_file.close()
            self.TransferFunctions= {}
            for Channels in self.fitResults:
                self.GetTransferFunction(Channels)

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

    def CallFreq(self, Freq, Ampl, Channels, offset=0, phase=0, SamplingFreq=2002,duration=10):
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
        self.DR.AllSensors[self.SensorID].SetCallback(self.BufferPushCB)
        time.sleep(2)
        self.Scope.forceTrigger()
        # disable callback
        time.sleep(15)
        self.DR.AllSensors[self.SensorID].UnSetCallback()
        self.DeltaTime = np.zeros(self.bufferCount)
        unix_time_start = self.buffer[0].unix_time
        unix_nsecs_start = self.buffer[0].unix_time_nsecs
        self.ADCdata=np.zeros([3,self.bufferCount])
        for i in range(self.bufferCount):
            self.DeltaTime[i] = (
                self.buffer[i].unix_time
                - unix_time_start
                + (0 - (unix_nsecs_start - self.buffer[i].unix_time_nsecs) * 1e-9)
            )
            # self.DeltaTime[i]=i*(1/SamplingFreq)+0.5
            self.ADCdata[0,i] = self.buffer[i].Data_11
            self.ADCdata[1,i] = self.buffer[i].Data_12
            self.ADCdata[2,i] = self.buffer[i].Data_13
        ADCDataIDXByName={'ADC1':0,'ADC2':1,'ADC3':2}
        # detect glitches due to message packing in stm32 board
        maxDeltaT = 1 / SamplingFreq * 1.25  # 25% more than set sampling clock
        deltaDeltaT = np.diff(ADCCall.DeltaTime)
        # find maximum
        DeltaDeltaTmax = deltaDeltaT.max()
        if DeltaDeltaTmax > maxDeltaT:
            print("oldPackets found")
            pos = deltaDeltaT.argmax()
            self.ADCdata = self.ADCdata[:,pos + 1 :]
            self.DeltaTime = self.DeltaTime[pos + 1 :]
        self.DeltaTime = self.DeltaTime - self.DeltaTime[0] + (0.5 * 1 / (SamplingFreq))
        retVals=[]
        for Channel in Channels:
            tmp=self.ADCdata[ADCDataIDXByName[Channel],:]
            FIT = st.threeparsinefit(tmp, self.DeltaTime, Freq)
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
        return

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

    def PlotTransferfunction(self,Channel,PlotType='lin',interpolSteps=1000,fig=None,ax=[None,None],LabelExtension='',TitleExtension=''):
        BoardID=self.metadata['BordID']
        tf=self.GetTransferFunction(Channel)
        if PlotType=='log':
            XInterPol=np.power(10,np.linspace(np.log10(np.min(tf['Frequencys'])),np.log10(np.max(tf['Frequencys'])-1),interpolSteps))
        else:
            XInterPol=np.logspace(np.min(tf['Frequencys']),np.max(tf['Frequencys']),interpolSteps)
        interPolAmp=np.zeros(interpolSteps)
        interPolAmpErrMin=np.zeros(interpolSteps)
        interPolAmpErrMax=np.zeros(interpolSteps)
        interPolPhase=np.zeros(interpolSteps)
        interPolPhaseErrMin=np.zeros(interpolSteps)
        interPolPhaseErrMax=np.zeros(interpolSteps)
        for i in range(interpolSteps):
            tmp=self.getNearestTF(Channel,XInterPol[i])
            interPolAmp[i]=tmp['AmplitudeCoefficent']
            interPolAmpErrMin[i]=interPolAmp[i]-tmp['AmplitudeCoefficentUncer']
            interPolAmpErrMax[i]=interPolAmp[i]+tmp['AmplitudeCoefficentUncer']
            interPolPhase[i]=tmp['Phase']
            interPolPhaseErrMin[i]=interPolPhase[i]-tmp['PhaseUncer']
            interPolPhaseErrMax[i]=interPolPhase[i]+tmp['PhaseUncer']
        if fig==None and ax==[None,None]:
            Fig, (ax1, ax2) = plt.subplots(2, 1)
        else:
            Fig=fig
            ax1=ax[0]
            ax2=ax[1]
        if PlotType=='log':
            ax1.set_xscale("log")
            ax2.set_xscale("log")
        ax1.plot(XInterPol,interPolAmp,label='Interpolated'+LabelExtension)
        lastcolor=ax1.get_lines()[-1].get_color()
        ax1.fill_between(XInterPol, interPolAmpErrMin, interPolAmpErrMax,alpha=0.3,color=lastcolor)
        ax1.errorbar(tf['Frequencys'], tf['AmplitudeCoefficent'],yerr=tf['AmplitudeCoefficentUncer'], fmt='o', markersize=4,label='Mesured Values'+LabelExtension, uplims=True, lolims=True,color=lastcolor)
        Fig.suptitle("Transfer function of "+str(Channel)+" of Board with ID"+hex(BoardID)+TitleExtension)
        ax1.set_ylabel("Relative magnitude $|S|$")
        ax1.grid(True)
        ax2.plot(XInterPol,interPolPhase/ np.pi * 180,label='Interpolated'+LabelExtension)
        ax2.fill_between(XInterPol, interPolPhaseErrMin/ np.pi * 180, interPolPhaseErrMax/ np.pi * 180,alpha=0.3,color=lastcolor)
        ax2.errorbar(tf['Frequencys'], tf['Phase'] / np.pi * 180,yerr=tf['PhaseUncer']/ np.pi * 180,fmt='o', markersize=3,label='Mesured Values'+LabelExtension, uplims=True, lolims=True,color=lastcolor)
        ax2.set_xlabel(r"Frequency $f$ in Hz")
        ax2.set_ylabel(r"Phase $\Delta\varphi$ in °")
        ax2.grid(True)
        ax1.legend(numpoints=1, fontsize=8,ncol=3)
        ax2.legend(numpoints=1, fontsize=8,ncol=3)
        plt.show()
        return Fig,[ax1,ax2]

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
            A,AErr=self.getInterPolatedAmplitude(Channel,freq)
            P,PErr=self.getInterPolatedPhase(Channel,freq)
            return {'Frequency':freq,
                    'AmplitudeCoefficent':np.asscalar(A),
                    'AmplitudeCoefficentUncer':np.asscalar(AErr),
                    'Phase':np.asscalar(P),
                    'PhaseUncer':np.asscalar(PErr),
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
        phaseUncer=self.TransferFunctions[Channel]['PhaseUncer']
        popt, pcov = curve_fit(PhaseFunc, freqs, phases,sigma=phaseUncer,absolute_sigma=True)
        return[popt,pcov]

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
            assert RuntimeWarning(str(freq)+" is to SMALL->Extrapolation is not recomended! minimal Frequency is "+str(Freqs[0]))
            return[Ampls[0],AmplErr[0]]
        if testFreqIDX+DeltaInterpolIDX>=Freqs.size:
            raise ValueError(str(freq)+" is to BIG->Extrapolation not supported! maximal Frequency is "+str(Freqs[-1]))
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

    def getInterPolatedPhase(self,Channel,freq):
        Freqs=self.TransferFunctions[Channel]['Frequencys']
        Phases=self.TransferFunctions[Channel]['Phase']
        PhasesErr=self.TransferFunctions[Channel]['PhaseUncer']
        testFreqIDX=np.argmin(abs(Freqs-freq))
        DeltaInterpolIDX=0
        if freq-Freqs[testFreqIDX]<0:
            DeltaInterpolIDX=-1
        if freq-Freqs[testFreqIDX]>0:
            DeltaInterpolIDX=1
        if testFreqIDX+DeltaInterpolIDX<0:
            assert RuntimeWarning(str(freq)+" is to SMALL->Extrapolation is not recomended! minimal Frequency is "+str(Freqs[0]))
            return[Phases[0],PhasesErr[0]]
        if testFreqIDX+DeltaInterpolIDX>=Freqs.size:
            raise ValueError("Extrapolation not supported! maximal Frequency is"+Freqs[-1])
        if DeltaInterpolIDX==0:
            return [self.TransferFunctions[Channel]['AmplitudeCoefficent'][testFreqIDX],self.TransferFunctions[Channel]['AmplitudeCoefficentUncer'][testFreqIDX]]
        elif DeltaInterpolIDX ==-1:
            IDX=[testFreqIDX-1,testFreqIDX]
        elif DeltaInterpolIDX==1:
            IDX=[testFreqIDX,testFreqIDX+1]
        x=Freqs[IDX]
        P=Phases[IDX]
        PErr=PhasesErr[IDX]
        fP = interpolate.interp1d(x, P)
        fPErr = interpolate.interp1d(x, PErr)
        logging.info('Interpolateded transferfunction for Channel '+str(Channel)+'at Freq '+str(freq))  # will not print anything
        return[fP(freq),fPErr(freq)]




if __name__ == "__main__":
    # SMALL_SIZE = 12
    # MEDIUM_SIZE = 15
    # BIGGER_SIZE = 18
    # plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
    # plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
    # plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
    # plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    # plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    # plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
    # plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title
    # ADCCall = Met4FOFADCCall(None,None,None,None,Filename='cal_data/1FE4_AC_CAL/200318_1FE4_ADC123_19V5_1V95_V195_1HZ_1MHZ.json')
    # ADCCall.PlotTransferfunction('ADC1',PlotType='log')
    # ADCCall.PlotTransferfunction('ADC2',PlotType='log')
    # ADCCall.PlotTransferfunction('ADC3',PlotType='log')
    DR = Datareceiver.DataReceiver("", 7654)
    Fgen = FGEN.DG4xxx("192.168.0.62")
    Scope = MSO.MSO5xxx("192.168.0.72")
    time.sleep(5)
    testfreqs = FGEN.generateDIN266Freqs(1,1e6, SigDigts=2)
    loops=20
    nptestfreqs=np.array([])
    for i in np.arange(loops):
        nptestfreqs = np.append(nptestfreqs,np.array(testfreqs))
    ADCCall = Met4FOFADCCall(Scope, Fgen, DR, 0x1FE40000)
    for freqs in nptestfreqs:
        ADCCall.CallFreq(freqs, 19.5, ['ADC1','ADC2','ADC3'])
    ADCCall.SaveFitresults('cal_data/1FE4_AC_CAL/200323_1FE4_ADC123_3CLCES_19V5_1HZ_1MHZ.json')
    ADCCall.PlotTransferfunction('ADC1',PlotType='log')
    ADCCall.PlotTransferfunction('ADC2',PlotType='log')
    ADCCall.PlotTransferfunction('ADC3',PlotType='log')
    del ADCCall
    ADCCall = Met4FOFADCCall(Scope, Fgen, DR, 0x1FE40000)
    for freqs in nptestfreqs:
        ADCCall.CallFreq(freqs, 1.95, ['ADC1','ADC2','ADC3'])
    ADCCall.SaveFitresults('cal_data/1FE4_AC_CAL/200323_1FE4_ADC123_3CYCLES_1V95_1HZ_1MHZ.json')
    ADCCall.PlotTransferfunction('ADC1',PlotType='log')
    ADCCall.PlotTransferfunction('ADC2',PlotType='log')
    ADCCall.PlotTransferfunction('ADC3',PlotType='log')
    del ADCCall
    ADCCall = Met4FOFADCCall(Scope, Fgen, DR, 0x1FE40000)
    for freqs in nptestfreqs:
        ADCCall.CallFreq(freqs, 0.195, ['ADC1','ADC2','ADC3'])
    ADCCall.SaveFitresults('cal_data/1FE4_AC_CAL/200323_1FE4_ADC123_3CYCLES_V195_1HZ_1MHZ.json')
    ADCCall.PlotTransferfunction('ADC1',PlotType='log')
    ADCCall.PlotTransferfunction('ADC2',PlotType='log')
    ADCCall.PlotTransferfunction('ADC3',PlotType='log')