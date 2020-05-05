#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Aug  7 12:34:21 2019

@author: seeger01
"""

# !/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import scipy as scp
from scipy.optimize import curve_fit
import SineTools as st
import csv
import timeit
from MET4FOF_ADC_AC_CAL import Met4FOFADCCall as ADCCal
import time

from uncertainties import ufloat


# from termcolor import colored
# plt.rcParams.update({"font.size": 30})
# plt.rc("text", usetex=True)

class ReferencTransferFunction:
    def __init__(self, typ="hf_ref", filename="filename"):
        self.CSVData = pd.read_csv(filename, delimiter=";").set_index(
            ["loop", "frequency"]
        )

    def __GetNearestData(self, loop, freq):
        freqs = self.CSVData.loc[loop].index.values
        frqIDX = abs(freqs - freq).argmin()
        Nearestfreq = freqs[frqIDX]
        return self.CSVData.loc[loop, Nearestfreq]

    def __getitem__(self, key):
        if len(key) == 2:
            return self.__GetNearestData(key[0], key[1])
        if len(key) == 3:
            return self.__GetNearestData(key[0], key[1])[key[2]]


class CalTimeSeries:
    def SinFunc(self, x, A, B, f, phi):
        return A * np.sin(2 * np.pi * f * x + phi) + B

    def __init__(self):
        """


        Returns
        -------
        None.

        """
        self.length = 0
        self.flags = {
            "timeCalculated": False,
            "FFTCalculated": False,
            "BlockPushed": 0,
            "SineFitCalculated": [False, False, False, False],
        }

    def pushBlock(self, Datablock):
        """


        Parameters
        ----------
        Datablock : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        """
        if self.flags["BlockPushed"] > 0:
            self.Data = np.append(self.Data, Datablock, axis=0)
            self.flags["BlockPushed"] = self.flags["BlockPushed"] + 1
        if self.flags["BlockPushed"] == 0:
            self.Data = np.copy(Datablock)
            # IMPORTANT copy is nesseary otherwise only a reference is copyid
            # when pushing the second block the reference is changed
            # and the second block gets added two times
            self.flags["BlockPushed"] = self.flags["BlockPushed"] + 1

    def CalcTime(self):
        """


        Returns
        -------
        tmpTimeDiffs : TYPE
            DESCRIPTION.

        """
        self.Data[:, 4] = self.Data[:, 4] - self.Data[0, 4]
        # calculate time differences between samples
        tmpTimeDiffs = self.Data[1:, 4] - self.Data[:-1, 4]
        self.MeanDeltaT = np.mean(tmpTimeDiffs)  # use with numpy.fft.fftfreq
        self.DeltaTJitter = np.std(tmpTimeDiffs)
        self.flags["timeCalculated"] = True
        return tmpTimeDiffs

    def CalcFFT(self):
        """


        Returns
        -------
        None.

        """
        if not (self.flags["timeCalculated"]):  # recusive calling
            self.CalcTime()  # if time calculation hasent been done do it
        self.FFTData = np.fft.fft(self.Data[:, :4], axis=0) / (self.Data.shape[0])
        # FFT calculation
        self.fftFreqs = np.fft.fftfreq(self.Data.shape[0], self.MeanDeltaT)
        # calculate to fft bins coresponding freqs
        self.flags["FFTCalculated"] = True
        fftPeakindexiposfreq = int(self.Data.shape[0] / 3)
        # TODO axis is hard coded this is not good
        self.REFFfftPeakIndex = (
            np.argmax(abs(self.FFTData[1:fftPeakindexiposfreq, 3])) + 1
        )
        # print(self.REFFfftPeakIndex)
        # +1 is needed sice we use relative index in the array passed to np.argmax
        # ingnore dc idx>0 only positive freqs idx<int(length/2)
        # print("Max peak at axis 3 has index " + str(self.REFFfftPeakIndex))
        self.FFTFreqPeak = self.fftFreqs[self.REFFfftPeakIndex]
        # für OW der index wird richtig berechnent
        self.FFTAmplitudePeak = abs(self.FFTData[self.REFFfftPeakIndex, :])
        self.FFTPhiPeak = np.angle(self.FFTData[self.REFFfftPeakIndex, :])
        # für OW diese atribute haben ikorrekte werte
        # print("Found Peak at " + str(self.FFTFreqPeak) + " Hz")
        # print("Amplidues for all channels are" + str(self.FFTAmplitudePeak))
        # print("Phase angle for all channels are" + str(self.FFTPhiPeak))

    def PlotFFT(self):
        """


        Returns
        -------
        None.

        """
        if not (self.flags["timeCalculated"]):  # recusive calling
            self.CalcFFT()  # if fft calculation hasent been done do it
        fig, ax = plt.subplots()
        ax.plot(self.fftFreqs, abs(self.FFTData[:, 0]), label="Sensor x")
        ax.plot(self.fftFreqs, abs(self.FFTData[:, 1]), label="Sensor y")
        ax.plot(self.fftFreqs, abs(self.FFTData[:, 2]), label="Sensor z")
        ax.plot(self.fftFreqs, abs(self.FFTData[:, 3]), label="Ref")
        ax.set(
            xlabel="Frequency / Hz", ylabel="|fft| / AU", title="FFT of CalTimeSeries"
        )
        ax.grid()
        ax.legend()
        fig.show()

    def SinFit(self, EndCutOut, Methode="ST4SEQ", SimulateTimingErrors=False):
        """
        

        Parameters
        ----------
        EndCutOut : TYPE
            DESCRIPTION.
        Methode : TYPE, optional
            DESCRIPTION. The default is 'ST4'.

        Returns
        -------
        None.

        """
        if SimulateTimingErrors == "1000Hz":
            print("WARNING SIMULATING TIMING ERROR 100Hz")
            starttime = self.Data[0, 4]
            self.Data[:, 4] = np.arange(self.Data.shape[0]) * 1e-3 + starttime

        if SimulateTimingErrors == "constdist":
            print("WARNING SIMULATING TIMING ERROR CONST dist")
            starttime = self.Data[0, 4]
            endtime = self.Data[-1, 4]
            deltaT = endtime - starttime
            step = deltaT / (self.Data.shape[0] - 1)
            self.Data[:, 4] = np.arange(self.Data.shape[0]) * step + starttime
        self.popt = np.zeros([4, 4])
        self.pcov = np.zeros([4, 4, 4])
        # self.poptRaw=[None,None,None,None]
        self.sinFitEndCutOut = EndCutOut
        if not (self.flags["FFTCalculated"]):  # recusive calling
            self.CalcFFT()
        tmpbounds = (
            [0, -12, self.FFTFreqPeak - 3 * self.fftFreqs[1], -np.pi],
            [20, 12, self.FFTFreqPeak + 3 * self.fftFreqs[1], np.pi],
        )

        # TODO get bounds from FFT params
        for i in range(4):  # TODO remove this hard coded 4
            if Methode == "curve_fit":
                tmpp0 = [
                    self.FFTAmplitudePeak[i],
                    np.mean(self.Data[i]),
                    self.FFTFreqPeak,
                    self.FFTPhiPeak[i],
                ]
                try:
                    self.popt[i], self.pcov[i] = curve_fit(
                        self.SinFunc,
                        self.Data[:-EndCutOut, 4],
                        self.Data[:-EndCutOut, i],
                        bounds=tmpbounds,
                        p0=tmpp0,
                    )
                    self.flags["SineFitCalculated"][i] = True
                except RuntimeError as ErrorCode:
                    print("Runtime Error:" + str(ErrorCode))
                    self.flags["SineFitCalculated"][i] = False
                # print("Fiting at Freq " + str(self.FFTFreqPeak) + " at Axis" + str(i))
            if Methode == "ST4":
                # print("Fiting at Freq " + str(self.FFTFreqPeak) + " at Axis" + str(i))
                try:
                    tmpparams = st.fourparsinefit(
                        self.Data[:-EndCutOut, i],
                        self.Data[:-EndCutOut, 4],
                        self.FFTFreqPeak,
                    )
                except AssertionError as error:
                    print(error)
                    print("Skipping this fit")
                    tmpparams = [0, 0, 0, 0]
                Complex = tmpparams[1] + 1j * tmpparams[0]
                DC = tmpparams[2]
                Freq = tmpparams[3]
                self.popt[i] = [abs(Complex), DC, Freq, np.angle(Complex)]
            if Methode == "ST4SEQ":
                # print("Fiting at Freq " + str(self.FFTFreqPeak) + " at Axis" + str(i))
                try:
                    tmpparams = st.seq_fourparsinefit(
                        self.Data[:-EndCutOut, i],
                        self.Data[:-EndCutOut, 4],
                        self.FFTFreqPeak,
                        tol=1.0e-7,
                        nmax=1000,
                        periods=50,
                    )
                except AssertionError as error:
                    print(error)
                    print("Skipping this fit")
                    tmpparams = np.zeros((4, 4))
                Complex = tmpparams[:, 1] + 1j * tmpparams[:, 0]
                DC = tmpparams[:, 2]
                Freq = tmpparams[:, 3]
                self.popt[i] = [
                    np.mean(abs(Complex)),
                    np.mean(DC),
                    np.mean(Freq),
                    np.mean(np.unwrap(np.angle(Complex))),
                ]
                # np.fill_diagonal(self.pcov[i],tmpParamsSTD)
                # self.poptRaw[i]=CoVarData=np.stack((abs(Complex), DC, Freq, np.unwrap(np.angle(Complex))), axis=0)
                CoVarData = np.stack(
                    (abs(Complex), DC, Freq, np.unwrap(np.angle(Complex))), axis=0
                )
                self.pcov[i] = np.cov(
                    CoVarData, bias=True
                )  # bias=True Nomation With N like np.std

        self.flags["SineFitCalculated"] = True

    def PlotSinFit(self, AxisofIntrest):
        fig, ax = plt.subplots()
        ax.plot(
            self.Data[: -self.sinFitEndCutOut, 4],
            self.Data[: -self.sinFitEndCutOut, AxisofIntrest],
            ".-",
            label="Raw Data",
        )
        ax.plot(
            self.Data[: -self.sinFitEndCutOut, 4],
            self.SinFunc(
                self.Data[: -self.sinFitEndCutOut, 4], *self.popt[AxisofIntrest]
            ),
            label="Fit",
        )
        ax.set(
            xlabel="Time /s",
            ylabel="Amplitude /AU",
            title="Rawdata and IEEE 1075 4 Param sine fit",
        )
        ax.legend()
        fig.show()

    def PlotRaw(self, startIDX=0, stopIDX=0):
        fig, ax = plt.subplots()
        if startIDX == 0 and stopIDX == 0:
            ax.plot(self.Data[:, 4], self.Data[:, 0], label="Sensor x")
            ax.plot(self.Data[:, 4], self.Data[:, 1], label="Sensor y")
            ax.plot(self.Data[:, 4], self.Data[:, 2], label="Sensor z")
            ax.plot(self.Data[:, 4], self.Data[:, 3], label="Ref")
        else:
            ax.plot(
                self.Data[startIDX:stopIDX, 4],
                self.Data[startIDX:stopIDX, 0],
                label="Sensor x",
            )
            ax.plot(
                self.Data[startIDX:stopIDX, 4],
                self.Data[startIDX:stopIDX, 1],
                label="Sensor y",
            )
            ax.plot(
                self.Data[startIDX:stopIDX, 4],
                self.Data[startIDX:stopIDX, 2],
                label="Sensor z",
            )
            ax.plot(
                self.Data[startIDX:stopIDX, 4],
                self.Data[startIDX:stopIDX, 3],
                label="Ref z",
            )
        ax.set(xlabel="Time /s", ylabel="Amplitude /AU", title="Rawdata CalTimeSeries")
        ax.grid()
        ax.legend()
        fig.show()


class Databuffer:
    # TODO reject packages after i= Dataarraysize
    def __init__(self):
        """


        Returns
        -------
        None.

        """
        self.params = {
            "IntegrationLength": 64,
            "MaxChunks": 100000,
            "axixofintest": 1,
            "stdvalidaxis": 2,
            "minValidChunksInRow": 10,
            "minSTDforVailid": 1,
            "defaultEndCutOut": 750,
        }
        self.flags = {
            "AllSinFitCalculated": False,
            "AllFFTCalculated": False,
            "RefTransFunctionSet": False,
            "ADCTFSet": False,
        }
        self.DataLoopBuffer = np.zeros([self.params["IntegrationLength"], 5])
        self.i = 0
        self.IntegratedPacketsCount = 0

        self.FFTArray = np.zeros(
            [self.params["MaxChunks"], self.params["IntegrationLength"], 4]
        )
        self.STDArray = np.zeros([self.params["MaxChunks"], 4])
        # min STD to be an vaild chunk (there is AC-Component in the Signal)
        self.params["minValidChunksInRow"] = 10
        # at least this amount of chunks in a row have to be vaild to create an
        # CalTimeSeries object in CalData
        self.isValidCalChunk = np.zeros([self.params["MaxChunks"], 4])
        # bool array for is valid data flag for procesed blocks
        self.CalData = []  # list for vaild calibration data Chunks as CalTimeSeries

    def pushData(self, x, y, z, REF, t):
        """


        Parameters
        ----------
        x : TYPE
            DESCRIPTION.
        y : TYPE
            DESCRIPTION.
        z : TYPE
            DESCRIPTION.
        REF : TYPE
            DESCRIPTION.
        t : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        """
        if self.i < self.params["IntegrationLength"]:
            self.DataLoopBuffer[self.i] = [x, y, z, REF, t]
            # self.DataLoopBuffer[self.i, 0] = x
            # self.DataLoopBuffer[self.i, 1] = y
            # self.DataLoopBuffer[self.i, 2] = z
            # self.DataLoopBuffer[self.i, 3] = REF
            # self.DataLoopBuffer[self.i, 4] = t
            self.i = self.i + 1
            if self.i == self.params["IntegrationLength"]:
                self.calc()

    def pushBlock(self, arrayx, arrayy, arrayz, arrayREF, arrayt):
        """


        Parameters
        ----------
        arrayx : TYPE
            DESCRIPTION.
        arrayy : TYPE
            DESCRIPTION.
        arrayz : TYPE
            DESCRIPTION.
        arrayREF : TYPE
            DESCRIPTION.
        arrayt : TYPE
            DESCRIPTION.

        Raises
        ------
        ValueError
            DESCRIPTION.

        Returns
        -------
        None.

        """
        if not (
            self.params["IntegrationLength"]
            == arrayt.size
            == arrayy.size
            == arrayy.size
            == arrayz.size
        ):
            raise ValueError(
                "Array size must be INTEGRATIONTIME "
                + str(self.params["IntegrationLength"])
                + "and not "
                + str(arrayt.size)
            )
        self.DataLoopBuffer[:, 0] = arrayx
        self.DataLoopBuffer[:, 1] = arrayy
        self.DataLoopBuffer[:, 2] = arrayz
        self.DataLoopBuffer[:, 3] = arrayREF
        self.DataLoopBuffer[:, 4] = arrayt
        self.i = self.params["IntegrationLength"]
        self.calc()

    def calc(self):
        """


        Returns
        -------
        None.

        """
        if self.IntegratedPacketsCount < self.params["MaxChunks"]:
            self.STDArray[self.IntegratedPacketsCount] = np.std(
                self.DataLoopBuffer[:, :4], axis=0
            )
            if self.IntegratedPacketsCount > self.params["minValidChunksInRow"]:
                # check if we had sufficent at at least self.params['minValidChunksInRow']
                # blocks befor so we do not produce an segvault
                IPC = self.IntegratedPacketsCount
                VCIR = self.params["minValidChunksInRow"]
                isVaild = np.all(
                    np.greater(
                        self.STDArray[IPC - VCIR : IPC, self.params["stdvalidaxis"]],
                        self.params["minSTDforVailid"],
                    )
                )
                self.isValidCalChunk[self.IntegratedPacketsCount] = isVaild
                # check if we have an new valid data Chunk
                if self.isValidCalChunk[IPC, self.params["stdvalidaxis"]]:
                    # ok this are valid data
                    if not (self.isValidCalChunk[IPC - 1, self.params["stdvalidaxis"]]):
                        # we don't had an vaild data set befor this one so
                        # create a new CalTimeSeries Objec
                        self.CalData.append(CalTimeSeries())
                    self.CalData[-1].pushBlock(self.DataLoopBuffer)
                    # push data to CalTimeSeries Object
            self.IntegratedPacketsCount = self.IntegratedPacketsCount + 1
            self.i = 0

    def DoAllSinFit(self, EndCutOut):
        for item in self.CalData:
            item.SinFit(EndCutOut)
        self.flags["AllSinFitCalculated"] = True

    def DoAllFFT(self):
        for item in self.CalData:
            item.CalcFFT()
        self.flags["AllFFTCalculated"] = True

    def setRefTransferFunction(self, transferCSV, typ="ref_hf"):
        tmptype = typ
        self.RefTransferFunction = ReferencTransferFunction(
            typ=tmptype, filename=transferCSV
        )
        self.flags["RefTransFunctionSet"] = True
        self.flags["RefGroupDelaySet"] = False

    def setRefADCTF(self, CaldataFileList, ADCChannel="ADC1"):
        self.ADCTF = ADCCal(None, None, None, None, Filenames=CaldataFileList)
        self.flags["ADCTFSet"] = True
        self.ADCChannel = ADCChannel

    def getADCPhase(self, freq):
        if self.flags["ADCTFSet"] == False:
            print("WARN!!!! ADC TF NOT SET USEFOR DEBUGGING ONLY")
            return 0, 0
        else:
            Freqpoint = self.ADCTF[self.ADCChannel, freq]
            return Freqpoint["Phase"], Freqpoint["PhaseUncer"]

    def setRefGroupDelay(self, GropuDelay):
        # TODO improve selection between Ref Freq an group delay
        self.flags["RefTransFunctionSet"] = False
        self.flags["RefGroupDelaySet"] = True
        self.params["RefGroupDelay"] = GropuDelay

    def getNearestReFTPoint(self, freq, loop=0):
        if self.flags["RefTransFunctionSet"] == True:
            if not self.flags["RefGroupDelaySet"]:
                return (
                    self.RefTransferFunction[loop, freq, "acceleration"],
                    self.RefTransferFunction[loop, freq, "acceleration_std"],
                    self.RefTransferFunction[loop, freq, "phase"] / 180 * np.pi,
                    self.RefTransferFunction[loop, freq, "phase_std"] / 180 * np.pi,
                )
            else:
                # print("Using GROUPDELAY WITHOUT AMPLITUDE AS REFERENCE TRANSFER FUNCTION")
                return (
                    1,  # Asume Amplitude as One
                    0,  # No amplitude Error asumed
                    freq
                    * self.params["RefGroupDelay"]
                    * 2
                    * np.pi
                    * -1,  # -1 because we are seeing this aus transferfunction of the Ref Mesurment
                    # so an positiv grouddelay results in in linear increasing negative phase
                    0,
                )  # No Phase Error asumed
        else:
            raise RuntimeWarning("REFTransferFunction NOT SET RETURNING 0,0")
            return (1, 0, 0, 0)

    def getTransferCoevs(
        self, axisDUT, AxisRef=3, RefPhaseDC=-np.pi
    ):
        if not self.flags["AllSinFitCalculated"]:
            print(
                "Doing Sin Fit with default  end cut out length "
                + str(self.params["defaultEndCutOut"])
            )
            self.DoAllSinFit(self.params["defaultEndCutOut"])
        self.TransferFreqs = np.zeros(len(self.CalData))
        self.TransferAmpl = np.zeros(len(self.CalData))
        self.TransferAmplErr = np.zeros(len(self.CalData))
        self.TransferPhase = np.zeros(len(self.CalData))
        self.TransferPhaseErr = np.zeros(len(self.CalData))
        self.TransferRunCount = np.zeros(len(self.CalData))
        i = 0  # number of packets processed
        Runcount = (
            0  # number of loops processed so far loop is an series of rising frequencys
        )
        for item in self.CalData:
            Freq = self.TransferFreqs[i] = self.CalData[i].popt[axisDUT, 2]
            RefData = self.getNearestReFTPoint(Freq, loop=self.TransferRunCount[i])
            AmplTF = RefData[0]
            AmplTFErr = RefData[1]*2# 2* for 95% coverage
            PhaseTF = RefData[2]
            PhaseTFErr = RefData[3]*2# 2* for 95% coverage
            DUTFit=self.CalData[i].popt[axisDUT, 0]
            DUTFitErr=np.sqrt(abs(self.CalData[i].pcov[axisDUT, 0, 0]))*2 # 2* for 95% coverage
            # S=DUT/REF
            self.TransferAmpl[i] =DUTFit/AmplTF
            # A/B da-->1/B
            COEV1=(1/AmplTF*DUTFitErr)
            # A/B db -->-A/B^2
            COEV2=(-1*DUTFit/(AmplTF*AmplTF)*AmplTFErr)
            self.TransferAmplErr[i] = np.sqrt(
                COEV1*COEV1+
                COEV2*COEV2
            )
            TransferPhasetmp = (
                self.CalData[i].popt[axisDUT, 3] - self.CalData[i].popt[AxisRef, 3]
            )
            TransferPhasetmpErr = (self.CalData[i].pcov[AxisRef, 3,3])*2# 2* for 95% coverage

            ADCPhase, ADCPhaseErr = self.getADCPhase(Freq)
            self.TransferPhase[i] = TransferPhasetmp - PhaseTF + ADCPhase + RefPhaseDC
            self.TransferPhaseErr[i]=np.sqrt((TransferPhasetmpErr*TransferPhasetmpErr) #A-B+C+D dA--> 1
                                             +(PhaseTFErr*PhaseTFErr) #A-B+C+D dB--> -1
                                             +(ADCPhaseErr*ADCPhaseErr))  #A-B+C+D dC--> 1
                                            # RefPhaseDC is an constant and has an uncertainty of 0
            # detect run count based on frequency drop to do right unwraping
            self.TransferRunCount[i] = Runcount
            # print("Freq:"+str(Freq)+"Ampl Fit: "+str(self.CalData[i].popt[axisDUT, 0])+"Ampl Ref: "+str(AmplTF))
            if i > 0:
                if (
                    self.TransferFreqs[i] * 1.01 < self.TransferFreqs[i - 1]
                ):  # multiply with 1.01 in case of same freqs
                    # ok the freq now is smaller the the last freq we have enterd a new loop
                    Runcount = Runcount + 1
                    self.TransferRunCount[i] = Runcount
            i = i + 1
        for run in range(Runcount + 1):
            runIDX = self.TransferRunCount == run
            transferfunctionunwraped = np.unwrap(DB1.TransferPhase[runIDX])
            if all(transferfunctionunwraped <= (-2 * np.pi)):
                transferfunctionunwraped = transferfunctionunwraped + 2 * np.pi
            self.TransferPhase[runIDX] = transferfunctionunwraped

    def GetTransferFunction(self,axisDUT, AxisRef=3, RefPhaseDC=-np.pi):
        self.getTransferCoevs(axisDUT, AxisRef=AxisRef, RefPhaseDC=RefPhaseDC)
        nominalFreqs=[]
        for freqs in self.TransferFreqs:
            if np.rint(freqs*10)/10 in nominalFreqs:# *10/10 for x.1 digit fixpoint resolution
                pass
            else:
                nominalFreqs.append(np.rint(freqs*10)/10)# *10/10 for x.1 digit fixpoint resolution
        FreqNum=len(nominalFreqs)
        self.Transferfunction={'Frequencys':np.array(nominalFreqs),
                          'AmplitudeCoefficent':np.zeros(FreqNum),
                          'AmplitudeCoefficentUncer':np.zeros(FreqNum),
                          'Phase':np.zeros(FreqNum),
                          'PhaseUncer':np.zeros(FreqNum),
                          'N':np.zeros(FreqNum)}
        roundedFreqs=np.rint(self.TransferFreqs*10)/10
        for i in range(len(nominalFreqs)):
            IDX=np.where(roundedFreqs==nominalFreqs[i])
            npIDX=np.array(IDX[0])
            N=npIDX.size
            TMPAmplitudes=self.TransferAmpl[IDX]
            TMPAmplitudesErr = self.TransferAmplErr[IDX]
            Amps=np.full(N,ufloat(0,0))
            ampsum=ufloat(0,0)
            weights=np.full(N,ufloat(0,0))
            weighted = np.full(N,ufloat(0,0))
            for j in range(N):
                Amps[j]=ufloat(TMPAmplitudes[j],TMPAmplitudesErr[j])
                ampsum=ampsum+ufloat(TMPAmplitudes[j],TMPAmplitudesErr[j])
                weights[j]=ufloat(1/TMPAmplitudesErr[j],0)
                weighted[j]=Amps[j]*weights[j]
            amp = ampsum/N
            wightsum=np.sum(weights)
            ampWMean=np.sum(weighted)/wightsum
            tmpmean=np.mean(Amps)
            diff=weighted-np.mean(weighted)
            tmpstd=np.sum(diff)
            #tmpstd = np.sum(diff)/wightsum
            TMPPhases=self.TransferPhase[IDX]
            TMPPhasesErr=self.TransferPhaseErr[IDX]
            Phases = np.full(N,ufloat(0,0))
            phasessum = ufloat(0, 0)
            for j in range(N):
                Phases[j] = ufloat(TMPPhases[j], TMPPhasesErr[j])
                phasessum = phasessum + ufloat(TMPPhases[j], TMPPhasesErr[j])
            phase=np.mean(Phases)
            phasetmp =phasessum/N
            self.Transferfunction['AmplitudeCoefficent'][i]=amp.nominal_value
            self.Transferfunction['AmplitudeCoefficentUncer'][i]=amp.std_dev
            self.Transferfunction['Phase'][i]= phase.nominal_value
            self.Transferfunction['PhaseUncer'][i] = phase.std_dev
            self.Transferfunction['N'][i] = len(IDX)
        return self.Transferfunction

    def PlotTransferFunction(self, PlotType="lin"):
        fig, (ax1, ax2) = plt.subplots(2, 1)
        if PlotType == "logx":
            ax1.set_xscale("log")
            ax2.set_xscale("log")
        fig.suptitle("Transfer function ")
        ax1.set_ylabel("Relative magnitude $|S|$")
        ax1.grid(True)
        for run in range(int(np.max(self.TransferRunCount)) + 1):
            runIDX = self.TransferRunCount == run
            ax1.errorbar(
                self.TransferFreqs[runIDX],
                self.TransferAmpl[runIDX],
                yerr=self.TransferAmplErr[runIDX],
                fmt=".",
                markersize=20,
                label=str(run),
            )

            ax2.errorbar(
                self.TransferFreqs[runIDX],
                self.TransferPhase[runIDX] / np.pi * 180,
                yerr=self.TransferPhaseErr[runIDX]/ np.pi * 180,
                fmt=".",
                markersize=20,
                label=str(run),
            )

        ax1.errorbar(
            self.Transferfunction['Frequencys'],
            self.Transferfunction['AmplitudeCoefficent'],
            yerr=self.Transferfunction['AmplitudeCoefficentUncer'],
            fmt='s',
            markersize=5,
            label='Mean',
        )
        ax2.errorbar(
            self.Transferfunction['Frequencys'],
            self.Transferfunction['Phase']/ np.pi * 180,
            yerr=self.Transferfunction['PhaseUncer']/ np.pi * 180,
            fmt='s',
            markersize=5,
            label='Mean',
        )
        ax2.set_xlabel(r"Frequency $f$ in Hz")
        ax2.set_ylabel(r"Phase $\Delta\varphi$ in °")
        ax2.grid(True)
        ax1.legend(numpoints=1, fontsize=8, ncol=3)
        ax2.legend(numpoints=1, fontsize=8, ncol=3)
        plt.show()

    def PrintTransferParams(self):
        PDTransferParams = pd.DataFrame(
            {
                "Freq": self.TransferFreqs,
                "Ampl": self.TransferAmpl,
                "AmplErr": self.TransferAmplErr,
                "Phase Deg": self.TransferPhase / np.pi * 180,
                "Phase Deg Err": self.TransferPhaseErr / np.pi * 180,
                "Phase Rad": self.TransferPhase,
                "Phase Rad Err": self.TransferPhase,
            }
        )
        print(PDTransferParams)
        return PDTransferParams

    def PlotSTDandValid(self, startIDX=0, stopIDX=0):
        fig, (ax1, ax2) = plt.subplots(2, 1)
        self.Chunktimes = np.arange(self.STDArray.shape[0])
        tmpdeltaT = (
            DB1.DataLoopBuffer[self.params["IntegrationLength"] - 1, 4]
            - DB1.DataLoopBuffer[0, 4]
        )
        print(tmpdeltaT)
        self.Chunktimes = self.Chunktimes * tmpdeltaT
        # calculationg delta time from last vaild chunk
        if startIDX == 0 and stopIDX == 0:
            ax1.plot(self.Chunktimes, self.STDArray[:, 0], label="Sensor x")
            ax1.plot(self.Chunktimes, self.STDArray[:, 1], label="Sensor y")
            ax1.plot(self.Chunktimes, self.STDArray[:, 2], label="Sensor z")
            ax1.plot(self.Chunktimes, self.STDArray[:, 3], label="Ref")
        else:
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 0],
                label="Sensor x",
            )
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 1],
                label="Sensor y",
            )
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 2],
                label="Sensor z",
            )
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 3],
                label="Ref",
            )
        ax1.title.set_text(
            "Short term standard deviation  $\sigma$ (width of "
            + str(self.params["IntegrationLength"])
            + ") of signal amplitude"
        )
        ax1.set_ylabel("STD  $\sigma$ in a.u.")
        ax1.legend()
        ax1.grid(True)
        if startIDX == 0 and stopIDX == 0:
            ax2.plot(self.Chunktimes, self.isValidCalChunk)
        else:
            ax2.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.isValidCalChunk[startIDX:stopIDX],
                ".",
            )
            ax2.set_yticks([0, 1])
        ax2.title.set_text("Valid data")
        ax2.set_xlabel("Time in s")
        ax2.set_ylabel("Valid = 1")
        ax2.xaxis.grid()  # vertical lines
        plt.subplots_adjust(hspace=0.3)
        plt.show()

    def PlotSTD(self, startIDX=0, stopIDX=0):
        fig, ax1 = plt.subplots(1, 1)
        self.Chunktimes = np.arange(self.STDArray.shape[0])
        tmpdeltaT = (
            DB1.DataLoopBuffer[self.params["IntegrationLength"] - 1, 4]
            - DB1.DataLoopBuffer[0, 4]
        )
        self.Chunktimes = self.Chunktimes * tmpdeltaT
        # calculationg delta time from last vaild chunk
        if startIDX == 0 and stopIDX == 0:
            ax1.plot(self.Chunktimes, self.STDArray[:, 0], label="Sensor x")
            ax1.plot(self.Chunktimes, self.STDArray[:, 1], label="Sensor y")
            ax1.plot(self.Chunktimes, self.STDArray[:, 2], label="Sensor z")
            ax1.plot(self.Chunktimes, self.STDArray[:, 3], label="Ref z")
        else:
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 0],
                label="Sensor x",
            )
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 1],
                label="Sensor y",
            )
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 2],
                label="Sensor z",
            )
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 3],
                label="Ref z",
            )
        ax1.title.set_text(
            "Short term standard deviation  $\sigma$ (width of "
            + str(self.params["IntegrationLength"])
            + ") of signal amplitude"
        )
        ax1.set_ylabel("STD  $\sigma$ in a.u.")
        ax1.set_xlabel("Time in s")
        ax1.legend()
        ax1.grid(True)
        plt.show()


def DataReaderGYROdump(ProtoCSVFilename):
    sdf = pd.read_csv(ProtoCSVFilename, delimiter=";", index_col=False, skiprows=1)
    print(sdf.columns.values)
    chunkSize = DB1.params["IntegrationLength"]
    for Index in np.arange(0, len(sdf), chunkSize)[
        :-1
    ]:  # don't use last chuck since this will propably not have chnukSize elements
        DB1.pushBlock(
            sdf["GYR_x"][Index : Index + chunkSize] * (180 / np.pi),
            sdf["GYR_y"][Index : Index + chunkSize] * (180 / np.pi),
            sdf["GYR_z"][Index : Index + chunkSize] * (180 / np.pi),
            sdf["ADC_1"][Index : Index + chunkSize] * 100,
            (sdf["unix_time"][Index : Index + chunkSize])
            + (sdf["unix_time_nsecs"][Index : Index + chunkSize]) * 1e-9,
        )


def DataReaderGYROdumpLARGE(Databuffer, ProtoCSVFilename, linestoread=0):
    reader = csv.reader(open(ProtoCSVFilename), delimiter=";")
    print("reading first to rows")
    print(next(reader))
    print(next(reader))
    # we sacrifice a line to set the start time stamp
    line = next(reader)
    startsec = float(line[2])
    i = 0
    if i == 0:
        linestoread = (
            Databuffer.params["IntegrationLength"] * Databuffer.params["MaxChunks"]
        )
    # TODO add "static" var for first time stamp to have relative times to avoid quantisation error
    while i < linestoread:
        # ['id', 'sample_number', 'unix_time', 'unix_time_nsecs', 'time_uncertainty', 'Data_01', 'Data_02', 'Data_03', 'Data_04', 'Data_05', 'Data_06', 'Data_07', 'Data_08', 'Data_09', 'Data_10', 'Data_11', 'Data_12', 'Data_13', 'Data_14', 'Data_15', 'Data_16']
        try:
            line = next(reader)
            time = float(line[2]) - startsec + (float(line[3]) * 1e-9)
            # pushData(self, x, y, z, REF, t)
            Databuffer.pushData(
                float(line[8]) * (180 / np.pi),
                float(line[9]) * (180 / np.pi),
                float(line[10]) * (180 / np.pi),
                float(line[15]) * 100,
                time,
            )
            i = i + 1
        except Exception as e:
            print(e)
            break
    print(str(i) + " Lines read Data Parsing finsihed")
    return


def DataReaderACCdumpLARGE(Databuffer, ProtoCSVFilename, linestoread=0):
    chunksize = Databuffer.params["IntegrationLength"]
    reader = csv.reader(open(ProtoCSVFilename), delimiter=";")
    print("reading first to rows")
    print(next(reader))
    print(next(reader))
    # we sacrifice a line to set the start time stamp
    line = next(reader)
    startsec = float(line[2])
    j = 0
    i = 0
    x = np.zeros(chunksize)
    y = np.zeros(chunksize)
    z = np.zeros(chunksize)
    REF = np.zeros(chunksize)
    t = np.zeros(chunksize)

    if i == 0:
        linestoread = (
            Databuffer.params["IntegrationLength"] * Databuffer.params["MaxChunks"]
        )
    # TODO add "static" var for first time stamp to have relative times to avoid quantisation error
    while i < linestoread:
        # ['id', 'sample_number', 'unix_time', 'unix_time_nsecs', 'time_uncertainty', 'Data_01', 'Data_02', 'Data_03', 'Data_04', 'Data_05', 'Data_06', 'Data_07', 'Data_08', 'Data_09', 'Data_10', 'Data_11', 'Data_12', 'Data_13', 'Data_14', 'Data_15', 'Data_16']
        #  0      1                2               3                    4               5           6          7         8          9         10
        try:
            line = next(reader)
            time = float(line[2]) - startsec + (float(line[3]) * 1e-9)
            # pushData(self, x, y, z, REF, t)
            if j < chunksize:
                x[j] = float(line[5])
                y[j] = float(line[6])
                z[j] = float(line[7])
                REF[j] = float(line[15]) * 100
                t[j] = time
                j = j + 1
            if j == chunksize:
                Databuffer.pushBlock(x, y, z, REF, t)
                j = 0
                x = np.zeros(chunksize)
                y = np.zeros(chunksize)
                z = np.zeros(chunksize)
                REF = np.zeros(chunksize)
                t = np.zeros(chunksize)
                x[j] = float(line[5])
                y[j] = float(line[6])
                z[j] = float(line[7])
                REF[j] = float(line[15]) * 100.0
                t[j] = time
                j = j + 1
            # Databuffer.pushData(float(line[5]),
            #                     float(line[6]),
            #                     float(line[7]),
            #                     float(line[15])*100,
            #                     time)
            i = i + 1
        except Exception as e:
            print(e)
            break
    print(
        str(i)
        + "for a max of "
        + str(linestoread)
        + " Lines read Data Parsing finsihed"
    )
    return


# Column Names
# 'id'
# 'sample_number'
# 'unix_time'
# 'unix_time_nsecs'
# 'time_uncertainty'
# 'Data_01'
# 'Data_02'
# 'Data_03'
# 'Data_04'
# 'Data_05'
# 'Data_06'
# 'Data_07'
# 'Data_08'
# 'Data_09'
# 'Data_10'
# 'Data_11'
# 'Data_12'
# 'Data_13'
# 'stimfeq'
# 'stimampl'
# 'stimtype'

if __name__ == "__main__":
    start_time = time.time()
    DB1 = Databuffer()
    DB1.setRefADCTF(
        [
            "cal_data/1FE4_AC_CAL/200318_1FE4_ADC123_19V5_1HZ_1MHZ.json",
            "cal_data/1FE4_AC_CAL/200319_1FE4_ADC123_19V5_1HZ_1MHZ.json",
        ],
        ADCChannel="ADC1",
    )
    DB1.setRefTransferFunction(
        "D:/data/2020-03-03_Messungen_MPU9250_SN_IMEKO_Frequenzgang_Firmware_0.3.0/mpu9250_imeko_10_hz_250_hz_6wdh.csv"
    )
    # DataReaderPROTOdump('data/20190826_300Hz_LP_10-250Hz_10ms2.csv')
    # DataReaderPROTOdump('data/20190819_1500_10_250hz_10_ms2_woairatstart.dump')
    # DataReaderPROTOdump('data/20190827_300Hz_LP_10x_10_250Hz_10ms2.csv')
    # DataReaderPROTOdump('data/20190904_300Hz_LP_1x_10_250Hz_10ms2_1_4bar.csv')
    # DataReaderPROTOdump('data/20190904_300Hz_LP_1x_10_250Hz_10ms2_10_4bar.csv')
    # DataReaderPROTOdump('data/20190904_300Hz_LP_1x_10_250Hz_10ms2_10_4bar_2.csv')
    # DataReaderPROTOdump('data/20190904_300Hz_LP_1x_10_250Hz_10ms_BK4809_1.csv')
    # DataReaderPROTOdump("data/20190918_10_250_HZ_10_ms2_300HzTP_neuer_halter.csv")
    # DataReaderPROTOdump("data/191001_BMA280_10_250_10ms2_1.csv")
    # DataReaderGYROdump("data/20191112_frequenzgang_0.4Hz-100Hz_100deg_s.csv")
    # DataReaderGYROdump("data/20191112_10Hz_amplgang_100_200_400_800_1600_degSek.csv")
    # DataReaderGYROdumpLARGE(DB1,"/media/seeger01/Part1/191216_MPU_9250_Z_Achse/191612_MPU_9250_Z_Rot_150_Wdh/20191216153445_MPU_9250_0x1fe40000.dump")
    # DataReaderGYROdumpLARGE(DB1,"/data/191218_MPU_9250_X_Achse_150_Wdh/20191218134946_MPU_9250_0x1fe40000.dump")
    # DataReaderGYROdumpLARGE(DB1,"/data/20191217100017_MPU_9250_0x1fe40000.dump")#/191617_MPU_9250_Y_Rot_100_Wdh/
    DataReaderACCdumpLARGE(
        DB1,
        "D:/data/2020-03-03_Messungen_MPU9250_SN_IMEKO_Frequenzgang_Firmware_0.3.0/mpu9250_imeko_10_hz_250_hz_6wdh.dump",
    )
    # DataReaderACCdumpLARGE(DB1,"D:/data/2020-03-03_Messungen_MPU9250_SN12 Frequenzgang_Firmware_0.3.0/mpu9250_12_10_hz_250_Hz_6wdh.dump")

    # reading data from file and proces all Data
    DB1.DoAllFFT()
    DB1.getTransferCoevs(2, RefPhaseDC=-np.pi)
    DB1.GetTransferFunction(2, RefPhaseDC=-np.pi)
    DB1.PlotTransferFunction()
    DB1.PlotTransferFunction(PlotType="logx")
    print("--- %s seconds ---" % (time.time() - start_time))
    # fstats=yappi.get_func_stats()
    # fstats.save(datetime.datetime.now().strftime("%Y%m%d%H%M%S")+'performance.out.', 'CALLGRIND')
    # DB2 = Databuffer()
    # DataReaderACCdumpLARGE(DB2,"D:/data/2020-03-03_Messungen_MPU9250_SN12 Frequenzgang_Firmware_0.3.0/mpu9250_12_10_hz_250_Hz_6wdh.dump")
    # DB2.setRefTransferFunction("D:/data/2020-03-03_Messungen_MPU9250_SN12 Frequenzgang_Firmware_0.3.0/mpu9250_12_10_hz_250_Hz_6wdh.csv")
    # reading data from file and proces all Data
    # DB2.DoAllFFT()
    # DB2.getTransferFunction(2,RefPhaseDC=-np.pi)
    # DB2.PlotTransferFunction()
    # DB2.PlotTransferFunction(PlotType="logx")

    # callculate all the ffts
