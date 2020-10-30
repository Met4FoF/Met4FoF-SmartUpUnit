#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Aug  7 12:34:21 2019

@author: seeger01
"""

# !/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Imu
import sys
import os
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
from scipy.stats import chi2
import json
from pathlib import Path
from cycler import cycler # for colored markers

#from uncertainties import ufloat


# from termcolor import colored
scalefactor=3
SMALL_SIZE = 8*scalefactor
MEDIUM_SIZE = 10*scalefactor
BIGGER_SIZE = 12*scalefactor

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)
plt.rc('lines', linewidth=scalefactor)
# plt.rc("text", usetex=True)


#https://stackoverflow.com/questions/26646362/numpy-array-is-not-json-serializable
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

def GetNearestTestFreq(freq,TestFreqs=[4.0,5.0,6.3,8.0,10.0,12.5,16.0,20.0,25.0,31.5,40.0,46.7,50.0,53.3,63.0,80.0,100.0,125.0,160.0,200.0,250.0]):
    frqIDX = abs(TestFreqs - freq).argmin()
    #print("got "+str(freq)+" will return "+str(TestFreqs[frqIDX]))
    return TestFreqs[frqIDX]

class ReferencTransferFunction:
    def __init__(self, typ="hf_ref", filename="filename"):
        self.CSVData = pd.read_csv(filename, delimiter=";",skiprows=[1,2]).set_index(
            ["loop", "frequency"]
        )

    def __GetNearestData(self, loop, freq):
        print("loop "+str(loop)+" freq "+str(freq))
        freqs = self.CSVData.loc[loop].index.values
        frqIDX = abs(freqs - freq).argmin()
        Nearestfreq = freqs[frqIDX]
        return self.CSVData.loc[loop, Nearestfreq]

    def __getitem__(self, key):
        if len(key) == 2:
            return self.__GetNearestData(key[0], key[1])
        if len(key) == 3:
            return self.__GetNearestData(key[0], key[1])[key[2]]

    def Plot(self, PlotType="lin"):
        fig, (ax1, ax2) = plt.subplots(2, 1)
        if PlotType == "logx":
            ax1.set_xscale("log")
            ax2.set_xscale("log")
        fig.suptitle("Analog Calibration System Data")
        ax1.set_ylabel("Absolute Amplitude $|A|$")
        ax1.grid(True)
        freq=[lis[1] for lis in self.CSVData.index.values]
        ax1.errorbar(
            freq,
            self.CSVData['ex_amp'].values,
            yerr=self.CSVData['ex_amp_std'].values,
            markersize=20,
            fmt=".",
        )
        ax2.errorbar(
            freq,
            self.CSVData['phase'].values,
            yerr=self.CSVData['phase_std'],
            markersize=20,
            fmt=".",
        )
        ax2.set_xlabel(r"Frequency $f$ in Hz")
        ax2.set_ylabel(r"Phase $\Delta\varphi$ in °")
        ax2.grid(True)
        ax1.legend(numpoints=1, fontsize=8, ncol=3)
        ax2.legend(numpoints=1, fontsize=8, ncol=3)
        plt.tight_layout()
        plt.show()


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
        self.params={ "sinFitEndCutOut":0}
        print("CalTimeSeries created")

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
        self.length=self.length+Datablock.shape[0]

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
        plt.tight_layout()
        fig.show()

    def SinFit(self, EndCutOut, Methode="ST3SEQ", SimulateTimingErrors=False):
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
        self.params['sinFitEndCutOut'] = EndCutOut
        if not (self.flags["FFTCalculated"]):  # recusive calling
            self.CalcFFT()
        tmpbounds = (
            [0, -12, self.FFTFreqPeak - 3 * self.fftFreqs[1], -np.pi],
            [20, 12, self.FFTFreqPeak + 3 * self.fftFreqs[1], np.pi],
        )
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
                    if int(self.FFTFreqPeak)==0:
                        N=10;
                    else:
                        N=int(self.FFTFreqPeak)*3
                    tmpparams = st.seq_fourparsinefit(
                        self.Data[:-EndCutOut, i],
                        self.Data[:-EndCutOut, 4],
                        self.FFTFreqPeak,
                        tol=1.0e-9,
                        nmax=5000,
                        periods=N,
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
            if Methode == "ST3SEQ" or Methode == "ST3SEQFitFreq":

                try:
                    if int(self.FFTFreqPeak)==0:
                        N=10;
                    else:
                        N=int(self.FFTFreqPeak)*5
                    testFfreq=GetNearestTestFreq(self.FFTFreqPeak)
                    if Methode=="ST3SEQFitFreq":
                        abcd=st.fourparsinefit(self.Data[:-EndCutOut, i],
                            self.Data[:-EndCutOut, 4],
                            testFfreq,
                            tol=1.0e-8,
                            nmax=10000,)
                        testFfreq=abcd[3]
                    print("Fiting at Freq " + str(testFfreq) + " at Axis" + str(i))
                    tmpparams = st.seq_threeparsinefit(
                        self.Data[:-EndCutOut, i],
                        self.Data[:-EndCutOut, 4],
                        testFfreq,
                        periods=N,
                    )
                except AssertionError as error:
                    print(error)
                    print("Skipping this fit")
                    tmpparams = np.zeros((4, 4))
                except np.linalg.LinAlgError as error:
                    print(error)
                    print("Skipping this fit")
                    tmpparams = np.zeros((4, 4))
                Complex = tmpparams[:, 1] + 1j * tmpparams[:, 0]
                DC = tmpparams[:, 2]
                Freq = np.ones_like(DC)*testFfreq
                tmpAngles=np.angle(Complex)
                DeltatmpAngles=(tmpAngles-np.mean(tmpAngles))/np.pi*180
                self.popt[i] = [
                    np.mean(abs(Complex)),
                    np.mean(DC),
                    np.mean(Freq),
                    np.mean(np.unwrap(np.angle(Complex))),
                ]
                # np.fill_diagonal(self.pcov[i],tmpParamsSTD)
                # self.poptRaw[i]=CoVarData=np.stack((abs(Complex), DC, Freq, np.unwrap(np.angle(Complex))), axis=0)
                self.MultiFitResults = np.stack(
                    (abs(Complex), DC, Freq, np.unwrap(np.angle(Complex))), axis=0
                )
                self.pcov[i] = np.cov(
                    self.MultiFitResults, bias=True
                )  # bias=True Nomation With N like np.std
                #print(self.pcov[i])
        self.flags["SineFitCalculated"] = True

    def PlotSinFit(self, AxisofIntrest):
        fig, ax = plt.subplots()
        ax.plot(
            self.Data[: -self.params['sinFitEndCutOut'] , 4],
            self.Data[: -self.params['sinFitEndCutOut'] , AxisofIntrest],
            ".-",
            label="Raw Data",
        )
        ax.plot(
            self.Data[: -self.params['sinFitEndCutOut'] , 4],
            self.SinFunc(
                self.Data[: -self.params['sinFitEndCutOut'] , 4], *self.popt[AxisofIntrest]
            ),
            label="Fit",
        )
        ax.set(
            xlabel="Time /s",
            ylabel="Amplitude /AU",
            title="Rawdata and IEEE 1075 4 Param sine fit",
        )
        ax.legend()
        plt.tight_layout()
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
                label="x Axis",
            )
            ax.plot(
                self.Data[startIDX:stopIDX, 4],
                self.Data[startIDX:stopIDX, 1],
                label="Y Axis",
            )
            ax.plot(
                self.Data[startIDX:stopIDX, 4],
                self.Data[startIDX:stopIDX, 2],
                label="Z Axis",
            )
            ax.plot(
                self.Data[startIDX:stopIDX, 4],
                self.Data[startIDX:stopIDX, 3],
                label="Sync",
            )
        ax.set(xlabel="Time /s", ylabel="Amplitude /AU", title="Rawdata CalTimeSeries")
        ax.grid()
        ax.legend()
        plt.tight_layout()
        fig.show()

    def PlotSamplingFreq(self,fig = None,ax = None,label=None,legend=True,color='blue',fset=1000,average=0):
        if fig==None and ax==None:
            fig, ax = plt.subplots()
        if label==None:
            label = "Sampling frequency"
        time=self.Data[:-1, 4]
        time=time-self.Data[0, 4]
        DeltaT=np.diff(time)
        DeltaFSample=1/DeltaT-fset
        T = self.Data[-1, 4]-self.Data[0, 4]
        if average!=0:
            DeltaFSample = np.convolve(DeltaFSample,np.ones(average), 'valid')/average
            time=time[:-average]
        ax.plot(
            time,
            DeltaFSample,
            label=label,
            linewidth=1,
            color=color
            )
        ax.set(xlabel="Time /s", ylabel="Sampling frequency deviation in Hz")# ,title="Sampling frequency deviation from set Value ("+str(fset)+" Hz)")
        if legend:
            ax.legend()
        fig.show()
        return fig,ax




class Databuffer:
    # TODO reject packages after i= Dataarraysize
    def __init__(self,params={}):
        """


        Returns
        -------
        None.

        """
        self.Defaultparams = {
            "IntegrationLength": 128,
            "MaxChunks": 250000,
            "axixofintest": 2,
            "stdvalidaxis": 3,
            "minValidChunksInRow": 100,
            "minSTDforVailid": 10,
            "defaultEndCutOut": 128*30,
        }
        self.params = self.Defaultparams.copy()  # start with x's keys and values
        self.params.update(params)
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
            DESCRIPTION.5
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
        print(freq,loop)
        if self.flags["RefTransFunctionSet"] == True:
            if not self.flags["RefGroupDelaySet"]:
                return np.array([
                    self.RefTransferFunction[loop, freq, "ex_amp"],
                    self.RefTransferFunction[loop, freq, "ex_amp_std"],
                    self.RefTransferFunction[loop, freq, "phase"] / 180 * np.pi,
                    self.RefTransferFunction[loop, freq, "phase_std"] / 180 * np.pi,
                ]).flatten()
            else:
                # print("Using GROUPDELAY WITHOUT AMPLITUDE AS REFERENCE TRANSFER FUNCTION")
                return np.array([
                    1,  # Asume Amplitude as One
                    0,  # No amplitude Error asumed
                    freq
                    * self.params["RefGroupDelay"]
                    * 2
                    * np.pi
                    * -1,  # -1 because we are seeing this aus transferfunction of the Ref Mesurment
                    # so an positiv grouddelay results in in linear increasing negative phase
                    0,
                ]).flatten() # No Phase Error asumed
        else:
            raise RuntimeWarning("REFTransferFunction NOT SET RETURNING 0,0")
            return np.array([1, 0, 0, 0]).flatten()

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
        self.ExAmpl = np.zeros(len(self.CalData))
        self.TransferAmplErr = np.zeros(len(self.CalData))
        self.TransferPhase = np.zeros(len(self.CalData))
        self.TransferPhaseErr = np.zeros(len(self.CalData))
        self.TransferRunCount = np.zeros(len(self.CalData))
        i = 0  # number of packets processed
        Runcount = (
            0  # number of loops processed so far loop is an series of rising frequencys
        )
        for item in self.CalData:
            # detect run count based on frequency drop to do right unwraping
            self.TransferRunCount[i] = Runcount
            Freq = self.TransferFreqs[i] = self.CalData[i].popt[axisDUT, 2]
            # print("Freq:"+str(Freq)+"Ampl Fit: "+str(self.CalData[i].popt[axisDUT, 0])+"Ampl Ref: "+str(AmplTF))
            if i > 0:
                if (
                        self.TransferFreqs[i] * 1.01 < self.TransferFreqs[i - 1]
                ):  # multiply with 1.01 in case of same freqs
                    # ok the freq now is smaller the the last freq we have enterd a new loop
                    Runcount = Runcount + 1
                    self.TransferRunCount[i] = Runcount



            RefData = self.getNearestReFTPoint(Freq, loop=int(self.TransferRunCount[i]))
            AmplTF = RefData[0]
            self.ExAmpl[i]=AmplTF
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
            i = i + 1

        for run in range(Runcount + 1):
            runIDX = self.TransferRunCount == run
            arctanResult=np.unwrap(np.arctan2(np.sin(DB1.TransferPhase[runIDX]),np.cos(DB1.TransferPhase[runIDX])))
            transferfunctionunwraped = np.unwrap(arctanResult)
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
                          'AmplitudeCoefficentUncerRelPercent': np.zeros(FreqNum),
                          'ExAmp': np.zeros(FreqNum),
                          'Phase':np.zeros(FreqNum),
                          'PhaseUncer':np.zeros(FreqNum),
                          'PhaseDeg': np.zeros(FreqNum),
                          'PhaseUncerDeg': np.zeros(FreqNum),
                          'N':np.zeros(FreqNum),
                          'AmpChiSquarePassed':[False] * FreqNum,
                          'PhaseChiSquarePassed':[False] * FreqNum,
                          'Tau': np.zeros(FreqNum)}
        roundedFreqs=np.rint(self.TransferFreqs*10)/10
        for i in range(len(nominalFreqs)):
            IDX=np.where(roundedFreqs==nominalFreqs[i])
            npIDX=np.array(IDX[0])
            N=npIDX.size
            TMPAmplitudes=self.TransferAmpl[IDX]
            TMPExAmplitudes = self.ExAmpl[IDX]
            TMPAmplitudesErr = self.TransferAmplErr[IDX]
            Amps=np.zeros(N)
            weights=np.zeros(N)
            weighted = np.zeros(N)
            #calculate wigthed mean of phase and amplitude values
            for j in range(N):
                Amps[j]=TMPAmplitudes[j]
                weights[j]=1/TMPAmplitudesErr[j]
                weighted[j]=Amps[j]*weights[j]
            ampmean=np.sum(weighted)/np.sum(weights)
            residualAmp=Amps-ampmean
            ampstd=np.sqrt(np.sum(residualAmp*residualAmp)/N)
            TMPPhases=self.TransferPhase[IDX]
            TMPPhasesErr=self.TransferPhaseErr[IDX]
            Phases = np.zeros(N)
            phaseweights= np.zeros(N)
            phaseweighted = np.zeros(N)
            for j in range(N):
                Phases[j] = np.arctan2(np.sin(TMPPhases[j]),np.cos(TMPPhases[j]))
                phaseweights[j]=1/TMPPhasesErr[j]
                phaseweighted[j]=Phases[j]*phaseweights[j]
            phasemean= np.sum(phaseweighted)/np.sum(phaseweights)
            residualPhase=(Phases-phasemean)
            RphaseSqare=residualPhase * residualPhase
            phasestd=np.sqrt(np.sum(RphaseSqare)/N)

            #do an CHI² test to check konsistency
            TAmplitude=np.sum((residualAmp*residualAmp)/TMPAmplitudesErr)
            TPhase = np.sum((residualPhase * residualPhase) / TMPPhasesErr)
            AmplitudeChi2Pass=False
            PhaseChi2Pass=False
            if TAmplitude<chi2.pdf(N, N-1) and TAmplitude<0.05:
                AmplitudeChi2Pass = True
            if TPhase<chi2.pdf(N, N-1) and TPhase<0.05:
                PhaseChi2Pass = True
            self.Transferfunction['AmplitudeCoefficent'][i]=ampmean
            self.Transferfunction['AmplitudeCoefficentUncer'][i]=ampstd*2
            self.Transferfunction['AmplitudeCoefficentUncerRelPercent'][i] = (ampstd * 2)/ampmean*100
            self.Transferfunction['ExAmp'][i] = np.mean(TMPExAmplitudes)
            self.Transferfunction['Phase'][i] = phasemean
            self.Transferfunction['PhaseUncer'][i] =phasestd*2
            self.Transferfunction['PhaseDeg'][i] = phasemean/np.pi*180
            self.Transferfunction['PhaseUncerDeg'][i] =phasestd*2/np.pi*180
            self.Transferfunction['N'][i] = N
            self.Transferfunction['AmpChiSquarePassed'][i] = AmplitudeChi2Pass
            self.Transferfunction['PhaseChiSquarePassed'][i] = PhaseChi2Pass
            self.Transferfunction['Tau'][i]=(self.Transferfunction['Phase'][i]/(2*np.pi))*(1/self.Transferfunction['Frequencys'][i])
        return self.Transferfunction

    def PlotTransferFunction(self, PlotType="lin",fileName=None,unwrapMean=True):
        fig, (ax1, ax2) = plt.subplots(2, 1)
        if PlotType == "logx":
            ax1.set_xscale("log")
            ax2.set_xscale("log")
        #fig.suptitle("Transfer function ")
        ax1.set_ylabel("Relative magnitude $|S|$")
        ax1.grid(True)
        jet = plt.cm.jet
        AmpMarker=[None]*len(self.Transferfunction['AmpChiSquarePassed'])
        for i in range (len(self.Transferfunction['AmpChiSquarePassed'])):
            if(self.Transferfunction['AmpChiSquarePassed'][i]==False):
                AmpMarker[i] = True
            else:
                AmpMarker[i] = False
        PhaseMarker=[None]*len(self.Transferfunction['PhaseChiSquarePassed'])
        for i in range(len(self.Transferfunction['PhaseChiSquarePassed'])):
            if(self.Transferfunction['PhaseChiSquarePassed'][i]==False):
                PhaseMarker[i] = True
            else:
                PhaseMarker[i] = False
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
            markersize=5,
            fmt=".",
            label='Mean',
            uplims=AmpMarker, lolims=AmpMarker
        )
        if not unwrapMean:
            ax2.errorbar(
                self.Transferfunction['Frequencys'],
                self.Transferfunction['Phase']/ np.pi * 180,
                yerr=self.Transferfunction['PhaseUncer']/ np.pi * 180,
                markersize=5,
                fmt=".",
                label='Mean',
                uplims=PhaseMarker, lolims=PhaseMarker
            )
        else:
            ax2.errorbar(
                self.Transferfunction['Frequencys'],
                np.unwrap(self.Transferfunction['Phase'])/ np.pi * 180,
                yerr=self.Transferfunction['PhaseUncer']/ np.pi * 180,
                markersize=5,
                fmt=".",
                label='Mean',
                uplims=PhaseMarker, lolims=PhaseMarker
            )
        ax2.set_xlabel(r"Frequency $f$ in Hz")
        ax2.set_ylabel(r"Phase $\Delta\varphi$ in deg")
        ax2.grid(True)
        ax1.legend(numpoints=1, fontsize=8, ncol=3)
        ax2.legend(numpoints=1, fontsize=8, ncol=3)
        plt.tight_layout()
        if fileName != None:
            fig.savefig(fileName+'.png')
            fig.savefig(fileName+'.svg')
            fig.show()
        else:
            fig.show()

    def PrintTransferParamsRaw(self):
        PDTransferParams = pd.DataFrame(
            {
                "Freq": self.TransferFreqs,
                "Ampl": self.TransferAmpl,
                "AmplErr": self.TransferAmplErr,
                "Phase Deg": self.TransferPhase / np.pi * 180,
                "Phase Deg Err": self.TransferPhaseErr / np.pi * 180,
                "Phase Rad": self.TransferPhase,
                "Phase Rad Err": self.TransferPhaseErr,
            }
        )
        print(PDTransferParams)
        return PDTransferParams

    def ExportTFasLatex(self,filename):
        all=pd.DataFrame.from_dict(self.Transferfunction)
        out=all[['Frequencys', 'ExAmp', 'AmplitudeCoefficent', 'AmplitudeCoefficentUncerRelPercent', 'PhaseDeg',
         'PhaseUncerDeg', 'Tau']]
        out['Tau']*=1000
        out.to_latex(filename+'_TF.tex',index=False,float_format="{:0.3g}".format)

    def exportTF(self,filename):
        tmp = pd.DataFrame.from_dict(self.Transferfunction)
        tmp.to_csv(filename+'TF.csv',index=False)
        with open(filename+'TF.json', 'w') as f:
            json.dump(self.Transferfunction, f,cls=NumpyEncoder)
        TFRaw={
            "Freq": self.TransferFreqs,
            "Ampl": self.TransferAmpl,
            "AmplErr": self.TransferAmplErr,
            "Phase Deg": self.TransferPhase / np.pi * 180,
            "Phase Deg Err": self.TransferPhaseErr / np.pi * 180,
            "Phase Rad": self.TransferPhase,
            "Phase Rad Err": self.TransferPhaseErr,
        }
        tmp = pd.DataFrame.from_dict(TFRaw)
        tmp.to_csv(filename+'TF_RAW.csv',index=False)
        with open(filename+'TF_RAW.json', 'w') as f:
            json.dump(TFRaw, f,cls=NumpyEncoder)

    def exportFitResults(self,filename):
        Fitresults=np.zeros([len(self.CalData),16])
        for i in range(len(self.CalData)):
            Fitresults[i] = self.CalData[i].popt.flatten()

        #self.popt[i] = [
        #    np.mean(abs(Complex)),
        #    np.mean(DC),
        #    np.mean(Freq),
        #    np.mean(np.unwrap(np.angle(Complex))),
        #]
        exportfilename=filename+'_FITPARAMS_RAW.csv'
        np.savetxt(exportfilename,Fitresults , delimiter=';', header='XAmp;XDC;XFreq;XPhaserad;YAmp;YDC;YFreq;YPhaserad;ZAmp;ZDC;ZFreq;ZPhaserad;REFAmp;REFDC;REFFreq;REFPhaserad')
        print("Fit raw data saved at "+exportfilename)
        return Fitresults

    def PlotSTDandValid(self, startIDX=0, stopIDX=0,fileName=None):
        fig, (ax1) = plt.subplots(1, 1)
        fig.set_size_inches(16, 9)
        ax1.set_xlabel("Time in s")
        ax2 = ax1.twinx()
        self.Chunktimes = np.arange(self.STDArray.shape[0])
        tmpdeltaT = (
            DB1.DataLoopBuffer[self.params["IntegrationLength"] - 1, 4]
            - DB1.DataLoopBuffer[0, 4]
        )
        print(tmpdeltaT)
        self.Chunktimes = self.Chunktimes * tmpdeltaT
        # calculationg delta time from last vaild chunk
        if startIDX == 0 and stopIDX == 0:
            ax1.plot(self.Chunktimes, self.STDArray[:, 0], label="X Axis",color='green')
            ax1.plot(self.Chunktimes, self.STDArray[:, 1], label="Y Axis",color='red')
            ax1.plot(self.Chunktimes, self.STDArray[:, 2], label="Z Axis",color='purple')
            ax1.plot(self.Chunktimes, self.STDArray[:, 3], label="Sync in A.U.",color='orange')
        else:
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 0],
                label="X Axis",
            )
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 1],
                label="Y Axis",
            )
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 2],
                label="Z Axis",
            )
            ax1.plot(
                self.Chunktimes[startIDX:stopIDX],
                self.STDArray[startIDX:stopIDX, 3],
                label="Sync in A.U.",
            )
        #ax1.title.set_text(
        #    "Short term standard deviation $\sigma$ (width "
        #    + str(self.params["IntegrationLength"])
        #    + " samples) of signal amplitude."
        #)
        ax1.set_ylabel("STD  $\sigma$ in m/s^2")
        ax1.legend(loc='lower center')

        ax1.grid(True)
        # calculate valide after cut out
        coutOutCunks=np.rint(self.params["defaultEndCutOut"]/self.params["IntegrationLength"])
        self.isValidCalChunkAfterCutOut=np.zeros(self.isValidCalChunk.shape)
        for i in range(int(len(self.isValidCalChunk)-coutOutCunks)):
            start=int(i)
            stop=int(i+coutOutCunks)
            valideToTest=self.isValidCalChunk[start:stop]
            tmp=np.all(valideToTest)
            self.isValidCalChunkAfterCutOut[i]=tmp

        if startIDX == 0 and stopIDX == 0:
            #ax2.plot(self.Chunktimes, self.isValidCalChunk[:,0],label="beforeCutOut")
            ax2.plot(self.Chunktimes, self.isValidCalChunkAfterCutOut[:,0],":", label="Data validity")
        else:
            #ax2.plot(
            #    self.Chunktimes[startIDX:stopIDX,0],
            #    self.isValidCalChunk[startIDX:stopIDX,0],
            #    ".", label="beforeCutOut"
            #)
            ax2.plot(
                self.Chunktimes[startIDX:stopIDX,0],
                self.isValidCalChunkAfterCutOut[startIDX:stopIDX,0],
                ":", label="Data validity"
            )
        ax2.set_yticks([0,1])
        ax2.set_ylabel("Data used for sine aproximation", color=ax2.get_lines()[0].get_color())
        ax2.set_yticklabels(['False','True'], color=ax2.get_lines()[0].get_color())
        ax2.xaxis.grid()
        plt.tight_layout()
        if fileName!=None:
            fig.savefig(fileName+'.png')
            fig.savefig(fileName+'.svg')
            fig.show()
        else:
            fig.show()

    def PlotSamplingFreq(self,fset=1000,averages=10,fileName=None):
        fig, ax = plt.subplots(1, 1)
        fig.set_size_inches(16, 9)
        colors = plt.cm.jet(np.linspace(0, 1, len(self.CalData)))
        i=0
        for CTS in self.CalData:
            #(self,fig = None,ax = None,label=None,legend=True,fset=1000):
            CTS.PlotSamplingFreq(fig=fig,ax=ax,legend=False,fset=fset,color=colors[i],average=averages)
            i=i+1
        plt.tight_layout()
        if fileName!=None:
            fig.savefig(fileName+'.png')
            fig.savefig(fileName+'.svg')
            fig.show()


def DataReaderACCdumpLARGE(Databuffer, ProtoCSVFilename, linestoread=0):
    chunksize = Databuffer.params["IntegrationLength"]
    reader = csv.reader(open(ProtoCSVFilename), delimiter=";")
    print("reading first two rows")
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

    if linestoread == 0:
        linestoread = (
            Databuffer.params["IntegrationLength"] * Databuffer.params["MaxChunks"]
        )
    # TODO add "static" var for first time stamp to have relative times to avoid quantisation error
    while i < linestoread:
        if i%1e5==0:
            print(str(i)+" lines read so far.")
        # ['id', 'sample_number', 'unix_time', 'unix_time_nsecs', 'time_uncertainty', 'Data_01', 'Data_02', 'Data_03', 'Data_04', 'Data_05', 'Data_06', 'Data_07', 'Data_08', 'Data_09', 'Data_10', 'Data_11', 'Data_12', 'Data_13', 'Data_14', 'Data_15', 'Data_16']
        #  0      1                2               3                    4               5           6          7         8          9         10            11          12         13       14          15          16          17      18          19          20
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
                REF[j] = float(line[15])*100
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

def DataReaderGYROdumpLARGE(Databuffer, ProtoCSVFilename, linestoread=0):
    chunksize = Databuffer.params["IntegrationLength"]
    reader = csv.reader(open(ProtoCSVFilename), delimiter=";")
    print("reading first two rows")
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
    if linestoread==0:
        linestoread = (
            Databuffer.params["IntegrationLength"] * Databuffer.params["MaxChunks"]
        )
    # TODO add "static" var for first time stamp to have relative times to avoid quantisation error
    while i < linestoread:
        if i%1e5==0:
            print(str(i)+"Lines read so far.")
        # ['id', 'sample_number', 'unix_time', 'unix_time_nsecs', 'time_uncertainty', 'Data_01', 'Data_02', 'Data_03', 'Data_04', 'Data_05', 'Data_06', 'Data_07', 'Data_08', 'Data_09', 'Data_10', 'Data_11', 'Data_12', 'Data_13', 'Data_14', 'Data_15', 'Data_16']
        #  0      1                2               3                    4               5           6          7         8          9         10
        try:
            line = next(reader)
            time = float(line[2]) - startsec + (float(line[3]) * 1e-9)
            # pushData(self, x, y, z, REF, t)
            if j < chunksize:
                x[j] = float(line[8])/np.pi*180
                y[j] = float(line[9])/np.pi*180
                z[j] = float(line[10])/np.pi*180
                REF[j] = float(line[15])*60
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
                x[j] = float(line[8])/np.pi*180
                y[j] = float(line[9])/np.pi*180
                z[j] = float(line[10])/np.pi*180
                REF[j] = float(line[15])*60
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
    testAxis=2
    refPhase=-np.pi
    DB1 = Databuffer(params={"stdvalidaxis": testAxis,"minSTDforVailid":1,"IntegrationLength": 128,"minValidChunksInRow": 100})
    DB1.setRefADCTF(
        [
            r"cal_data\1FE4_AC_CAL\200615_1FE4_ADC123_3CLCES_19V5_1HZ_1MHZ.json",
            r"cal_data\1FE4_AC_CAL\200615_1FE4_ADC123_3CYCLES_1V95_1HZ_1MHZ.json",
        ],
        ADCChannel="ADC1",
    )
    folder=r'D:\data\200907_mpu9250_BMA280_cal\2020-09-01 Messungen MPU9250_SN20_Zweikanalig\WDH5'
    os.chdir(folder)
    refName=r'20200901171918_MPU_9250_0x1fe40000_SN20_WDH5_Ref_TF.csv'
    dumpName="20200901171918_MPU_9250_0x1fe40000_SN20_WDH5"
    DB1.setRefTransferFunction(folder+'\\'+refName)
    DataReaderACCdumpLARGE(DB1,folder+'\\'+dumpName+'.dump',linestoread=7700000 )#
    DB1.DoAllFFT()
    DB1.getTransferCoevs(2, RefPhaseDC=refPhase)
    DB1.GetTransferFunction(2, RefPhaseDC=refPhase)
    DB1.PlotTransferFunction()
    DB1.PlotTransferFunction(PlotType="logx")
    DB1.PlotSTDandValid()
    DB1.PlotSamplingFreq()
    resultfolder=os.path.join(folder, "results")
    if not os.path.exists(resultfolder):
        os.mkdir(resultfolder)
        print("Directory ", resultfolder, " Created ")
    else:
        print("Directory ", resultfolder, " already exists")
    #DB1.exportTF(resultfolder+r'\\'+dumpName)
    #DB1.ExportTFasLatex(resultfolder+r'+dumpName)
    DB1.exportFitResults(os.path.join(resultfolder,dumpName))
    print("--- %s seconds ---" % (time.time() - start_time))


