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

# from termcolor import colored
plt.rcParams.update({"font.size": 30})
plt.rc("text", usetex=True)


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
        print(self.REFFfftPeakIndex)
        # +1 is needed sice we use relative index in the array passed to np.argmax
        # ingnore dc idx>0 only positive freqs idx<int(length/2)
        print("Max peak at axis 3 has index " + str(self.REFFfftPeakIndex))
        self.FFTFreqPeak = self.fftFreqs[self.REFFfftPeakIndex]
        # für OW der index wird richtig berechnent
        self.FFTAmplitudePeak = abs(self.FFTData[self.REFFfftPeakIndex, :])
        self.FFTPhiPeak = np.angle(self.FFTData[self.REFFfftPeakIndex, :])
        # für OW diese atribute haben ikorrekte werte
        print("Found Peak at " + str(self.FFTFreqPeak) + " Hz")
        print("Amplidues for all channels are" + str(self.FFTAmplitudePeak))
        print("Phase angle for all channels are" + str(self.FFTPhiPeak))

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

    def SinFit(self, EndCutOut, Methode="SineTools",SimulateTimingErrors=False):
        """
        

        Parameters
        ----------
        EndCutOut : TYPE
            DESCRIPTION.
        Methode : TYPE, optional
            DESCRIPTION. The default is 'SineTools1'.

        Returns
        -------
        None.

        """
        if SimulateTimingErrors=='1000Hz':
            print("WARNING SIMULATING TIMING ERROR 100Hz")
            starttime=self.Data[0, 4]
            self.Data[:,4]=np.arange(self.Data.shape[0])*1e-3+starttime

        if SimulateTimingErrors=='constdist':
            print("WARNING SIMULATING TIMING ERROR CONST dist")
            starttime=self.Data[0, 4]
            endtime=self.Data[-1, 4]
            deltaT=endtime-starttime
            step=deltaT/(self.Data.shape[0]-1)
            self.Data[:,4]=np.arange(self.Data.shape[0])*step+starttime
        self.popt = np.zeros([4, 4])
        self.pcov = np.zeros([4, 4, 4])
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
                print("Fiting at Freq " + str(self.FFTFreqPeak) + " at Axis" + str(i))
                print(tmpp0)
            if Methode == "SineTools":
                print("Fiting at Freq " + str(self.FFTFreqPeak) + " at Axis" + str(i))
                try:
                    tmpparams = st.fourparsinefit(
                    self.Data[:-EndCutOut, i],
                    self.Data[:-EndCutOut, 4],
                    self.FFTFreqPeak
                )
                except AssertionError as error:
                    print(error)
                    print("Skipping this fit")
                Complex = tmpparams[1] + 1j * tmpparams[0]
                DC = tmpparams[2]
                Freq = tmpparams[3]
                print(Complex)
                self.popt[i] = [abs(Complex), DC, Freq, np.angle(Complex)]
            print(self.popt)

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
            "IntegrationLength": 1024,
            "MaxChunks": 100000,
            "axixofintest": 1,
            "stdvalidaxis": 3,
            "minValidChunksInRow": 3,
            "minSTDforVailid": 5,
            "defaultEndCutOut": 750,
        }
        self.flags = {
            "AllSinFitCalculated": False,
            "AllFFTCalculated": False,
            "RefTrnaferFunctionSet": False,
        }
        self.DataLoopBuffer = np.zeros([self.params["IntegrationLength"], 5])
        self.i = 0
        self.IntegratedPacketsCount = 0

        self.FFTArray = np.zeros(
            [self.params["MaxChunks"], self.params["IntegrationLength"], 4]
        )
        self.STDArray = np.zeros([self.params["MaxChunks"], 4])
        # min STD to be an vaild chunk (there is AC-Component in the Signal)
        self.params["minValidChunksInRow"] = 5
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
            self.DataLoopBuffer[self.i, 0] = x
            self.DataLoopBuffer[self.i, 1] = y
            self.DataLoopBuffer[self.i, 2] = z
            self.DataLoopBuffer[self.i, 3] = REF
            self.DataLoopBuffer[self.i, 4] = t
            self.i = self.i + 1
            if self.i == self.params["IntegrationLength"]:
                self.calc()
                print()

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


    def setRefTransferFunction(self, transferCSV):
        #'STM32Toolchain/projects/Met4FoF-SmartUpUnit/tools/data/messkette_cal.csv'
        self.RefTransferFunction = pd.read_csv(
            transferCSV, sep="\t", skiprows=[1, 2, 3], decimal=","
        )
        self.flags["RefTrnaferFunctionSet"] = True
        self.flags["RefGroupDelaySet"] = False

    def setRefGroupDelay(self,GropuDelay):
        # TODO improve selection between Ref Freq an group delay
        self.flags["RefTrnaferFunctionSet"] = True
        self.flags["RefGroupDelaySet"] = True
        self.params["RefGroupDelay"]=GropuDelay

    def getNearestReFTPoint(self, freq):
        if self.flags["RefTrnaferFunctionSet"] == True:
            if not self.flags['RefGroupDelaySet']:
                tmp = abs(self.RefTransferFunction["Frequenz"] - freq)
                IDX = np.where(tmp == np.amin(tmp))[0]
                return (
                        self.RefTransferFunction["Spannungs-ÜTK"][IDX] / 100,
                        self.RefTransferFunction["Phasenverschiebung"][IDX] / 180 * np.pi,
                        )
            else:
                #print("Using GROUPDELAY WITHOUT AMPLITUDE AS REFERENCE TRANSFER FUNCTION")
                return (
                        1,#Asume Amplitude as One
                        freq*self.params["RefGroupDelay"]*2*np.pi*-1,#-1 because we are seeing this aus transferfunction of the Ref Mesurment
                                                                    # so an positiv grouddelay results in in linear increasing negative phase
                                                                    )
        else:
            raise RuntimeError("REFTransferFunction NOT SET RETURNING 0,0")


    def getTransferFunction(
        self, axisDUT, AxisRef=3, RefScalefactor=1, RefPhaseDC=-np.pi
    ):
        if not self.flags["AllSinFitCalculated"]:
            print(
                "Doing Sin Fit with default  end cut out length "
                + str(self.params["defaultEndCutOut"])
            )
            self.DoAllSinFit(self.params["defaultEndCutOut"])
        self.TransferFreqs = np.zeros(len(self.CalData))
        self.TransferAmpl = np.zeros(len(self.CalData))
        self.TransferPhase = np.zeros(len(self.CalData))
        self.TransferPhaseDEBUG = np.zeros(len(self.CalData))
        self.TransferPhaseDEBUG2 = np.zeros(len(self.CalData))
        self.TransferRunCount = np.zeros(len(self.CalData))
        i = 0 #number of packets processed
        Runcount = 0 #number of loops processed so far loop is an series of rising frequencys
        if self.flags["RefTrnaferFunctionSet"] == False:
            for item in self.CalData:
                print(
                    "WARING REFTransferFunction NOT SET USE THIS RESULTS JUST FOR DEBUGGING!!!"
                )
                self.TransferFreqs[i] = self.CalData[i].popt[axisDUT, 2]
                #
                self.TransferAmpl[i] = (self.CalData[i].popt[axisDUT, 0]) / (
                    self.CalData[i].popt[AxisRef, 0] * RefScalefactor
                )
                self.TransferPhase[i] = (
                  self.CalData[i].popt[axisDUT, 3] - self.CalData[i].popt[AxisRef, 3]
                )
                self.TransferPhaseDEBUG[i]=self.TransferPhase[i]
                self.TransferPhase[i] = self.TransferPhase[i] + RefPhaseDC
                #detect run count based on frequency drop to do right unwraping
                self.TransferRunCount[i]=Runcount
                self.TransferPhaseDEBUG2[i]=self.TransferPhase[i]
                if(i>0):
                    if self.TransferFreqs[i]*1.01<self.TransferFreqs[i-1]:#multiply with 1.01 in case of same freqs
                        #ok the freq now is smaller the the last freq we have enterd a new loop
                        Runcount=Runcount+1
                        self.TransferRunCount[i]=Runcount
                i = i + 1

        if self.flags["RefTrnaferFunctionSet"] == True:
            for item in self.CalData:
                self.TransferFreqs[i] = self.CalData[i].popt[axisDUT, 2]
                AmplTF = self.getNearestReFTPoint(self.TransferFreqs[i])[0]
                PhaseTF = self.getNearestReFTPoint(self.TransferFreqs[i])[1]
                print(PhaseTF)
                self.TransferAmpl[i] = (self.CalData[i].popt[axisDUT, 0]) / (
                    (self.CalData[i].popt[AxisRef, 0] * RefScalefactor) / AmplTF
                )
                self.TransferPhase[i] = (
                     self.CalData[i].popt[axisDUT, 3]-self.CalData[i].popt[AxisRef, 3]
                )
                self.TransferPhaseDEBUG[i]=self.TransferPhase[i]
                self.TransferPhase[i] = self.TransferPhase[i] + PhaseTF + RefPhaseDC
                                #detect run count based on frequency drop to do right unwraping
                self.TransferRunCount[i]=Runcount
                self.TransferPhaseDEBUG2[i]=self.TransferPhase[i]
                if(i>0):
                    if self.TransferFreqs[i]*1.01<self.TransferFreqs[i-1]:#multiply with 1.01 in case of same freqs
                        #ok the freq now is smaller the the last freq we have enterd a new loop
                        Runcount=Runcount+1
                        self.TransferRunCount[i]=Runcount
                i = i + 1
        for run in range(Runcount):
            runIDX=self.TransferRunCount==run
            transferfunctionunwraped=np.unwrap(DB1.TransferPhase[runIDX])
            if all (transferfunctionunwraped<=(-2*np.pi)):
                transferfunctionunwraped=transferfunctionunwraped+2*np.pi
            self.TransferPhase[runIDX]=transferfunctionunwraped
        # TODO implement bettwer way vor unwraping and function for data set labeling
        # self.TransferPhase = np.unwrap(self.TransferPhase)

    def PlotTransferFunction(self, PlotType="lin"):
        fig, (ax1, ax2) = plt.subplots(2, 1)
        if PlotType == "lin":
             for run in range(int(np.max(self.TransferRunCount))):
                 runIDX=self.TransferRunCount==run
                 ax1.plot(self.TransferFreqs[runIDX], self.TransferAmpl[runIDX], ".", markersize=20,label=str(run))
        if PlotType == "logx":
            for run in range(int(np.max(self.TransferRunCount))):
                runIDX=self.TransferRunCount==run
                ax1.semilogx(self.TransferFreqs[runIDX], self.TransferAmpl[runIDX], ".", markersize=20,label=str(run))
        fig.suptitle("Transfer function ")
        ax1.set_ylabel("Relative magnitude $|S|$")
        ax1.grid(True)
        if PlotType == "lin":
            for run in range(int(np.max(self.TransferRunCount))):
                runIDX=self.TransferRunCount==run
                ax2.plot(
                     self.TransferFreqs[runIDX], self.TransferPhase[runIDX] / np.pi * 180, ".", markersize=20,label=str(run)
            )
        if PlotType == "logx":
            for run in range(int(np.max(self.TransferRunCount))):
                runIDX=self.TransferRunCount==run
                ax2.semilogx(
                     self.TransferFreqs[runIDX], self.TransferPhase[runIDX] / np.pi * 180, ".", markersize=20,label=str(run)
            )
        ax2.set_xlabel(r"Frequency $f$ in Hz")
        ax2.set_ylabel(r"Phase $\Delta\varphi$ in °")
        ax2.grid(True)
        ax1.legend(numpoints=1, fontsize=8,ncol=3)
        ax2.legend(numpoints=1, fontsize=8,ncol=3)
        plt.show()

    def PrintTransferParams(self):
        self.TransferParams=pd.DataFrame({'Freq':self.TransferFreqs,'Ampl':self.TransferAmpl,'Phase Deg':self.TransferPhase / np.pi * 180,'Phase Rad':self.TransferPhase})
        return self.TransferParams

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


def callback(data):
    print(data)
    DB1.pushData(
        data.linear_acceleration.x,
        data.linear_acceleration.y,
        data.linear_acceleration.z,
        0,
        data.header.stamp / 1e9,
    )
    return


# def listener():
# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
# rospy.init_node('listener', anonymous=True)

# rospy.Subscriber("IMU0", Imu, callback)
# rospy.Subscriber("IMU1", Imu, callback)
# spin() simply keeps python from exiting until this node is stopped
# rospy.spin()


def DataReaderROS(RosCSVFilename):
    sdf = pd.read_csv(RosCSVFilename)
    print(sdf.columns.values)
    chunkSize = DB1.params["IntegrationLength"]
    for Index in np.arange(0, len(sdf), chunkSize):
        DB1.pushBlock(
            sdf["field.linear_acceleration.x"][Index : Index + chunkSize],
            sdf["field.linear_acceleration.y"][Index : Index + chunkSize],
            sdf["field.linear_acceleration.z"][Index : Index + chunkSize],
            0,
            sdf["field.header.stamp"][Index : Index + chunkSize] / 1e9,
        )


# Column Names
#        'time',
#        'field.header.seq',
#        'field.header.stamp',
#        'field.header.frame_id',
#        'field.orientation.x',
#        'field.orientation.y',
#        'field.orientation.z',
#        'field.orientation.w',
#        'field.orientation_covariance0',
#        'field.orientation_covariance1',
#        'field.orientation_covariance2',
#        'field.orientation_covariance3',
#        'field.orientation_covariance4',
#        'field.orientation_covariance5',
#        'field.orientation_covariance6',
#        'field.orientation_covariance7',
#        'field.orientation_covariance8',
#        'field.angular_velocity.x',
#        'field.angular_velocity.y',
#        'field.angular_velocity.z',
#        'field.angular_velocity_covariance0',
#        'field.angular_velocity_covariance1',
#        'field.angular_velocity_covariance2',
#        'field.angular_velocity_covariance3',
#        'field.angular_velocity_covariance4',
#        'field.angular_velocity_covariance5',
#        'field.angular_velocity_covariance6',
#        'field.angular_velocity_covariance7',
#        'field.angular_velocity_covariance8',
#        'field.linear_acceleration.x',
#        'field.linear_acceleration.y',
#        'field.linear_acceleration.z',
#        'field.linear_acceleration_covariance0',
#        'field.linear_acceleration_covariance1',
#        'field.linear_acceleration_covariance2',
#        'field.linear_acceleration_covariance3',
#        'field.linear_acceleration_covariance4',
#        'field.linear_acceleration_covariance5',
#        'field.linear_acceleration_covariance6',
#        'field.linear_acceleration_covariance7',
#        'field.linear_acceleration_covariance8'


def DataReaderPROTOdump(ProtoCSVFilename):
    sdf = pd.read_csv(ProtoCSVFilename, delimiter=";",index_col=False)
    print(sdf.columns.values)
    chunkSize = DB1.params["IntegrationLength"]
    for Index in np.arange(0, len(sdf), chunkSize)[
        :-1
    ]:  # don't use last chuck since this will propably not have chnukSize elements
        DB1.pushBlock(
            sdf["Data_01"][Index : Index + chunkSize],
            sdf["Data_02"][Index : Index + chunkSize],
            sdf["Data_03"][Index : Index + chunkSize],
            sdf["Data_11"][Index : Index + chunkSize]*10,
            (sdf["unix_time"][Index : Index + chunkSize])
            + (sdf["unix_time_nsecs"][Index : Index + chunkSize]) * 1e-9,
        )


def DataReaderGYROdump(ProtoCSVFilename):
    sdf = pd.read_csv(ProtoCSVFilename, delimiter=";",index_col=False,skiprows=1)
    print(sdf.columns.values)
    chunkSize = DB1.params["IntegrationLength"]
    for Index in np.arange(0, len(sdf), chunkSize)[
        :-1
    ]:  # don't use last chuck since this will propably not have chnukSize elements
        DB1.pushBlock(
            sdf["GYR_x"][Index : Index + chunkSize]*(180/np.pi),
            sdf["GYR_y"][Index : Index + chunkSize]*(180/np.pi),
            sdf["GYR_z"][Index : Index + chunkSize]*(180/np.pi),
            sdf["ADC_1"][Index : Index + chunkSize]*100,
            (sdf["unix_time"][Index : Index + chunkSize])
            + (sdf["unix_time_nsecs"][Index : Index + chunkSize]) * 1e-9,
        )

def  DataReaderGYROdumpLARGE(Databuffer,ProtoCSVFilename,linestoread=0):
    reader = csv.reader(open(ProtoCSVFilename),delimiter=';')
    print("reading first to rows")
    print(next(reader))
    print(next(reader))
    #we sacrifice a line to set the start time stamp
    line=next(reader)
    startsec=float(line[2])
    i=0
    if(i==0):
        linestoread=Databuffer.params["IntegrationLength"]*Databuffer.params["MaxChunks"]
    #TODO add "static" var for first time stamp to have relative times to avoid quantisation error
    while i<linestoread:
        #['id', 'sample_number', 'unix_time', 'unix_time_nsecs', 'time_uncertainty', 'Data_01', 'Data_02', 'Data_03', 'Data_04', 'Data_05', 'Data_06', 'Data_07', 'Data_08', 'Data_09', 'Data_10', 'Data_11', 'Data_12', 'Data_13', 'Data_14', 'Data_15', 'Data_16']
        try:
            line=next(reader)
            time=float(line[2])-startsec+(float(line[3])*1e-9)
            #pushData(self, x, y, z, REF, t)
            Databuffer.pushData(float(line[8])*(180/np.pi),
                                float(line[9])*(180/np.pi),
                                float(line[10])*(180/np.pi),
                                float(line[15])*100,
                                time)
            i=i+1
        except Exception as e:
            print(e)
            break
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
    DB1 = Databuffer()
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
    DataReaderGYROdumpLARGE(DB1,"/data/20191217100017_MPU_9250_0x1fe40000.dump")#/191617_MPU_9250_Y_Rot_100_Wdh/
    DB1.setRefGroupDelay(220e-6)
    # DB1.setRefTransferFunction("data/messkette_cal.csv")
    # reading data from file and proces all Data
    DB1.DoAllFFT()
    DB1.getTransferFunction(1,RefPhaseDC=-np.pi)
    DB1.PlotTransferFunction()
    DB1.PlotTransferFunction(PlotType="logx")
    end = timeit.timeit()
    # callculate all the ffts
