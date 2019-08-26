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
import pandas
from scipy import signal
from scipy.optimize import curve_fit
import SineTools as st
# from termcolor import colored


class CalTimeSeries:
    def SinFunc(self, x, A, B, f, phi):
        return A*np.sin(2*np.pi*f*x+phi)+B

    def __init__(self):
        """


        Returns
        -------
        None.

        """
        self.length = 0
        self.flags = {'timeCalculated': False,
                      'FFTCalculated': False,
                      'BlockPushed': 0,
                      'SineFitCalculated': [False,False,False,False]}

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
        if(self.flags['BlockPushed'] > 0):
            self.Data = np.append(self.Data, Datablock, axis=0)
            self.flags['BlockPushed'] = self.flags['BlockPushed'] + 1
        if(self.flags['BlockPushed'] == 0):
            self.Data = np.copy(Datablock)
            # IMPORTANT copy is nesseary otherwise only a reference is copyid
            # when pushing the second block the reference is changed
            # and the second block gets added two times
            self.flags['BlockPushed'] = self.flags['BlockPushed'] + 1

    def CalcTime(self):
        """


        Returns
        -------
        tmpTimeDiffs : TYPE
            DESCRIPTION.

        """
        self.Data[:, 4] = self.Data[:, 4]-self.Data[0, 4]
        # calculate time differences between samples
        tmpTimeDiffs = self.Data[1:, 4]-self.Data[:-1, 4]
        self.MeanDeltaT = np.mean(tmpTimeDiffs)  # use with numpy.fft.fftfreq
        self.DeltaTJitter = np.std(tmpTimeDiffs)
        self.flags['timeCalculated'] = True
        return tmpTimeDiffs

    def CalcFFT(self):
        """


        Returns
        -------
        None.

        """
        if not(self.flags['timeCalculated']):  # recusive calling
            self.CalcTime()  # if time calculation hasent been done do it
        self.FFTData = np.fft.fft(self.Data[:, :4], axis=0)/(self.Data.shape[0])
        # FFT calculation
        self.fftFreqs = np.fft.fftfreq(self.Data.shape[0], self.MeanDeltaT)
        # calculate to fft bins coresponding freqs
        self.flags['FFTCalculated'] = True
        fftPeakindexiposfreq = int(self.Data.shape[0]/2)
        # TODO axis is hard coded this is not good
        self.REFFfftPeakIndex = np.argmax(abs(self.FFTData[1:fftPeakindexiposfreq,2]),)+1
        print(self.REFFfftPeakIndex)
        # +1 is needed sice we use relative index in the array passed to np.argmax
        # ingnore dc idx>0 only positive freqs idx<int(length/2)
        print('Max peak at axis 2 has index '+str(self.REFFfftPeakIndex))
        self.FFTFreqPeak = self.fftFreqs[self.REFFfftPeakIndex]
        # für OW der index wird richtig berechnent
        self.FFTAmplitudePeak = abs(self.FFTData[self.REFFfftPeakIndex, :])
        self.FFTPhiPeak = np.angle(self.FFTData[self.REFFfftPeakIndex, :])
        # für OW diese atribute haben ikorrekte werte
        print("Found Peak at "+str(self.FFTFreqPeak)+' Hz')
        print("Amplidues for all channels are"+str(self.FFTAmplitudePeak))
        print("Phase angle for all channels are"+str(self.FFTPhiPeak))

    def PlotFFT(self):
        """


        Returns
        -------
        None.

        """
        if not(self.flags['timeCalculated']):  # recusive calling
            self.CalcFFT()  # if fft calculation hasent been done do it
        fig, ax = plt.subplots()
        ax.plot(self.fftFreqs, abs(self.FFTData[:, 0]), label='Sensor x')
        ax.plot(self.fftFreqs, abs(self.FFTData[:, 1]), label='Sensor y')
        ax.plot(self.fftFreqs, abs(self.FFTData[:, 2]), label='Sensor z')
        ax.plot(self.fftFreqs, abs(self.FFTData[:, 3]), label='Ref z')
        ax.set(xlabel='Frequency f in Hz', ylabel='Abs of fft val in AU',
               title='FFT of CalTimeSeries')
        ax.grid()
        ax.legend()
        fig.show()

    def SinFit(self,EndCutOut,Methode='SineTools1'):
        self.popt = np.zeros([4, 4])
        self.pcov = np.zeros([4, 4, 4])
        self.sinFitEndCutOut=EndCutOut
        if not(self.flags['FFTCalculated']):  # recusive calling
            self.CalcFFT()
        tmpbounds = ([0, -12, self.FFTFreqPeak-3*self.fftFreqs[1], -np.pi],
                     [20, 12, self.FFTFreqPeak+3*self.fftFreqs[1], np.pi])

        # TODO get bounds from FFT params
        for i in range(4):#TODO remove this hard coded 4
            if(Methode=='curve_fit'):
                tmpp0=[self.FFTAmplitudePeak[i],np.mean(self.Data[i]),
                       self.FFTFreqPeak,self.FFTPhiPeak[i]]
                try:
                    self.popt[i], self.pcov[i] = curve_fit(self.SinFunc,
                                                       self.Data[:-EndCutOut, 4],
                                                       self.Data[:-EndCutOut, i],
                                                       bounds=tmpbounds,
                                                       p0=tmpp0)
                    self.flags['SineFitCalculated'][i]=True
                except RuntimeError as ErrorCode:
                        print("Runtime Error:" +str(ErrorCode))
                        self.flags['SineFitCalculated'][i]=False
                pass
                print('Fiting at Freq ' + str(self.FFTFreqPeak) +' at Axis' + str(i))
                print(tmpp0)
            if(Methode=='SineTools1'):
                print('Fiting at Freq ' + str(self.FFTFreqPeak) +' at Axis' + str(i))
                tmpparams=st.fourparsinefit1(self.Data[:-EndCutOut, i],self.Data[:-EndCutOut, 4],self.FFTFreqPeak,df_max=None,max_iter=20,eps=1.0e-6)
                Complex=tmpparams[0]+1j*tmpparams[1]
                DC=tmpparams[2]
                Freq=tmpparams[3]
                print(Complex)
                self.popt[i]=[abs(Complex),DC,Freq,np.angle(Complex)]
            print(self.popt)

        self.flags['SineFitCalculated'] = True

    def PlotSinFit(self, AxisofIntrest):
        plt.plot(self.Data[:-self.sinFitEndCutOut, 4], self.Data[:-self.sinFitEndCutOut, AxisofIntrest], label='Raw Data')
        plt.plot(self.Data[:-self.sinFitEndCutOut, 4], self.SinFunc(self.Data[:-self.sinFitEndCutOut, 4],
                 *self.popt[AxisofIntrest]), label='Fit')
        plt.show()


class Databuffer:
    # TODO reject packages after i= Dataarraysize
    def __init__(self):
        """


        Returns
        -------
        None.

        """
        self.params={'IntegrationLength':512,
                     'MaxChunks': 900,
                     'axixofintest': 2,
                     'minValidChunksInRow':3,
                     'minSTDforVailid':5,
                     'defaultEndCutOut':750
                }
        self.flags={'AllSinFitCalculated':False,
                    'AllFFTCalculated':False
                }
        self.DataLoopBuffer = np.zeros([self.params['IntegrationLength'], 5])
        self.i = 0
        self.IntegratedPacketsCount = 0

        self.FFTArray = np.zeros([self.params['MaxChunks'], self.params['IntegrationLength'], 4])
        self.STDArray = np.zeros([self.params['MaxChunks'], 4])
        self.params['minSTDforVailid'] = 3
        # min STD to be an vaild chunk (there is AC-Component in the Signal)
        self.params['minValidChunksInRow'] = 5
        # at least this amount of chunks in a row have to be vaild to create an
        # CalTimeSeries object in CalData
        self.isValidCalChunk = np.zeros([self.params['MaxChunks'], 3])
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
        if(self.i < self.params['IntegrationLength']):
            self.DataLoopBuffer[self.i, 0] = x
            self.DataLoopBuffer[self.i, 1] = y
            self.DataLoopBuffer[self.i, 2] = z
            self.DataLoopBuffer[self.i, 3] = REF
            self.DataLoopBuffer[self.i, 4] = t
            self.i = self.i+1
            if(self.i == self.params['IntegrationLength']):
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
        if not (self.params['IntegrationLength'] == arrayt.size == arrayy.size == arrayy.size == arrayz.size):
            raise ValueError('Array size must be INTEGRATIONTIME '
                             + str(self.params['IntegrationLength']) + 'and not '
                             + str(arrayt.size))
        self.DataLoopBuffer[:, 0] = arrayx
        self.DataLoopBuffer[:, 1] = arrayy
        self.DataLoopBuffer[:, 2] = arrayz
        self.DataLoopBuffer[:, 3] = arrayREF
        self.DataLoopBuffer[:, 4] = arrayt
        self.i = self.params['IntegrationLength']
        self.calc()

    def calc(self):
        """


        Returns
        -------
        None.

        """
        if(self.IntegratedPacketsCount < self.params['MaxChunks']):
            self.STDArray[self.IntegratedPacketsCount] = np.std(self.DataLoopBuffer[:, :4], axis=0)
            if(self.IntegratedPacketsCount > self.params['minValidChunksInRow']):
                # check if we had sufficent at at least self.params['minValidChunksInRow']
                # blocks befor so we do not produce an segvault
                IPC = self.IntegratedPacketsCount
                VCIR = self.params['minValidChunksInRow']
                isVaild = np.all(np.greater(self.STDArray[IPC-VCIR:IPC, self.params['axixofintest']], self.params['minSTDforVailid']))
                self.isValidCalChunk[self.IntegratedPacketsCount] = isVaild
                # check if we have an new valid data Chunk
                if(self.isValidCalChunk[IPC, self.params['axixofintest']]):
                    # ok this are valid data
                    if not (self.isValidCalChunk[IPC-1, self.params['axixofintest']]):
                        # we don't had an vaild data set befor this one so
                        # create a new CalTimeSeries Objec
                        self.CalData.append(CalTimeSeries())
                    self.CalData[-1].pushBlock(self.DataLoopBuffer)
                    # push data to CalTimeSeries Object
            self.IntegratedPacketsCount = self.IntegratedPacketsCount+1
            self.i = 0

    def DoAllSinFit(self,EndCutOut):
        for item in self.CalData:
            item.SinFit(EndCutOut)
        self.flags['AllSinFitCalculated']=True

    def DoAllFFT(self):
        for item in self.CalData:
            item.CalcFFT()
        self.flags['AllFFTCalculated']=True

    def getTransferFunction(self,axisDUT,AxisRef=3,RefScalefactor=10,RefPhaseDC=np.pi):
        if not self.flags['AllSinFitCalculated']:
            print("Doing Sin Fit with default  end cut out length " +str(self.params['defaultEndCutOut']))
            self.DoAllSinFit(self.params['defaultEndCutOut'])
        self.TransferFreqs=np.zeros(len(self.CalData))
        self.TransferAmpl=np.zeros(len(self.CalData))
        self.TransferPhase=np.zeros(len(self.CalData))
        i=0
        for item in self.CalData:
            self.TransferFreqs[i]=self.CalData[i].popt[axisDUT,2]
            #
            self.TransferAmpl[i]=(self.CalData[i].popt[axisDUT,0])/(self.CalData[i].popt[AxisRef,0]*RefScalefactor)
            self.TransferPhase[i]=(self.CalData[i].popt[AxisRef,3]-self.CalData[i].popt[axisDUT,3])
            self.TransferPhase[i]=self.TransferPhase[i]+RefPhaseDC
            i=i+1
        self.TransferPhase=np.unwrap(self.TransferPhase)

    def PlotTransferFunction(self):
        plt.subplot(2, 1, 1)
        plt.plot(self.TransferFreqs, self.TransferAmpl, 'o-')
        plt.title('Transfer Funktion ')
        plt.ylabel('Amplitude')

        plt.subplot(2, 1, 2)
        plt.plot(self.TransferFreqs,self.TransferPhase/np.pi*180, '.-')
        plt.xlabel('Frequency / Hz')
        plt.ylabel('Phase /°')

        plt.show()




def callback(data):
    print(data)
    DB1.pushData(data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z,0,data.header.stamp/1e9)
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
    sdf = pandas.read_csv(RosCSVFilename)
    print(sdf.columns.values)
    chunkSize = DB1.params['IntegrationLength']
    for Index in np.arange(0, len(sdf), chunkSize):
        DB1.pushBlock(sdf['field.linear_acceleration.x'][Index:Index+chunkSize],
                      sdf['field.linear_acceleration.y'][Index:Index+chunkSize],
                      sdf['field.linear_acceleration.z'][Index:Index+chunkSize],
                      0,
                      sdf['field.header.stamp'][Index:Index+chunkSize]/1e9)
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
    sdf=pandas.read_csv(ProtoCSVFilename,delimiter=';')
    print(sdf.columns.values)
    chunkSize=DB1.params['IntegrationLength']
    for Index in np.arange(0,len(sdf),chunkSize)[:-1]:#don't use last chuck since this will propably not have chnukSize elements
        DB1.pushBlock(sdf['Data_01'][Index:Index+chunkSize],
                      sdf['Data_02'][Index:Index+chunkSize],
                      sdf['Data_03'][Index:Index+chunkSize],
                      sdf['Data_11'][Index:Index+chunkSize],
                      (sdf['unix_time'][Index:Index+chunkSize])+(sdf['unix_time_nsecs'][Index:Index+chunkSize])*1e-9)


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

if __name__ == '__main__':
    DB1 = Databuffer()
    DataReaderPROTOdump('data/20190826_300Hz_LP_10-250Hz_10ms2.csv')
    #DataReaderPROTOdump('data/20190819_1500_10_250hz_10_ms2_woairatstart.dump')
    # reading data from file and proces all Data
    DB1.DoAllFFT()
    # callculate all the ffts


