#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Aug  7 12:34:21 2019

@author: seeger01
"""

#!/usr/bin/env python
#import rospy
#from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
import pandas
from scipy import signal
from scipy.optimize import curve_fit
#from termcolor import colored


class CalTimeSeries:
    def SinFunc(self,x,A,B,f,phi):
        return A*np.sin(2*np.pi*f*x+phi)+B
    def __init__(self):
        """
        

        Returns
        -------
        None.

        """
        self.length=0
        self.flags={'timeCalculated':False,'FFTCalculated':False,'BlockPushed':0,'SineFitCalculated':False}
    def pushBlock(self,Datablock):
        """
        

        Parameters
        ----------
        Datablock : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        """
        if(self.flags['BlockPushed']>0):
            self.Data=np.append(self.Data,Datablock, axis=0)
            self.length=self.length+Datablock.shape[0]
            self.flags['BlockPushed']=self.flags['BlockPushed']+1
        if(self.flags['BlockPushed']==0):
            self.Data=np.copy(Datablock)#IMPORTANT copy is nesseary otherwise only a reference is copyid witch cahnges in the next call
            self.length=self.length+Datablock.shape[0]
            self.flags['BlockPushed']=self.flags['BlockPushed']+1

    def CalcTime(self):
        """
        

        Returns
        -------
        tmpTimeDiffs : TYPE
            DESCRIPTION.

        """
        self.Data[:,4]=self.Data[:,4]-self.Data[0,4]#substract timestamp fromfirst timestamp
        tmpTimeDiffs=self.Data[1:,4]-self.Data[:-1,4]
        self.MeanDeltaT=np.mean(tmpTimeDiffs)# mean value of Delta T in betwean of two samples to use with numpy.fft.fftfreq
        self.DeltaTJitter=np.std(tmpTimeDiffs)
        self.flags['timeCalculated']=True
        return tmpTimeDiffs
    def CalcFFT(self):
        """
        

        Returns
        -------
        None.

        """
        if not(self.flags['timeCalculated']): #recusive calling
            self.CalcTime() #if time calculation hasent been done do it
        self.FFTData=np.fft.fft(self.Data[:,:4],axis=0)/(self.Data.shape[0])
        self.fftFreqs=np.fft.fftfreq(self.Data.shape[0],self.MeanDeltaT)
        self.flags['FFTCalculated']=True
        fftPeakindexiposfreq=int(self.Data.shape[0]/2)
        #TODO axis is hard coded this is not good
        self.REFFfftPeakIndex=np.argmax(self.FFTData[1:fftPeakindexiposfreq,2])
        print(self.REFFfftPeakIndex)
        self.FFTFreqPeak=self.fftFreqs[self.REFFfftPeakIndex] # ingnore dc only positive freqs
        self.FFTAmplitudePeak=abs(self.FFTData[self.REFFfftPeakIndex,:])
        self.FFTPhiPeak=np.angle(self.FFTData[self.REFFfftPeakIndex,:])
        print("Found Peak at "+str(self.FFTFreqPeak)+' Hz')
        print("Amplidues for all channels are"+str(self.FFTAmplitudePeak))
        print("Phase angle for all channels are"+str(self.FFTPhiPeak))
    def PlotFFT(self):
        """
        

        Returns
        -------
        None.

        """
        if not(self.flags['timeCalculated']): #recusive calling
            self.CalcFFT() #if fft calculation hasent been done do it
        fig, ax = plt.subplots()
        ax.plot(self.fftFreqs, abs(self.FFTData[:,0]),label='Sensor x')
        ax.plot(self.fftFreqs, abs(self.FFTData[:,1]),label='Sensor y')
        ax.plot(self.fftFreqs, abs(self.FFTData[:,2]),label='Sensor z')
        ax.plot(self.fftFreqs, abs(self.FFTData[:,3]),label='Ref z')
        ax.set(xlabel='Frequency f in Hz', ylabel='Abs of fft val in AU',
               title='FFT of CalTimeSeries')
        ax.grid()
        ax.legend()
        fig.show()

    def SinFit(self):
        self.popt=np.zeros([4,4])
        self.pcov=np.zeros([4,4,4])
        if not(self.flags['FFTCalculated']): #recusive calling
            self.CalcFFT()
        tmpbounds=([0,-12,self.FFTFreqMax-0.5,-np.pi],[20,12,self.FFTFreqMax+0.5,np.pi])
        for i in range(4):
            self.popt[i],self.pcov[i]=curve_fit(self.SinFunc,self.Data[:,4],self.Data[:,i],bounds=tmpbounds)
        self.flags['SineFitCalculated']=True

    def plotSinFit(self,AxisofIntrest):
        plt.plot(self.Data[:,4],self.Data[:,AxisofIntrest],label='Raw Data')
        plt.plot(self.Data[:,4],self.SinFunc(self.Data[:,4],*self.popt[AxisofIntrest]),label='Fit')
        plt.show()


class Databuffer:
    #TODO reject packages after i= Dataarraysize
    def __init__(self):
        """
        

        Returns
        -------
        None.

        """
        self.INTEGRATIONTIME = 512
        self.DataLoopBuffer=np.zeros([self.INTEGRATIONTIME,5])
        self.i=0
        self.IntegratedPacketsCount=0
        self.DATAARRYSIZE=900
        self.FFTArray=np.zeros([self.DATAARRYSIZE,self.INTEGRATIONTIME,4])
        self.STDArray=np.zeros([self.DATAARRYSIZE,4])
        self.minRMSforVailid=3.0
        self.minValidChunksInRow=5
        self.axixofintest=2
        self.isValidCalChunk=np.zeros([self.DATAARRYSIZE,3])
        self.CalData=[]




    def pushData(self,x,y,z,REF,t):
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
        if(self.i<self.INTEGRATIONTIME):
            self.DataLoopBuffer[self.i,0]=x
            self.DataLoopBuffer[self.i,1]=y
            self.DataLoopBuffer[self.i,2]=z
            self.DataLoopBuffer[self.i,3]=REF
            self.DataLoopBuffer[self.i,4]=t
            self.i=self.i+1
            if(self.i==self.INTEGRATIONTIME):
                self.dalc()

    def pushBlock(self,arrayx,arrayy,arrayz,arrayREF,arrayt):
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
        if not (self.INTEGRATIONTIME==arrayt.size==arrayy.size==arrayy.size==arrayz.size):
            raise ValueError('Array size must be INTEGRATIONTIME '+str(self.INTEGRATIONTIME) +'and not '+str(arrayt.size))
        self.DataLoopBuffer[:,0]=arrayx
        self.DataLoopBuffer[:,1]=arrayy
        self.DataLoopBuffer[:,2]=arrayz
        self.DataLoopBuffer[:,3]=arrayREF
        self.DataLoopBuffer[:,4]=arrayt
        self.i=self.INTEGRATIONTIME
        self.calc()

    def calc(self):
        """
        

        Returns
        -------
        None.

        """
        if(self.IntegratedPacketsCount<self.DATAARRYSIZE):
            self.STDArray[self.IntegratedPacketsCount]=np.std(self.DataLoopBuffer[:,:4],axis=0)
            self.FFTArray[self.IntegratedPacketsCount,:,:]=np.fft.fft(self.DataLoopBuffer[:,:4],axis=0)#maybe disable this fft calculation
        #print("Mean="+str(self.STDArray[self.IntegratedPacketsCount,2]))
        #print(str(self.IntegratedPacketsCount) +' STD= '+str(np.std(self.DataLoopBuffer,axis=0)))
        if(self.IntegratedPacketsCount>self.minValidChunksInRow):
        #check if we had sufficent at at least self.minValidChunksInRow blocks befor so we do not produce an segvault
            self.isValidCalChunk[self.IntegratedPacketsCount]=np.all(np.greater(self.STDArray[self.IntegratedPacketsCount-self.minValidChunksInRow:self.IntegratedPacketsCount,self.axixofintest],self.minRMSforVailid))
            #check if we have an new valid data Chunk
            if(self.isValidCalChunk[self.IntegratedPacketsCount,self.axixofintest]):
            #ok this are valid data
                if(self.isValidCalChunk[self.IntegratedPacketsCount-1,self.axixofintest]==False):
                    #we don't had an vaild data set befor this one so create a new CalTimeSeries Objec
                    self.CalData.append(CalTimeSeries())
                self.CalData[-1].pushBlock(self.DataLoopBuffer)
                #push data to CalTimeSeries Object
        self.IntegratedPacketsCount=self.IntegratedPacketsCount+1
        self.i=0

def callback(data):
    """
    

    Parameters
    ----------
    data : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
    """
    
    Parameters
    ----------
    data : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
    print(data)
    DB1.pushData(data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z,0,data.header.stamp/1e9)
    return

def listener():
    """
    

    Returns
    -------
    None.

    """

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("IMU0", Imu, callback)
    #rospy.Subscriber("IMU1", Imu, callback)
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

def DataReaderROS(RosCSVFilename):
    sdf=pandas.read_csv(RosCSVFilename)
    print(sdf.columns.values)
    chunkSize=DB1.INTEGRATIONTIME
    for Index in np.arange(0,len(sdf),chunkSize):
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
    chunkSize=DB1.INTEGRATIONTIME
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

def doAllFFT(DB):
    for item in DB.CalData:
        item.CalcFFT()

def doAllSinFit(DB):
    for item in DB.CalData:
        item.SinFit()

if __name__ == '__main__':
    DB1=Databuffer()
    DataReaderPROTOdump('data/20190819_1500_10_250hz_10_ms2_woairatstart.dump')