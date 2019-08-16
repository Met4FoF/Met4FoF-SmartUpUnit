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

class CalTimeSeries():
    def __init__(self,DB):
        self.Data=np.zeros([DB.INTEGRATIONTIME,4])
        self.length=0
    def pushBlock(self,Datablock):
        if(self.length==0):
            self.Data=Datablock
        else:
            self.Data=np.append(self.Data,Datablock, axis=0)
        self.length=self.length+Datablock.shape[0]


class Databuffer:
    def __init__(self):
        self.INTEGRATIONTIME = 512
        self.DataLoopBuffer=np.zeros([self.INTEGRATIONTIME,4])
        self.i=0
        self.IntegratedPacketsCount=0
        self.DATAARRYSIZE=900
        self.FFTArray=np.zeros([self.DATAARRYSIZE,self.INTEGRATIONTIME,3])
        self.STDArray=np.zeros([self.DATAARRYSIZE,3])
        self.minRMSforVailid=4.0
        self.minValidChunksInRow=4
        self.axixofintest=2
        self.isValidCalChunk=np.zeros([self.DATAARRYSIZE,3])
        self.CalData=[]




    def pushData(self,x,y,z,t):
        if(self.i<self.INTEGRATIONTIME):
            self.DataLoopBuffer[self.i,0]=x
            self.DataLoopBuffer[self.i,1]=y
            self.DataLoopBuffer[self.i,2]=z
            self.DataLoopBuffer[self.i,3]=t
            self.i=self.i+1
            if(self.i==self.INTEGRATIONTIME):
                self.dalc()

    def pushBlock(self,arrayx,arrayy,arrayz,arrayt):
        if not (self.INTEGRATIONTIME==arrayt.size==arrayy.size==arrayy.size==arrayz.size):
            raise ValueError('Array size must be INTEGRATIONTIME '+str(self.INTEGRATIONTIME) +'and not '+str(arrayt.size))
        self.DataLoopBuffer[:,0]=arrayx
        self.DataLoopBuffer[:,1]=arrayy
        self.DataLoopBuffer[:,2]=arrayz
        self.DataLoopBuffer[:,3]=arrayt
        self.i=self.INTEGRATIONTIME
        self.calc()

    def calc(self):
        if(self.IntegratedPacketsCount<self.DATAARRYSIZE):
            self.STDArray[self.IntegratedPacketsCount]=np.std(self.DataLoopBuffer[:,:3],axis=0)
            self.FFTArray[self.IntegratedPacketsCount,:,:]=np.fft.fft(self.DataLoopBuffer[:,:3],axis=0)
        #print("Mean="+str(self.STDArray[self.IntegratedPacketsCount,2]))
        print(str(self.IntegratedPacketsCount) +' STD= '+str(np.std(self.DataLoopBuffer,axis=0)))
        self.IntegratedPacketsCount=self.IntegratedPacketsCount+1
        self.i=0
        if(self.IntegratedPacketsCount>self.minValidChunksInRow):
            self.isValidCalChunk[self.IntegratedPacketsCount]=np.all(np.greater(self.STDArray[self.IntegratedPacketsCount-self.minValidChunksInRow:self.IntegratedPacketsCount,self.axixofintest],self.minRMSforVailid))
            #check if we have an new valid data Chunk
            if(self.isValidCalChunk[self.IntegratedPacketsCount-1,self.axixofintest]==False and self.isValidCalChunk[self.IntegratedPacketsCount,self.axixofintest]==True):
                print(self.IntegratedPacketsCount)
                self.CalData.append(CalTimeSeries(self))
                self.CalData[-1].pushBlock(self.DataLoopBuffer)
            if(self.isValidCalChunk[self.IntegratedPacketsCount-1,self.axixofintest]==True and self.isValidCalChunk[self.IntegratedPacketsCount,self.axixofintest]==True):
                self.CalData[-1].pushBlock(self.DataLoopBuffer)


#DB0=Databuffer()
DB1=Databuffer()

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
    print(data)
    DB1.pushData(data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z,data.header.stamp/1e9)
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
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("IMU0", Imu, callback)
    rospy.Subscriber("IMU1", Imu, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def DataReader(RosCSVFilename):
    sdf=pandas.read_csv(RosCSVFilename)
    print(sdf.columns.values)
    chunkSize=DB1.INTEGRATIONTIME
    Chunkindexes=np.arange(0,len(sdf),chunkSize)
    for Index in Chunkindexes[:-1]:#cutting out last chung since this this is porpaply not 512 datapoints long 
        DB1.pushBlock(sdf['field.linear_acceleration.x'][Index:Index+chunkSize],sdf['field.linear_acceleration.y'][Index:Index+chunkSize],sdf['field.linear_acceleration.z'][Index:Index+chunkSize],sdf['field.header.stamp'][Index:Index+chunkSize]/1e9)


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
#        'field.linear_acceleration_covariance8']

if __name__ == '__main__':
    DataReader('C:/temp/IMU0.csv')