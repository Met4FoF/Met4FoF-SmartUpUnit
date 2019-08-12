#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Aug  7 12:34:21 2019

@author: seeger01
"""

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt

class Databuffer:
    def __init__(self):
        self.INTEGRATIONTIME = 512
        self.DataLoopBuffer=np.zeros([self.INTEGRATIONTIME,3])
        self.i=0
        self.IntegratedPacketsCount=0
        self.DATAARRYSIZE=1000
        self.FFTArray=np.zeros([self.DATAARRYSIZE,self.INTEGRATIONTIME,3])
        self.STDArray=np.zeros([self.DATAARRYSIZE,3])

    def pushData(self,x,y,z):
        if(self.i<self.INTEGRATIONTIME):
            self.DataLoopBuffer[self.i,0]=x
            self.DataLoopBuffer[self.i,1]=y
            self.DataLoopBuffer[self.i,2]=z
            self.i=self.i+1
        if(self.i==self.INTEGRATIONTIME):
            if(self.IntegratedPacketsCount<self.DATAARRYSIZE):
                self.STDArray[self.IntegratedPacketsCount]=np.std(self.DataLoopBuffer,axis=0)
                self.FFTArray[self.IntegratedPacketsCount]=np.fft.fft(self.DataLoopBuffer)
            print("Mean="+str(np.mean(self.DataLoopBuffer,axis=0)))
            print("STD="+str(np.std(self.DataLoopBuffer,axis=0)))
            self.IntegratedPacketsCount=self.IntegratedPacketsCount+1
            self.i=0


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
    DB1.pushData(data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z)
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

if __name__ == '__main__':
    listener()