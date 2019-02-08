#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from struct import *
import socket
import csv

LOGFILENAME ='log_files/k050HZTekAFG3101ext10MHz'
ACCLOGFILENAME = LOGFILENAME+'acc.csv'
GPSLOGFILENAME = LOGFILENAME+'gpst.csv'
SYNCLOGFILENAME = LOGFILENAME+'SYNC.csv'
LOGGINENABLED = True
UDP_IP = "192.168.0.1"
UDP_PORT = 7000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))


def imu_publisher():
    GPSTCount=0
    RefCount=0
    pub_imu = rospy.Publisher("IMU", Imu, queue_size=1)
    rospy.init_node('imu_publisher', anonymous=True)
    if(LOGGINENABLED): 
        with open(ACCLOGFILENAME, mode='a') as DtataCSV:
            while not rospy.is_shutdown():
                data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
                if(data.startswith("ACC3")):
                    unpackeddata=unpack('IIIIIIHffff',data)
                    imu_msg = Imu()
                    imu_msg.linear_acceleration.x=unpackeddata[6]
                    imu_msg.linear_acceleration.y=unpackeddata[7]
                    imu_msg.linear_acceleration.z=unpackeddata[8]
                    imu_msg.header.stamp = rospy.Time(unpackeddata[1],unpackeddata[2])
                    imu_msg.header.frame_id = "Imu"
                    imu_msg.header.seq=unpackeddata[6]
                    pub_imu.publish(imu_msg)
                    DtataCSVwriter = csv.writer(DtataCSV, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    DtataCSVwriter.writerow(unpackeddata)
                if(data.startswith("GYR3")):
                    unpackeddata=unpack('IIIIIIHffff',data)
                    print(unpackeddata)
                    #DtataCSVwriter = csv.writer(DtataCSV, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    #DtataCSVwriter.writerow(unpackeddata) 
                if(data.startswith("GPST")):
                    unpackeddata=unpack('II',data)
                    print(GPSTCount,unpackeddata[1],"GPST")
                    with open(LOGFILENAME, mode='a') as GPSTimeDtataCSV:
                        GPSCSV_writer = csv.writer(GPSTimeDtataCSV, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        GPSCSV_writer.writerow(["GPST",GPSTCount,unpackeddata[1]])    
                    GPSTCount=GPSTCount+1
                if(data.startswith("SYNT")):
                    unpackeddata=unpack('II',data)
                    print(GPSTCount,unpackeddata[1],"SYNT")
                    with open(SYNCLOGFILENAME, mode='a') as SYNCTimeDtataCSV:
                        SYNCCSV_writer = csv.writer(SYNCTimeDtataCSV, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        SYNCCSV_writer.writerow(["SYNCT",RefCount,unpackeddata[1]])    
                    RefCount=RefCount+1            
            

        
if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
