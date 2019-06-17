#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 17 07:02:07 2019

@author: seeger01
"""
from struct import *
import socket


LOGFILENAME ='log_files/20190617_sensor_cooling'
ACCLOGFILENAME = LOGFILENAME+'ACC.csv'
GPSLOGFILENAME = LOGFILENAME+'GPST.csv'
SYNCLOGFILENAME = LOGFILENAME+'REFT.csv'
UDP_IP = "192.168.0.1"
UDP_PORT = 7000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

def data_receiver():
    GPSTCount=0
    RefCount=0
    with open(ACCLOGFILENAME, mode='a') as DtataCSV:
        while True:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            #print(data)
            if(data.startswith("ACC3")):
                DtataCSV.write(data)
                print(data)
            if(data.startswith("REFT")):
                print(data)
                with open(SYNCLOGFILENAME, mode='a') as REFTDtataCSV:
                    REFTDtataCSV.write(data)    
 
                    
if __name__ == '__main__':
    try:
        data_receiver()
    except KeyboardInterrupt:
        pass