#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  5 14:21:29 2018

@author: seeger01
"""
from struct import *
import socket
import numpy as np
import collections
import matplotlib
from matplotlib import pyplot as plt

UDP_IP = "192.168.0.1"
UDP_PORT = 7000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
BUFFERSIZE = 5000
Buffer = collections.deque(maxlen=BUFFERSIZE)
DataSinceLastPlot=0;
# INIT figue
fig = plt.gcf()
fig.show()
fig.canvas.draw()
while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    Buffer.append(unpack('IIIffff',data))
    DataSinceLastPlot=DataSinceLastPlot+1
    if(DataSinceLastPlot>=BUFFERSIZE):
        plt.clf()
        plt.plot(np.arange(0,BUFFERSIZE,1),np.asarray([Buffer[i][3] for i in range(0, BUFFERSIZE)]),np.arange(0,BUFFERSIZE,1),np.asarray([Buffer[i][4] for i in range(0, BUFFERSIZE)]),np.arange(0,BUFFERSIZE,1),np.asarray([Buffer[i][5] for i in range(0, BUFFERSIZE)]))  
        fig.canvas.draw()
        DataSinceLastPlot=0
        break