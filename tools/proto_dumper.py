#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 25 15:42:10 2019

@author: seeger01
"""

import messages_pb2
import socket
import numpy as np
import matplotlib.pyplot as plt
UDP_IP = "192.168.0.1"
UDP_PORT = 7000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

def proto_dumper():
    with open('log_files/20190626_Timing_dump3.protodump', 'w') as protodumpfile:   
        while True:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            ProtoData = messages_pb2.DataMessage()
            ProtoData.ParseFromString(data)
            protodumpfile.write(data+"\n")
            print(ProtoData)
    return 