#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  3 11:42:10 2019

@author: seeger01
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 11:21:54 2019

@author: seeger01
"""

import numpy as np
import socket
import messages_pb2


UDP_IP = "192.168.2.100"
UDP_PORT = 7000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
def writeHeader(filename):
    dumpfile = open(filename, "a")
    dumpfile.write("id;sample_number;unix_time;unix_time_nsecs;time_uncertainty;Data_01;Data_02;Data_03;Data_04;Data_05;Data_06;Data_07;Data_08;Data_09;Data_10;Data_11;Data_12;Data_13;stimfeq;stimampl;stimtype\n")
    dumpfile.close()    
def datadumper_reader(samplecount,stimfreq,stimampl,stimtype,filename):
    BUFFERFLUSHSIZE=2000#trow awy first 1000 samples because this could be old values from the Ethernetbuffer
    i=0
    dumpfile = open(filename, "a")
    try:
        while i<samplecount+BUFFERFLUSHSIZE:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            ProtoData = messages_pb2.DataMessage()
            ProtoData.ParseFromString(data)
            if(int(ProtoData.id/65536)== 13616):
                if(i>=BUFFERFLUSHSIZE):
                    dumpfile.write(str(ProtoData.id)+';'+str(ProtoData.sample_number)+';'+str(ProtoData.unix_time)+';'+
                                   str(ProtoData.unix_time_nsecs)+';'+str(ProtoData.time_uncertainty)+';'+
                                   str(ProtoData.Data_01)+';'+str(ProtoData.Data_02)+';'+str(ProtoData.Data_03)+';'+
                                   str(ProtoData.Data_04)+';'+str(ProtoData.Data_05)+';'+str(ProtoData.Data_06)+';'+
                                   str(ProtoData.Data_07)+';'+str(ProtoData.Data_08)+';'+str(ProtoData.Data_09)+';'+
                                   str(ProtoData.Data_10)+';'+str(ProtoData.Data_11)+';'+str(ProtoData.Data_12)+';'+
                                   str(ProtoData.Data_13)+';'+str(stimfreq)+';'+str(stimampl)+';'+str(stimtype)+'\n')
                i=i+1
        dumpfile.close()
    except KeyboardInterrupt:
        dumpfile.close()
        print('interrupted!')
