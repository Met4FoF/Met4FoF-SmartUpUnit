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

def analog_reader():
    i=0
    values=np.zeros(1000)
    try:
        while True:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            ProtoData = messages_pb2.DataMessage()
            ProtoData.ParseFromString(data)
            if(int(ProtoData.id/65536)== 13616):
                values[i]=ProtoData.Data_11
                i=i+1
                if(i==values.size):
                    print("MEAN:="+str(np.mean(values)))
                    print("STD:="+str(np.std(values)))
                    i=0
    except KeyboardInterrupt:
        print('interrupted!')
