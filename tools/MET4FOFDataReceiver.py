#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  4 09:20:12 2019

Data receiver for Met4FoF Protobuff Data
@author: seeger01
"""

import sys
import socket
import threading
import messages_pb2
from datetime import datetime
import threading
from multiprocessing import  Queue
#mp.set_start_method('spawn')

class DataReceiver:

    def __init__(self,IP,Port):
        self.flags={'Networtinited':False}
        self.params={'IP':IP,
                     'Port':Port,
                     'PacketrateUpdateCount':1000
                }
        self.socket=socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        try:
            self.socket.bind((IP, Port))
        except OSError as err:
            print("OS error: {0}".format(err))
            if(err.errno == 99):
                print("most likely no network card of the system has the ip address"
                      +str(IP)+" check this with >>> ifconfig on linux or with >>> ipconfig on Windows")
            if(err.errno == 98):
                print('an other task is blocking the connection on linux use >>> sudo netstat -ltnp | grep -w \':'
                      +str(Port)+'\' on windows use in PowerShell >>> Get-Process -Id (Get-NetTCPConnection -LocalPort '
                      +str(Port)+').OwningProcess')
            raise(err)
            #we need to raise an exception to prevent __init__ from returning
            #otherwise a broken class instance will be created
        except:
            print("Unexpected error:", sys.exc_info()[0])
            raise("Unexpected error:", sys.exc_info()[0])
        self.flags['Networtinited']=True
        self.AllSensors={}
        self.ActiveSensors={}
        self.msgcount=0
        self.lastTimestamp=0
        self.Datarate=0
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start()
        #self.udpReceiverTask = Process(target=self.SrartUDPReceiverTask, args=())

    def run(self):
        #implement stop routine
        while True:
            data, addr = self.socket.recvfrom(1024) # buffer size is 1024 bytes
            wasValidData=False
            wasValidDescription=False
            ProtoData = messages_pb2.DataMessage()
            ProtoDescription=messages_pb2.DescriptionMessage()
            SensorID=0
            #TODO check error handling
            #TODO add state machine
            try:
                ProtoData.ParseFromString(data)
                wasValidData=True
                SensorID=ProtoData.id
                message=ProtoData
            except:
                pass #? no exception for wrong data type !!
            #  todo improve parsing
            #  try:
            #      ProtoDescription.ParseFromString(data)
            #      wasValidDescription=True
            #      SensorID=ProtoDescription.id
            #      message=ProtoDescription
            # except:
            #     pass

            if not(wasValidData or wasValidDescription):
                print("INVALID PROTODATA")
                pass # invalid data leave parsing routine
            if SensorID in self.AllSensors:
                self.AllSensors[SensorID].buffer.put(message)
            else:
                self.AllSensors[SensorID]=Sensor(SensorID)
                print("FOUND NEW SENSOR WITH ID="+str(SensorID))
            self.msgcount=self.msgcount+1

            if (self.msgcount%self.params['PacketrateUpdateCount']==0):
                print('received '+str(self.params['PacketrateUpdateCount'])+' packets')
                if(self.lastTimestamp!=0):
                    timeDIFF=datetime.now()-self.lastTimestamp
                    timeDIFF=timeDIFF.seconds+timeDIFF.microseconds*1e-6
                    self.Datarate=self.params['PacketrateUpdateCount']/timeDIFF
                    print('Update rate is '+str(self.Datarate)+' Hz')
                    self.lastTimestamp=datetime.now()
                else:
                    self.lastTimestamp=datetime.now()




    def __del__(self):
        self.socket.close()


class Sensor:
    #TODO implement multi therading and callbacks
    def __init__(self,ID,BufferSize=1e7):
        self.buffer=Queue()
        self.ID=ID

