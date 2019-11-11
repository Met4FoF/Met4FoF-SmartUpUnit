#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  4 09:20:12 2019

Data receiver for Met4FoF Protobuff Data
@author: seeger01
"""

import sys
import traceback
import os
import socket
import threading
import messages_pb2
import google.protobuf as pb
from google.protobuf.internal.encoder import _VarintBytes
from google.protobuf.internal.decoder import _DecodeVarint32
from datetime import datetime
import threading
import time
from multiprocessing import Queue
import copy


class DataReceiver:
    def __init__(self, IP, Port):
        self.flags = {"Networtinited": False}
        self.params = {"IP": IP, "Port": Port, "PacketrateUpdateCount": 10000}
        self.socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM  # Internet
        )  # UDP
        try:
            self.socket.bind((IP, Port))
        except OSError as err:
            print("OS error: {0}".format(err))
            if err.errno == 99:
                print(
                    "most likely no network card of the system has the ip address"
                    + str(IP)
                    + " check this with >>> ifconfig on linux or with >>> ipconfig on Windows"
                )
            if err.errno == 98:
                print(
                    "an other task is blocking the connection on linux use >>> sudo netstat -ltnp | grep -w ':"
                    + str(Port)
                    + "' on windows use in PowerShell >>> Get-Process -Id (Get-NetTCPConnection -LocalPort "
                    + str(Port)
                    + ").OwningProcess"
                )
            raise (err)
            # we need to raise an exception to prevent __init__ from returning
            # otherwise a broken class instance will be created
        except:
            print("Unexpected error:", sys.exc_info()[0])
            raise ("Unexpected error:", sys.exc_info()[0])
        self.flags["Networtinited"] = True
        self.AllSensors = {}
        self.ActiveSensors = {}
        self.msgcount = 0
        self.lastTimestamp = 0
        self.Datarate = 0
        self._stop_event = threading.Event()
        thread = threading.Thread(target=self.run, args=())
        thread.start()

    def stop(self):
        print("Stopping DataReceiver")
        self._stop_event.set()
        # wait 1 second to ensure that all ques are empty before closing them
        # other wise SIGPIPE is raised by os
        # IMPORVEMNT use signals for this
        time.sleep(1)
        for key in self.AllSensors:
            self.AllSensors[key].stop()
        self.socket.close()

    def run(self):
        # implement stop routine
        while not self._stop_event.is_set():
            data, addr = self.socket.recvfrom(1024)  # buffer size is 1024 bytes
            wasValidData = False
            wasValidDescription = False
            ProtoData = messages_pb2.DataMessage()
            ProtoDescription = messages_pb2.DescriptionMessage()
            SensorID = 0
            BytesProcessed = 4  # we need an offset of 4 sice
            if data[:4] == b"DATA":
                while BytesProcessed < len(data):
                    msg_len, new_pos = _DecodeVarint32(data, BytesProcessed)
                    BytesProcessed = new_pos

                    try:
                        msg_buf = data[new_pos : new_pos + msg_len]
                        ProtoData.ParseFromString(msg_buf)
                        wasValidData = True
                        SensorID = ProtoData.id
                        message = {'ProtMsg':copy.deepcopy(ProtoData),'Type':'Data'}
                        BytesProcessed += msg_len
                    except:
                        pass  # ? no exception for wrong data type !!
                    if not (wasValidData or wasValidDescription):
                        print("INVALID PROTODATA")
                        pass  # invalid data leave parsing routine

                    if SensorID in self.AllSensors:
                        try:
                            self.AllSensors[SensorID].buffer.put_nowait(message)
                        except:
                            print("packet lost for sensor ID:" + str(SensorID))
                    else:
                        self.AllSensors[SensorID] = Sensor(SensorID)
                        print("FOUND NEW SENSOR WITH ID=hex" + hex(SensorID)+'==>dec:'+str(SensorID))
                    self.msgcount = self.msgcount + 1

                    if self.msgcount % self.params["PacketrateUpdateCount"] == 0:
                        print(
                            "received "
                            + str(self.params["PacketrateUpdateCount"])
                            + " packets"
                        )
                        if self.lastTimestamp != 0:
                            timeDIFF = datetime.now() - self.lastTimestamp
                            timeDIFF = timeDIFF.seconds + timeDIFF.microseconds * 1e-6
                            self.Datarate = (
                                self.params["PacketrateUpdateCount"] / timeDIFF
                            )
                            print("Update rate is " + str(self.Datarate) + " Hz")
                            self.lastTimestamp = datetime.now()
                        else:
                            self.lastTimestamp = datetime.now()
            elif data[:4] == b"DSCP":
                while BytesProcessed < len(data):
                    msg_len, new_pos = _DecodeVarint32(data, BytesProcessed)
                    BytesProcessed = new_pos
                    try:
                        msg_buf = data[new_pos : new_pos + msg_len]
                        ProtoDescription.ParseFromString(msg_buf)
                        wasValidData = True
                        SensorID = ProtoDescription.id
                        message = {'ProtMsg':ProtoDescription,'Type':'Description'}
                        BytesProcessed += msg_len
                    except:
                        pass  # ? no exception for wrong data type !!
                    if not (wasValidData or wasValidDescription):
                        print("INVALID PROTODATA")
                        pass  # invalid data leave parsing routine

                    if SensorID in self.AllSensors:
                        try:
                            self.AllSensors[SensorID].buffer.put_nowait(message)
                        except:
                            print("packet lost for sensor ID:" + hex(SensorID))
                    else:
                        self.AllSensors[SensorID] = Sensor(SensorID)
                        print("FOUND NEW SENSOR WITH ID=hex" + hex(SensorID)+' dec==>:'+str(SensorID))
                    self.msgcount = self.msgcount + 1

                    if self.msgcount % self.params["PacketrateUpdateCount"] == 0:
                        print(
                            "received "
                            + str(self.params["PacketrateUpdateCount"])
                            + " packets"
                        )
                        if self.lastTimestamp != 0:
                            timeDIFF = datetime.now() - self.lastTimestamp
                            timeDIFF = timeDIFF.seconds + timeDIFF.microseconds * 1e-6
                            self.Datarate = (
                                self.params["PacketrateUpdateCount"] / timeDIFF
                            )
                            print("Update rate is " + str(self.Datarate) + " Hz")
                            self.lastTimestamp = datetime.now()
                        else:
                            self.lastTimestamp = datetime.now()
            else:
                print("unrecognized packed preamble"+str(data[:5]))

    def getsenorIDs(self):
        return [*self.AllSensors]

    def __del__(self):
        self.socket.close()


class Sensor:
    # TODO implement multi therading and callbacks
    def __init__(self, ID, BufferSize=1e4):
        self.buffer = Queue(int(BufferSize))
        self.flags = {
            "DumpToFile": False,
            "PrintProcessedCounts": True,
            "callbackSet": False,
        }
        self.params = {"ID": ID, "BufferSize": BufferSize, "DumpFileName": ""}

        self._stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, args=())
        # self.thread.daemon = True
        self.thread.start()
        self.ProcessedPacekts = 0
        self.lastPacketTimestamp = datetime.now()
        self.deltaT = (
            self.lastPacketTimestamp - datetime.now()
        )  # will b 0 but has deltaTime type witch is intended
        self.datarate = 0

    def StartDumpingToFile(self, filename):
        # check if the path is valid
        # if(os.path.exists(os.path.dirname(os.path.abspath('data/dump.csv')))):
        self.Dumpfile = open(filename, "a")
        self.params["DumpFileName"] = filename
        self.flags["DumpToFile"] = True

    def StopDumpingToFile(self):
        self.flags["DumpToFile"] = False
        self.params["DumpFileName"] = ""
        self.Dumpfile.close()

    def run(self):
        while not self._stop_event.is_set():
            # problem when we are closing the queue this function is waiting for data and raises EOF error if we delet the q
            # work around adding time out so self.buffer.get is returning after a time an thestop_event falg can be checked
            try:
                message = self.buffer.get(timeout=0.1)
                tmpTime = datetime.now()
                self.deltaT = (
                    tmpTime - self.lastPacketTimestamp
                )  # will b 0 but has deltaTime type witch is intended
                self.datarate = 1 / (self.deltaT.seconds + 1e-6 * self.deltaT.microseconds)
                self.lastPacketTimestamp = datetime.now()
                self.ProcessedPacekts = self.ProcessedPacekts + 1
                if self.flags["PrintProcessedCounts"]:
                    if self.ProcessedPacekts % 10000 == 0:
                        print(
                            "processed 10000 packets in receiver for Sensor ID:"
                            + hex(self.params["ID"])
                        )
                if self.flags["callbackSet"]:
                    if(message['Type']=='Data'):
                        try:
                            self.callback(message['ProtMsg'])
                        except Exception:
                            print (" Sensor id:"+hex(self.params["ID"])+"Exception in user callback:")
                            print('-'*60)
                            traceback.print_exc(file=sys.stdout)
                            print('-'*60)
                            pass
            except Exception:
                pass

    def SetCallback(self, callback):
        self.flags["callbackSet"] = True
        self.callback = callback

    def stop(self):
        print("Stopping Sensor " + hex(self.params["ID"]))
        self._stop_event.set()
        # sleeping until run function is exiting due to timeout
        time.sleep(0.2)
        # thrash all data in queue
        while not self.buffer.empty():
            try:
                self.buffer.get(False)
            except:
                pass
        self.buffer.close()

    def join(self, *args, **kwargs):
        self.stop()

def DumpDataMPU9250(message):
    PRINTDEVIDER=200
    filename='data/DataDump.log'
    if not (os.path.exists(filename)):
        dumpfile = open(filename, "a+")
        dumpfile.write("id;sample_number;unix_time;unix_time_nsecs;time_uncertainty;ACC_x;ACC_y,;ACC_z,;GYR_x;GYR_y;GYR_z;MAG_x;MAG_y;MAG_z;TEMP;ADC_1;ADC_2;ADC_3\n")
    else:
        dumpfile = open(filename, "a")
        if(message.sample_number%PRINTDEVIDER==0):
            print('=====DATA PACKET====','\n',
                    hex(message.id),message.sample_number,message.unix_time,message.unix_time_nsecs,message.time_uncertainty,
                  '\n ACC:',message.Data_01,message.Data_02,message.Data_03,
                  '\n GYR:',message.Data_04,message.Data_05,message.Data_06,
                  '\n MAG:',message.Data_07,message.Data_08,message.Data_09,
                  '\n TEMP:',message.Data_10,
                  '\n ADC:',message.Data_11,message.Data_12,message.Data_13),

        dumpfile.write(str(message.id)+';'+
                       str(message.sample_number)+';'+
                       str(message.unix_time)+';'+
                       str(message.unix_time_nsecs)+';'+
                       str(message.time_uncertainty)+';'+
                       str(message.Data_01)+';'+
                       str(message.Data_02)+';'+
                       str(message.Data_03)+';'+
                       str(message.Data_04)+';'+
                       str(message.Data_05)+';'+
                       str(message.Data_06)+';'+
                       str(message.Data_07)+';'+
                       str(message.Data_08)+';'+
                       str(message.Data_09)+';'+
                       str(message.Data_10)+';'+
                       str(message.Data_11)+';'+
                       str(message.Data_12)+';'+
                       str(message.Data_13)+';'+
                       "\n")
    dumpfile.close()

def DumpDataGPSDummySensor(message):
    if not (os.path.exists('data/GPSLog.log')):
        dumpfile = open('data/GPSLog.log', "a+")
        dumpfile.write("id;sample_number;unix_time;unix_time_nsecs;time_uncertainty;GPSCount\n")
    else:
        dumpfile = open('data/GPSLog', "a")
        #2^48=281474976710656 2^32=4294967296 2^16=65536
        gpscount=message.Data_01*281474976710656+message.Data_02*4294967296+message.Data_03*65536+message.Data_04
        print(hex(message.id),message.sample_number,message.unix_time,message.unix_time_nsecs,message.time_uncertainty,gpscount)
        dumpfile.write(str(message.id)+';'+
                       str(message.sample_number)+';'+
                       str(message.unix_time)+';'+
                       str(message.unix_time_nsecs)+';'+
                       str(message.time_uncertainty)+';'+
                       str(gpscount)+"\n")
    dumpfile.close()