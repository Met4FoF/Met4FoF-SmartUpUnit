#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  4 09:20:12 2019

Data receiver for Met4FoF Protobuff Data
@author: seeger01
"""

import sys
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
                        n = new_pos
                        ProtoData.ParseFromString(msg_buf)
                        wasValidData = True
                        SensorID = ProtoData.id
                        message = ProtoData
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
                        print("FOUND NEW SENSOR WITH ID=" + hex(SensorID))
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
                print("unrecognized packed preamble"+data[:4].hex())

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
            # problem when wee are closing the queue this function is waiting for data and raises EOF error if we delet the q
            # work around adding time out so self.buffer.get is returning after a time an thestop_event falg can be checked
            try:
                message = self.buffer.get(timeout=0.1)
            except Exception:
                break
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
                self.callback(message)

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
