# -*- coding: utf-8 -*-
"""
Created on Fri Sep 27 13:42:10 2019

@author: benes
"""

import sys
import os
import socket
import threading
import messages_pb2
import google.protobuf as pb
import time
import pause
import numpay as np


class SensorSimulator:
    def __init__(self, IP, Port, ID):
        self.flags = {"Networtinited": False}
        self.params = {"IP": IP, "Port": Port, "ID": ID}
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
        self.msgcount = 0
        self.lastTimestamp = 0

    def ConstSine(Freqs, Phases, Ampls, SampleRate, Duration):
        Freqs = self.ShapeInputArray(Freqs, 15)
        Phases = self.ShapeInputArray(Phases, 15)
        Ampls = self.ShapeInputArray(Ampls, 15)
        samplesToSend = int(SampleRate * Duration)
        SampleCount = 0
        starttime = time.time_ns()
        ProtoData = messages_pb2.DataMessage()
        while SampleCount < samplesToSend:
            DeltaT = SampleCount / SampleRate
            Values = np.sin(Freqs * 2 * np.pi * DeltaT + Phases) * Ampls
            print(Values)
            pause.until(starttime / 1e9 + DeltaT)
            SampleCount = SampleCount + 1

    def ShapeInputArray(Array, expLen):
        RetArray = np.zeros(expLen)
        if Array.size <= expLen:
            RetArray[: Array.size] = np.copy(Array)
        else:
            RetArray = np.copy(Array[:expLen])
        return RetArray
