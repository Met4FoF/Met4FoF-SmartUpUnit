#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 23 09:27:10 2019

@author: seeger01
"""
#this script should be used to controal an RIGOL DG4162 FGen
# wire shark debung filter options (ip.src == 192.168.2.3 || ip.dst == 192.168.2.3 ) && !(tcp.flags.ack && tcp.len <= 1)
import pyvisa #pip install -U pyvisa-py
import time
TCPIPSTRING='TCPIP0::192.168.2.3::INSTR'
rm = pyvisa.ResourceManager()


class DG4xxx:

    def __init__(self,rm,IP):

        self.params={'IP':IP}
        self.rm=rm
        self.visa=self.rm.open_resource('TCPIP0::'+IP+'::INSTR')
        self.visa.timeout=10000
        retval=self.visa.query('*IDN?')
        self.params['IP']
        self.params['Vendor']=retval.split(",")[0]
        self.params['Type']=retval.split(",")[1]
        self.params['SN']=retval.split(",")[2]
        self.params['FWVersion']=retval.split(",")[3].replace('\n', '')

    def queryCommand(self,CMD):
        return self.visa.query(CMD)

    def sendCommand(self,CMD):
        return self.visa.write(CMD)

    def fiereBurst(self,freq,ampl,offset,phase,cycles,channel=1,delay=False):
        minAmpl=-ampl/2+offset
        maxAmpl=ampl/2+offset
        if(minAmpl<0):
            raise ValueError('Output voltage should never be negative but is at minium '+str(minAmpl))
        if(maxAmpl>5):
            raise ValueError('Output voltage should never be grater than 5 Volt but is at max '+str(maxAmpl))
        if(freq<0.001):
            raise ValueError('Output frequency is to low (<1 mHz) or negative')
        self.sendCommand(':OUTPut'+str(channel)+':STATe OFF')
        self.sendCommand(':SOURce'+str(channel)+':APPLy:SINusoid '+str(freq)+','+str(ampl)+','+str(offset)+','+str(phase))
        self.sendCommand(':SOURce'+str(channel)+':BURSt:NCYCles '+str(cycles))
        self.sendCommand(':SOURce'+str(channel)+':BURSt:TRIGger:SOURce MANual') #activate manual TRIGger
        self.sendCommand(':SOURce'+str(channel)+':BURSt ON')#activate burst mode
        self.sendCommand(':OUTPut'+str(channel)+':STATe ON')
        self.sendCommand(':SOURce'+str(channel)+':BURSt:TRIGger')
        if delay:
            Duration=1/freq*cycles
            print("Burst Duration="+str(Duration)+' seconds \nsleeping this time')
            time.sleep(Duration)
            self.sendCommand(':OUTPut OFF')
            return
        else:
            return

def Fireloop():
    for x in range(0, 20):
        DG4162.fiereDifferentialBurst(100.0,0.01,0.005,0,10,True)
