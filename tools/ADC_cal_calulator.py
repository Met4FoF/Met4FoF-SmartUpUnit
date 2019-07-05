#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  3 13:44:55 2019

@author: seeger01
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import stats

#CalData=pd.read_csv('./STM32Toolchain/projects/Met4FoF-SmartUpUnit/tools/Board1_ADC2_DC_cal.csv',sep=';')
plt.plot(CalData['ADCCOunts'],CalData['Vin'], 'o', label='original data')
plt.plot(CalData['ADCCOunts'], intercept + slope*CalData['ADCCOunts'], 'r', label='ADC Fitt')
plt.legend()
plt.show()
slope, intercept, r_value, p_value, std_err = stats.linregress(CalData['ADCCOunts'], CalData['Vin'])
print("Slope= "+str(slope)+" V/LSB")
print("Min Val= "+str(intercept)+" V  [ADC=0 LSB]")
MAXVal=4095*slope+intercept
print("MAx Val= "+str(MAXVal)+" V [ADC=4095 LSB]")
print("R^2-Value= "+ str(r_value*r_value))
print("P-Value= "+ str( p_value))
print("std Error= "+ str(std_err))
