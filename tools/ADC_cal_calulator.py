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

CalData = pd.read_csv("Board_001_DC_CAL.dump", sep=";")
# slope, intercept, r_value, p_value, std_err
ADC1_fit = stats.linregress(CalData["Data_11"], CalData["stimampl"])
ADC2_fit = stats.linregress(CalData["Data_12"], CalData["stimampl"])
ADC3_fit = stats.linregress(CalData["Data_13"], CalData["stimampl"])
plt.plot(CalData["Data_11"], CalData["stimampl"], "o", label="ADC 1original data")
plt.plot(
    CalData["Data_11"], ADC1_fit[1] + ADC1_fit[0] * CalData["Data_11"], label="ADC1 Fit"
)
ADC1Residi = np.square(
    CalData["stimampl"] - (ADC1_fit[1] + ADC1_fit[0] * CalData["Data_11"])
)
ADC1Residi = np.mean(np.sqrt(ADC1Residi))  # 95,449 9736 %
plt.plot(CalData["Data_12"], CalData["stimampl"], "o", label="ADC 2 original data")
plt.plot(
    CalData["Data_12"], ADC2_fit[1] + ADC2_fit[0] * CalData["Data_12"], label="ADC3 Fit"
)
ADC2Residi = np.square(
    CalData["stimampl"] - (ADC2_fit[1] + ADC2_fit[0] * CalData["Data_12"])
)
ADC2Residi = np.mean(np.sqrt(ADC2Residi))  # 95,449 9736 %
plt.plot(CalData["Data_13"], CalData["stimampl"], "o", label="ADC 3 original data")
plt.plot(
    CalData["Data_13"], ADC3_fit[1] + ADC3_fit[0] * CalData["Data_13"], label="ADC3 Fit"
)
ADC3Residi = np.square(
    CalData["stimampl"] - (ADC3_fit[1] + ADC3_fit[0] * CalData["Data_13"])
)
ADC3Residi = np.mean(np.sqrt(ADC3Residi))  # 95,449 9736 %
plt.legend()
plt.show()
print("===ADC 1 ===")
print("Slope= " + str(ADC1_fit[0]) + " V/LSB")
print("Min Val= " + str(ADC1_fit[1]) + " V [ADC=0 LSB]")
MAXVal = 4095 * ADC1_fit[0] + ADC1_fit[1]
print("Max Val= " + str(MAXVal) + " V [ADC=4095 LSB]")
print("R^2-Value= " + str(ADC1_fit[2] * ADC1_fit[2]))
print("Fit std Error= " + str(ADC1_fit[4]))
print(" RMS Noise = " + str(ADC1Residi * 1000) + " mV")
print("===ADC 2 ===")
print("Slope= " + str(ADC2_fit[0]) + " V/LSB")
print("Min Val= " + str(ADC2_fit[1]) + " V [ADC=0 LSB]")
MAXVal = 4095 * ADC1_fit[0] + ADC2_fit[1]
print("Max Val= " + str(MAXVal) + " V [ADC=4095 LSB]")
print("R^2-Value= " + str(ADC2_fit[2] * ADC2_fit[2]))
print("std Error= " + str(ADC2_fit[4]))
print("RMS Noise = " + str(ADC2Residi * 1000) + " mV")
print("===ADC 3 ===")
print("Slope= " + str(ADC3_fit[0]) + " V/LSB")
print("Min Val= " + str(ADC3_fit[1]) + " V [ADC=0 LSB]")
MAXVal = 4095 * ADC3_fit[0] + ADC3_fit[1]
print("Max Val= " + str(MAXVal) + " V [ADC=4095 LSB]")
print("R^2-Value= " + str(ADC3_fit[2] * ADC3_fit[2]))
print("std Error= " + str(ADC3_fit[4]))
print("RMS Noise = " + str(ADC3Residi * 1000) + " mV")

# ===ADC 1 ===
# Slope= 0.00488040211169927 V/LSB
# Min Val= -10.029208660668372 V  [ADC=0 LSB]
# Max Val= 9.95603798674014 V [ADC=4095 LSB]
# R^2-Value= 0.9999990233862752
# Fit std Error= 2.3534388883844257e-08
# RMS Noise = 4.6824163159348675 mV
# ===ADC 2 ===
# Slope= 0.004864769104581888 V/LSB
# Min Val= -9.911472983085314 V  [ADC=0 LSB]
# Max Val= 10.073773664323198 V [ADC=4095 LSB]
# R^2-Value= 0.9999919259294022
# std Error= 6.74521740799918e-08
# RMS Noise = 13.68572038605262 mV
# ===ADC 3 ===
# Slope= 0.004884955868836948 V/LSB
# Min Val= -10.031544601902738 V  [ADC=0 LSB]
# Max Val= 9.972349680984568 V [ADC=4095 LSB]
# R^2-Value= 0.9999989979123382
# std Error= 2.386159197105857e-08
# RMS Noise = 4.721804326558252 mV
