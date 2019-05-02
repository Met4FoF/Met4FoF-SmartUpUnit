# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
i=0
ISRNames=""
with open(r"ISR_names_from_startup_stm32f767xx.s.txt", "r") as ins:
    for line in ins:
        print("SEGGER_SYSVIEW_SendSysDesc(\"I#"+str(i)+"="+str(line.replace('\n',''))+"\");")
        i=i+1