/*
/*
 * bma280.cpp
 *
 *  Created on: 08.10.2018
 *      Author: seeger01
 */


/*
 * BMA280.cpp
 *
 *
 *  Created on: 01.08.2018
 *      Author: seeger01
 *
 *
 * 06/16/2017 Copyright Tlera Corporation
 *
 *  Created by Kris Winer
 *
 *  The BMA280 is an inexpensive (~$1), three-axis, high-resolution (14-bit) acclerometer in a tiny 2 mm x 2 mm LGA12 package with 32-slot FIFO,
 *  two multifunction interrupts and widely configurable sample rate (15 - 2000 Hz), full range (2 - 16 g), low power modes,
 *  and interrupt detection behaviors. This accelerometer is nice choice for low-frequency sound and vibration analysis,
 *  tap detection and simple orientation estimation.
 *
 *  Library may be used freely and without limit with attribution.
 *
 */
#include "dummy_sensor.h"

DummySensor::DummySensor(uint16_t BaseID){
	  _BaseID=BaseID;
	  _ID=(uint32_t)_BaseID<<16;
}

int DummySensor::setBaseID(uint16_t BaseID)
{
	_BaseID=BaseID;
	_ID=(uint32_t)_BaseID<<16;
	return 0;
}


int DummySensor::getData(DataMessage * Message,uint64_t RawTimeStamp,uint32_t CaptureCount){
	int result=0;
	Message->id=_ID;
	Message->unix_time=0XFFFFFFFF;
	Message->time_uncertainty=(uint32_t)((RawTimeStamp & 0xFFFFFFFF00000000) >> 32);//high word
	Message->unix_time_nsecs=(uint32_t)(RawTimeStamp & 0x00000000FFFFFFFF);// low word
	Message->sample_number=CaptureCount;
	//Oh, that code hurts your eyes.
	//This code should transfer the 64 bit timestamp of the GPS timing unit to the Python software as easy as possible without data loss and without protobuff protocol extension.
	//This is the reason why we convert the 64 bit timestamp into 4 16 bit fragments.
	//A typecast from uint16_t to 32 bit IEEE 754 float is possible without any loss because the mantissa of the float is 22 bit long.
	uint16_t w1=(uint16_t)((RawTimeStamp & 0xFFFF000000000000) >> 48);
	uint16_t w2=(uint16_t)((RawTimeStamp & 0x0000FFFF00000000) >> 32);
	uint16_t w3=(uint16_t)((RawTimeStamp & 0x00000000FFFF0000) >> 16);
	uint16_t w4=(uint16_t)((RawTimeStamp & 0x000000000000FFFF));
	Message->Data_01=float(w1);
	Message->has_Data_02=true;
	Message->Data_02=float(w2);
	Message->has_Data_03=true;
	Message->Data_03=float(w3);
	Message->has_Data_04=true;
	Message->Data_04=float(w4);
	result=1;
	return result;
}

int DummySensor::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	int retVal=0;
	Message->id=_ID;
	strncpy(Message->Sensor_name,"GPS TimeStamping Dummy Sensor\0",sizeof(Message->Sensor_name));
	Message->Description_Type=DESCRIPTION_TYPE;
	Message->has_str_Data_01=false;
	Message->has_str_Data_02=false;
	Message->has_str_Data_03=false;
	Message->has_str_Data_04=false;
	Message->has_str_Data_05=false;
	Message->has_str_Data_06=false;
	Message->has_str_Data_07=false;
	Message->has_str_Data_08=false;
	Message->has_str_Data_09=false;
	Message->has_str_Data_10=false;
	Message->has_str_Data_11=false;
	Message->has_str_Data_12=false;
	Message->has_str_Data_13=false;
	Message->has_str_Data_14=false;
	Message->has_str_Data_15=false;
	Message->has_str_Data_16=false;
	Message->has_f_Data_01=false;
	Message->has_f_Data_02=false;
	Message->has_f_Data_03=false;
	Message->has_f_Data_04=false;
	Message->has_f_Data_05=false;
	Message->has_f_Data_06=false;
	Message->has_f_Data_07=false;
	Message->has_f_Data_08=false;
	Message->has_f_Data_09=false;
	Message->has_f_Data_10=false;
	Message->has_f_Data_11=false;
	Message->has_f_Data_12=false;
	Message->has_f_Data_13=false;
	Message->has_f_Data_14=false;
	Message->has_f_Data_15=false;
	Message->has_f_Data_16=false;
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_01,"Bit [63:49] of GPS Timstamp Raw Val as float32\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Bit [48:32] of GPS Timstamp Raw Val as float32\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"Bit [31:16] of GPS Timstamp Raw Val as float32\0",sizeof(Message->str_Data_03));
		strncpy(Message->str_Data_04,"Bit [15:0]  of GPS Timstamp Raw Val as float32\0",sizeof(Message->str_Data_04));
	}
	else
	{
		Message->has_str_Data_01=false;
		Message->has_str_Data_02=false;
		Message->has_str_Data_03=false;
		Message->has_str_Data_04=false;
		Message->has_str_Data_05=false;
		Message->has_str_Data_06=false;
		Message->has_str_Data_07=false;
		Message->has_str_Data_08=false;
		Message->has_str_Data_09=false;
		Message->has_str_Data_10=false;
		Message->has_str_Data_11=false;
		Message->has_str_Data_12=false;
		Message->has_str_Data_13=false;
		Message->has_str_Data_14=false;
		Message->has_str_Data_15=false;
		Message->has_str_Data_16=false;
		Message->has_f_Data_01=false;
		Message->has_f_Data_02=false;
		Message->has_f_Data_03=false;
		Message->has_f_Data_04=false;
		Message->has_f_Data_05=false;
		Message->has_f_Data_06=false;
		Message->has_f_Data_07=false;
		Message->has_f_Data_08=false;
		Message->has_f_Data_09=false;
		Message->has_f_Data_10=false;
		Message->has_f_Data_11=false;
		Message->has_f_Data_12=false;
		Message->has_f_Data_13=false;
		Message->has_f_Data_14=false;
		Message->has_f_Data_15=false;
		Message->has_f_Data_16=false;


	}
	return retVal;
}



