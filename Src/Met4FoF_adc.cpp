/*
/*
 * Met4FoF_adc.cpp
 *
 *  Created on: 08.10.2018
 *      Author: seeger01
 */

#include <stdint.h>
#include <cstring>
#include <math.h>
#include "Met4FoF_adc.h"
#include "stm32f7xx_hal.h"
#include "adc.h"
#include "main.h"

Met4FoF_adc::Met4FoF_adc(ADC_HandleTypeDef * hadc1,ADC_HandleTypeDef * hadc2,ADC_HandleTypeDef* hadc3,uint32_t BaseID){
	  	  _hadc1=hadc1;
	  	  _hadc2=hadc2;
	  	  _hadc3=hadc3;
		_ID=_BaseID+(uint32_t)_SetingsID;
		_SetingsID=0;
}





int Met4FoF_adc::setBaseID(uint32_t BaseID)
{
	_BaseID=BaseID;
	_ID=_BaseID+(uint32_t)_SetingsID;
	return 0;
}



uint32_t Met4FoF_adc::getSampleCount(){
	return _SampleCount;
}

int Met4FoF_adc::getData(DataMessage * Message,uint64_t RawTimeStamp){
	memcpy(Message,&empty_DataMessage,sizeof(DataMessage));//Copy default values into array
	int result=0;
	_SampleCount++;
	Message->id=_ID;
	Message->unix_time=0XFFFFFFFF;
	Message->time_uncertainty=(uint32_t)((RawTimeStamp & 0xFFFFFFFF00000000) >> 32);//high word
	Message->unix_time_nsecs=(uint32_t)(RawTimeStamp & 0x00000000FFFFFFFF);// low word
	Message->sample_number=	_SampleCount;
	float adcVal1 = (float) HAL_ADC_GetValue(_hadc1);
	float adcVal2 = (float) HAL_ADC_GetValue(_hadc2);
	float adcVal3 = (float) HAL_ADC_GetValue(_hadc3);
	adcVal1=adcVal1*_Slopes[0]+_Offsets[0];
	adcVal2=adcVal2*_Slopes[1]+_Offsets[1];
	adcVal3=adcVal3*_Slopes[2]+_Offsets[2];
	Message->Data_01=adcVal1;
	Message->has_Data_02=true;
	Message->Data_02=adcVal2;
	Message->has_Data_03=true;
	Message->Data_03=adcVal3;

	return result;
}


int Met4FoF_adc::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	memcpy(Message,&empty_DescriptionMessage,sizeof(DescriptionMessage));//Copy default values into array
	int retVal=0;
	Message->id=_ID;
	strncpy(Message->Sensor_name,"STM32 Internal ADC\0",sizeof(Message->Sensor_name));
	Message->Description_Type=DESCRIPTION_TYPE;
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		strncpy(Message->str_Data_01,"Voltage Ch 1\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Voltage Ch 2\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"Voltage Ch 3\0",sizeof(Message->str_Data_03));
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_UNIT)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		strncpy(Message->str_Data_01,"\\volt\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"\\volt\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"\\volt\0",sizeof(Message->str_Data_03));
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->f_Data_01=4096;
		Message->f_Data_02=4096;
		Message->f_Data_03=4096;
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->f_Data_01=_Offsets[0];
		Message->f_Data_02=_Offsets[1];
		Message->f_Data_03=_Offsets[2];
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->has_f_Data_02=true;
		Message->has_f_Data_03=true;
		Message->f_Data_01=_Offsets[0]+_Slopes[0]*4096;
		Message->f_Data_02=_Offsets[1]+_Slopes[1]*4096;
		Message->f_Data_03=_Offsets[2]+_Slopes[2]*4096;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_HIERARCHY)
	{
		Message->has_str_Data_01=true;
		Message->has_str_Data_02=true;
		Message->has_str_Data_03=true;
		strncpy(Message->str_Data_01,"Voltage/0\0",sizeof(Message->str_Data_01));
		strncpy(Message->str_Data_02,"Voltage/1\0",sizeof(Message->str_Data_02));
		strncpy(Message->str_Data_03,"Voltage/2\0",sizeof(Message->str_Data_03));
	}
	return retVal;
}






