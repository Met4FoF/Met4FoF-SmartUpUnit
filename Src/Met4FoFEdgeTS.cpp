/*
/*
 * Met4FoFEdgeTS.cpp
 *
 *  Created on: 08.10.2018
 *      Author: seeger01
 */

#include <stdint.h>
#include <cstring>
#include <math.h>
#include "Met4FoFEdgeTS.h"
#include "stm32f7xx_hal.h"
#include "adc.h"
#include "main.h"

Met4FoFEdgeTS::Met4FoFEdgeTS(float EdgeDirection,uint32_t BaseID){
		_BaseID=BaseID;
		_ID=_BaseID+(uint32_t)_SetingsID;
		_SetingsID=0;
		_EdgeDirection=EdgeDirection;
		_publish_time_ticks=true;
}





int Met4FoFEdgeTS::setBaseID(uint32_t BaseID)
{
	_BaseID=BaseID;
	_ID=_BaseID+(uint32_t)_SetingsID;
	return 0;
}

void Met4FoFEdgeTS::setEdgeDirection(float EdgeDirection)
{
	_EdgeDirection=EdgeDirection;
}


uint32_t Met4FoFEdgeTS::getSampleCount(){
	return _SampleCount;
}

int Met4FoFEdgeTS::getData(DataMessage * Message,uint64_t RawTimeStamp){
	int result=0;
	_SampleCount++;
	if (Message==0){
		return result;
	}
	memcpy(Message,&empty_DataMessage,sizeof(DataMessage));//Copy default values into array
	Message->id=_ID;
	Message->unix_time=0XFFFFFFFF;
	Message->time_uncertainty=(uint32_t)((RawTimeStamp & 0xFFFFFFFF00000000) >> 32);//high word
	Message->unix_time_nsecs=(uint32_t)(RawTimeStamp & 0x00000000FFFFFFFF);// low word
	Message->sample_number=	_SampleCount;
	Message->Data_01=_EdgeDirection;
	if(_publish_time_ticks==true){
	Message->has_time_ticks=true;
	Message->time_ticks=RawTimeStamp;
	}
	return result;
}

int Met4FoFEdgeTS::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	memcpy(Message,&empty_DescriptionMessage,sizeof(DescriptionMessage));//Copy default values into array
	int retVal=0;
	Message->id=_ID;
	strncpy(Message->Sensor_name,"STM32 GPIO Input\0",sizeof(Message->Sensor_name));
	Message->Description_Type=DESCRIPTION_TYPE;
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY)
	{
		Message->has_str_Data_01=true;
		strncpy(Message->str_Data_01,"Edge Direction\0",sizeof(Message->str_Data_01));
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_UNIT)
	{
		Message->has_str_Data_01=true;
		strncpy(Message->str_Data_01,"\\boolean\0",sizeof(Message->str_Data_01));
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION)
	{
		Message->has_f_Data_01=true;
		Message->f_Data_01=2;
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->f_Data_01=0.0;
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->f_Data_01=1.0;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_HIERARCHY)
	{
		Message->has_str_Data_01=true;
		strncpy(Message->str_Data_01,"Edge/0\0",sizeof(Message->str_Data_01));
	}
	if(_publish_time_ticks==true){
	Message->has_time_ticks=true;
	}
	return retVal;
}






