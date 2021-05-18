/*
/*
 * Met4FoFGPSPub.cpp
 *
 *  Created on: 08.10.2018
 *      Author: seeger01
 */


#include <Met4FoFGPSPub.hpp>


Met4FoFGPSPub::Met4FoFGPSPub(struct tref * GPS_ref,uint32_t BaseID){
		_ID=_BaseID+(uint32_t)_SetingsID;
		_SetingsID=0;
		_GPSTimeRef=GPS_ref;
}


int Met4FoFGPSPub::setBaseID(uint32_t BaseID)
{
	_BaseID=BaseID;
	_ID=_BaseID+(uint32_t)_SetingsID;
	return 0;
}


uint32_t Met4FoFGPSPub::getSampleCount(){
	return _SampleCount;
}

int Met4FoFGPSPub::getData(DataMessage * Message,uint64_t RawTimeStamp){
	lgw_gps_get(&_utc_time, &_gps_time, &_coords, NULL);
	memcpy(Message,&empty_DataMessage,sizeof(DataMessage));//Copy default values into array
	int result=0;
	_SampleCount++;
	Message->id=_ID;
	Message->unix_time=0XFFFFFFFF;
	Message->time_uncertainty=(uint32_t)((RawTimeStamp & 0xFFFFFFFF00000000) >> 32);//high word
	Message->unix_time_nsecs=(uint32_t)(RawTimeStamp & 0x00000000FFFFFFFF);// low word
	Message->sample_number=	_SampleCount;
	Message->Data_01=_utc_time.tv_sec;
	Message->Data_02=_utc_time.tv_nsec;
	Message->Data_03= _coords.lat;
	Message->Data_04= _coords.lon;
	Message->Data_05= _coords.alt;
	return result;
}

int Met4FoFGPSPub::getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE){
	memcpy(Message,&empty_DescriptionMessage,sizeof(DescriptionMessage));//Copy default values into array
	int retVal=0;
	Message->id=_ID;
	strncpy(Message->Sensor_name,"STM32 GPIO Input Edge\0",sizeof(Message->Sensor_name));
	Message->Description_Type=DESCRIPTION_TYPE;
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY)
	{
		Message->has_str_Data_01=true;
		strncpy(Message->str_Data_01,"Time seconds\0",sizeof(Message->str_Data_01));
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_02,"Time Secs\0",sizeof(Message->str_Data_01));
		Message->has_str_Data_03=true;
		strncpy(Message->str_Data_03,"Latitude\0",sizeof(Message->str_Data_01));
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_04,"Longitude\0",sizeof(Message->str_Data_01));
		Message->has_str_Data_05=true;
		strncpy(Message->str_Data_05,"Altitude\0",sizeof(Message->str_Data_01));

	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_UNIT)
	{
		Message->has_str_Data_01=true;
		strncpy(Message->str_Data_01,"\\sceconds\0",sizeof(Message->str_Data_01));
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_02,"\\nanoseconds\0",sizeof(Message->str_Data_02));
		Message->has_str_Data_03=true;
		strncpy(Message->str_Data_03,"\\degree\0",sizeof(Message->str_Data_03));
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_04,"\\degree\0",sizeof(Message->str_Data_04));
		Message->has_str_Data_05=true;
		strncpy(Message->str_Data_05,"\\metre\0",sizeof(Message->str_Data_05));
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION)
	{
		Message->has_f_Data_01=true;
		Message->f_Data_01=4294967296;
		Message->has_f_Data_02=true;
		Message->f_Data_02=1e9;
		Message->has_f_Data_03=true;
		Message->f_Data_03=NAN;
		Message->has_f_Data_04=true;
		Message->f_Data_04=NAN;
		Message->has_f_Data_05=true;
		Message->f_Data_05=NAN;

	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->f_Data_01=0;
		Message->has_f_Data_02=true;
		Message->f_Data_02=0;
		Message->has_f_Data_03=true;
		Message->f_Data_03=NAN;
		Message->has_f_Data_04=true;
		Message->f_Data_04=NAN;
		Message->has_f_Data_05=true;
		Message->f_Data_05=NAN;
	}
	else if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE)
	{
		Message->has_f_Data_01=true;
		Message->f_Data_01=4294967296;
		Message->has_f_Data_02=true;
		Message->f_Data_02=1e9;
		Message->has_f_Data_03=true;
		Message->f_Data_03=NAN;
		Message->has_f_Data_04=true;
		Message->f_Data_04=NAN;
		Message->has_f_Data_05=true;
		Message->f_Data_05=NAN;
	}
	if(DESCRIPTION_TYPE==DescriptionMessage_DESCRIPTION_TYPE_HIERARCHY)
	{
		Message->has_str_Data_01=true;
		strncpy(Message->str_Data_01,"Time/0\0",sizeof(Message->str_Data_01));
		Message->has_str_Data_02=true;
		strncpy(Message->str_Data_02,"Time/1\0",sizeof(Message->str_Data_02));
		Message->has_str_Data_03=true;
		strncpy(Message->str_Data_03,"Coordinates/0\0",sizeof(Message->str_Data_03));
		Message->has_str_Data_04=true;
		strncpy(Message->str_Data_04,"Coordinates/1\0",sizeof(Message->str_Data_04));
		Message->has_str_Data_05=true;
		strncpy(Message->str_Data_05,"Coordinates/2\0",sizeof(Message->str_Data_05));
	}
	return retVal;
}






