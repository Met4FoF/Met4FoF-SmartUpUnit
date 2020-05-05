/*
 * dummy_sensor.h
 *
 *  Created on: Oct 14, 2019
 *      Author: seeger01
 */

#ifndef DUMMY_SENSOR_H_
#define DUMMY_SENSOR_H_
#include <math.h>

#include "pb.h"
#include "message.pb.h"

class DummySensor
{
  public:
  DummySensor(uint16_t BaseID);
  int getData(DataMessage * Message,uint64_t RawTimeStamp,uint32_t CaptureCount);
  int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);
  int setBaseID(uint16_t BaseID);
  uint32_t _ID;
  uint16_t _BaseID;
};


#endif /* DUMMY_SENSOR_H_ */
