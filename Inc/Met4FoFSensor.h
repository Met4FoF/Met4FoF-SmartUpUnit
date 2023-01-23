/*
 * Met4FoFSensor.h
 *
 *  Created on: 15.09.2020
 *      Author: seeger01
 */

#ifndef MET4FOFSENSOR_H_
#define MET4FOFSENSOR_H_

#include <list>
#include "pb.h"
#include "message.pb.h"

namespace Met4FoFSensors{
class Met4FoFSensor {
public:
	Met4FoFSensor(uint8_t baseID){_baseID=baseID;_ID=baseID;};
	//  =0 is needed to generate vtable for linking with only virtual functions
  virtual int getData(DataMessage * Message,uint64_t RawTimeStamp)= 0; //data getter function handels sensor communication
  virtual int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE)= 0;// get the protobuff description
  float getNominalSamplingFreq(){return _NominalSamplingFreq;}
  void setBaseID(uint32_t BaseID){_baseID=BaseID;_ID=_baseID+(uint32_t)_SetingsID;};
  uint32_t getSampleCount(){return _SampleCount;};
  float _NominalSamplingFreq=NAN;
  uint8_t _baseID=0;
  uint32_t _ID=0;
  uint16_t _SetingsID=0;
  uint32_t _SampleCount=0;
protected:

  bool _publish_time_ticks=false;
};

static std::list<Met4FoFSensor *> listMet4FoFSensors;
}




#endif /* MET4FOFSENSOR_H_ */
