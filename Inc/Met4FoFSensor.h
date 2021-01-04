/*
 * Met4FoFSensor.h
 *
 *  Created on: 15.09.2020
 *      Author: seeger01
 */

#ifndef MET4FOFSENSOR_H_
#define MET4FOFSENSOR_H_


class Met4FoFSensor {
public:
  virtual int getData(DataMessage * Message,uint64_t RawTimeStamp)=0; //data getter function handels sensor communication
  virtual int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE)=0;// get the protobuff description
  virtual uint32_t getSampleCount()=0;// get sample count
  virtual float getNominalSamplingFreq()=0;// get nominal sampling freq
  virtual int setBaseID(uint32_t BaseID)=0;// set base id
};


#endif /* MET4FOFSENSOR_H_ */
