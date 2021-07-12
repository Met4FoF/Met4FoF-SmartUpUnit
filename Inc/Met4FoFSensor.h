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
  virtual int getData(DataMessage * Message,uint64_t RawTimeStamp); //data getter function handels sensor communication
  virtual int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);// get the protobuff description
  virtual uint32_t getSampleCount();// get sample count
  virtual float getNominalSamplingFreq();// get nominal sampling freq
  virtual int setBaseID(uint32_t BaseID);// set base id
  virtual void increaseCaptureCountWORead();// increade capture count even if we dont capture the data use if MailAlloc fails
  //virtual ~Met4FoFSensor()=0; // Pure virtual destructor
protected:
	bool _publish_time_ticks=false;
};


#endif /* MET4FOFSENSOR_H_ */
