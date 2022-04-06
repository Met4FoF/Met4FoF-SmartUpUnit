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
	//  =0 is needed to generate vtable for call with only virtual functions
  virtual int getData(DataMessage * Message,uint64_t RawTimeStamp)= 0; //data getter function handels sensor communication
  virtual int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE)= 0;// get the protobuff description
  virtual uint32_t getSampleCount()= 0;// get sample count
  virtual float getNominalSamplingFreq()= 0;// get nominal sampling freq
  virtual int setBaseID(uint32_t BaseID)= 0;// set base id
  virtual void increaseCaptureCountWORead()= 0;// increade capture count even if we dont capture the data use if MailAlloc fails
protected:
	bool _publish_time_ticks=false;
};


#endif /* MET4FOFSENSOR_H_ */
