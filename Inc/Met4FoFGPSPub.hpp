
#ifndef MET4FOFGPSPUB_HPP_
#define MET4FOFGPSPUB_HPP_

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "message.pb.h"
#include "Met4FoFSensor.hpp"

#include "NMEAPraser.h"

class Met4FoFGPSPub: public Met4FoFSensor{
  public:
  Met4FoFGPSPub(struct tref * GPS_ref,uint32_t BaseID);
  int getData(DataMessage * Message,uint64_t RawTimeStamp);
  int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);
  uint32_t getSampleCount();
  int setBaseID(uint32_t BaseID);
  float getNominalSamplingFreq(){return 0.0;};
  private:
  uint32_t _ID;
  uint32_t _BaseID;
  uint16_t _SetingsID;
  uint32_t _SampleCount=0;
  struct tref * _GPSTimeRef;
  struct timespec _utc_time;
  struct timespec _gps_time;
  struct coord_s _coords;

};

#endif /*MET4FOF_ADC_H_ */
