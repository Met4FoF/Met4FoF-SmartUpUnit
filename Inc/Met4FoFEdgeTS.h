
#ifndef MET4FOFEDGETS_H_
#define MET4FOFEDGETS_H_

#include "adc.h"
#include "stm32f7xx_hal.h"

#include <stdint.h>
#include <cstring>
#include <math.h>

#include "pb.h"
#include "message.pb.h"
#include "Met4FoFSensor.h"

class Met4FoFEdgeTS: public Met4FoFSensor
{
  public:
  Met4FoFEdgeTS(float EdgeDirection,uint32_t BaseID);
  int getData(DataMessage * Message,uint64_t RawTimeStamp);
  int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);
  uint32_t getSampleCount();
  void increaseCaptureCountWORead(){_SampleCount++;return ;};
  int setBaseID(uint32_t BaseID);
  void setEdgeDirection(float EdgeDirection);
  float getNominalSamplingFreq(){return 0.0;};
  private:
  uint32_t _ID;
  uint32_t _BaseID;
  uint16_t _SetingsID;
  uint32_t _SampleCount=0;
  float  _EdgeDirection=1.0f;
};

#endif /*MET4FOF_ADC_H_ */
