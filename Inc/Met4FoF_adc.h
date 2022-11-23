
#ifndef MET4FOF_ADC_H_
#define MET4FOF_ADC_H_

#include "adc.h"
#include "stm32f7xx_hal.h"

#include <stdint.h>
#include <cstring>
#include <math.h>

#include "pb.h"
#include "message.pb.h"
#include "Met4FoFSensor.h"

class Met4FoF_adc: public Met4FoFSensors::Met4FoFSensor
{
  public:
  Met4FoF_adc(ADC_HandleTypeDef * hadc1,ADC_HandleTypeDef * hadc2,ADC_HandleTypeDef* hadc3,uint32_t BaseID);
  int getData(DataMessage * Message,uint64_t RawTimeStamp);
  int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);
  void setSlopes(float Slope1, float Slope2,float Slope3){_Slopes[0]=Slope1;_Slopes[1]=Slope2;_Slopes[2]=Slope3;}
  void setOffsets(float Offset1, float Offset2,float Offset3){_Offsets[0]=Offset1;_Offsets[1]=Offset2;_Offsets[2]=Offset3;}
  void increaseCaptureCountWORead(){_SampleCount++;return ;};
  float getNominalSamplingFreq(){return 0.0;};
  private:

  ADC_HandleTypeDef * _hadc1;
  ADC_HandleTypeDef * _hadc2;
  ADC_HandleTypeDef * _hadc3;
  float _Slopes[3]={3.3/4096.0,3.3/4096.0,3.3/4096.0};
  float _Offsets[3]={0.0,0.0,0.0};
};

#endif /*MET4FOF_ADC_H_ */
