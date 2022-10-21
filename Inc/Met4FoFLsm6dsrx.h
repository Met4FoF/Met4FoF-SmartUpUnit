/*
 * lsm6dsrx.h
 *
 *  Created on: 21.10.2022
 *      Author: seeger01
 */

#ifndef MET4FOFLSM6DSRX_H_
#define MET4FOFLSM6DSRX_H_


#include "stm32f7xx_hal.h"

#include <stdint.h>
#include <cstring>
#include <math.h>

#include "pb.h"
#include "message.pb.h"
#include "Met4FoFSensor.h"

#include "lsm6dsrx_reg.h"

class Met4FoFLsm6dsrx: public Met4FoFSensor
{
  public:
  Met4FoFLsm6dsrx(GPIO_TypeDef* SPICSTypeDef, uint16_t SPICSPin,SPI_HandleTypeDef* spiIfaceHandle,uint32_t BaseID);
  int getData(DataMessage * Message,uint64_t RawTimeStamp);
  int getDescription(DescriptionMessage * Message,DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE);
  uint32_t getSampleCount();
  void increaseCaptureCountWORead(){_SampleCount++;return ;};
  int setBaseID(uint32_t BaseID);
  float getNominalSamplingFreq(){return 0.0;};
  int setUp();
  private:
  int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,uint16_t len);
  int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);
  GPIO_TypeDef* _SPICSPort;
  uint16_t _SPICSPin;
  SPI_HandleTypeDef* _spi;
  uint32_t _ID;
  uint32_t _BaseID;
  uint16_t _SetingsID;
  uint32_t _SampleCount=0;
  stmdev_ctx_t _dev_ctx;
  uint8_t _whoamI;

};



#endif /* MET4FOFLSM6DSRX_H_ */
