/*
 * WS2812B_spi.h
 *
 *  Created on: 20.05.2021
 *      Author: benes
 */

#ifndef WS2812B_SPI_H_
#define WS2812B_SPI_H_


#include "spi.h"
#include "stm32f7xx_hal.h"


class WS2812B
{
  public:
  const uint32_t mask=0b000000000100100100100100100100100;
  WS2812B(SPI_HandleTypeDef* spi,uint16_t numLeds);
  void calculateData();
  void writeData();
  void setLed(uint16_t led,uint8_t r,uint8_t g,uint8_t b);
  uint8_t * ledData=NULL;
  private:
  uint16_t _numLeds;
  uint8_t * outputData=NULL;
  SPI_HandleTypeDef* _spi;
  bool bufferAlloced=false;


};

#endif /* WS2812B_SPI_H_ */
