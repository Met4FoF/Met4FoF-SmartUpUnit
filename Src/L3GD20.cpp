/*
 * L3GD20.h
 *
 *  Created on: 05.02.2019
 *      Author: seeger01
 *
 *BASED ON:
/***************************************************
  This is a library for the L3GD20 GYROSCOPE

  Designed specifically to work with the Adafruit L3GD20 Breakout
  ----> https://www.adafruit.com/products/1032

  These sensors use I2C or SPI to communicate, 2 pins (I2C)
  or 4 pins (SPI) are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "L3GD20.h"


#define USE_MULTIBYTE_READ 1

  L3GD20::L3GD20(GPIO_TypeDef* SPICSTypeDef,
		uint16_t SPICSPin,
		SPI_HandleTypeDef* L3GD20spi):_SPICSTypeDef(SPICSTypeDef),_SPICSPin(SPICSPin),_L3GD20spi(L3GD20spi) {
;
_gyrorange=DEFAULT_GYRORANGE ;
}

  bool L3GD20::init(gyroRange_t gyrorange,gyroUpdateFreq_t gyroUpdateFreq){
	  _gyrorange=gyrorange;
	  _gyroUpdateFreq=gyroUpdateFreq;
	  /* Make sure we have the correct chip ID since this checks
	     for correct address and that the IC is properly connected */
	  uint8_t id = readByte(GYRO_REGISTER_WHO_AM_I);
	  //Serial.println(id, HEX);
	  if ((id != L3GD20_ID) && (id != L3GD20H_ID))
	  {
	    return false;
	  }
	  /* Set CTRL_REG1 (0x20)
	     ====================================================================
	     BIT  Symbol    Description                                   Default
	     ---  ------    --------------------------------------------- -------
	     7-6  DR1/0     Output data rate                                   00
	     5-4  BW1/0     Bandwidth selection                                00
	       3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
	       2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
	       1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
	       0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

	    /* Reset then switch to normal mode and enable all three channels */
	  writeByte(GYRO_REGISTER_CTRL_REG1, 0x00);
	  writeByte(GYRO_REGISTER_CTRL_REG1, 0x0F|_gyroUpdateFreq<<4);
	  if(_gyroUpdateFreq==GYRO_UPDATE_50_HZ_CUTOFF_16_6||_gyroUpdateFreq==GYRO_UPDATAE_25_HZ||_gyroUpdateFreq==GYRO_UPDATAE_25_HZ)
	  {
		  writeByte(GYRO_REGISTER_LOW_ODR,0x01);
	  }
	  else
	  {
		  writeByte(GYRO_REGISTER_LOW_ODR,0x00);
	  }
	    /* ------------------------------------------------------------------ */

	    /* Set CTRL_REG2 (0x21)
	     ====================================================================
	     BIT  Symbol    Description                                   Default
	     ---  ------    --------------------------------------------- -------
	     5-4  HPM1/0    High-pass filter mode selection                    00
	     3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

	    /* Nothing to do ... keep default values */
	    /* ------------------------------------------------------------------ */

	    /* Set CTRL_REG3 (0x22)
	     ====================================================================
	     BIT  Symbol    Description                                   Default
	     ---  ------    --------------------------------------------- -------
	       7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
	       6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
	       5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
	       4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
	       3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
	       2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
	       1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
	       0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

	  writeByte(GYRO_REGISTER_CTRL_REG3, 0x88);
	    /* ------------------------------------------------------------------ */

	    /* Set CTRL_REG4 (0x23)
	     ====================================================================
	     BIT  Symbol    Description                                   Default
	     ---  ------    --------------------------------------------- -------
	       7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
	       6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
	     5-4  FS1/0     Full scale selection                               00
	                                    00 = 250 dps
	                                    01 = 500 dps
	                                    10 = 2000 dps
	                                    11 = 2000 dps
	       0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

	    /* Adjust resolution if requested */
	    switch(_gyrorange)
	    {
	      case GYRO_RANGE_250DPS:
	    	  writeByte(GYRO_REGISTER_CTRL_REG4, 0x00);
	        break;
	      case GYRO_RANGE_500DPS:
	    	  writeByte(GYRO_REGISTER_CTRL_REG4, 0x10);
	        break;
	      case GYRO_RANGE_2000DPS:
	    	  writeByte(GYRO_REGISTER_CTRL_REG4, 0x20);
	        break;
	    }
	    /* ------------------------------------------------------------------ */

	    /* Set CTRL_REG5 (0x24)
	     ====================================================================
	     BIT  Symbol    Description                                   Default
	     ---  ------    --------------------------------------------- -------
	       7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
	       6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
	       4  HPen      High-pass filter enable (0=disable,1=enable)        0
	     3-2  INT1_SEL  INT1 Selection config                              00
	     1-0  OUT_SEL   Out selection config                               00 */

	    /* Nothing to do ... keep default values */
	    /* ------------------------------------------------------------------ */

	    return true;
	  }

  GyroData L3GD20::GetData()
  {
	  GyroData data={float NAN, float NAN ,float NAN ,float NAN };
	  int16_t tmp_x,tmp_y,tmp_z=0;
	  int8_t tmp_tmperature=0;
	  /*
      GYRO_REGISTER_OUT_TEMP            = 0x26,   //            r
      GYRO_REGISTER_STATUS_REG          = 0x27,   //            r

      GYRO_REGISTER_OUT_X_L             = 0x28,   //            r
      GYRO_REGISTER_OUT_X_H             = 0x29,   //            r

      GYRO_REGISTER_OUT_Y_L             = 0x2A,   //            r
      GYRO_REGISTER_OUT_Y_H             = 0x2B,   //            r

      GYRO_REGISTER_OUT_Z_L             = 0x2C,   //            r
      GYRO_REGISTER_OUT_Z_H             = 0x2D,   //            r
      */

	#if  USE_MULTIBYTE_READ == 1
    uint8_t Raw_data[8];
    readBytes(GYRO_REGISTER_OUT_TEMP,8,Raw_data);
    // Shift values to create properly formed integer (low byte first)
    tmp_x = (int16_t)(Raw_data[2] | (Raw_data[3] << 8));
    tmp_y = (int16_t)(Raw_data[4] | (Raw_data[5] << 8));
    tmp_z = (int16_t)(Raw_data[6] | (Raw_data[7] << 8));
    tmp_tmperature=(int8_t)(Raw_data[0]);
	#else
    uint8_t raw[2]={0};
    tmp_tmperature=(int8_t)readByte(GYRO_REGISTER_OUT_TEMP);
    readByte(GYRO_REGISTER_STATUS_REG);
    raw[0]=readByte(GYRO_REGISTER_OUT_X_L);
    raw[1]=readByte(GYRO_REGISTER_OUT_X_H);
    tmp_x = (int16_t)(raw[0] | (raw[1] << 8));
    raw[0]=readByte(GYRO_REGISTER_OUT_Y_L);
    raw[1]=readByte(GYRO_REGISTER_OUT_Y_H);
    tmp_y = (int16_t)(raw[0] | (raw[1] << 8));
    raw[0]=readByte(GYRO_REGISTER_OUT_Z_L);
    raw[1]=readByte(GYRO_REGISTER_OUT_Z_H);
    tmp_z = (int16_t)(raw[0] | (raw[1] << 8));
	#endif

    // Compensate values depending on the resolution
    switch(_gyrorange)
    {
      case GYRO_RANGE_250DPS:
        data.x = GYRO_SENSITIVITY_250DPS*(float)tmp_x;
        data.y = GYRO_SENSITIVITY_250DPS*(float)tmp_y;
        data.z = GYRO_SENSITIVITY_250DPS*(float)tmp_z;
        data.temperature=-tmp_tmperature;
        break;
      case GYRO_RANGE_500DPS:
        data.x = GYRO_SENSITIVITY_500DPS*(float)tmp_x;
        data.y = GYRO_SENSITIVITY_500DPS*(float)tmp_y;
        data.z = GYRO_SENSITIVITY_500DPS*(float)tmp_z;
        data.temperature=-tmp_tmperature;
        break;
      case GYRO_RANGE_2000DPS:
        data.x = GYRO_SENSITIVITY_2000DPS*(float)tmp_x;
        data.y = GYRO_SENSITIVITY_2000DPS*(float)tmp_y;
        data.z = GYRO_SENSITIVITY_2000DPS*(float)tmp_z;
        data.temperature=-tmp_tmperature;
        break;
    }
    return data;
  }

  GyroDataStamped  L3GD20::GetStampedData(uint32_t UnixSecs,uint64_t RawTimerCount,uint32_t CaptureCount,uint16_t ADCVal){
	GyroDataStamped returnVal{0,0,0,0};
  	returnVal.Data=L3GD20::GetData();
  	returnVal.UnixSecs=UnixSecs;
  	returnVal.RawTimerCount=RawTimerCount;
  	returnVal.CaptureCount=CaptureCount;
  	returnVal.ADCValue=ADCVal;
  	return returnVal;
  }

  void L3GD20::writeByte(uint8_t subAddress, uint8_t data) {
  	uint8_t buffer[2] = { L3GD20_SPI_WRITE | subAddress, data };
  	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
  	HAL_SPI_Transmit(_L3GD20spi, buffer, 2, SPI_TIMEOUT);
  	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
  }

  uint8_t L3GD20::readByte(uint8_t subAddress) {
  	uint8_t tx[2] = {(L3GD20_SPI_READ | subAddress),0};
  	uint8_t rx[2] = {0,0};

  	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
  	HAL_SPI_TransmitReceive(_L3GD20spi,tx, rx, 2, SPI_TIMEOUT);
  	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);

  	return rx[1];
  }

  bool L3GD20::readBytes(uint8_t subAddress,uint8_t count, uint8_t* dest) {
  	bool retVal=false;
  	uint8_t tx[count+1]={0};
  	uint8_t rx[count+1]={0};
  	tx[0] = {L3GD20_SPI_READ |L3GD20_SPI_MULTIREADINCREMENT| subAddress};
  	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
  	if(HAL_SPI_TransmitReceive(_L3GD20spi,tx, rx, count+1, SPI_TIMEOUT)==HAL_OK)
  	{
  		retVal=true;
  	}
  	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
  	memcpy(dest, &rx[1], count);
  	return retVal;
  }
