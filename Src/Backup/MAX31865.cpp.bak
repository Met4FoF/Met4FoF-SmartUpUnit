/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865
  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328
  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "MAX31865.h"

#include <stdlib.h>


/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI
    @param spi_cs the SPI CS pin to use
    @param spi_mosi the SPI MOSI pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_clk the SPI clock pin to use
*/
/**************************************************************************/
//
MAX31865::MAX31865(GPIO_TypeDef* SPICSTypeDef, uint16_t SPICSPin,SPI_HandleTypeDef* MAX31865spi) {
	  _SPICSTypeDef=SPICSTypeDef;
	  _SPICSPin=SPICSPin;
	  _MAX31865spi=MAX31865spi;
}

/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
bool MAX31865::begin(max31865_numwires_t wires) {
  setWires(wires);
  enableBias(false);
  autoConvert(true);
  clearFault();

  // Serial.print("config: ");
  // Serial.println(readRegister8(MAX31856_CONFIG_REG), HEX);
  return true;
}

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t MAX31865::readFault(void) {
  return readRegister8(MAX31856_FAULTSTAT_REG);
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void MAX31865::clearFault(void) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void MAX31865::enableBias(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31856_CONFIG_BIAS; // disable bias
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void MAX31865::autoConvert(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31856_CONFIG_MODEAUTO; // disable autoconvert
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want filter out 50Hz noise or 60Hz noise
    @param b If true, 50Hz noise is filtered, else 60Hz(default)
*/
/**************************************************************************/

void MAX31865::enable50Hz(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_FILT50HZ;
  } else {
    t &= ~MAX31856_CONFIG_FILT50HZ;
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void MAX31865::setWires(max31865_numwires_t wires) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31856_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31856_CONFIG_3WIRE;
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float MAX31865::temperature(float RTDnominal, float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = readRTD();
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in continous mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t MAX31865::readRTD(void) {
	/* not need since we use cont conversion
  clearFault();
  enableBias(true);
  osDelay(10);
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_1SHOT;
  writeRegister8(MAX31856_CONFIG_REG, t);
  osDelay(65);
  */

  uint16_t rtd = readRegister16(MAX31856_RTDMSB_REG);

  // remove fault
  rtd >>= 1;

  return rtd;
}

/**********************************************/

uint8_t MAX31865::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t MAX31865::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

bool MAX31865::readRegisterN(uint8_t addr, uint8_t buffer[],
                                      uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set
	bool retVal=false;
	uint8_t tx[n]={0};
	uint8_t rx[n]={0};
	tx[0] = addr;
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive(_MAX31865spi,tx, rx, n+1, SPI_TIMEOUT)==HAL_OK)
	{
		retVal=true;
	}
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
	memcpy(buffer, &rx[1], n);
	return retVal;

}

bool MAX31865::writeRegister8(uint8_t addr, uint8_t data) {
	bool retVal=false;
  addr |= 0x80; // make sure top bit is set

  uint8_t tx[2] = {addr, data};
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(_MAX31865spi,tx, 2, SPI_TIMEOUT)==HAL_OK)
	{
		retVal=true;
	}
	HAL_GPIO_WritePin(_SPICSTypeDef, _SPICSPin, GPIO_PIN_SET);
	return retVal;
}
