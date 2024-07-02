/***************************************************************************************************/
/*
   This is a C library for Aosong ASAIR AHT10/AHT15/AHT20/AHT21/AHT25/AM2301B/AM2311B
   Digital Humidity & Temperature Sensor

   written by : enjoyneering
   sourse code: https://github.com/enjoyneering/

   Aosong ASAIR AHT1x/AHT2x features:
   - AHT1x +1.8v..+3.6v, AHT2x +2.2v..+5.5v
   - AHT1x 0.25uA..320uA, AHT2x 0.25uA..980uA
   - temperature range -40C..+85C
   - humidity range 0%..100%
   - typical accuracy T +-0.3C, RH +-2%
   - typical resolution T 0.01C, RH 0.024%
   - normal operating range T -20C..+60C, RH 10%..80%
   - maximum operating rage T -40C..+80C, RH 0%..100%
   - response time 8..30sec*
   - I2C bus speed 100KHz..400KHz, 10KHz recommended minimum
     *measurement with high frequency leads to heating of the
      sensor, interval must be > 1 second to keep self-heating below 0.1C

   This device uses I2C bus to communicate, specials pins are required to interface


   GNU GPL license, all text above must be included in any redistribution,
   see link for details - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/


#include "AHTxx.h"


struct aht_st {
   AHTXX_I2C_SENSOR sensorType;
   uint8_t          address;
   uint8_t          status;
   uint8_t          rawData[7]; // = {0, 0, 0, 0, 0, 0, 0}; //{status, RH, RH, RH+T, T, T, CRC}, CRC for AHT2x only
} aht;


extern void     pause_ms(uint16_t ms);
extern uint32_t millis(void);
extern void     i2c_AHT_flush(void);
extern uint16_t i2c_AHT_available(void);
extern uint16_t i2c_AHT_read(uint8_t addr, uint8_t* buf);
extern uint16_t i2c_AHT_write(uint8_t addr, uint8_t* buf, uint16_t len);


static bool     softReset();
static void     aht_readMeasurement();
static bool     aht_setInitializationRegister(uint8_t value); 
static uint8_t  aht_readStatusRegister();
static uint8_t  aht_getCalibration();
static uint8_t  aht_getBusy(bool readAHT);
static bool     aht_checkCRC8();


/**************************************************************************/
/*
    Constructor
    Initialises the struct and the sensor

    NOTE:
    - call this function before doing anything else!!!
    - call this AHT2X_POWER_ON_DELAY after power on
    - speed in Hz, stretch in usec

    - returned value by "Wire.endTransmission()":
      - 0 success
      - 1 data too long to fit in transmit data buffer
      - 2 received NACK on transmit of address
      - 3 received NACK on transmit of data
      - 4 other error
*/
/**************************************************************************/
bool AHTxx(uint8_t address, AHTXX_I2C_SENSOR sensorType)
{
  bool retval     = 1;
  aht.address     = address;
  aht.sensorType  = sensorType;
  aht.status      = AHTXX_NO_ERROR;

  for (int i = 0; i < 7; ++i)
  {
    aht.rawData[i]    = 0;
  }

  softReset();            // soft reset is recommended at start (reset, set normal mode, set calibration bit & check calibration bit)

  pause_ms(20);
  
  setMode(AHT1X_INIT_CTRL_NORMAL_MODE);

  if (aht_getCalibration() == AHTXX_STATUS_CTRL_CAL_ON)
  {
    retval              = 0;
  }

  return retval;
}


/**************************************************************************/
/*
    readHumidity()

    Read relative humidity, in %

    NOTE:
    - relative humidity range........ 0%..100%
    - relative humidity resolution... 0.024%
    - relative humidity accuracy..... +-2%
    - response time............ 5..30sec
    - measurement with high frequency leads to heating of the
      sensor, must be > 2 seconds apart to keep self-heating below 0.1C
    - long-term exposure for 60 hours outside the normal range
      (humidity > 80%) can lead to a temporary drift of the
      signal +3%, sensor slowly returns to the calibrated state at normal
      operating conditions

    - sensors data structure:
      - {status, RH, RH, RH+T, T, T, CRC*}, *CRC for AHT2x only

    - normal operating range T -20C..+60C, RH 10%..80%
    - maximum operating rage T -40C..+80C, RH 0%..100%
*/
/**************************************************************************/
float readHumidity(bool readAHT)
{
  if (readAHT == AHTXX_FORCE_READ_DATA)
  {
    aht_readMeasurement();                // force to read data via I2C & update "_rawData[]" buffer
  }

  if (aht.status != AHTXX_NO_ERROR)
  {
    return AHTXX_ERROR;                   // no reason to continue, call "getStatus()" for error description
  }

  uint32_t humidity     = aht.rawData[1];                     // 20-bit raw humidity data
           humidity     <<= 8;
           humidity     |= aht.rawData[2];
           humidity     <<= 4;
           humidity     |= (aht.rawData[3] >> 4);

  if (humidity > 0x100000)
  {
    humidity            = 0x100000;       // check if (RH > 100), no need to check for (RH < 0) since "humidity" is "uint"
  }

  return ( (float) humidity / 0x100000) * 100;
}


/**************************************************************************/
/*
    readTemperature()

    Read temperature, in C 

    NOTE:
    - temperature range........ -40C..+85C
    - temperature resolution... 0.01C
    - temperature accuracy..... +-0.3C
    - response time............ 5..30sec
    - measurement with high frequency leads to heating of the
      sensor, must be > 2 seconds apart to keep self-heating below 0.1C

    - sensors data structure:
      - {status, RH, RH, RH+T, T, T, CRC*}, *CRC for AHT2x only
*/
/**************************************************************************/
float readTemperature(bool readAHT)
{
  if (readAHT == AHTXX_FORCE_READ_DATA)
  {
    aht_readMeasurement(); // force to read data via I2C & update "_rawData[]" buffer
  }
  if (aht.status != AHTXX_NO_ERROR)
  {
    return AHTXX_ERROR; // no reason to continue, call "getStatus()" for error description
  }

  uint32_t temperature      = (aht.rawData[3] & 0x0F);                // 20-bit raw temperature data
           temperature      <<= 8;
           temperature      |= aht.rawData[4];
           temperature      <<= 8;
           temperature      |= aht.rawData[5];

  return ( (float) temperature / 0x100000) * 200 - 50;
}


/**************************************************************************/
/*
    setMode()  
 
    AHT1X_INIT_CTRL_NORMAL_MODE - no info in datasheet, suspect this is one measurement & power down
    AHT1X_INIT_CTRL_CYCLE_MODE  - no info in datasheet, suspect this is continuous measurement
    AHT1X_INIT_CTRL_CMD_MODE    - no info in datasheet
*/
/**************************************************************************/
void setMode(uint8_t mode)
{
  mode      = (mode & 0xF0);

  aht_setInitializationRegister(AHTXX_INIT_CTRL_CAL_ON | mode);
}


/**************************************************************************/
/*
    softReset()  
 
    Restart sensor, without power off

    NOTE:
    - takes 20ms
    - all registers set to default
*/
/**************************************************************************/
static bool softReset()
{
  uint8_t write_buf[2];

  write_buf[0]          = AHTXX_SOFT_RESET_REG;

  i2c_AHT_write(aht.address, write_buf, 1);

  return 0;
}


/**************************************************************************/
/*
    getStatus()  
 
    Return sensor status

    NOTE:
    - returned statuse:
      - AHTXX_NO_ERROR   = 0x00, success, no errors
      - AHTXX_BUSY_ERROR = 0x01, sensor is busy
      - AHTXX_ACK_ERROR  = 0x02, sensor didn't return ACK
      - AHTXX_DATA_ERROR = 0x03, received data smaller than expected
      - AHTXX_CRC8_ERROR = 0x04, computed CRC8 not match received CRC8, for AHT2x only
*/
/**************************************************************************/
uint8_t getStatus()
{
  return aht.status;
}


/**************************************************************************/
/*
    setType()  
 
    Set sensor type on the fly

    NOTE:
    - AHT1x vs AHT2x:
      - AHT1x +1.8v..+3.6v, AHT2x 2.2v..5.5v
      - AHT1x 0.25uA..320uA, AHT2x 0.25uA..980uA
      - AHT2x support CRC8 check
*/
/**************************************************************************/
void setType(AHTXX_I2C_SENSOR sensorType)
{
  aht.sensorType = sensorType;
}


/**************************************************************************/
/*
    _readMeasurement()

    Start new measurement, read sensor data to buffer & collect errors

    NOTE:
    - sensors data structure:
      - {status, RH, RH, RH+T, T, T, CRC*}, *CRC for AHT2x only & for
        status description see "_readStatusRegister()" NOTE
*/
/**************************************************************************/
static void aht_readMeasurement()
{
  /* send measurement command */
  // Wire.beginTransmission(_address);

  // Wire.write(AHTXX_START_MEASUREMENT_REG);      //send measurement command, strat measurement
  // Wire.write(AHTXX_START_MEASUREMENT_CTRL);     //send measurement control
  // Wire.write(AHTXX_START_MEASUREMENT_CTRL_NOP); //send measurement NOP control

  // if (Wire.endTransmission(true) != 0)          //collision on I2C bus
  // {
  //   _status = AHTXX_ACK_ERROR;                  //update status byte, sensor didn't return ACK

  //   return;                                     //no reason to continue
  // }

  /* check busy bit */
  aht.status = aht_getBusy(AHTXX_FORCE_READ_DATA);                                                //update status byte, read status byte & check busy bit

  // if      (_status == AHTXX_BUSY_ERROR) {delay(AHTXX_MEASUREMENT_DELAY - AHTXX_CMD_DELAY);}
  // else if (_status != AHTXX_NO_ERROR)   {return;}                                           //no reason to continue, received data smaller than expected

  /* read data from sensor */
  uint8_t dataSize;

  if   (aht.sensorType == AHT1x_SENSOR) {dataSize = 6;}   //{status, RH, RH, RH+T, T, T, CRC*}, *CRC for AHT2x only
  else                               {dataSize = 7;}

  // Wire.requestFrom(_address, dataSize, (uint8_t)true); //read n-byte to "wire.h" rxBuffer, true-send stop after transmission

  // if (Wire.available() != dataSize)
  // {
  //   _status = AHTXX_DATA_ERROR;                        //update status byte, received data smaller than expected

  //   return;                                            //no reason to continue
  // }

  /* read n-bytes from "wire.h" rxBuffer */
  // Wire.readBytes(_rawData, dataSize);                  //"readBytes()", from Stream Class 

  /* check busy bit after measurement dalay */
  aht.status = aht_getBusy(AHTXX_USE_READ_DATA);             //update status byte, read status byte & check busy bit

  if (aht.status != AHTXX_NO_ERROR) {return;}             //no reason to continue, sensor is busy

  /* check CRC8, for AHT2x only */
  if ((aht.sensorType == AHT2x_SENSOR) && (aht_checkCRC8() != true))
  {
    aht.status = AHTXX_CRC8_ERROR; //update status byte
  }
}


/**************************************************************************/
/*
    _setInitializationRegister()
 
    Set initialization register

    NOTE:
    - true=success, false=I2C error
*/
/**************************************************************************/
static bool aht_setInitializationRegister(uint8_t value)
{
  // delay(AHTXX_CMD_DELAY);

  // Wire.beginTransmission(aht._address);

  // if   (aht._sensorType == AHT1x_SENSOR) {Wire.write(AHT1X_INIT_REG);} //send initialization command, for AHT1x only
  // else                               {Wire.write(AHT2X_INIT_REG);} //send initialization command, for AHT2x only

  // Wire.write(value);                                               //send initialization register controls
  // Wire.write(AHTXX_INIT_CTRL_NOP);                                 //send initialization register NOP control

  // return (Wire.endTransmission(true) == 0);                        //true=success, false=I2C error
  return 1;
}


/**************************************************************************/
/*
    _readStatusRegister()

    Read status register

    NOTE:
    - AHT1x status register controls:
      7    6    5    4   3    2   1   0
      BSY, MOD, MOD, xx, CAL, xx, xx, xx
      - BSY:
        - 1, sensor busy/measuring
        - 0, sensor idle/sleeping
      - MOD:
        - 00, normal mode
        - 01, cycle mode
        - 1x, command mode
      - CAL:
        - 1, calibration on
        - 0, calibration off

    - AHT2x status register controls:
      7    6   5   4   3    2   1  0
      BSY, xx, xx, xx, CAL, xx, xx, xx

    - under normal conditions status is 0x18 & 0x80 if the sensor is busy
*/
/**************************************************************************/
static uint8_t aht_readStatusRegister()
{
  // delay(AHTXX_CMD_DELAY);

  // Wire.beginTransmission(aht._address);

  // Wire.write(AHTXX_STATUS_REG);

  // if (Wire.endTransmission(true) != 0) {return AHTXX_ERROR;} //collision on I2C bus, sensor didn't return ACK

  // Wire.requestFrom(aht._address, (uint8_t)1, (uint8_t)true);     //read 1-byte to "wire.h" rxBuffer, true-send stop after transmission

  // if (Wire.available() == 1) {return Wire.read();}           //read 1-byte from "wire.h" rxBuffer
  //                             return AHTXX_ERROR;            //collision on I2C bus, "wire.h" rxBuffer is empty
  return 0;
}


/**************************************************************************/
/*
    _getCalibration()

    Read calibration bits from status register

    NOTE:
    - 0x08 = loaded, 0x00 = not loaded, 0xFF = I2C error
    - calibration status check should only be performed at power-up,
      rechecking is not required during data collection
*/
/**************************************************************************/
static uint8_t aht_getCalibration()
{
  uint8_t value     = aht_readStatusRegister();

  if (value != AHTXX_ERROR)
  {
    return (value & AHTXX_STATUS_CTRL_CAL_ON); // 0x08 = loaded, 0x00 = not loaded
  }
  return AHTXX_ERROR;                         // collision on I2C bus, sensor didn't return ACK
}


/**************************************************************************/
/*
    _getBusy()

    Read/check busy bit after measurement command

    NOTE:
    - part of "readRawMeasurement()" function!!!
    - 0x80=busy, 0x00=measurement completed, etc
*/
/**************************************************************************/
static uint8_t aht_getBusy(bool readAHT)
{
  if (readAHT == AHTXX_FORCE_READ_DATA)                    //force to read data via I2C & update "_rawData[]" buffer
  {
    // delay(AHTXX_CMD_DELAY);

    // Wire.requestFrom(aht._address, (uint8_t)1, (uint8_t)true); //read 1-byte to "wire.h" rxBuffer, true-send stop after transmission

    // if (Wire.available() != 1) {return AHTXX_DATA_ERROR;}  //no reason to continue, "return" terminates the entire function & "break" just exits the loop

    // _rawData[0] = Wire.read();                             //read 1-byte from "wire.h" rxBuffer
  }

  if   ((aht.rawData[0] & AHTXX_STATUS_CTRL_BUSY) == AHTXX_STATUS_CTRL_BUSY) {aht.status = AHTXX_BUSY_ERROR;} //0x80=busy, 0x00=measurement completed
  else                                                                    {aht.status = AHTXX_NO_ERROR;}

  return aht.status;
}


/**************************************************************************/
/*
    _checkCRC8()

    Check CRC-8-Maxim of AHT2X measured data

    NOTE:
    - part of "readRawMeasurement()" function!!!
    - only AHT2x sensors have CRC
    - initial value=0xFF, polynomial=(x8 + x5 + x4 + 1) ie 0x31 CRC [7:0] = 1+X4+X5+X8
*/
/**************************************************************************/
static bool aht_checkCRC8()
{
  if (aht.sensorType == AHT2x_SENSOR)
  {
    uint8_t crc = 0xFF;                                      //initial value

    for (uint8_t byteIndex = 0; byteIndex < 6; byteIndex ++) //6-bytes in data, {status, RH, RH, RH+T, T, T, CRC}
    {
      crc ^= aht.rawData[byteIndex];

      for(uint8_t bitIndex = 8; bitIndex > 0; --bitIndex)    //8-bits in byte
      {
        if   (crc & 0x80) {crc = (crc << 1) ^ 0x31;}         //0x31=CRC seed/polynomial 
        else              {crc = (crc << 1);}
      }
    }

    return (crc == aht.rawData[6]);
  }

  return true;
}
