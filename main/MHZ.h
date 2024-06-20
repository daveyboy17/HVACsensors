/* MHZ library

    By Tobias Schürg
*/


#ifndef MHZ_H
#define MHZ_H


#include "esp_system.h"     // needed for uint8_t etc.

// #include <limits.h>


#define MHZ_BAUDRATE    (9600)


//  public:
 enum SensorType {
  MHZ14A,
  MHZ14B,
  MHZ16,
  MHZ1911A,
  MHZ19B,
  MHZ19C,
  MHZ19D,
  MHZ19E
};

 enum StatusCode {
    STATUS_NO_RESPONSE            = -2,
    STATUS_CHECKSUM_MISMATCH      = -3,
    STATUS_INCOMPLETE             = -4,
    STATUS_NOT_READY              = -5,
    STATUS_PWM_NOT_CONFIGURED     = -6,
    STATUS_SERIAL_NOT_CONFIGURED  = -7
  };

 enum MeasuringRange {
    RANGE_2K      = 2000,
    RANGE_5K      = 5000,
    RANGE_10K     = 10000,
    RANGE_50K     = 50000
  };


void MHZ_init(enum SensorType type, enum MeasuringRange range);
void MHZ_setDebug(bool enable);
void MHZ_setBypassCheck(bool isBypassPreheatingCheck, bool isBypassResponseTimeCheck);

// bool isPreHeating(void);
bool MHZ_isReady(void);
void setAutoCalibrate(bool b);
void calibrateZero(void);
void setRange(int range);

int32_t readCO2UART(void);
int getLastTemperature(void);
void setTemperatureOffset(uint8_t offset);
int32_t getLastCO2(void);
void activateAsyncUARTReading(void);


#endif
