/* MHZ library

    By Tobias Schürg
*/


#include "MHZ.h"
#include "freertos/FreeRTOS.h"    // needed for vTaskDelay


//  private:
static const unsigned long MHZ14A_PREHEATING_TIME     = (3L * 60L * 1000L);
static const unsigned long MHZ14B_PREHEATING_TIME     = (1L * 30L * 1000L);
static const unsigned long MHZ16_PREHEATING_TIME      = (1L * 10L * 1000L);
static const unsigned long MHZ1911A_PREHEATING_TIME   = (1L * 60L * 1000L);
static const unsigned long MHZ19B_PREHEATING_TIME     = (3L * 60L * 1000L);
static const unsigned long MHZ19C_PREHEATING_TIME     = (1L * 60L * 1000L);
static const unsigned long MHZ19D_PREHEATING_TIME     = (1L * 60L * 1000L);
static const unsigned long MHZ19E_PREHEATING_TIME     = (1L * 60L * 1000L);

static const unsigned long MHZ14A_RESPONSE_TIME       = (60L * 1000L);
static const unsigned long MHZ14B_RESPONSE_TIME       = (0);
static const unsigned long MHZ16_RESPONSE_TIME        = (30L * 1000L);
static const unsigned long MHZ1911A_RESPONSE_TIME     = (120L * 1000L);
static const unsigned long MHZ19B_RESPONSE_TIME       = (120L * 1000L);
static const unsigned long MHZ19C_RESPONSE_TIME       = (120L * 1000L);
static const unsigned long MHZ19D_RESPONSE_TIME       = (120L * 1000L);
static const unsigned long MHZ19E_RESPONSE_TIME       = (120L * 1000L);

struct MHZ_st
{
  enum SensorType type;
  uint8_t temperature;
  uint8_t temperatureOffset;
  enum MeasuringRange range;
  bool debug;
  bool isBypassPreheatingCheck;
  bool isBypassResponseTimeCheck;
  unsigned long sLastPpm;
} MHZ;

unsigned long lastRequest       = 0;


extern uint32_t millis(void);
extern void serial_MHZ_flush(void);
extern uint16_t serial_MHZ_available(void);
extern uint16_t serial_MHZ_read(uint8_t* buf);
extern uint16_t serial_MHZ_write(uint8_t* buf, uint16_t len);


uint8_t getCheckSum(uint8_t *packet);
static unsigned long getTimeDiff(unsigned long start, unsigned long stop);
static bool isPreHeating(void);


static unsigned long getTimeDiff(unsigned long start, unsigned long stop)
{
  unsigned long retval;

  if (stop < start)
  {
    retval      = (ULONG_MAX - start) + stop;
  }
  else
  {
    retval      = (stop - start);
  }

  return retval;
} // end of getTimeDiff ------------------------------------


void MHZ_init(enum SensorType type, enum MeasuringRange range)
{
  MHZ.type                        = type;
  MHZ.temperatureOffset           = 44;
  MHZ.temperature                 = STATUS_CHECKSUM_MISMATCH;
  MHZ.range                       = range;
  MHZ.debug                       = false;
  MHZ.isBypassPreheatingCheck     = false;
  MHZ.isBypassResponseTimeCheck   = false;
  MHZ.sLastPpm                    = 0;
} // end of MHZ_init ---------------------------------------


/**
 * Enables or disables the debug mode (more logging).
 */
void MHZ_setDebug(bool enable)
{
  MHZ.debug           = enable;

  if (MHZ.debug)
  {
    printf("MHZ: debug mode ENABLED\n");
  }
  else
  {
    printf("MHZ: debug mode DISABLED\n");
  }
} // end of MHZ_setDebug -----------------------------------


static bool isPreHeating(void)
{
  if (MHZ.isBypassPreheatingCheck)
  {
    return false;
  }
  else if (MHZ.type == MHZ14A)
  {
    return millis() < (MHZ14A_PREHEATING_TIME);
  }
  else if (MHZ.type == MHZ14B)
  {
    return millis() < (MHZ14B_PREHEATING_TIME);
  }
  else if (MHZ.type == MHZ16)
  {
    return millis() < (MHZ16_PREHEATING_TIME);
  }
  else if (MHZ.type == MHZ1911A)
  {
    return millis() < (MHZ1911A_PREHEATING_TIME);
  }
  else if (MHZ.type == MHZ19B)
  {
    return millis() < (MHZ19B_PREHEATING_TIME);
  }
  else if (MHZ.type == MHZ19C)
  {
    return millis() < (MHZ19C_PREHEATING_TIME);
  }
  else if (MHZ.type == MHZ19D)
  {
    return millis() < (MHZ19D_PREHEATING_TIME);
  }
  else if (MHZ.type == MHZ19E)
  {
    return millis() < (MHZ19E_PREHEATING_TIME);
  }
  else
  {
    printf("MHZ::isPreHeating() => UNKNOWN SENSOR\n");
    return false;
  }
}


bool MHZ_isReady(void)
{
  if (isPreHeating())
  {
    return false;
  }
  else if (MHZ.isBypassResponseTimeCheck)
  {
    return true;
  }
  else if (MHZ.type == MHZ14A)
  {
    return getTimeDiff(lastRequest, millis()) > MHZ14A_RESPONSE_TIME;
  }
  else if (MHZ.type == MHZ14B)
  {
    return getTimeDiff(lastRequest, millis()) > MHZ14B_RESPONSE_TIME;
  }
  else if (MHZ.type == MHZ16)
  {
    return getTimeDiff(lastRequest, millis()) > MHZ16_RESPONSE_TIME;
  }
  else if (MHZ.type == MHZ1911A)
  {
    return getTimeDiff(lastRequest, millis()) > MHZ1911A_RESPONSE_TIME;
  }
  else if (MHZ.type == MHZ19B)
  {
    return getTimeDiff(lastRequest, millis()) > MHZ19B_RESPONSE_TIME;
  }
  else if (MHZ.type == MHZ19C)
  {
    return getTimeDiff(lastRequest, millis()) > MHZ19C_RESPONSE_TIME;
  }
  else if (MHZ.type == MHZ19D)
  {
    return getTimeDiff(lastRequest, millis()) > MHZ19D_RESPONSE_TIME;
  }
  else if (MHZ.type == MHZ19E)
  {
    return getTimeDiff(lastRequest, millis()) > MHZ19E_RESPONSE_TIME;
  }
  else
  {
    printf("MHZ::isReady() => UNKNOWN SENSOR \"%u\"\n", MHZ.type);
    return true;
  }
} // end of MHZ_isReady ------------------------------------


int32_t readCO2UART(void)
{
  if (!MHZ_isReady()) return STATUS_NOT_READY;

  // Clearing the uart reading buffer to avoid:
  // - processing the unwanted data the sensor sends during startup
  // - reading an old response already in the reading buffer
  if (MHZ.debug)
  {
    printf("MHZ: - clearing uart reading buffer\n");
  }
  uint8_t dummy_buf[9];
  serial_MHZ_read(dummy_buf);

  if (MHZ.debug)
  {
    printf("-- read CO2 uart ---\n");
  }
  uint8_t cmd[9]      = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  uint8_t response[9];  // for answer

  if (MHZ.debug)
  {
    printf("  >> Sending CO2 request\n");
  }
  serial_MHZ_write(cmd, 9);  // request PPM CO2
  lastRequest         = millis();

  // clear the buffer
  for (int i = 0; i < 9; i++)
  {
    response[i]     = 0;
  }

  int waited        = 0;
  while (serial_MHZ_available() == 0)
  {
    if (MHZ.debug)
    {
      printf(".");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  // wait a short moment to avoid false reading
    waited++;
    if (waited > 10)
    {
      if (MHZ.debug)
      {
        printf("No response after 1 second\n");
      }
      serial_MHZ_flush();
      return STATUS_NO_RESPONSE;
    }
  }

  // The serial stream can get out of sync. The response starts with 0xff, try
  // to resync.
  // TODO: I think this might be wrong any only happens during initialization?
  // bool skip = false;
  // while (_serial->available() > 0 && (unsigned char)_serial->peek() != 0xFF) {
  //   if (!skip) {
  //     printf("MHZ: - skipping unexpected readings:\n");
  //     skip = true;
  //   }
  //   _console->print(" ");
  //   _console->print(_serial->peek(), HEX);
  //   _serial->read();
  // }
  // if (skip) printf("\n");

  if (serial_MHZ_available() > 0)
  {
    int count         = serial_MHZ_read(response);
    if (count < 9)
    {
      serial_MHZ_flush();
      return STATUS_INCOMPLETE;
    }
  }
  else
  {
    serial_MHZ_flush();
    return STATUS_INCOMPLETE;
  }

  if (MHZ.debug)
  {
    // print out the response in hexa
    printf("  << ");
    for (int i = 0; i < 9; i++)
    {
      printf("%X", response[i]);
      printf("  ");
    }
    printf("\n");
  }

  // checksum
  uint8_t check       = getCheckSum(response);
  if (response[8] != check)
  {
    printf("MHZ: Checksum not OK!\n");
    printf("MHZ: Received: %X\n", response[8]);
    printf("MHZ: Should be: %X\n", check);
    MHZ.temperature       = STATUS_CHECKSUM_MISMATCH;
    serial_MHZ_flush();
    return STATUS_CHECKSUM_MISMATCH;
  }

  int32_t ppm_uart    = (256 * (int32_t)response[2]) + response[3];

  MHZ.sLastPpm        = ppm_uart;
  MHZ.temperature         = response[4] - MHZ.temperatureOffset;

  uint8_t status      = response[5];
  if (MHZ.debug)
  {
    printf(" # PPM UART: %lu\n", ppm_uart);
    printf(" # Temperature? %u\n", MHZ.temperature);
  }

  // Is always 0 for version 14a  and 19b
  // Version 19a?: status != 0x40
  if (MHZ.debug && status != 0)
  {
    printf(" ! Status maybe not OK ! %u\n", status);
  }
  else if (MHZ.debug)
  {
    printf(" Status  OK: %u\n", status);
  }

  serial_MHZ_flush();

  return ppm_uart;
}


int getLastTemperature(void)
{
  if (isPreHeating())
  {
    return STATUS_NOT_READY;
  }
  return MHZ.temperature;
}


void setTemperatureOffset(uint8_t offset)
{
  MHZ.temperatureOffset    = offset;
}


void MHZ_setBypassCheck(bool isBypassPreheatingCheck, bool isBypassResponseTimeCheck)
{
  MHZ.isBypassPreheatingCheck      = isBypassPreheatingCheck;
  MHZ.isBypassResponseTimeCheck    = isBypassResponseTimeCheck;
}


int32_t getLastCO2(void)
{
  return MHZ.sLastPpm;
}


uint8_t getCheckSum(uint8_t* packet)
{
  unsigned char checksum    = 0;

  if (MHZ.debug)
  {
    printf("  getCheckSum()");
  }

  for (uint8_t i = 1; i < 8; i++)
  {
    checksum                += packet[i];
  }
  checksum                  = 0xff - checksum;
  checksum                  += 1;
  return checksum;
}


void setAutoCalibrate(bool b)  // only available for MHZ-19B with firmware < 1.6, MHZ-19C and MHZ 14a
{
  uint8_t cmd_enableAutoCal[9]    = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};
  uint8_t cmd_disableAutoCal[9]   = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};
  if (b)
  {
    serial_MHZ_write(cmd_enableAutoCal, 9);
  }
  else
  {
    serial_MHZ_write(cmd_disableAutoCal, 9);
  }
}


void setRange(int range)  // only available for MHZ-19B < 1.6 and MH-Z 14a
{
  uint8_t cmd_2K[9]     = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F};
  uint8_t cmd_5K[9]     = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88, 0xCB};
  uint8_t cmd_10K[9]    = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x27, 0x10, 0x2F};

  switch (range)
  {
    case 1:
      serial_MHZ_write(cmd_2K, 9);
      break;

    case 2:
      serial_MHZ_write(cmd_5K, 9);
      break;

    case 3:
      serial_MHZ_write(cmd_10K, 9);
  }
}


void calibrateZero(void)
{
  uint8_t cmd[9]    = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
  serial_MHZ_write(cmd, 9);
}


