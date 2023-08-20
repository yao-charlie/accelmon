#pragma once

#include <cstdint>
#include "packetcontainer.h"

#define TYPE_ID_ADXL1005  0x01
#define TYPE_ID_KX134     0x02

// IO Pins for SAMD, i.e PAxx
#define ADXL1005_SELF_TEST_IOPIN 3    // D1/A1
#define ADXL1005_STBY_IOPIN 4         // D2/A2
#define ADXL1005_OR_IOPIN 5           // D3/A3
#define ADXL1005_ADC_CONV_IOPIN 6     // D6/A6/TX
#define ADXL1005_DEBUG_GCLK_IOPIN 11  // D8/A8/SCK

#define KX134_DRDY_IOPIN 7            // D7/A7/RX
#define KX134_NCS_IOPIN 16            // D4/SDA -- for SPI interface, re-use SDA as CS
// leave SPI & I2C pins free

class AccelerometerBase{
public:
  struct QueryResponse
  {
    QueryResponse() : type(RESP_TYPE_NONE), val(0) {}
    uint8_t type;
    uint32_t val;
  };

  struct DataBuffer 
  {
    DataBuffer() : buf(0), count(0) {}
    uint16_t const* buf;
    int16_t count;
  };

  AccelerometerBase() {}

  virtual uint8_t type_id() const = 0;

  virtual bool init() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;

  virtual DataBuffer process();

  virtual void set(char key, uint32_t val) = 0;
  virtual QueryResponse get(char key) const = 0;

private:
  DataBuffer null_buffer_;

};

