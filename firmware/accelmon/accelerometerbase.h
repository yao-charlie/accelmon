#pragma once

#include <cstdint>

#define TYPE_ID_ADXL1005  0x01
#define TYPE_ID_KX134     0x02

class AccelerometerBase{
public:
  struct QueryResponse
  {
    QueryResponse() : type(0), val(0) {}
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

