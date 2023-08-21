#ifndef __KX134_h
#define __KX134_h

#include "accelerometerbase.h"

#include <SparkFun_KX13X.h> 

#define KX134_BUS_SPI

class KX134 : public AccelerometerBase {
public:

  using QueryResponse = AccelerometerBase::QueryResponse;
  using DataBuffer = AccelerometerBase::DataBuffer;
  using VoidFunc_T = void (*)();

  KX134(VoidFunc_T callback, int int_pin = 6);

  struct Config
  {
    Config() : 
      odata_rate(0x07), /* 100Hz */
      g_range(0x01)     /* 16g */
    { 

    }
    uint8_t odata_rate;   // only lower four bits are used
    uint8_t g_range;      // lower 2 bits, 0=8g to 3=64g
  };

  uint8_t type_id() const override { return TYPE_ID_KX134; }

  bool init() override;
  void start() override;
  void stop() override;

  DataBuffer process() override;

  void set(char key, uint32_t val) override;
  QueryResponse get(char key) const override;

private:

#ifdef KX134_BUS_SPI
  SparkFun_KX134_SPI accel_;
#else
  SparkFun_KX134 accel_;
#endif

  Config cfg_;
  VoidFunc_T callback_;
  int8_t drdy_int_pin_;
  rawOutputData raw_data_;
  uint16_t raw_data_buffer_[3];
  DataBuffer raw_data_buffer_ref_;
};


#endif //__KX134_h
