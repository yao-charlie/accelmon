#pragma once

#include "accelerometerbase.h"
#include "gclk_util.h"

// testing the free-running ADC on the Trinket M0
// https://forcetronic.blogspot.com/2016/10/utilizing-advanced-adc-capabilities-on.html
// https://community.atmel.com/forum/samd21-adc-interrupt-routine
// https://www.eevblog.com/forum/microcontrollers/pin-multiplexing-in-samd21/
// https://github.com/ataradov/mcu-starter-projects/blob/master/samd21/hal_gpio.h
// https://forum.arduino.cc/t/arduino-m0-pro-adc-free-running-with-interrupt/471688

class ADXL1005 : public AccelerometerBase 
{
public:

  using QueryResponse = AccelerometerBase::QueryResponse;

  ADXL1005() {}

  struct Config
  {
    Config() 
      : clk_div(240), 
        clk_divsel(GCLK_DIVSEL_DIRECT), 
        adc_prescaler(0),
        adc_samplen(0),
        fclk(48000000)
    {
      
    }
    
    uint32_t adc_clk_est() const {
      return fclk >> (adc_prescaler + 2);
    }

    uint8_t clk_div;
    DIVSEL_T clk_divsel;
    uint8_t adc_prescaler;
    uint8_t adc_samplen;
    uint32_t fclk;
  };

  uint8_t type_id() const override { return TYPE_ID_ADXL1005; }

  bool init() override;
  void start() override;
  void stop() override;

  void set(char key, uint32_t val) override;
  QueryResponse get(char key) const override;

private:
  Config cfg_;

};

