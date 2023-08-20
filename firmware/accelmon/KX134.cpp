#include "KX134.h"

// config response types (0x00-0x02 reserved)
#define RESP_TYPE_ODR   0x03
#define RESP_TYPE_GSEL  0x04

KX134::KX134(VoidFunc_T callback, int int_pin /* = 6*/) 
  : callback_(callback), drdy_int_pin_(int_pin) 
{
  raw_data_buffer_ref_.buf = &raw_data_buffer_[0];
  raw_data_buffer_ref_.count = 3;
}

bool KX134::init()
{
  Wire.begin();
  delay(50);

  if (!accel_.begin()) {
    return false;
  }
  
  //Serial.println("Ready.");

  accel_.softwareReset();

  // Give some time for the accelerometer to reset.
  // It needs two, but give it five for good measure.
  delay(5);

  stop();

  // hardware interrupt on INT1, 
  accel_.configureInterruptPin(0x30);         // INC1 register: enabled, active HI, latched
  accel_.routeHardwareInterrupt(0x10);        // INC4 register: route DRDY signal to INT1

  accel_.setOutputDataRate(cfg_.odata_rate);  // 100Hz default
  
}

void KX134::start()
{

  attachInterrupt(drdy_int_pin_, callback_, RISING);

  // CTL1 register
  uint8_t const PC1_EN = (1 << 7);
  uint8_t const RES_HI_PERFORMANCE = (1 << 6);
  uint8_t const DRDY_EN = (1 << 5);
  uint8_t const GSEL_RANGE = (cfg_.g_range << 3);
  accel_.writeRegisterByte(SFE_KX13X_CNTL1, (PC1_EN | RES_HI_PERFORMANCE | DRDY_EN | GSEL_RANGE));
  
  delay(20);  // wait 1.5/ODR = 15ms after CTL1.PC1 0->1
}

void KX134::stop()
{
  detachInterrupt(drdy_int_pin_);
  accel_.enableAccel(false);            // CTL1.PC1 = 0
  accel_.clearInterrupt();    
}

KX134::DataBuffer KX134::process()
{
  accel_.getRawAccelData(&raw_data_);
  accel_.clearInterrupt();
  raw_data_buffer_[0] = raw_data_.xData;
  raw_data_buffer_[1] = raw_data_.yData;
  raw_data_buffer_[2] = raw_data_.zData;
  return raw_data_buffer_ref_;
}

void KX134::set(char const key, uint32_t const val) 
{
  if (key == 'F') {
    uint8_t const fval = val & 0x0F;
    if (fval != cfg_.odata_rate) {        
      cfg_.odata_rate = fval;
      stop();
      accel_.setOutputDataRate(cfg_.odata_rate); 
    }
  } else if (key == 'G') {
    uint8_t const fval = val & 0x03;    // choices are 0=8g, 1=16g, 2=32g, 3=64g
    if (fval != cfg_.g_range) {        
      cfg_.g_range = fval;
      stop();
      // write on start
    }
  }
}

KX134::QueryResponse KX134::get(char const key) const
{
  QueryResponse r;
  uint8_t count = 0;
  if (key == 'F') {
    r.type = RESP_TYPE_ODR;
    r.val = cfg_.odata_rate;
  } else if (key == 'G') {
    r.type = RESP_TYPE_GSEL;
    r.val = cfg_.g_range;
  } 
  return r;
}
