#include "ADXL1005.h"
#include <Arduino.h>
#include "port_util.h"
#include "adc_util.h"



// config response types
#define RESP_TYPE_FCLK          0x3
#define RESP_TYPE_DIV           0x4
#define RESP_TYPE_DIV_MODE      0x5
#define RESP_TYPE_ADC_PRE       0x6
#define RESP_TYPE_ADC_SAMPLEN   0x7

bool ADXL1005::init()
{
  init_pin_for_D_out(ADXL1005_ADC_CONV_IOPIN);
  init_pin_for_D_out(ADXL1005_STBY_IOPIN);
  init_pin_for_D_out(ADXL1005_SELF_TEST_IOPIN);
  init_pin_for_D_in(ADXL1005_OR_IOPIN);
  
  //Serial.println("Hello accelo");

  init_pin_for_CLK_out();
  cfg_.fclk = init_GCLK(5, cfg_.clk_div, cfg_.clk_divsel, true);  

  init_pin_for_ADC_in();
  init_ADC(GCLK_CLKCTRL_GEN_GCLK5);

  //Serial.print("fclk = ");
  //Serial.println(cfg_.fclk);
  return true;
}

void ADXL1005::start() 
{
  start_ADC();
}

void ADXL1005::stop() 
{
  stop_ADC();
}

void ADXL1005::set(char const key, uint32_t const val) 
{
  bool invalidate_clk = false;
  bool invalidate_adc = false;

  if (key == 'D') {
    cfg_.clk_div = val;
    invalidate_clk = true;
  } else if (key == 'M') {
    cfg_.clk_divsel = val == 0 ? GCLK_DIVSEL_DIRECT : GCLK_DIVSEL_POW2;
    invalidate_clk = true;
  } else if (key == 'P') {
    uint8_t const pval = val & 0xFF;
    if ((pval != cfg_.adc_prescaler) &&  (pval < 8)) {        
      cfg_.adc_prescaler = pval;
      invalidate_adc = true;
    }
  } else if (key == 'L') {
    uint8_t const lval = val & 0xFF;
    if ((lval != cfg_.adc_samplen) &&  (lval < 64)) {        
      cfg_.adc_samplen = lval;
      invalidate_adc = true;
    }
  } 

  if (invalidate_clk) {
    cfg_.fclk = init_GCLK(5, cfg_.clk_div, cfg_.clk_divsel, true);  
  }
  if (invalidate_adc) {
      stop_ADC();
      init_ADC(GCLK_CLKCTRL_GEN_GCLK5, cfg_.adc_prescaler, cfg_.adc_samplen);
  }

}

ADXL1005::QueryResponse ADXL1005::get(char const key) const
{
  QueryResponse r;
  uint8_t count = 0;
  if (key == 'F') {
    r.type = RESP_TYPE_FCLK;
    r.val = cfg_.adc_clk_est();
    //count = packet.write_resp(RESP_TYPE_FCLK, cfg_.adc_clk_est());        
  } else if (key == 'D') {
    r.type = RESP_TYPE_DIV;
    r.val = cfg_.clk_div;
    //count = packet.write_resp(RESP_TYPE_DIV, cfg_.clk_div);        
  } else if (key == 'M') {
    r.type = RESP_TYPE_DIV_MODE;
    r.val = cfg_.clk_divsel;
    //count = packet.write_resp(RESP_TYPE_DIV_MODE, cfg_.clk_divsel);        
  } else if (key == 'P') {
    r.type = RESP_TYPE_ADC_PRE;
    r.val = cfg_.adc_prescaler;
    //count = packet.write_resp(RESP_TYPE_ADC_PRE, cfg_.adc_prescaler);        
  } else if (key == 'L') {
    r.type = RESP_TYPE_ADC_SAMPLEN;
    r.val = cfg_.adc_samplen;
    //count = packet.write_resp(RESP_TYPE_ADC_SAMPLEN, cfg_.adc_samplen);        
  } 
  return r;
}