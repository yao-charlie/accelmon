// testing the free-running ADC on the Trinket M0
// https://forcetronic.blogspot.com/2016/10/utilizing-advanced-adc-capabilities-on.html
// https://community.atmel.com/forum/samd21-adc-interrupt-routine
// https://www.eevblog.com/forum/microcontrollers/pin-multiplexing-in-samd21/
// https://github.com/ataradov/mcu-starter-projects/blob/master/samd21/hal_gpio.h
// https://forum.arduino.cc/t/arduino-m0-pro-adc-free-running-with-interrupt/471688

#include <Adafruit_NeoPixel.h>

#include "adc_util.h"
#include "gclk_util.h"
#include "port_util.h"
#include "packetcontainer.h"

// IO Pins from SAMD
#define SELF_TEST_IOPIN 16  // D4/SDA
#define STBY_IOPIN 17       // D5/SCL
#define OR_IOPIN 6          // D6/A6/TX
#define ADC_CONV_IOPIN 7    // D7/A7/RX

void process_serial_buffer();

struct RunConfig
{
  RunConfig() 
    : started(false), clk_div(240), 
      clk_divsel(GCLK_DIVSEL_DIRECT), 
      adc_prescaler(0),
      adc_samplen(0),
      fclk(48000000)
  {
    
  }
  
  uint32_t adc_clk_est() const {
    return fclk >> (adc_prescaler + 2);
  }

  bool started;
  uint8_t clk_div;
  DIVSEL_T clk_divsel;
  uint8_t adc_prescaler;
  uint8_t adc_samplen;
  uint32_t fclk;
} cfg;

// create a pixel strand with 1 pixel on PIN_NEOPIXEL
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

volatile bool result_ready;
volatile uint16_t adc_val;
volatile uint32_t dropped_count;
volatile uint32_t timestamp_us;

void ADC_Handler() 
{
  PORT->Group[PORTA].OUTTGL.reg = (1 << ADC_CONV_IOPIN); 
  if (!result_ready) {
    //timestamp_us = TC4->COUNT32.COUNT.reg;
    adc_val = 0x0FFF & ADC->RESULT.reg;     // uint16_t
    result_ready = true;
  } else {
    dropped_count++;
  }
  ADC->INTFLAG.bit.RESRDY = 1;  // write a bit to clear interrupt
}

void setup() 
{
  Serial.begin(115200);
  pixels.begin();
  
  delay(1000);

  pixels.setPixelColor(0, pixels.Color(0, 96, 0));
  pixels.show();
    
  init_pin_for_D_out(ADC_CONV_IOPIN);
  init_pin_for_D_out(STBY_IOPIN);
  init_pin_for_D_out(SELF_TEST_IOPIN);
  init_pin_for_D_in(OR_IOPIN);
  
  //Serial.println("Hello accelo");

  init_pin_for_CLK_out();
  cfg.fclk = init_GCLK(5, cfg.clk_div, cfg.clk_divsel, true);  

  init_pin_for_ADC_in();
  init_ADC(GCLK_CLKCTRL_GEN_GCLK5);

  //Serial.print("fclk = ");
  //Serial.println(cfg.fclk);

  delay(1000);
  pixels.clear();
  pixels.show();
  

}

void loop() 
{
  process_serial_buffer();

  if (result_ready) {
    uint16_t const sample = adc_val;
    result_ready = false;
    if (packet.append_sample(sample)) {
      Serial.write(packet.buffer(), packet.byte_count());
      packet.reset();    
    }        
    if ((packet.max_packets > 0) && (packet.sample_count >= packet.max_packets)) {
      stop_ADC();
      cfg.started = false;
      Serial.write(packet.buffer(), packet.byte_count());
      packet.reset();
    }
  }
}

// Message format
// H : halt, send last packet then halt packet
// R# : start with max packets count as string (0=no max), streams packets HDR_DATA | timestamp_ms | T .. | P ..
// C : configure
//  D# : clock divisor (0-255 for direct, 0-8 for pow2)
//  M# : clock divisor mode (0=direct, 1=pow2)
//  P# : ADC prescaler (2^(x + 4))
//  L# : ADC sample length (half-clock cycles)
// A : ask
//  F : ADC clock frequency
//  D : clock divisor
//  M : clock divisor mode (0=direct, 1 = pow2)
//  P : ADC prescaler setting
//  L : ADC sample length
// Z : reset the board
void process_serial_buffer()
{
  char const data_in = Serial.read();
  if (data_in == 'Z') {
    NVIC_SystemReset();
  } else if ((data_in == 'H') && (cfg.started)) {
      stop_ADC();
      cfg.started = false;

      if (dropped_count == 0) {
        pixels.clear();
      } else {
        pixels.setPixelColor(0, pixels.Color(96, 0, 0));
      }
      pixels.show();

  } else if (!cfg.started) {
    if (data_in == 'R') {
      char count_buf[4];
      int const rlen = Serial.readBytes(count_buf, 4);
      if (rlen == 4) {
        auto const cfg_val = *reinterpret_cast<uint32_t*>(count_buf);
        packet.max_packets = cfg_val >= 0 ? cfg_val : 0;
      } else {
        packet.max_packets = 0;
      }

      pixels.setPixelColor(0, pixels.Color(0, 0, 96));
      pixels.show();
     
      packet.sample_count = 0;
      cfg.started = true;
      dropped_count = 0;
      result_ready = false;
      start_ADC();
    } else if (data_in == 'C') {
      char const cfg_opt = Serial.read();      
      bool invalidate_clk = false;
      bool invalidate_adc = false;
      if (cfg_opt == 'D') {
        cfg.clk_div = Serial.parseInt();        
        invalidate_clk = true;
      } else if (cfg_opt == 'M') {
        cfg.clk_divsel = Serial.parseInt() == 0 ? GCLK_DIVSEL_DIRECT : GCLK_DIVSEL_POW2;
        invalidate_clk = true;
      } else if (cfg_opt == 'P') {
        uint8_t const pval = Serial.parseInt();
        if ((pval != cfg.adc_prescaler) &&  (pval < 8)) {        
          cfg.adc_prescaler = pval;
          invalidate_adc = true;
        }
      } else if (cfg_opt == 'L') {
        uint8_t const lval = Serial.parseInt();
        if ((lval != cfg.adc_samplen) &&  (lval < 64)) {        
          cfg.adc_samplen = lval;
          invalidate_adc = true;
        }
      } 

      if (invalidate_clk) {
        cfg.fclk = init_GCLK(5, cfg.clk_div, cfg.clk_divsel, true);  
      }
      if (invalidate_adc) {
          stop_ADC();
          init_ADC(GCLK_CLKCTRL_GEN_GCLK5, cfg.adc_prescaler, cfg.adc_samplen);
      }
    } else if (data_in == 'A') {
      char const ask_opt = Serial.read();
      uint8_t count = 0;
      if (ask_opt == 'F') {
        //Serial.println(cfg.fclk);        
        count = packet.write_resp(RESP_TYPE_FCLK, cfg.adc_clk_est());        
      } else if (ask_opt == 'D') {
        //Serial.println(cfg.clk_div);
        count = packet.write_resp(RESP_TYPE_DIV, cfg.clk_div);        
      } else if (ask_opt == 'M') {
        //Serial.println((int)cfg.clk_divsel);
        count = packet.write_resp(RESP_TYPE_DIV_MODE, cfg.clk_divsel);        
      } else if (ask_opt == 'P') {
        count = packet.write_resp(RESP_TYPE_ADC_PRE, cfg.adc_prescaler);        
      } else if (ask_opt == 'L') {
        count = packet.write_resp(RESP_TYPE_ADC_SAMPLEN, cfg.adc_samplen);        
      } else if (ask_opt == 'C') {  // sample count
        count = packet.write_resp(RESP_TYPE_SAMPLE_COUNT, packet.sample_count);        
      } else if (ask_opt == 'B') {  /// board ID
        count = packet.write_resp(RESP_TYPE_ID, 0xB0A4D001);
      }
      if (count > 0) {
        Serial.write(packet.buffer(), count);
      }
    }
  }
}
