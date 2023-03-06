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

void process_serial_buffer();

struct RunConfig
{
  RunConfig() 
    : started(false), clk_div(240), 
      clk_divsel(GCLK_DIVSEL_DIRECT), 
      fclk(48000000)
  {
    
  }
  
  bool started;
  uint32_t clk_div;
  DIVSEL_T clk_divsel;
  uint32_t fclk;
} cfg;

// create a pixel strand with 1 pixel on PIN_NEOPIXEL
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

bool led_state = false;

volatile bool result_ready;
volatile uint16_t adc_val;
volatile uint32_t dropped_count;
volatile uint32_t timestamp_us;

void ADC_Handler() 
{
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
      led_state = !led_state;
    }        
    if ((packet.max_packets > 0) && (packet.sample_count >= packet.max_packets)) {
      stop_ADC();
      cfg.started = false;
      Serial.write(packet.buffer(), packet.byte_count());
      packet.reset();
      led_state = false;
    }
  }
}

// Message format
// H : halt, send last packet then halt packet
// R# : start with max packets count as string (0=no max), streams packets HDR_DATA | timestamp_ms | T .. | P ..
// C : configure
//  D# : clock divisor
//  M# : clock divisor mode (0=direct, 1=pow2)
// A : ask
//  F : clock frequency
//  D : clock divisor
//  M : clock divisor mode (0=direct, 1 = pow2)
// Z : reset the board
void process_serial_buffer()
{
  char const data_in = Serial.read();
  if (data_in == 'Z') {
    NVIC_SystemReset();
  } else if ((data_in == 'H') && (cfg.started)) {
      stop_ADC();
      cfg.started = false;

      pixels.clear();
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
      result_ready = false;
      start_ADC();
    } else if (data_in == 'C') {
      char const cfg_opt = Serial.read();      
      bool invalidate_clk = false;
      if (cfg_opt == 'D') {
        cfg.clk_div = Serial.parseInt();        
        invalidate_clk = true;
      } else if (cfg_opt == 'M') {
        cfg.clk_divsel = Serial.parseInt() == 0 ? GCLK_DIVSEL_DIRECT : GCLK_DIVSEL_POW2;
        invalidate_clk = true;
      } 

      if (invalidate_clk) {
        cfg.fclk = init_GCLK(5, cfg.clk_div, cfg.clk_divsel, true);  
      }
    } else if (data_in == 'A') {
      char const ask_opt = Serial.read();
      uint8_t count = 0;
      if (ask_opt == 'F') {
        //Serial.println(cfg.fclk);        
        count = packet.write_resp(RESP_TYPE_FCLK, cfg.fclk);        
      } else if (ask_opt == 'D') {
        //Serial.println(cfg.clk_div);
        count = packet.write_resp(RESP_TYPE_DIV, cfg.clk_div);        
      } else if (ask_opt == 'M') {
        //Serial.println((int)cfg.clk_divsel);
        count = packet.write_resp(RESP_TYPE_DIV_MODE, cfg.clk_divsel);        
      } else if (ask_opt == 'B') {  /// board ID
        count = packet.write_resp(RESP_TYPE_ID, 0xB0A4D001);
      }
      if (count > 0) {
        Serial.write(packet.buffer(), count);
      }
    }
  }
}
