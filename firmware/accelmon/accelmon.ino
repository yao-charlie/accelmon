// testing the free-running ADC on the Trinket M0
// https://forcetronic.blogspot.com/2016/10/utilizing-advanced-adc-capabilities-on.html
// https://community.atmel.com/forum/samd21-adc-interrupt-routine
// https://www.eevblog.com/forum/microcontrollers/pin-multiplexing-in-samd21/
// https://github.com/ataradov/mcu-starter-projects/blob/master/samd21/hal_gpio.h
// https://forum.arduino.cc/t/arduino-m0-pro-adc-free-running-with-interrupt/471688


#include "adc_util.h"
#include "gclk_util.h"
#include "port_util.h"
#include "packetcontainer.h"

#define MSG_BUF_LEN 24
char msg_buffer[MSG_BUF_LEN];
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

/*
inline void send_data_packet() 
{
  packet.buf[0] |= HDR_TYPE_DATA;
  Serial.write((uint8_t*)(&packet.buf[0]), PACKET_LENGTH);
}

inline void send_sync_packet(bool const halt = false)
{
  packet.buf[0] = (halt ? HDR_TYPE_HALT : HDR_TYPE_SYNC) | 0x0C;  // 0x0C = 12 bytes
  *(uint32_t*)(&packet.buf[1]) = packet.count;
  *(uint32_t*)(&packet.buf[5]) = timestamp_us;
  *(uint32_t*)(&packet.buf[9]) = dropped_count;
  Serial.write((uint8_t*)(&packet.buf[0]), 13);
}


void finalize_transmission() 
{
  if (result_ready) {
    packet.append(adc_val);
  }
  if (packet.pos > 1) {
    send_data_packet();
    packet.count++;
  }
  send_sync_packet(true);   // send HALT packet?
}
*/
void setup() 
{
  Serial.begin(250000);

  delay(2000);
  Serial.println("Hello accelo");

  //init_pin_for_ADC_in();
  init_pin_for_CLK_out();
  cfg.fclk = init_GCLK5(cfg.clk_div, cfg.clk_divsel, true);  

  Serial.print("fclk = ");
  Serial.println(cfg.fclk);

}

void loop() 
{
  process_serial_buffer();
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
  } else if (data_in == 'H') {
    if (cfg.started) {
      //stop_ADC();
      cfg.started = false;
      //finalize_transmission();
    }
  } else if (!cfg.started) {
    if (data_in == 'R') {
      cfg.started = true;
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
        cfg.fclk = init_GCLK5(cfg.clk_div, cfg.clk_divsel, true);  
      }
    } else if (data_in == 'A') {
      char const ask_opt = Serial.read();
      if (ask_opt == 'F') {
        Serial.println(cfg.fclk);        
      } else if (ask_opt == 'D') {
        Serial.println(cfg.clk_div);
      } else if (ask_opt == 'M') {
        Serial.println((int)cfg.clk_divsel);
      }
    }
  }
}
