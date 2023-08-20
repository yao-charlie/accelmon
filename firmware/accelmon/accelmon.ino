#include <Adafruit_NeoPixel.h>
#include <RingBuf.h>
#include <FlashStorage.h>
#include "packetcontainer.h"
#include "ADXL1005.h"
#include "KX134.h"

#define BOARD_ID_MASK 0x00FFFFFF

volatile bool data_ready;
volatile uint32_t dropped_count;
RingBuf<uint16_t, 1024> adc_val;

uint32_t serial_read_uint32();
void process_serial_buffer();

void ADC_Handler() 
{
  PORT->Group[PORTA].OUTTGL.reg = (1 << ADXL1005_ADC_CONV_IOPIN); 
  if (!adc_val.push(0x0FFF & ADC->RESULT.reg)) {
    dropped_count++;
  }
  ADC->INTFLAG.bit.RESRDY = 1;  // write a bit to clear interrupt
}

// uses the Arduino implementation for the EIC, which handles clearing the flag (see WInterrupts.c)
void data_ready_ISR()
{
  if (data_ready) { // not cleared
    dropped_count++;
  }
  data_ready = true;
}

// create a pixel strand with 1 pixel on PIN_NEOPIXEL
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

// create an accelerometer
//ADXL1005 accel;
KX134 accel(data_ready_ISR, KX134_DRDY_IOPIN);

FlashStorage(board_id_store, uint32_t);
uint32_t board_id;
bool is_running;

void setup() 
{
  is_running = false;
  data_ready = false;
  board_id = BOARD_ID_MASK & board_id_store.read();  // upper byte reserved for board type

  Serial.begin(921600);
  pixels.begin();
  
  delay(1000);

  pixels.setPixelColor(0, pixels.Color(0, 96, 0));
  pixels.show();

  delay(1000);

  if (!accel.init()) {
    pixels.setPixelColor(0, pixels.Color(96, 0, 0));
    pixels.show();
    while(1); // wait forever
  }

  delay(1000);
  pixels.clear();
  pixels.show();

}


void loop() 
{
  if (Serial.available() > 0) {
    process_serial_buffer();
  }

  // for digitial read (I2C or SPI) from KX134 
  // could probably write direct to packet here
  if (data_ready) {
    auto const data = accel.process();
    for (int16_t i = 0; i < data.count; ++i) {
      if (!adc_val.push(data.buf[i])) {
        dropped_count++;
      }
    }
    data_ready = false;
  }

  uint16_t sample;
  while (adc_val.lockedPop(sample)) {
    if (packet.append_sample(sample)) {
      Serial.write(packet.buffer(), packet.byte_count());
      packet.reset();
    }        
    if ((packet.max_packets > 0) && (packet.sample_count >= packet.max_packets)) {
      accel.stop();
      is_running = false;
      Serial.write(packet.buffer(), packet.byte_count());
      packet.reset();
    }
  }
}

int serial_read_uint32(uint32_t& val)
{
  uint8_t buf[4];
  int const rlen = Serial.readBytes(reinterpret_cast<char*>(&buf[0]), 4);
  val = *reinterpret_cast<uint32_t*>(buf);
  return rlen;
}

// Message format
// H : halt, send last packet then halt packet
// R# : start with max packets count as string (0=no max), streams packets HDR_DATA | timestamp_ms | T .. | P ..
// C : configure
//  general
//  B# : board ID (24 bits -- upper byte reserved for accelerometer type)
//  KX134
//  F# : output data rate (4-bit code, see docs)
//  G# : g range selection 0=8g, 1=16g, 2=32g, 3=64g
//  ADXL1005
//  D# : clock divisor (0-255 for direct, 0-8 for pow2)
//  M# : clock divisor mode (0=direct, 1=pow2)
//  P# : ADC prescaler (2^(x + 4))
//  L# : ADC sample length (half-clock cycles)
// A : ask
//  general
//  B : Accelerometer type [24:31] | board ID [0:23]
//  C : sample count
//  KX134
//  F : output data rate (4-bit code)
//  G : g range selection 0=8g, 1=16g, 2=32g, 3=64g
//  ADXL1005
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
  } else if ((data_in == 'H') && (is_running)) {
      accel.stop();
      is_running = false;

      if (dropped_count == 0) {
        pixels.clear();
      } else {
        pixels.setPixelColor(0, pixels.Color(96, 0, 0));
      }
      pixels.show();

  } else if (!is_running) {
    if (data_in == 'R') {
      uint32_t cfg_val = 0;
      if (serial_read_uint32(cfg_val) == 4) {
        packet.max_packets = cfg_val >= 0 ? cfg_val : 0;
      } else {
        packet.max_packets = 0;
      }

      pixels.setPixelColor(0, pixels.Color(0, 0, 96));
      pixels.show();
     
      packet.sample_count = 0;
      is_running = true;
      dropped_count = 0;
      accel.start();
    } else if (data_in == 'C') {
      char const cfg_key = Serial.read(); 
      uint32_t cfg_val = 0;
      if (serial_read_uint32(cfg_val) == 4) {
        if (cfg_key == 'B') {
          board_id = cfg_val & BOARD_ID_MASK;
          board_id_store.write(board_id);
        } else {
          accel.set(cfg_key, cfg_val);
        }
      }
    } else if (data_in == 'A') {
      char const ask_opt = Serial.read();
      uint8_t count = 0;
      if (ask_opt == 'C') {
        count = packet.write_resp(RESP_TYPE_SAMPLE_COUNT, packet.sample_count);                          
      } else if (ask_opt == 'B') {
        count = packet.write_resp(RESP_TYPE_ID, board_id | (accel.type_id() << 24));
      } else {
        auto const resp = accel.get(ask_opt);
        count = packet.write_resp(resp.type, resp.val); // can be none on fall through
      }
      if (count > 0) {
        Serial.write(packet.buffer(), count);
      }
    }
  }
}
