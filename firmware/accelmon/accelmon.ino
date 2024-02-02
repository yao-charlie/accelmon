#include <Adafruit_NeoPixel.h>
#include <RingBuf.h>
#include <FlashStorage.h>
#include "packetcontainer.h"
#include "ADXL1005.h"
#include "KX134.h"

#define BOARD_ID_MASK 0x00FFFFFF

volatile bool data_ready;
volatile uint32_t dropped_count;
volatile uint32_t timestamp_curr;
volatile uint32_t timestamp_prev;
volatile bool tach_edge;
volatile bool tach_state; 

RingBuf<uint16_t, 512> adc_val;

bool is_running;
uint8_t tach_blink_counter;

typedef struct {
  int16_t  skip_count;
  uint32_t n;    // count
  float    mu;   // mean
  float    M2;   // sum of squares diff from mean 
  uint32_t max_T;
  uint32_t min_T;
  void update(uint32_t new_interval_n)
  {
    float const new_interval = (float)new_interval_n;

    // Welfords algorithm https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    if (skip_count == 0) {
      n++;
      float const delta = new_interval - mu;
      mu += delta / (float)n;
      M2 += delta * (new_interval - mu );  // use updated mu for "delta2"
      if (new_interval_n > max_T) {
        max_T = new_interval_n;
      } else if (new_interval_n < min_T) {
        min_T = new_interval_n;
      }
    } else {
      skip_count--;
      if (skip_count == 0) {
        mu = new_interval;
        n = 1;
      }
    }
  }
  void reset(int16_t const skips=2) 
  {
    skip_count = skips;
    max_T = 0;
    min_T = UINT32_MAX;
    M2 = 0.0;
    mu = 0.0;
    n = 0;
  }
} timing_statistics_t;
timing_statistics_t timing_stats;


uint32_t serial_read_uint32();
void process_serial_buffer();
void halt_and_blink_until_reset(int8_t r, int8_t g, int8_t b);
void reset_for_data_collection();

void ADC_Handler() 
{
  timestamp_curr = micros();
  PORT->Group[PORTA].OUTTGL.reg = (1 << ADXL1005_ADC_CONV_IOPIN); 
  if (!adc_val.push(0x0FFF & ADC->RESULT.reg)) {
    dropped_count++;
  }
  ADC->INTFLAG.bit.RESRDY = 1;  // write a bit to clear interrupt
}

// uses the Arduino implementation for the EIC, which handles clearing the flag (see WInterrupts.c)
void data_ready_ISR()
{
  timestamp_prev = timestamp_curr;
  timestamp_curr = micros();
  
  if (data_ready) { // not cleared
    dropped_count++;
  }
  tach_state = digitalRead(TACH_OPEN_COLLECTOR_PIN);
  data_ready = true;
}

void tach_edge_ISR() 
{
  tach_edge = true;
}

// create a pixel strand with 1 pixel on PIN_NEOPIXEL
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

// create an accelerometer
//ADXL1005 accel;
KX134 accel(data_ready_ISR, KX134_DRDY_ARDUINO_PIN);

FlashStorage(board_id_store, uint32_t);
uint32_t board_id;


void setup() 
{
  is_running = false;
  data_ready = false;
  tach_edge = false;
  tach_blink_counter = 0;
  pinMode(TACH_OPEN_COLLECTOR_PIN, INPUT_PULLUP);
  pinMode(TACH_INDICATOR_LED_PIN, OUTPUT);
  digitalWrite(TACH_INDICATOR_LED_PIN, 0);

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
  if (is_running && data_ready) {
    uint16_t const tach_bit = tach_state ? 0x8000 : 0;
    //uint16_t const tach_bit = tach_edge ? 0x8000 : 0;
    tach_edge = false;  // reset it 

    auto const data = accel.process();  // 16us
    uint32_t const interval_us = timestamp_curr > timestamp_prev ? 
        timestamp_curr - timestamp_prev : (0xFFFFFFFF - timestamp_prev) + timestamp_curr;
    data_ready = false;

    // assume that the minimum sample rate is > 32Hz (<32768us period)
    //if (interval_us > 0x00007FFF) {
    //  halt_and_blink_until_reset(64,0,0);
    //}
    uint16_t const interval_with_flag = (0x7FFF & interval_us) | tach_bit;

    if (adc_val.push(interval_with_flag)) {
      for (int16_t i = 0; i < data.count; ++i) {
        if (!adc_val.push(data.buf[i])) {
          dropped_count++;
        }
      }
    } else {
      dropped_count += data.count;
    }

    timing_stats.update(interval_us);
  }

  digitalWrite(TACH_INDICATOR_LED_PIN, digitalRead(TACH_OPEN_COLLECTOR_PIN));

  uint16_t sample;
  while (adc_val.lockedPop(sample)) {
    if (packet.append_sample(sample)) {
      Serial.write(packet.buffer(), packet.byte_count());
      packet.reset();
    }        
    if ((packet.max_packets > 0) && (packet.sample_count >= packet.max_packets)) {
      is_running = false;
      accel.stop();
      Serial.write(packet.buffer(), packet.byte_count());
    }
  }
}

void halt_and_blink_until_reset(int8_t r, int8_t g, int8_t b)
{
  accel.stop(); // disable interrupts
  while (1) {
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
    delay(500);
    if (Serial.available() > 0) {
      char const data_in = Serial.read();
      if (data_in == 'Z') {
        break;
      }
    }

    pixels.clear();
    pixels.show();
    delay(500);
    if (Serial.available() > 0) {
      char const data_in = Serial.read();
      if (data_in == 'Z') {
        break;
      }
    }
  }

  NVIC_SystemReset();

}

void reset_for_data_collection()
{
  dropped_count = 0;
  data_ready = false;
  packet.reset();
  packet.sample_count = 0;
  timing_stats.reset(2);
  adc_val.clear();
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
//  X : dropped count
//  U : mean sample time
//  V : sample time variance
//  R : max T
//  S : min T
//  N : n for mean 
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
// ask/resp: A---E--HIJK--NO-QRST--W-YZ
void process_serial_buffer()
{
  char const data_in = Serial.read();
  if (data_in == 'Z') {
    NVIC_SystemReset();
  } else if ((data_in == 'H') && (is_running)) {
      is_running = false;
      accel.stop();
      
      //detachInterrupt(digitalPinToInterrupt(TACH_OPEN_COLLECTOR_PIN));

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

      //attachInterrupt(digitalPinToInterrupt(TACH_OPEN_COLLECTOR_PIN), tach_edge_ISR, FALLING);

      reset_for_data_collection();
      is_running = true;
      accel.start();
      timestamp_prev = micros();    // rough starting point
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
      } else if (ask_opt == 'X') {
        count = packet.write_resp(RESP_TYPE_DROPPED_COUNT, dropped_count);
      } else if (ask_opt == 'U') {
        count = packet.write_resp(RESP_TYPE_T_MEAN, timing_stats.mu);
      } else if (ask_opt == 'V') {
        float const unbiased_sample_variance = timing_stats.M2 / (float)(timing_stats.n - 1);
        count = packet.write_resp(RESP_TYPE_T_VARIANCE, unbiased_sample_variance);
      }  else if (ask_opt == 'R') {
        count = packet.write_resp(RESP_TYPE_T_MAX, timing_stats.max_T);
      } else if (ask_opt == 'S') {
        count = packet.write_resp(RESP_TYPE_T_MIN, timing_stats.min_T);
      } else if (ask_opt == 'N') {
        count = packet.write_resp(RESP_TYPE_T_N, timing_stats.n);
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
