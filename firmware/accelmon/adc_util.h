#ifndef _adc_util_h
#define _adc_util_h

#include <cstdint>

void init_ADC(uint16_t genclk_id, uint8_t adc_prescaler = 0); 
void start_ADC();
void stop_ADC();
void reset_ADC();


#endif // _adc_util_h
