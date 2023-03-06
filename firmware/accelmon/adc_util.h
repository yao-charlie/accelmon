#ifndef _adc_util_h
#define _adc_util_h

#include <cstdint>

void init_ADC(uint16_t genclk_id); 
void start_ADC();
void stop_ADC();
void reset_ADC();


#endif // _adc_util_h
