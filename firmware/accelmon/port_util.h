#ifndef _port_util_h
#define _port_util_h

void init_pin_for_ADC_in();           // hardcoded for A0/PA02
void init_pin_for_CLK_out();          // DEBUG ONLY, hardcoded to PA11 (SPI SCK)
void init_pin_for_D_out(int iopin);
void init_pin_for_D_in(int iopin);

#endif // _port_util_h
