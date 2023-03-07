#include "port_util.h"
#include <Arduino.h>

// ADC input setting 
//  + SAMD21: AIN[0]/PA02/pin 3
//  + Adafruit QTPy: Pin 0/A0
void init_pin_for_ADC_in()
{
  int const iopin = 2;
  PORT->Group[PORTA].DIRCLR.reg = (1 << iopin);     // input PA02
  PORT->Group[PORTA].PINCFG[iopin].bit.PMUXEN = 1;  // enable peripheral mux to define behaviour
  PORT->Group[PORTA].PMUX[iopin >> 1].reg = PORT_PMUX_PMUXE_B | PORT_PMUX_PMUXO_B;  // type B for ADC
}


// GCLK_IO
//  + SAMD21: PA11
//  + Adafruit QTPy: Pin 8/SCK
void init_pin_for_CLK_out()
{
  int const iopin = 11;
  PORT->Group[PORTA].DIRSET.reg = (1 << iopin);     // output PA11
  PORT->Group[PORTA].PINCFG[iopin].bit.PMUXEN = 1;  // enable peripheral mux to define behaviour
  PORT->Group[PORTA].PMUX[iopin >> 1].reg = PORT_PMUX_PMUXE_H | PORT_PMUX_PMUXO_H;  // type H for CLK
}

// digital out
//  + SAMD21: PA07
//  + Adafruit QTPy: Pin 7
void init_pin_for_D_out(int const iopin)
{
  PORT->Group[PORTA].DIRSET.reg = (1 << iopin);     // output PA07
  PORT->Group[PORTA].PINCFG[iopin].bit.PMUXEN = 0;  // disable peripheral mux
}
