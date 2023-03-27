#include "adc_util.h"
#include <Arduino.h>

void init_ADC(uint16_t const genclk_id, uint8_t const adc_prescaler /* = 0*/, uint8_t const adc_samplen /* = 0*/) 
{
  // select the generic clock generator used as clock source GCLK_CLKCTRL_GEN_GCLK5
  GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_CLKEN | genclk_id | GCLK_CLKCTRL_ID_ADC);
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Select reference, internal VCC/2
  ADC->REFCTRL.reg |= ADC_REFCTRL_REFSEL_INTVCC1; // VDDANA/2, combine with gain DIV2 for full VCC range

  // Average control 1 sample, no right-shift
  // ADC->AVGCTRL.reg |= ADC_AVGCTRL_ADJRES(0) | ADC_AVGCTRL_SAMPLENUM_1;

  // Sampling time = (SAMPLEN+1)*(CLK_ADC/2), adds half clock-cycles
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(adc_samplen);
  
  // Input control: set gain to div by two so ADC has measurement range of VCC, no diff measurement so set neg to gnd, pos input set to pin 0 or A0
  // ADC_INPUTCTRL_GAIN_DIV2 |   
  ADC->INPUTCTRL.reg |= ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;
  while (ADC->STATUS.bit.SYNCBUSY);
  
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER(adc_prescaler) | ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_FREERUN; 
  while (ADC->STATUS.bit.SYNCBUSY);

  // setup the interrupt
  ADC->INTENSET.reg |= ADC_INTENSET_RESRDY; // enable ADC interrupt on result ready
  while (ADC->STATUS.bit.SYNCBUSY);

  NVIC_SetPriority(ADC_IRQn, 3); //set priority of the interrupt
  NVIC_EnableIRQ(ADC_IRQn); // enable ADC interrupts
  
}

void start_ADC() 
{
  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY);
  
  ADC->SWTRIG.reg = ADC_SWTRIG_START; // Start ADC conversion
  while(ADC->STATUS.bit.SYNCBUSY);    // Wait for sync
}

void stop_ADC() 
{
  ADC->CTRLA.bit.ENABLE = 0;
  while (ADC->STATUS.bit.SYNCBUSY);
}

void reset_ADC()
{
  ADC->CTRLA.bit.SWRST = 1;
  while (ADC->STATUS.bit.SYNCBUSY);
}
