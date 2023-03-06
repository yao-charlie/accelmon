#include "gclk_util.h"
#include <Arduino.h>

#define FCLK_CPU 48000000

// https://blog.thea.codes/understanding-the-sam-d21-clocks/

/*
  \brief Initialize GCLK from the CPU 48MHz PLL with a specified clock divider
*/
uint32_t init_GCLK(
  int const clk_id, 
  uint8_t const clk_div, 
  DIVSEL_T const clk_divsel, 
  bool const en_gclk_io /*= false*/) 
{

  GCLK->GENCTRL.reg = GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(clk_id);  // disable
  GCLK->CLKCTRL.reg = GCLK_GENCTRL_ID(clk_id);  //disable
  while (GCLK->STATUS.bit.SYNCBUSY);

  // input checking: can't bit shift 48000000 more than 24 bits and have > 0 fgclk
  // leave the clock disabled
  if ((clk_divsel == 1) && (clk_div > 24)) {
    return uint32_t(-1);
  }

  GCLK->GENDIV.reg = GCLK_GENDIV_ID(clk_id) | GCLK_GENDIV_DIV(clk_div);
  while (GCLK->STATUS.bit.SYNCBUSY);  

  // configure the generator of the generic clock, which is 48MHz clock; 
  // if DIVSEL = 0 --> fgclk = 48/div_factor MHz (or fgclk = 48MHz if div_factor = 0)
  // if DIVSEL = 1 --> fgclk = 48/(1 << div_factor) MHz
  uint32_t reg_val = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(clk_id);
  if (clk_divsel == GCLK_DIVSEL_POW2) {
    reg_val |= GCLK_GENCTRL_DIVSEL;
  }
  if (en_gclk_io) {
    reg_val |= GCLK_GENCTRL_OE;
  } 
  GCLK->GENCTRL.reg = reg_val;    
  while (GCLK->STATUS.bit.SYNCBUSY);  
  
  if (clk_divsel == GCLK_DIVSEL_DIRECT) {
    return clk_div > 1 ? (FCLK_CPU / uint32_t(clk_div)) : FCLK_CPU;
  } else if (clk_divsel == GCLK_DIVSEL_POW2) {
    return (FCLK_CPU >> (clk_div+1));
  } 
  return uint32_t(-1);      // error otherwise

} 