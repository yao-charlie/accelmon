#ifndef _gclk_util_h
#define _gclk_util_h

#include <cstdint>

enum DIVSEL_T {GCLK_DIVSEL_DIRECT=0, GCLK_DIVSEL_POW2=1};

uint32_t init_GCLK(int clk_id, uint8_t clk_div, DIVSEL_T clk_divsel, bool en_gclk_io = false);

#endif // _gclk_util_h