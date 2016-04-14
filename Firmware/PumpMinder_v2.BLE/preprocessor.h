#pragma once

#include <stdint.h>

#define WPS_TIMEOUT_VALUE		50
#define DEBUG_BREAK				1

#ifdef DEBUG_BREAK
#define BREAK_OUT()		asm("bkpt 255")
#else
#define BREAK_OUT()
#endif


extern uint32_t seconds_with_water;
extern uint32_t seconds;
