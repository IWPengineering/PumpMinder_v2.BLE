#pragma once

#include "preprocessor.h"
#include <stdint.h>
#include <stdbool.h>
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"

#define PULSE_WIDTH_THRESHOLD		1000

typedef struct 
{
	uint32_t sense_channel;
	uint32_t control_channel;
} wps_channel_t;

void wps_init(const wps_channel_t *p_channel_list, uint8_t channel_num);
bool is_water_present(const wps_channel_t *p_channel);
