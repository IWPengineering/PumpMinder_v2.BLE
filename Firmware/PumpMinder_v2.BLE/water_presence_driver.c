#include "water_presence_driver.h"
#include "app_error.h"

nrf_drv_timer_t wps_timer = NRF_DRV_TIMER_INSTANCE(1);

void wps_timer_handler(nrf_timer_event_t event_type, void *p_context)
{
}

void wps_init(const wps_channel_t *p_channel_list, uint8_t channel_num)
{
	uint32_t err_code;
	
	// Initialize timer
	
	nrf_drv_timer_config_t config = {
		.frequency = NRF_TIMER_FREQ_500kHz,
		.mode = NRF_TIMER_MODE_TIMER,
		.bit_width = NRF_TIMER_BIT_WIDTH_16,
		.interrupt_priority = 8
	};
	
	err_code = nrf_drv_timer_init(&wps_timer, &config, wps_timer_handler);
	APP_ERROR_CHECK(err_code);
	
	// Initialize channels
	
	for (int i = 0; i < channel_num; i++)
	{
		// init
		nrf_gpio_cfg_output(p_channel_list[i].control_channel);
		nrf_gpio_cfg_input(p_channel_list[i].sense_channel, NRF_GPIO_PIN_NOPULL);
		
		nrf_gpio_pin_clear(p_channel_list[i].control_channel);
	}
}

bool is_water_present(const wps_channel_t *p_channel)
{
	nrf_drv_timer_enable(&wps_timer); // start the timer
	nrf_gpio_pin_set(p_channel->control_channel); // turn on the WPS
	
	bool pin_state = nrf_gpio_pin_read(p_channel->sense_channel);
	
	// Wait for the edge
	uint16_t timeout_val = 0;
	while (nrf_gpio_pin_read(p_channel->sense_channel) == pin_state)
	{
		timeout_val++;
		if (timeout_val > TIMEOUT_TICKS)
		{
			break;
		}
	}
	
	// Once we see an edge, record the time
	uint32_t prev_time = nrf_drv_timer_capture(&wps_timer, NRF_TIMER_CC_CHANNEL0);
	
	// Wait for the other edge
	timeout_val = 0;
	while (nrf_gpio_pin_read(p_channel->sense_channel) != pin_state)
	{
		timeout_val++;
		if (timeout_val > TIMEOUT_TICKS)
		{
			break;
		}
	}
	
	// Record the new time
	uint32_t cur_time = nrf_drv_timer_capture(&wps_timer, NRF_TIMER_CC_CHANNEL0);
	
	// Turn off the timer, turn off the WPS
	nrf_drv_timer_disable(&wps_timer);
	nrf_gpio_pin_clear(p_channel->control_channel);
	
	// calculate our pulse width
	uint32_t pulse_width;
	if (cur_time >= prev_time)
	{
		pulse_width = cur_time - prev_time;
	}
	else
	{
		pulse_width = (cur_time + 0xFFFF) - prev_time;
	}
	
	return (pulse_width <= PULSE_WIDTH_THRESHOLD);
}
