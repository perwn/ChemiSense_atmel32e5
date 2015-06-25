/*
 * Sensors.c
 *
 * Created: 6/18/2015 5:27:26 PM
 *  Author: ChemiSenseUser
 */ 


#include <avr/io.h>
#include "Sensors.h"


int main(void)
{
	init_oscillator();
	//sysclk_init();
	adc_init();
    while(1)
    {
     uint16_t result;
     adc_enable(&MY_ADC);
     adc_start_conversion(&MY_ADC, MY_ADC_CH);
     adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);
     result = adc_get_result(&MY_ADC, MY_ADC_CH);
    }
}