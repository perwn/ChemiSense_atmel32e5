/*
 * final_board_first.c
 *
 * Created: 6/22/2015 3:58:01 PM
 *  Author: ChemiSenseUser
 */ 

#define F_CPU 32000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "final_board_first.h"

static volatile uint8_t echo_char = 42;

int main(void)
{
	init_oscillator();
	init_leds();
	
	// enable clock out on port PC7
	PORTCFG.CLKOUT = (PORTCFG.CLKOUT & ~PORTCFG_CLKOUTSEL_gm) | PORTCFG_CLKOUT_PC7_gc;
	// set PC7 as output
	PORTA.DIRSET = PIN4_bm;
	
	init_usart();
	init_interrupts();
	
    while(1)
    {
		usart_write_string("test\n");
		led0_on();
		led1_off();	
		_delay_ms(1000);
		led0_off();
		led1_on();
		_delay_ms(1000);
        //TODO:: Please write your application code 
    }
}

// USART RX receive interrupt handler
ISR(USARTD0_RXC_vect) {
	echo_char = USARTD0.DATA;
}