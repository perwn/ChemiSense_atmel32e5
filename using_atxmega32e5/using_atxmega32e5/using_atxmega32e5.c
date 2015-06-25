#ifndef F_CPU
#define F_CPU 32000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "test.h"

static volatile uint8_t echo_char = 42;

void toggle(PORT_t *io, uint8_t pin_bm){
	io->OUTTGL = pin_bm;
}

int main(void){
	init_oscillator();
	//initButton();
	initLeds();
	//ADCBInit();
	// enable clock out on port PC7
//	PORTCFG.CLKOUT = (PORTCFG.CLKOUT & ~PORTCFG_CLKOUTSEL_gm) | PORTCFG_CLKOUT_PC7_gc;
	// set PC7 as output
//	PORTC.DIRSET = PIN7_bm;

//	init_usart();
	_delay_ms(500);                  // waiting 500ms
	PORTD.OUTSET    =   PIN0_bm;     // setting bit in the new way
	PORTD.OUTSET    =   PIN2_bm;
	_delay_ms(500);                  // waiting 500ms
	PORTD.OUTCLR    =   PIN0_bm;
	PORTD.OUTCLR    =   PIN2_bm;

	PORTA.DIR = ~(1<<0) ; // All out save to measure Solar Cell.
	PORTA_PIN0CTRL = 7 ;
	
	ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc; // Gain = 1, Single Ended
	ADCA.CH0.MUXCTRL =  ADC_CH_MUXINT0_bm //0x08 // (Channel<<3);//(Channel<<3) ;
	ADCA.CH0.INTCTRL = 0 ; // No interrupt
/*
while(1){

	ADCB.CH0.CTRL |= 0x80; //Starer en konvertering
	
	ADCB.CH0.CTRL = 0x01;
		ADCB.CH0.MUXCTRL = 0b00010000; //Light sensor

		ADCB.CH0.CTRL = 0x00;
		ADCB.CH0.MUXCTRL = 0x00; //Temperatur sensor

		ADCB.CH0.CTRL = 0x01;
		ADCB.CH0.MUXCTRL = ADC_CH_MUXINT0_bm; //Potentiometer
}
*/

/*
	while (1) {
	if((PORTD.IN & PIN0_bm)){
		 //PORTD.OUTSET = PIN4_bm;
		toggle(&PORTD, PIN4_bm);
	}
	else {
		toggle(&PORTD, PIN4_bm);
	}
	if((PORTD.IN & PIN2_bm)){
		//PORTD.OUTSET = PIN5_bm;
		//_delay_ms( 500 );
		toggle(&PORTD, PIN5_bm);
	}
	else {
		toggle(&PORTD, PIN5_bm);
		
	}
	//PORTD.OUTCLR = PIN4_bm;
	//PORTD.OUTCLR = PIN5_bm;
	//_delay_ms(500);
	}
}
*/
// USART RX receive interrupt handler




ISR(USARTD0_RXC_vect) {
	echo_char = USARTD0.DATA;
}	

// Interrupt Service Routine for handling the LCD
ISR(LCD_INT_vect)
{
	//lcd_for_adc_callback();
}

// Interrupt Service Routine for handling the ADC conversion
ISR(ADCB_CH0_vect)
{
	adcb_callback();
}
