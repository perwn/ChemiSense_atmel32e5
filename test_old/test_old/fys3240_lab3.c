/* 
   Author       : Bent Furevik
   Company      : University of Oslo
   File name    : fys3240_lab3.c
   Date         : 23.12.2011
   Project      : FYS3240 Lab3
   Function     : Precode for a microcontrollerprogram interfacing the ADC
*/


#define F_CPU 32000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "fys3240_lcd.h"
#include "fys3240_led.h"
#include "fys3240_adbc.h"


// Interrupt Service Routine for handling the LCD 
ISR(LCD_INT_vect)
{	
  lcd_for_adc_callback();	
}

// Interrupt Service Routine for handling the ADC conversion
ISR(ADCB_CH0_vect)
{
  adcb_callback();		
}
	

int main(void)
{
  
  init_32M_clock();	
  init_lcd_interrupts();
  init_lcd();
  init_buttons_and_leds();
  ADCBInit();
	

  //Velger forskjellig sensor avhengig av tastetrykk
  while(1){

    ADCB.CH0.CTRL |= 0x80; //Starer en konvertering
	   
    uint8_t buttonInput = getButtonInput();
    if (buttonInput == 0b00001110) {
      ADCB.CH0.CTRL = 0x01;
      ADCB.CH0.MUXCTRL = 0b00010000; //Light sensor
      put_on_leds(0b00000001);
    }	
		
    if (buttonInput == 0b00001101) {
      ADCB.CH0.CTRL = 0x00; 
      ADCB.CH0.MUXCTRL = 0x00; //Temperatur sensor
      put_on_leds(0b00000010);
    }			

    if (buttonInput == 0b00001011) {
      ADCB.CH0.CTRL = 0x01;
      ADCB.CH0.MUXCTRL = ADC_CH_MUXINT0_bm; //Potentiometer
      put_on_leds(0b00000100);
    }		
  }
}
