/* 
   Author       : Bent Furevik
   Company      : University of Oslo
   File name    : fys3240_adc.h
   Date         : 23.12.2011
   Project      : FYS3240 Lab3
   Function     : Precode for a microcontrollerprogram interfacing the ADC
*/

/* We will reuse the code from lab 1 and 2. */

#include <avr/pgmspace.h>

#include <stddef.h>


volatile uint8_t val;
int called1;
int called2;
volatile uint8_t tmpVal1;
volatile uint8_t tmpVal2;

void ADCBInit(void) 
{


  //Conversion Complete mode is enabled and is assigned high priority
  ADCB.CH0.INTCTRL = ADC_CH_INTLVL1_bm | ADC_CH_INTLVL0_bm; // 0x03
	
  // Ensure the conversion complete flag is cleared
  ADCB.INTFLAGS = ADC_CH0IF_bm; // 0x01
	
  //Enable the ADC
  ADCB.CTRLA = ADC_ENABLE_bm; // 0x01
	
  //Single conversions and run in Signed mode.  The ADC is configured to display a 
  //8-bit, right justified result.  
  ADCB.CTRLB = 0b00110100; 
	
  // Configure PB1 as input; write '0' to pin position in Direction register
  // the potentiometer is connected to PORTB, Pin 1 on the Xmega
  PORTB.DIR &= ~0x02;
	
  //The ADC is placed in Single-Ended mode
  ADCB.CH0.CTRL = ADC_CH_INPUTMODE0_bm; // 0x01
	
  //The reference voltage to the ADC is set to Vcc/1.6
  ADCB.REFCTRL = ADC_REFSEL0_bm; // 0x10
	
  ADCB.REFCTRL |= 0x01; //Enable temp sensor

  //Disable event trigging
  ADCB.EVCTRL = 0x00;
	
  // Start a conversion using ADC Control A Register
  ADCB.CTRLA |= ADC_CH0START_bm; // 0x04



	

}

void adcb_callback(void)
{
	
  val = ADCB.CH0.RES;
}

void lcd_for_adc_callback(void)
{

  LCD_CLEAN;
  show_value(val);
	
	
}

static uint8_t conv_ascii(uint8_t val)
{
	
  //Fullført i forrige lab
}

void show_value(uint8_t val){
  lcd_write_int(val, 12);
}
