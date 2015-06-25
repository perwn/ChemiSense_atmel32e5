/* 
Author       : Bent Furevik
Company      : University of Oslo
File name    : fys3240_lab2.c
Date         : 23.12.2011
Project      : FYS3240 Lab2
Function     : Precode for a microcontrollerprogram interfacing the LCD
*/

#define DEVICE ATXMEGA128B1
#define F_CPU 32000000
#include <avr/io.h>
#include <util/delay.h>
#include "fys3240_lcd.h"

{
	init_lcd();
	init_buttons_and_leds();
	init_32M_clock();	
	
	while(1)
	{
		_delay_ms(50);		
		if(flash_on_keypress() == -1)
	}
	
void,.........