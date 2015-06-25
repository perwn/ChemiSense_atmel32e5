/* 
Author       : Bent Furevik
Company      : University of Oslo
File name    : fys3240_lcd.c
Date         : 23.12.2011
Project      : FYS3240 Lab1
Function     : Precode for the methods used to interface the LCD on the XMEGA-board
*/
#define F_CPU 32000000
#include "fys3240_led.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//! 14-segment with 4 commons terminals, start segment
#define START_14SEG_4C 12

// #define LCD_HOME 
// #define LCD_CLEAR
#define LCD_BACKLIGHT_TOGGLE (PORTE.OUT ^= (1 << 5))


void lcd_set_blink(uint8_t rate){
	/**/
}

void lcd_write_char_pos(const uint8_t data, uint8_t pos) 
{
	/**/
}

void lcd_write_char(const uint8_t data) 
{
	
	LCD.CTRLG = LCD_TDG_14S_4C_gc | FIRST_14SEG_4C;

	LCD.CTRLH = data;
}

void lcd_write_int(int i) 
{
	/**/
}

void init_lcd(void)
{
	//
	PR.PRGEN &= ~PR_LCD_bm; // ~0x80
	
	//enable lcd and segments
	LCD.CTRLA = LCD_ENABLE_bm | LCD_SEGON_bm; // 0x82
	//set framerate
	LCD.CTRLB = LCD_PRESC_bm | LCD_CLKDIV1_bm | LCD_CLKDIV0_bm | LCD_LPWAV_bm;
	//enable all segment drivers
	LCD
	
	CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm; // 0x01
	
	LCD.INTCTRL = LCD_XIME2_bm | LCD_XIME1_bm | LCD_XIME0_bm | LCD_FCINTLVL0_bm | LCD_FCINTLVL1_bm; 
	LCD.INTFLAGS |= LCD_FCIF_bm; // 0x01
	LCD_BACKLIGHT_TOGGLE;
//	PORTE.DIR = 0x20; // set PE5 to output

//	PORTE.OUTSET = 0x20; // turn backlight fully-on by setting bit 5

}
