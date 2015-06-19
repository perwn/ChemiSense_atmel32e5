/* 
Author       : Bent Furevik
Company      : University of Oslo
File name    : fys3240_lcd.c
Date         : 23.12.2011
Project      : FYS3240 Lab1
Function     : Precode for the methods used to interface the LCD on the XMEGA-board
*/
#define F_CPU 32000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
//! 14-segment with 4 commons terminals, start segment
#define START_14SEG_4C 12
#define HZ_4_BLINKRATE 0b00001000
#define HZ_2_BLINKRATE 0b00001001
#define HZ_1_BLINKRATE 0b00001010
#define HZ_05_BLINKRATE 0b00001011
#define LCD_CLEAN LCD.CTRLA ^= 0b00000100;



#define LCD_BACKLIGHT_TOGGLE (PORTE.OUT ^= (1 << 5))

volatile uint8_t count_callback = 0x00;

// setter hyppigheten på blinking av LCD
void lcd_set_blink(uint8_t rate){
	LCD_CTRLD = rate;
}

// skriver en char data på posisjon pos på LCD
void lcd_write_char_pos(const uint8_t data, uint8_t pos) 
{
	LCD.CTRLG = LCD_TDG_14S_4C_gc | pos;
	LCD.CTRLH = data;
}

// skriver en CHAR på LCD 1. posisjon
void lcd_write_char(const uint8_t data) 
{
	LCD.CTRLG = LCD_TDG_14S_4C_gc | START_14SEG_4C;
	LCD.CTRLH = data;
}

// en metode for å skrive ints på fra og med posisjonen pos
void lcd_write_int(int i, int pos) 
{
	int j = 0;
	char buffer[16];
	itoa(i, buffer, 10);
	while(j < strlen(buffer)) {
		lcd_write_char_pos(buffer[j++], pos);
		pos += 4;
	}		
}

void init_lcd_interrupts(void)
{
	sei();	
	//enable all interrupt levels 
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;

	

}

void init_lcd(void)
{
	//enable rtc clock to 1kHz is the lowest clock rate avalible and is fast enough for our use.
	CLK.RTCCTRL =  CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm; // 0x01
	//Ensure the lcd perifiral clock is running
	PR.PRGEN &= ~PR_LCD_bm; // ~0x80
	//clear the display memory
	LCD.CTRLA = LCD_CLRDT_bm;
	//framerate set to 64 khz
	LCD.CTRLB = LCD_PRESC_bm | LCD_CLKDIV1_bm | LCD_CLKDIV0_bm | LCD_LPWAV_bm;
	//enable all segment drivers
	LCD.CTRLC = LCD_PMSK_gm;
	//How often a interrupt occures
	LCD.INTCTRL = LCD_XIME2_bm | LCD_XIME1_bm | LCD_XIME0_bm | LCD_FCINTLVL_gm;
	//Disable hardware blink
	LCD.CTRLD = ~LCD_BLINKEN_bm;
	//Enable LCD, enable lcd segments
	LCD.CTRLA |= LCD_ENABLE_bm | LCD_SEGON_bm; // 0x82
	
	//Velger konstrast
	//LCD.CTRLF = 31;
	
}	
// inkrementerer og dekrementerer en counter ved bruk av CS1 og CS2 og skriver til LCD
/*void lcd_callback_lab2(void)
{
	//deaktiverer interrupts
	cli();
	uint8_t buttonInput = getButtonInput();
		
    if (buttonInput == 0b00001101) {
      //Increment
      count_callback++;
	  lcd_write_int(count_callback, 12);			
    }
			
    else if (buttonInput == 0b00001011) {
      //Decrement
      count_callback--;
	  lcd_write_int(count_callback, 12);
			
    }
	//aktiverer innterupts
	sei();
		
}*/
// blir kalt på av en interrupt 
