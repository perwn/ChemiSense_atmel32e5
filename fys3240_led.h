/* 
   Author       : Bent Furevik
   Company      : University of Oslo
   File name    : fys3240_led.h
   Date         : 23.12.2011
   Project      : FYS3240 Lab1 Solution
   Function     : Code for the methods used to control the LEDS on the XMEGA-board
*/

#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>

uint8_t count = 0x00;



void init_32M_clock(void)
{
  //For å få tilgang til den interne 32MHz klokka må vi sette riktig verdi i CCP registeret.
  CCP = CCP_IOREG_gc;
	
  //Aktiver 32MHz klokke
  OSC.CTRL|=OSC_RC32MEN_bm;
		
  //Venter på at klokka blir klar     
  while (!(OSC_STATUS & OSC_RC32MRDY_bm)) 
    {		
    }
	
  //Velger 32MHz klokka som aktiv klokke.
  CLK.CTRL = CLK_SCLKSEL_RC32M_gc;		
}

void put_on_leds(uint8_t value)
{
  //Setter de fire øverse bittene.
  PORTB.OUT = value<<4;	
}

uint8_t getButtonInput(void)
{
  //Får inn button input og maskerer vekk de fire øverste bittene
  return (PORTE.IN & 0x0f);
}

int flash_on_keypress(void)
{
  uint8_t buttonInput = getButtonInput();

  //Sjekker om CS3 er trykket
  if (buttonInput == 0b00000111)
    {
      put_on_leds(0b11110111); //Lyser LED4 og avbryter while løkken i c filen
      _delay_ms(100);
      put_on_leds(0b11111111);
    return -1;		
  }
			
  put_on_leds(buttonInput);	//Hvis ikke, sender knappetrykk til leds
  return 1;
	
}

void led_counter(void)
{
	
	
	
  while(1) 
    {
		
    uint8_t buttonInput = getButtonInput();
		
    if (buttonInput == 0b00001101) {
      //Increment
      count++;
			
    }
		
		
    else if (buttonInput == 0b00001011) {
      //Decrement
      count--;
			
    }
    else if (buttonInput == 0b00000111) {
      //Quit;
			
      put_on_leds(0b11110111);
			
      break;
	
    }
    put_on_leds(~count);
    _delay_ms(7);
		
		
  }	
}

void init_buttons(void)
{
  PORTE.DIR = 0x00; //Setter buttons til å være innganger
  //Knappene er aktivt lave, kobler knappene til pullup-motstandere
  PORTE.PIN0CTRL =  PORT_OPC_PULLUP_gc;
  PORTE.PIN1CTRL =  PORT_OPC_PULLUP_gc;
  PORTE.PIN2CTRL =  PORT_OPC_PULLUP_gc;
  PORTE.PIN3CTRL =  PORT_OPC_PULLUP_gc;
	
}

void init_leds(void)
{
  //Setter led-lampene til å være utganger (De fire MSBene)
  PORTB.DIR = 0b11110000;
	
}

void init_buttons_and_leds(void)
{
  init_buttons();
  init_leds();
}
