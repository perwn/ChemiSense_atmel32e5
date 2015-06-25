#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

static inline void init_oscillator() {
	// enable 32Mhz internal oscillator
	OSC.CTRL |= OSC_RC32MEN_bm;
	// wait for it to be stable
	while (!(OSC.STATUS & OSC_RC32MRDY_bm));
	// tell the processor we want to change a protected register
	CCP=CCP_IOREG_gc;
	// and start using the 32Mhz oscillator
	CLK.CTRL=CLK_SCLKSEL_RC32M_gc;
	// disable the default 2Mhz oscillator
	OSC.CTRL&=(~OSC_RC2MEN_bm);
	// enable 32kHz calibrated internal oscillator
	OSC.CTRL|= OSC_RC32KEN_bm;
	while (!(OSC.STATUS & OSC_RC32KRDY_bm));
	// set bit to 0 to indicate we use the internal 32kHz
	// callibrated oscillator as auto-calibration source
	// for our 32Mhz oscillator
	OSC.DFLLCTRL = OSC_RC32MCREF_RC32K_gc;
	// enable auto-calibration for the 32Mhz oscillator
	DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
}

void init_leds(){
	PORTC.DIRSET = PIN7_bm | PIN6_bm;
	PORTC.OUTCLR = PIN7_bm | PIN6_bm;	
}

void led0_on(){
	PORTC.OUTSET = PIN7_bm;
}
void led0_off(){
	PORTC.OUTCLR = PIN7_bm;
}
void led1_on(){
	PORTC.OUTSET = PIN6_bm;
}
void led1_off(){
	PORTC.OUTCLR = PIN6_bm;
}

static void usart_write(uint8_t data)
{
	USARTD0.DATA = data;
	if(!(USARTD0.STATUS & USART_DREIF_bm)) {
		while(!(USARTD0.STATUS & USART_TXCIF_bm)); // wait for TX complete
	}
	USARTD0.STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag
}

static void usart_write_string(char *text){
	
	PORTA.OUTSET = PIN0_bm;
	while(*text)
	{
		usart_write(*text++);
	}
	PORTA.OUTCLR = PIN0_bm;
}
// enable clock out on port PC7
	PORTCFG.CLKOUT = (PORTCFG.CLKOUT & ~PORTCFG_CLKOUTSEL_gm) | PORTCFG_CLKOUT_PC7_gc;
	// set PC7 as output
	PORTC.DIRSET = PIN7_bm;