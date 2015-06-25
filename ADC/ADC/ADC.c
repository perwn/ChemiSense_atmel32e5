/*
 * ADC.c
 *
 * Created: 6/15/2015 10:30:47 AM
 *  Author: ChemiSenseUser
 */ 

#define F_CPU 32000000UL
//#define BAUD 9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


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

static inline void init_usart() {
	// enable clock out on port PC7
	PORTCFG.CLKOUT = (PORTCFG.CLKOUT & ~PORTCFG_CLKOUTSEL_gm) | PORTCFG_CLKOUT_PC7_gc;
	// set PC7 as output
	PORTC.DIRSET = PIN7_bm;

	// set PD7 as output for TX0
	PORTD.DIRSET = PIN7_bm;
	PORTD.OUTSET = PIN7_bm;
	// remap USARTD0 to PD[7-4]
	PORTD.REMAP |= PORT_USART0_bm;
	// set baud rate 9600: BSEL=12, BSCALE=4
	// as found in table in
	// Atmel-42005-8-and-16-bit-AVR-Microcontrollers-XMEGA-E_Manual.pdf
	USARTD0.BAUDCTRLA = 12; // BSEL
	USARTD0.BAUDCTRLB = 4 << USART_BSCALE_gp; // BSCALE
	// disable 2X
	USARTD0.CTRLB = USARTD0.CTRLB & ~USART_CLK2X_bm;
	// enable RX and TX
	USARTD0.CTRLB = USARTD0.CTRLB | USART_RXEN_bm | USART_TXEN_bm;
	// enable async UART 8N1
	USARTD0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	USARTD0.CTRLC &= ~USART_SBMODE_bm;
	USARTD0.CTRLD = 0; // No LUT

	// set interrupt level for RX
	USARTD0.CTRLA = (USARTD0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
}

static inline void init_interrupts() {
	// Enable PMIC interrupt level low
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	// enable interrupts
	sei();
}


void ConfigurePin()
{
	PORTA.DIR = ~(1<<0) ; // All out save to measure Solar Cell.
	PORTA_PIN0CTRL = 7 ;
}

uint8_t ReadSignatureByte(uint16_t Address)
{
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	__asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}

uint16_t ReadADC(uint8_t Channel, uint8_t ADCMode) // Mode = 1 for single ended, 0 for internal
{
	if ((ADCA.CTRLA & ADC_ENABLE_bm) == 0)
	{
		ADCA.CTRLA = ADC_ENABLE_bm ; // Enable the ADC
		ADCA.CTRLB = (1<<4); // Signed Mode
		ADCA.REFCTRL = 0; // Internal 1v ref
		ADCA.EVCTRL = 0 ; // no events
		ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc ;
		ADCA.CALL = ReadSignatureByte(0x20) ; //ADC Calibration Byte 0
		ADCA.CALH = ReadSignatureByte(0x21) ; //ADC Calibration Byte 1
		//ADCA.SAMPCTRL = This register does not exist
		_delay_us(400); // Wait at least 25 clocks
	}
	ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADCMode ; // Gain = 1, Single Ended
	ADCA.CH0.MUXCTRL = (Channel<<3);
	ADCA.CH0.INTCTRL = 0 ; // No interrupt
	//ADCA.CH0.SCAN Another bogus register
	for(uint8_t Waste = 0; Waste<2; Waste++)
	{
		ADCA.CH0.CTRL |= ADC_CH_START_bm; // Start conversion
		while (ADCA.INTFLAGS==0) ; // Wait for complete
		ADCA.INTFLAGS = ADCA.INTFLAGS ;
	}
	return ADCA.CH0RES;
}
/*
int main(void)
{
	ConfigurePin();
	while(1)
	{
		ReadADC(0b0000001, 0b00010000);
	}
}
*/
static volatile uint8_t echo_char = 42;
static volatile char DATA[50];

int main( void )
{
	init_oscillator();
	ConfigurePin();

	// enable clock out on port PC7
	PORTCFG.CLKOUT = (PORTCFG.CLKOUT & ~PORTCFG_CLKOUTSEL_gm) | PORTCFG_CLKOUT_PC7_gc;
	// set PC7 as output
	PORTC.DIRSET = PIN7_bm;

	init_usart();

	init_interrupts();

	// set PA0 as output
	PORTA.DIRSET = PIN0_bm;
	// blink LED on PA0 with 1 second on, 1 second off
	// write echo_char on USART on D7; defaults to 42(*)
	while (1) {
		memset(DATA, 0, sizeof(DATA));
		strcpy(DATA,ReadADC(0b00000001, 0b000000001));
		usart_write_string(DATA);
		usart_write_string("\n");
		_delay_ms(1000);
	}
}

// USART RX receive interrupt handler
ISR(USARTD0_RXC_vect) {
	echo_char = USARTD0.DATA;
}
