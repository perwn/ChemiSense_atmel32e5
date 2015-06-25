/*
 * Usart_test.c
 *
 * Created: 6/18/2015 1:07:29 PM
 *  Author: ChemiSenseUser
 */ 
#define F_CPU 32000000UL
//#define BAUD 9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//#include <avr/instdio.h>
//#include <util/setbaud.h>


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



static volatile uint8_t echo_char = 42;

int main( void )
{
	init_oscillator();

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
		usart_write_string("abc\n");
		_delay_ms(500);
	}
}

// USART RX receive interrupt handler
ISR(USARTD0_RXC_vect) {
	echo_char = USARTD0.DATA;
}









/*

static int uart_putchar(char c, FILE *stream);

// extern "C"{ FILE * uart_str; }


void setUp32MhzInternalOsc()
{
	OSC_CTRL |= OSC_RC32MEN_bm; //Setup 32Mhz crystal
	
	while(!(OSC_STATUS & OSC_RC32MRDY_bm));
	
	CCP = CCP_IOREG_gc; //Trigger protection mechanism
	CLK_CTRL = CLK_SCLKSEL_RC32M_gc; //Enable internal  32Mhz crystal
	
	
}

void setUpSerial()
{
	//For the sake of example, I'll just REMAP the USART pins from PC3 and PC2 to PC7 and PC6
	PORTC_REMAP |= 0x16; //See page 152 in datasheet, remaps the USART0
		
	PORTC_OUTSET = PIN7_bm; //Let's make PC7 as TX
	PORTC_DIRSET = PIN7_bm; //TX pin as output
		
	PORTC_OUTCLR = PIN6_bm;
	PORTC_DIRCLR = PIN6_bm; //PC6 as RX
	
	// Baud rate selection
	// BSEL = (32000000 / (2^0 * 16*9600) -1 = 207.333 -> BSCALE = 0
	// FBAUD = ( (32000000)/(2^0*16(207+1)) = 9615.384 -> it's alright
	
	USARTC0_BAUDCTRLB = 0; //Just to be sure that BSCALE is 0
	USARTC0_BAUDCTRLA = 0xCF; // 207
	
	
	//Disable interrupts, just for safety
	USARTC0_CTRLA = 0;
	//8 data bits, no parity and 1 stop bit
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
	
	//Enable receive and transmit
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm; // And enable high speed mode
}

void sendChar(char c)
{
	
	while( !(USARTC0_STATUS & USART_DREIF_bm) ); //Wait until DATA buffer is empty
	
	USARTC0_DATA = c;
	
}

void sendString(char *text)
{
	while(*text)
	{
		sendChar(*text++);
	}
}
 
 static int uart_putchar (char c, FILE *stream)
 {
	 if (c == '\n')
	 uart_putchar('\r', stream);
	 
	 // Wait for the transmit buffer to be empty
	 while (  !(USARTC0_STATUS & USART_DREIF_bm) );
	 
	 // Put our character into the transmit buffer
	 USARTC0_DATA = c;
	 
	 return 0;
 }

char usart_receiveByte()
{
	while( !(USARTC0_STATUS & USART_RXCIF_bm) ); //Interesting DRIF didn't work.
	return USARTC0_DATA;
}

int uart_getchar(FILE *stream)
{
	while( !(USARTC0_STATUS & USART_RXCIF_bm) ); //Interesting DRIF didn't work.
	char data = USARTC0_DATA;
	if(data == '\r'){
		data = '\n';		
	}
	uart_putchar(data, stream);
	return data;
}

FILE usart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main()
{
	setUp32MhzInternalOsc();
	setUpSerial();
	
//	File uart_str = fdevopen(uart_putchar, uart_getchar);
	stdout = stdin = &usart_str;
	
	printf("Ready!\n");
    printf("Ready!\n");
    while(1)
    {
	 printf ("A");
	 _delay_ms(500);
	  //  char str [80];
	  //  int i;
	  //  printf ("Enter your family name: ");
	  //  scanf ("%79s",str);
	  //  printf ("Enter your age: ");
	  //  scanf ("%d",&i);
	  //  printf ("Mr. %s , %d years old.\n",str,i);
	}
}
*/