#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>
//#include <board.h>
//#include <gpio.h>

volatile uint8_t adc_value;

void ADCBInit(void)
{
	//Conversion Complete mode is enabled and is assigned high priority
	ADCA.CH0.INTCTRL = ADC_CH_INTLVL1_bm | ADC_CH_INTLVL0_bm; // 0x03
	
	// Ensure the conversion complete flag is cleared
	ADCA.INTFLAGS = ADC_CH0IF_bm; // 0x01
	
	//Enable the ADC
	ADCA.CTRLA = ADC_ENABLE_bm; // 0x01
	
	//Single conversions and run in Signed mode.  The ADC is configured to display a
	//8-bit, right justified result.
	ADCA.CTRLB = 0b00110100;
	
	// Configure PB1 as input; write '0' to pin position in Direction register
	// the potentiometer is connected to PORTB, Pin 1 on the Xmega
	//PORTA.DIR &= ~0x02;
	
	//Light sensor
	PORTD.DIRCLR = PIN1_bm;
	
	//The ADC is placed in Single-Ended mode
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE0_bm; // 0x01
	
	//The reference voltage to the ADC is set to Vcc/1.6
	ADCA.REFCTRL = ADC_REFSEL0_bm; // 0x10
	
	ADCA.REFCTRL |= 0x01; //Enable temp sensor

	//Disable event trigging
	ADCA.EVCTRL = 0x00;
	
	// Start a conversion using ADC Control A Register
	ADCA.CTRLA |= ADC_CH0START_bm; // 0x04
}

void adcb_callback(void)
{
	adc_value = ADCA.CH0.RES;
}

uint8_t ReadSignatureByte(uint16_t Address)
{
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	__asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}

void initButton(void){
	PORTD.DIRCLR = PIN0_bm;
	PORTD.DIRCLR = PIN2_bm;
	PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN2CTRL = PORT_OPC_PULLUP_gc;
}
uint8_t GetButtonInput(void){
	
return PORTD.IN;
}

void initLeds(void){
	PORTD.DIRSET = PIN5_bm; 
	PORTD.DIRSET = PIN4_bm;
	PORTD.OUTCLR = PIN5_bm;
	PORTD.OUTCLR = PIN4_bm;
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


static void usart_write(uint8_t data)
{
	USARTD0.DATA = data;
	if(!(USARTD0.STATUS & USART_DREIF_bm)) {
		while(!(USARTD0.STATUS & USART_TXCIF_bm)); // wait for TX complete
	}
	USARTD0.STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag
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