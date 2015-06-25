

#include <avr/io.h>
#define MY_ADC ADCA
#define MY_ADC_CH ADC_CH_MUXINT0_bm



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

static void adc_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_BANDGAP);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
	adc_write_configuration(&MY_ADC, &adc_conf);
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
}
