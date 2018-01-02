/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <stdio.h>
#include <util/delay.h>
#include <adc_sensors/adc_sensors.h>

static char strbuf[128];

#define MY_ADC    ADCA
#define MY_ADC_CH ADC_CH0

static void adc_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);

	adcch_set_input(&adcch_conf, J2_PIN0, ADCCH_NEG_NONE, 1);

	adc_write_configuration(&MY_ADC, &adc_conf);
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
}

static uint16_t adc_read()
{
	uint16_t result;
	adc_enable(&MY_ADC);
	adc_start_conversion(&MY_ADC, MY_ADC_CH);
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);
	result = adc_get_result(&MY_ADC, MY_ADC_CH);
	return result;
}

/*untuk melakukan rutinitas*/
static int8_t aktuator(uint32_t light, int8_t moisture, int8_t sec)
{
	if (sec == 0)
	{
		return 0;
	}
	if ((sec%30) == 0)
	{
		if (moisture < 50)
		{
			gpio_set_pin_high(J3_PIN4);
			_delay_ms(2000);
			gpio_set_pin_low(J3_PIN4);
		}
		return 0;
	}
	if ((sec < 15) && (light < 50))
	{
		gpio_set_pin_high(LED0_GPIO);
		gpio_set_pin_high(LED1_GPIO);
		gpio_set_pin_high(LED2_GPIO);
	}
	else
	{
		gpio_set_pin_low(LED0_GPIO);
		gpio_set_pin_low(LED1_GPIO);
		gpio_set_pin_low(LED2_GPIO);
	}
	return sec;
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	board_init();

	/* Insert application code here, after the board has been initialized. */

	//init lcd
	gfx_mono_init();	
	adc_sensors_init();
	
	// Inisialisasi interrupt vector
	pmic_init();
	cpu_irq_enable();

	//set background lcd on
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

	//set display - output lcd
	gfx_mono_draw_string("Welcome!",0, 0, &sysfont);
	delay_ms(1000);
	gfx_mono_draw_string("We will use sensor",0, 8, &sysfont);
	delay_ms(1000);
	gfx_mono_draw_string("                   ",0, 0, &sysfont);
	gfx_mono_draw_string("                   ",0, 8, &sysfont);
	
	// Variable untuk sampling light sensor
	uint32_t intensity = 0;
	uint8_t iterations = 0;
	#define LIGHTSENSOR_NUM_SAMPLES 20
	
	adc_init();		/* initialize the ADC */
	char array[10];
	int8_t adc_value;
	float moisture;
	int counter = 0;
	int8_t sec = -1;
	int speed = 1000;

	while (1)
	{
		counter = counter%speed;
		if ((counter%speed) == 0)
		{
			sec = sec + 1;
		}
		snprintf(strbuf, sizeof(strbuf), "sec : %3d",sec);
		gfx_mono_draw_string(strbuf,0, 0, &sysfont);
		
		ntc_measure();												// Mengambil data dari pengukuran suhu oleh NTC temperature sensor
		while(!ntc_data_is_ready());								// Menunggu data sampai siap untuk ditampilkan		
		volatile int8_t temperature = ntc_get_temperature();	// Mengambil hasil olah data dalam Celcius

		snprintf(strbuf, sizeof(strbuf), "Tempr : %3d",temperature);
		gfx_mono_draw_string(strbuf,0, 8, &sysfont);

		lightsensor_measure();									// Mengambil data dari pengukuran intensitas oleh light sensor
		while(!lightsensor_data_is_ready());					// Menunggu data sampai siap untuk ditampilkan
		intensity += lightsensor_get_raw_value();				// Mengambil hasil olah data dalam raw ADC value

		// Dikarenakan hasil yang diperoleh merupakan data raw diperlukan sampling agar mendapatkan hasil yang baik
		if(iterations++ >= LIGHTSENSOR_NUM_SAMPLES) {
			iterations = 0;
			intensity /= LIGHTSENSOR_NUM_SAMPLES;
			
			snprintf(strbuf, sizeof(strbuf), "Light : %3d",intensity);
			gfx_mono_draw_string(strbuf,0, 16, &sysfont);

			intensity = 0;
		}
		
		adc_value = adc_read();	/* Copy the ADC value */
		moisture = 100-(adc_value*100.00)/1023.00; /* Calculate moisture in % */
		int a = moisture;
		snprintf(strbuf, sizeof(strbuf), "Moisture: %3d",a);
		gfx_mono_draw_string(strbuf,0, 24, &sysfont);
		
		sec = aktuator(intensity, a, sec);
		counter = counter + 1;
		_delay_ms(1);
	}
	

}