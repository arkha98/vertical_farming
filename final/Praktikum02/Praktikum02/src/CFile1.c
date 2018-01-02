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
#include <adc_sensors/adc_sensors.h>

static char strbuf[128];

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
	gfx_mono_draw_string("    Sensor Read    ",0, 0, &sysfont);
	gfx_mono_draw_string("                   ",0, 8, &sysfont);
	
	// Variable untuk sampling light sensor
	uint32_t intensity = 0;
	uint8_t iterations = 0;
	#define LIGHTSENSOR_NUM_SAMPLES 20
	

	while (1)
	{
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
			
		
	}
	

}