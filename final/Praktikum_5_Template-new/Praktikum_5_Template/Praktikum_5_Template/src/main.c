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
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <FreeRTOS.h>
#include <stdio.h>
#include <task.h>
#include <timers.h>
#include <adc_sensors/adc_sensors.h>

#define MY_ADC    ADCA
#define MY_ADC_CH ADC_CH0

#define MY_ADC2    ADCA
#define MY_ADC2_CH ADC_CH1

#define MY_ADC3    ADCA
#define MY_ADC3_CH ADC_CH2

// Variable untuk sampling light sensor
uint32_t intensity = 0;
uint8_t iterations = 0;
#define LIGHTSENSOR_NUM_SAMPLES 20

uint16_t result = 0;
uint16_t result2 = 0;
uint16_t result3 = 0;
int door = 0;
long increment = 0;
static char strbuf[201];

static portTASK_FUNCTION_PROTO(testLamp, p_);
static portTASK_FUNCTION_PROTO(testLCD, p_);
static portTASK_FUNCTION_PROTO(testLightS, p_);
static portTASK_FUNCTION_PROTO(testTempS, p_);
static portTASK_FUNCTION_PROTO(testServo, p_);
static portTASK_FUNCTION_PROTO(testMoisture, p_);

void vTimerCallback(){
	increment++;
}

void PWM_Init(void)
{
	/* Set output */
	PORTC.DIR |= PIN0_bm;

	/* Set Register */
	TCC0.CTRLA = (PIN2_bm) | (PIN0_bm);
	TCC0.CTRLB = (PIN4_bm) | (PIN2_bm) | (PIN1_bm);
	
	/* Set Period */
	TCC0.PER = 1000;

	/* Set Compare Register value*/
	TCC0.CCA = 375;
}

static void adc_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);

	adcch_set_input(&adcch_conf, J3_PIN0, ADCCH_NEG_NONE, 1);

	adc_write_configuration(&MY_ADC, &adc_conf);
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
}

static void adc_init2(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&MY_ADC2, &adc_conf);
	adcch_read_configuration(&MY_ADC2, MY_ADC2_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);

	adcch_set_input(&adcch_conf, J3_PIN1, ADCCH_NEG_NONE, 1);

	adc_write_configuration(&MY_ADC2, &adc_conf);
	adcch_write_configuration(&MY_ADC2, MY_ADC2_CH, &adcch_conf);
}

static void adc_init3(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&MY_ADC3, &adc_conf);
	adcch_read_configuration(&MY_ADC3, MY_ADC3_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);

	adcch_set_input(&adcch_conf, J3_PIN2, ADCCH_NEG_NONE, 1);

	adc_write_configuration(&MY_ADC3, &adc_conf);
	adcch_write_configuration(&MY_ADC3, MY_ADC3_CH, &adcch_conf);
}

static uint16_t adc_read(){
	uint16_t result;
	adc_enable(&MY_ADC);
	adc_start_conversion(&MY_ADC, MY_ADC_CH);
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);
	result = adc_get_result(&MY_ADC, MY_ADC_CH);
	return result;
}

//
static uint16_t adc_read2(){
	uint16_t result;
	ntc_measure();
	adc_enable(&MY_ADC2);
	adc_start_conversion(&MY_ADC2, MY_ADC2_CH);
	adc_wait_for_interrupt_flag(&MY_ADC2, MY_ADC2_CH);
	result = adc_get_result(&MY_ADC2, MY_ADC2_CH);
	return result;
}

static uint16_t adc_read3(){
	uint16_t result;
	adc_enable(&MY_ADC3);
	adc_start_conversion(&MY_ADC3, MY_ADC3_CH);
	adc_wait_for_interrupt_flag(&MY_ADC3, MY_ADC3_CH);
	result = adc_get_result(&MY_ADC3, MY_ADC3_CH);
	return result;
}

int main (void)
{
	//Inintialize Board
	board_init();
	pmic_init();
	
	// Inintialize adc sensor
	adc_init();
	adc_init2();
	adc_init3();
	
	//Inintialize LED board
	gfx_mono_init();
	
	//Membuat Ping yang dapat digunakan sebagai timer.
	TimerHandle_t timerPing = xTimerCreate("tPing", 2/portTICK_PERIOD_MS, pdTRUE, (void *) 0, vTimerCallback);
	
	xTaskCreate(testLamp,"",500,NULL,1,NULL);		//inintialize function lampu pada Board
	xTaskCreate(testLCD,"",500,NULL,1,NULL);		//inintialize  function LCD
	xTaskCreate(testLightS,"",500,NULL,1,NULL);		//inintialize  function lampu untuk pencahayaan tanaman
	xTaskCreate(testTempS,"",500,NULL,1,NULL);		//inintialize  function sensor suhu
	xTaskCreate(testMoisture,"",500,NULL,1,NULL);	//inintialize  function Sensor kelembapan tanah 
	
	xTimerStart(timerPing, 0);			// Memulai timer waktu
	
	vTaskStartScheduler();

	// Code here
	
}

static portTASK_FUNCTION(testLamp, p_){
	//ioport_set_pin_level(LCD_BACKLIGHT_ENABLE_PIN, false);
	
	while(1){
		gpio_set_pin_low(LED0_GPIO);
		vTaskDelay(100/portTICK_PERIOD_MS);
		gpio_set_pin_high(LED0_GPIO);
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(testLCD, p_){
	ioport_set_pin_level(LCD_BACKLIGHT_ENABLE_PIN, 1);
	while(1){
		
		//print light
		snprintf(strbuf, sizeof(strbuf), "Read Light : %3d",result);
		gfx_mono_draw_string(strbuf,0, 0, &sysfont);
		
		//print temp
		snprintf(strbuf, sizeof(strbuf), "Read Temp : %3d",result2);
		gfx_mono_draw_string(strbuf,0, 10, &sysfont);
		
		//print timer
		//snprintf(strbuf, sizeof(strbuf), "Timer : %3d",increment);
		snprintf(strbuf, sizeof(strbuf), "Read Moisture : %3d",result3);
		gfx_mono_draw_string(strbuf,0, 20, &sysfont);
		
		vTaskDelay(5/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(testLightS, p_){
	while(1){
		lightsensor_measure();							// Mengambil data dari pengukuran intensitas oleh light sensor
		while(!lightsensor_data_is_ready());			// Menunggu data sampai siap untuk ditampilkan
		intensity += lightsensor_get_raw_value();		// Mengambil hasil olah data dalam raw ADC value

		// Dikarenakan hasil yang diperoleh merupakan data raw diperlukan sampling agar mendapatkan hasil yang baik
		if(iterations++ >= LIGHTSENSOR_NUM_SAMPLES) {
			iterations = 0;
			intensity /= LIGHTSENSOR_NUM_SAMPLES;
			
			result = intensity;

			intensity = 0;
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(testTempS, p_){
	while(1){
		ntc_measure();								// Mengambil data dari pengukuran suhu oleh NTC temperature sensor
		while(!ntc_data_is_ready());				// Menunggu data sampai siap untuk ditampilkan
		volatile int8_t temperature = ntc_get_temperature();	// Mengambil hasil olah data dalam Celcius
		result2 = temperature;
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(testMoisture, p_){
	while(1){
		int8_t adc_value;
		float moisture;
		adc_value = adc_read();	/* Copy the ADC value */
		moisture = 100-(adc_value*100.00)/1023.00; /* Calculate moisture in % */
		int a = moisture;
		result3 = a;
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}
