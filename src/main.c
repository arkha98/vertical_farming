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
#include <stdio.h>
#include <string.h>
#include <adc_sensors/adc_sensors.h>
#include <usart.h>


#define USART_SERIAL_EXAMPLE             &USARTC0
#define USART_SERIAL_EXAMPLE_BAUDRATE    9600
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT            false                          

static char strbuf[201];
static char reads[100];
int result = 0;
char in = 'x';

char *str1 = "atas ";
char *str2 = "bawah ";

void setup_timer(void);
void print_message(void);

//Fungsi setup timer
void setup_timer(void){
    tc_enable(&TCC0);
    tc_set_overflow_interrupt_callback(&TCC0,print_message);
    tc_set_wgm(&TCC0, TC_WG_NORMAL);
    tc_write_period(&TCC0, 58);
    tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_HI);
    tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);    
}

void setUpSerial()
{
    // Baud rate selection
    // BSEL = (2000000 / (2^0 * 16*9600) -1 = 12.0208... ~ 12 -> BSCALE = 0
    // FBAUD = ( (2000000)/(2^0*16(12+1)) = 9615.384 -> mendekati lah ya
    
    USARTC0_BAUDCTRLB = 0; //memastikan BSCALE = 0
    USARTC0_BAUDCTRLA = 0x0C; // 12
    
    //USARTC0_BAUDCTRLB = 0; //Just to be sure that BSCALE is 0
    //USARTC0_BAUDCTRLA = 0xCF; // 207
    
    
    //Disable interrupts, just for safety
    USARTC0_CTRLA = 0;
    //8 data bits, no parity and 1 stop bit
    USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
    
    //Enable receive and transmit
    USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

void sendString(char *text)
{
    while(*text)
    {
        //sendChar(*text++);
        usart_putchar(USART_SERIAL_EXAMPLE, *text++);
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
    gfx_mono_draw_string("    Sensor Read    ",0, 0, &sysfont);
    gfx_mono_draw_string("                   ",0, 8, &sysfont);
    
    // Variable untuk sampling light sensor
    uint32_t intensity = 0;
    uint8_t iterations = 0;
    #define LIGHTSENSOR_NUM_SAMPLES 20
    

    while (1)
    {
        ntc_measure();                                          // Mengambil data dari pengukuran suhu oleh NTC temperature sensor
        while(!ntc_data_is_ready());                            // Menunggu data sampai siap untuk ditampilkan      
        volatile int8_t temperature = ntc_get_temperature();    // Mengambil hasil olah data dalam Celcius

        snprintf(strbuf, sizeof(strbuf), "Tempr : %3d",temperature);
        gfx_mono_draw_string(strbuf,0, 8, &sysfont);

        lightsensor_measure();                                  // Mengambil data dari pengukuran intensitas oleh light sensor
        while(!lightsensor_data_is_ready());                    // Menunggu data sampai siap untuk ditampilkan
        intensity += lightsensor_get_raw_value();               // Mengambil hasil olah data dalam raw ADC value

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