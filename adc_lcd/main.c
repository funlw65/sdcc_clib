/* 
 * File:   main.c
 * Author: Vasile Guta-Ciucur
 *
 */

/*
 * Description: Reading analog values of RA0 pin and displaying on 
 *              16x2 LCD
 *
 */

#define ONBOARD 1 /* enable the default onboard definitions */
/* #define NOBOOT 1 */
#undef NOBOOT
#include <rosso_sdcc.h> /* processor type, speed, configuration bits, hardware, app_offset. */
#include <stdlib.h>
#include <rosso_sdcc_lcd4.h>
//#define USE_OR_MASKS
#include <rosso_sdcc_adc.h>
#include <crt0i.c> /* don't ever remove this! let it be the last, under other includes */

uint8_t s[5]; /* buffer for number conversion */

const uint8_t sf[]="Analog RA0:\0"; /* this should be stored in FLASH */

#ifdef USE_OR_MASKS
uint8_t config = ADC_FOSC_64 | ADC_RIGHT_JUST | ADC_20_TAD;
uint8_t config2 = ADC_CH0 | ADC_INT_OFF;
uint8_t config3 = ADC_TRIG_CCP5 | ADC_REF_VDD_VDD | ADC_REF_VDD_VSS;
#else
uint8_t config = ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD;
uint8_t config2 = ADC_CH0 & ADC_INT_OFF;
uint8_t config3 = ADC_TRIG_CCP5 & ADC_REF_VDD_VDD & ADC_REF_VDD_VSS;
#endif

void main() {
    uint16_t adcval = 0;
    AllDigital(); /* all pins digital */
#ifdef ONBOARD
    OnBoardLED_dir = 0; /* output */
    OnBoardButton_dir = 1; /* input */
    OnBoardLED = 0;
#endif
	/* Setting RA0 as input and analog */
    TRISAbits.RA0 = 1;
    ANSELAbits.ANSA0 = 1;
    /* end setting RA0 */
    adc_init(config, config2, config3);
    lcd_init(LCD_HD44780);
    lcd_cursor_position(0, 0);
    lcd_write_strF(sf); /* reading the string from the FLASH */
    adc_setch(ADC_CH0);
    while (1) {
        /* Add your repeating code... */
        adc_conv();
        do{;}while(adc_busy());
        adcval = adc_read();
        lcd_cursor_position(1, 0);
        uitoa(adcval, s, 10);
        /* formatting - aligning the number to right */
		if(adcval < 10) { 
			lcd_write_str("   ");
		}
		if((adcval > 9) && (adcval < 100)) { 
			lcd_write_str("  ");
		}
		if((adcval > 99) && (adcval < 1000)) { 
			lcd_write_str(" ");
		}
		/* end formating */
		lcd_write_str(s);
		_delay_ms(1000);
    }
}


