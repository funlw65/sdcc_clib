/* 
 * File:   main.c
 * Author: Vasile Guta-Ciucur
 *
 * Created on December 16, 2014, 9:34 PM
 */

/*
 * Description: a standard "Hello world" program for testing the 16x2 LCD.
 * It also tests the reading and displaying a string stored in the FLASH (pro-
 * gram space) opposed to one stored in RAM (well, I guess is just me, now learning
 * how to do things in SDCC...).
 *
 */

#define ONBOARD 1 /* enable the default onboard definitions */
/* #define NOBOOT 1 */
#undef NOBOOT
#include <rosso_sdcc.h> /* processor type, speed, configuration bits, hardware, app_offset. */
#include <rosso_sdcc_lcd4.h>
//#include <rosso_sdcc_conversion.h>
#include <stdlib.h>
#include <crt0i.c> /* don't ever remove this! let it be the last, under other includes */

uint8_t s[4]; /* buffer for number conversion */

const uint8_t sf[]=" ENG\0"; /* this should be stored in FLASH */
uint8_t sr[]="Hello World!\0";

void main() {
    uint8_t counter = 0;
    AllDigital(); /* all pins digital */
#ifdef ONBOARD
    OnBoardLED_dir = 0; /* output */
    OnBoardButton_dir = 1; /* input */
    OnBoardLED = 0;
#endif
    lcd_init(LCD_HD44780);
    lcd_cursor_position(0, 0);
    lcd_write_str(sr);  /* reading the string from the RAM */
    lcd_write_strF(sf); /* reading the string from the FLASH */
    while (1) {
        counter += 1; /* count up to 255 and start again from zer0 */
        lcd_cursor_position(1, 0);
        uitoa(counter, s, 10);
        /* formating - aligning the number to right */
		if(counter < 10) { 
			lcd_write_str("  ");
		}
		if((counter > 9) && (counter < 100)) { 
			lcd_write_str(" ");
		}
		/* end formating */
        lcd_write_str(s);
        delay_150ms();
        delay_150ms();
    }
}


