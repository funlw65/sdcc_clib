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
#include <rosso_sdcc_conversion.h>
#include <crt0i.c> /* don't ever remove this! let it be the last, under other includes */

uint8_t s[4]; /* buffer for number conversion */

const uint8_t sf[]=" ENG\0"; /* this should be stored in FLASH */
uint8_t sr[]="Hello World!\0";

/* ------ Relocating Interrupts - Don't touch! --------------------- */
//#ifdef NOBOOT
//#pragma code high_isr_proxy 0x008
//void high_isr_proxy(void) __naked __interrupt 1 {
//	__asm goto _high_isr __endasm;
//}

//#pragma code low_isr_proxy 0x018
//void low_isr_proxy(void) __naked __interrupt 2 {
//	__asm goto _low_isr __endasm;
//}
//#else
//#pragma code high_isr_proxy 0x308
//void high_isr_proxy(void) __naked __interrupt 1 {
//	__asm goto _high_isr __endasm;
//}

//#pragma code low_isr_proxy 0x318
//void low_isr_proxy(void) __naked __interrupt 2 {
//	__asm goto _low_isr __endasm;
//}
//#endif
/* ----------------------------------------------------------------- */
 
//void high_isr(void) __shadowregs  __interrupt {
//    /* add here any high priority interrupt management functions */
//}

//void low_isr(void) __shadowregs  __interrupt {
//    /* add here any low priority interrupt management functions */
//}


void main() {
    uint8_t i = 0;
    uint8_t counter = 0;
    AllDigital(); /* all pins digital */
#ifdef ONBOARD
    OnBoardLED_dir = 0; /* output */
    OnBoardButton_dir = 1; /* input */
    OnBoardLED = 0;
#endif
    /* sei(); */ /* enable general interrupts if needed */
    /* Add other initializations you may have... */
    lcd_init(LCD_HD44780);
    /* signal the start four times */
    for (i = 0; i < 4; i++) {
        OnBoardLED = 1;
        delay_100ms();
        delay_100ms();
        OnBoardLED = 0;
        delay_100ms();
        delay_100ms();
    }
    lcd_cursor_position(0, 0);
    lcd_write_str(sr);  /* reading the string from the RAM */
    lcd_write_strF(sf); /* reading the string from the FLASH */
    while (1) {
        /* Add your repeating code... */
        counter += 1; /* count up to 255 and start again from zer0 */
        lcd_cursor_position(1, 0);
        byte2dec(counter, s);
        for (i = 0; i < 3; i++) _lcd_write_data(s[i]);
        delay_150ms();
        delay_150ms();
    }
}


