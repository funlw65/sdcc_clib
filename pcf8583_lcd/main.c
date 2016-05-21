/* 
 * Copy this file on your folder's project and 
 * rename it to main.c file. Copy also the *.sh  files 
 * in the same folder.
 */
#define ONBOARD 1
/* #define NOBOOT 1 */
#undef NOBOOT
#include <rosso_sdcc.h>
#include <rosso_sdcc_conversion.h>
#include <stdlib.h>
#include <rosso_sdcc_i2c_master.h>
#define PCF8583_USE_I2C
#define PCF8583_PHYSICAL_ADDRESS 0xA2 // 0x00 may be used by I2C EEPROM
#include <rosso_sdcc_pcf8583.h>
#include <rosso_sdcc_lcd4.h>
#include <crt0i.c>/* don't ever remove this! let it be the last, under other includes */

uint8_t s[4]; // buffer used for conversions



void main(void){
	uint8_t  current_sec = 0, old_sec = 0;
    AllDigital();
#ifdef ONBOARD
    OnBoardLED_dir = OUTPUT; 
    OnBoardButton_dir = INPUT; 
    OnBoardLED = OFF;
#endif
    //sei(); /* enable global interrupts */
	i2c_init(I2C_100KHZ);
	lcd_init(LCD_HD44780);
    // =====================================================================
    // if you need to set the RTC, uncomment the following and run once
    RTC_seconds = 0; // 0 to 59
    RTC_minutes = 5; // 0 to 59
    RTC_hours = 17; // 0 to 23
    RTC_day = 13; // 1 to 31
    RTC_month = 5; // 1 to 12
    RTC_century = 20; //
    RTC_year = 14; // 0 to 99
    RTC_dayofweek = 2; // 0 to 6 (Sun, Mon, etc..)
    RTC_leapyear = 2; // 0 to 3 (o - is leapyear and 1,2,3 not)
    // 2012 was leapyear    ( leapyear = 0)
    // 2014 is not leapyear ( leapyear = 2)
    //
    pcf8583_set_datetime(RTC_hours, RTC_minutes, RTC_seconds, RTC_dayofweek, RTC_day, RTC_month, RTC_leapyear, RTC_century, RTC_year);
    // =====================================================================

    
    while (1) {
        // Add your repeating code...
        OnBoardLED = !OnBoardLED; // blink seconds
        pcf8583_get_datetime(&RTC_hours, &RTC_minutes, &RTC_seconds, &RTC_dayofweek, &RTC_day, &RTC_month, &RTC_leapyear, &RTC_century, &RTC_year);
        current_sec = RTC_seconds;
        if(current_sec != old_sec){
            old_sec = current_sec;
            // show time
            lcd_cursor_position(0, 0);
            uitoa(RTC_hours, s, 10);
            if(RTC_hours < 10) _lcd_write_data('0');
            lcd_write_str(s);
            _lcd_write_data(':');
            uitoa(RTC_minutes, s, 10);
            if(RTC_minutes < 10) _lcd_write_data('0');
            lcd_write_str(s);
            _lcd_write_data(':');
            uitoa(RTC_seconds, s, 10);
            if(RTC_seconds < 10) _lcd_write_data('0');
            lcd_write_str(s);
            // show date on the second line
            lcd_cursor_position(1, 0);
            uitoa(RTC_day, s, 10);
            if(RTC_day < 10) _lcd_write_data('0');
            lcd_write_str(s);
            _lcd_write_data('/');
            uitoa(RTC_month, s, 10);
            if(RTC_month < 10) _lcd_write_data('0');
            lcd_write_str(s);
            _lcd_write_data('/');
            uitoa(RTC_century, s, 10);
            lcd_write_str(s);
            uitoa(RTC_year, s, 10);
            lcd_write_str(s);
        }
        delay_150ms();
        delay_150ms();
        delay_100ms();
        delay_100ms();
    }
}

