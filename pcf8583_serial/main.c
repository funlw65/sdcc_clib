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
#include <rosso_sdcc_i2c_master.h>
#define PCF8583_USE_I2C
#define PCF8583_PHYSICAL_ADDRESS 0xA2 // 0x00 may be used by I2C EEPROM
#include <rosso_sdcc_pcf8583.h>
#define USART_BAUDRATE 19200
#include <rosso_sdcc_hwserial.h>
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
	USART_HW_init();
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
            byte2dec(RTC_hours, s);
            if(s[1] == ' ') USART_HW_write('0');
            else USART_HW_write(s[1]);
            USART_HW_write(s[2]);
            USART_HW_write(':');
            byte2dec(RTC_minutes, s);
            if(s[1] == ' ') USART_HW_write('0');
            else USART_HW_write(s[1]);
            USART_HW_write(s[2]);
            USART_HW_write(':');
            byte2dec(RTC_seconds, s);
            if(s[1] == ' ') USART_HW_write('0');
            else USART_HW_write(s[1]);
            USART_HW_write(s[2]);
            // show date
            USART_HW_write(' ');
            byte2dec(RTC_day, s);
            if(s[1] == ' ') USART_HW_write('0');
            else USART_HW_write(s[1]);
            USART_HW_write(s[2]);
            USART_HW_write('/');
            byte2dec(RTC_month, s);
            if(s[1] == ' ') USART_HW_write('0');
            else USART_HW_write(s[1]);
            USART_HW_write(s[2]);
            USART_HW_write('/');
            byte2dec(RTC_century, s);
            USART_HW_write(s[1]);
            USART_HW_write(s[2]);
            byte2dec(RTC_year, s);
            USART_HW_write(s[1]);
            USART_HW_write(s[2]);
            USART_HW_write(13);
            USART_HW_write(10);
        }
        delay_150ms();
        delay_150ms();
        delay_100ms();
        delay_100ms();
    }
}

