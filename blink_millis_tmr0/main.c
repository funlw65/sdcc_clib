/*
 * File:   main.c
 * Author: sotea
 *
 * Created on June 21, 2014, 4:18 AM
 */
 
#define ONBOARD 1
/*#define NOBOOT 1*/ 
#undef NOBOOT
#include <rosso_sdcc.h> 
#define TMR0_MILLIS // automatically sets the RATE to 1000
#include <rosso_sdcc_isr_delay.h>
#include <crt0i.c>/* don't ever remove this! let it be the last, under other includes */

uint32_t previous = 0, current = 0;

/* ------ Relocating Interrupts - Don't touch! --------------------- */
#ifdef NOBOOT
#pragma code high_isr_proxy 0x008
void high_isr_proxy(void) __naked __interrupt 1 {
	__asm goto _high_isr __endasm;
}

#pragma code low_isr_proxy 0x018
void low_isr_proxy(void) __naked __interrupt 2 {
	__asm goto _low_isr __endasm;
}
#else
#pragma code high_isr_proxy 0x308
void high_isr_proxy(void) __naked __interrupt 1 {
	__asm goto _high_isr __endasm;
}

#pragma code low_isr_proxy 0x318
void low_isr_proxy(void) __naked __interrupt 2 {
	__asm goto _low_isr __endasm;
}
#endif
/* ----------------------------------------------------------------- */
 
void high_isr(void) __shadowregs  __interrupt {
    /* add here any high priority interrupt management functions */
    tmr0_isr_intr();
}

void low_isr(void) __shadowregs  __interrupt {
    /* add here any low priority interrupt management functions */
}


void main() {
    AllDigital();
#ifdef ONBOARD
    OnBoardLED_dir    = 0; //output
    OnBoardButton_dir = 1; //input
    OnBoardLED = 0;
#endif
    sei(); // enable general interrupts
    tmr0_isr_init();
    previous = 0;
    while (1) {
        current = tmr0_millis();
        if((current - previous) >= 1000){
            previous = current;
            OnBoardLED = !OnBoardLED;
        }
    }
}

