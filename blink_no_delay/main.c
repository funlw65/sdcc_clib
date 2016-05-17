#define ONBOARD 1
/*#define NOBOOT 1*/ 
#undef NOBOOT
#include <rosso_sdcc.h>
#define TMR0_SLOTED /* using sloted delays */
#define TMR0_DELAY_SLOTS 1
#define TMR0_ISR_RATE 1000
#include <rosso_sdcc_isr_delay.h>
#include <crt0i.c>/* don't ever remove this! let it be the last, under other includes */

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

void main(void){
    AllDigital();
#ifdef ONBOARD
    OnBoardLED_dir = OUTPUT; 
    OnBoardButton_dir = INPUT; 
    OnBoardLED = 0;
#endif
    sei(); /* enable global interrupts */
    tmr0_isr_init();
    tmr0_set_delay(0, 250);
    while (1) {
        if (tmr0_check_delay(0) == TRUE) {
            tmr0_set_delay(0, 250);
            OnBoardLED = !OnBoardLED;
        }
    }
}

