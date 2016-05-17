/* 
 * Copy this file on your folder's project and 
 * rename it to main.c file. Copy also the *.sh  files 
 * in the same folder.
 */
#define ONBOARD 1
/* #define NOBOOT 1 */
#undef NOBOOT
#include <rosso_sdcc.h>
/* add your other definitions and includes */
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
}

void low_isr(void) __shadowregs  __interrupt {
    /* add here any low priority interrupt management functions */
}


void main(void){
    AllDigital();
#ifdef ONBOARD
    OnBoardLED_dir = OUTPUT; 
    OnBoardButton_dir = INPUT; 
    OnBoardLED = OFF;
#endif
    sei(); /* enable global interrupts */
    
    /* add your other initializations and code */
    
    while (1) {
		/* add your ever repeating code */
    }
}

