;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.5.6 #9596 (Linux)
;--------------------------------------------------------
; PIC16 port for the Microchip 16-bit core micros
;--------------------------------------------------------
	list	p=18f46k22
	radix	dec


;--------------------------------------------------------
; public variables in this module
;--------------------------------------------------------
	global	___uflags
	global	__entry
	global	__startup
	global	_main
	global	_tmr0_isr_countdown
	global	_tmr0_load
	global	__delay_us
	global	__delay_ms
	global	_tmr0_set_delay
	global	_tmr0_check_delay
	global	_tmr0_isr_init
	global	_tmr0_isr_intr
	global	_high_isr_proxy
	global	_low_isr_proxy
	global	_high_isr
	global	_low_isr

;--------------------------------------------------------
; extern variables in this module
;--------------------------------------------------------
	extern	_ANSELAbits
	extern	_ANSELBbits
	extern	_ANSELCbits
	extern	_ANSELDbits
	extern	_ANSELEbits
	extern	_PMD2bits
	extern	_PMD1bits
	extern	_PMD0bits
	extern	_DACCON1bits
	extern	_VREFCON2bits
	extern	_DACCON0bits
	extern	_VREFCON1bits
	extern	_FVRCONbits
	extern	_VREFCON0bits
	extern	_CTMUICONbits
	extern	_CTMUICONHbits
	extern	_CTMUCON1bits
	extern	_CTMUCONLbits
	extern	_CTMUCON0bits
	extern	_CTMUCONHbits
	extern	_SRCON1bits
	extern	_SRCON0bits
	extern	_CCPTMRS1bits
	extern	_CCPTMRS0bits
	extern	_T6CONbits
	extern	_T5GCONbits
	extern	_T5CONbits
	extern	_T4CONbits
	extern	_CCP5CONbits
	extern	_CCP4CONbits
	extern	_PSTR3CONbits
	extern	_CCP3ASbits
	extern	_ECCP3ASbits
	extern	_PWM3CONbits
	extern	_CCP3CONbits
	extern	_SLRCONbits
	extern	_WPUBbits
	extern	_IOCBbits
	extern	_PSTR2CONbits
	extern	_CCP2ASbits
	extern	_ECCP2ASbits
	extern	_PWM2CONbits
	extern	_CCP2CONbits
	extern	_SSP2CON3bits
	extern	_SSP2MSKbits
	extern	_SSP2CON2bits
	extern	_SSP2CON1bits
	extern	_SSP2STATbits
	extern	_BAUD2CONbits
	extern	_BAUDCON2bits
	extern	_RC2STAbits
	extern	_RCSTA2bits
	extern	_TX2STAbits
	extern	_TXSTA2bits
	extern	_CM12CONbits
	extern	_CM2CON1bits
	extern	_CM2CONbits
	extern	_CM2CON0bits
	extern	_CM1CONbits
	extern	_CM1CON0bits
	extern	_PIE4bits
	extern	_PIR4bits
	extern	_IPR4bits
	extern	_PIE5bits
	extern	_PIR5bits
	extern	_IPR5bits
	extern	_PORTAbits
	extern	_PORTBbits
	extern	_PORTCbits
	extern	_PORTDbits
	extern	_PORTEbits
	extern	_LATAbits
	extern	_LATBbits
	extern	_LATCbits
	extern	_LATDbits
	extern	_LATEbits
	extern	_DDRAbits
	extern	_TRISAbits
	extern	_DDRBbits
	extern	_TRISBbits
	extern	_DDRCbits
	extern	_TRISCbits
	extern	_DDRDbits
	extern	_TRISDbits
	extern	_DDREbits
	extern	_TRISEbits
	extern	_OSCTUNEbits
	extern	_HLVDCONbits
	extern	_LVDCONbits
	extern	_PIE1bits
	extern	_PIR1bits
	extern	_IPR1bits
	extern	_PIE2bits
	extern	_PIR2bits
	extern	_IPR2bits
	extern	_PIE3bits
	extern	_PIR3bits
	extern	_IPR3bits
	extern	_EECON1bits
	extern	_EEADRbits
	extern	_EEADRHbits
	extern	_RC1STAbits
	extern	_RCSTAbits
	extern	_RCSTA1bits
	extern	_TX1STAbits
	extern	_TXSTAbits
	extern	_TXSTA1bits
	extern	_T3CONbits
	extern	_T3GCONbits
	extern	_ECCP1ASbits
	extern	_ECCPASbits
	extern	_PWM1CONbits
	extern	_PWMCONbits
	extern	_BAUD1CONbits
	extern	_BAUDCONbits
	extern	_BAUDCON1bits
	extern	_BAUDCTLbits
	extern	_PSTR1CONbits
	extern	_PSTRCONbits
	extern	_T2CONbits
	extern	_CCP1CONbits
	extern	_ADCON2bits
	extern	_ADCON1bits
	extern	_ADCON0bits
	extern	_SSP1CON2bits
	extern	_SSPCON2bits
	extern	_SSP1CON1bits
	extern	_SSPCON1bits
	extern	_SSP1STATbits
	extern	_SSPSTATbits
	extern	_SSP1MSKbits
	extern	_SSPMSKbits
	extern	_SSP1CON3bits
	extern	_SSPCON3bits
	extern	_T1GCONbits
	extern	_T1CONbits
	extern	_RCONbits
	extern	_WDTCONbits
	extern	_OSCCON2bits
	extern	_OSCCONbits
	extern	_T0CONbits
	extern	_STATUSbits
	extern	_INTCON3bits
	extern	_INTCON2bits
	extern	_INTCONbits
	extern	_STKPTRbits
	extern	_stack_end
	extern	_ANSELA
	extern	_ANSELB
	extern	_ANSELC
	extern	_ANSELD
	extern	_ANSELE
	extern	_PMD2
	extern	_PMD1
	extern	_PMD0
	extern	_DACCON1
	extern	_VREFCON2
	extern	_DACCON0
	extern	_VREFCON1
	extern	_FVRCON
	extern	_VREFCON0
	extern	_CTMUICON
	extern	_CTMUICONH
	extern	_CTMUCON1
	extern	_CTMUCONL
	extern	_CTMUCON0
	extern	_CTMUCONH
	extern	_SRCON1
	extern	_SRCON0
	extern	_CCPTMRS1
	extern	_CCPTMRS0
	extern	_T6CON
	extern	_PR6
	extern	_TMR6
	extern	_T5GCON
	extern	_T5CON
	extern	_TMR5
	extern	_TMR5L
	extern	_TMR5H
	extern	_T4CON
	extern	_PR4
	extern	_TMR4
	extern	_CCP5CON
	extern	_CCPR5
	extern	_CCPR5L
	extern	_CCPR5H
	extern	_CCP4CON
	extern	_CCPR4
	extern	_CCPR4L
	extern	_CCPR4H
	extern	_PSTR3CON
	extern	_CCP3AS
	extern	_ECCP3AS
	extern	_PWM3CON
	extern	_CCP3CON
	extern	_CCPR3
	extern	_CCPR3L
	extern	_CCPR3H
	extern	_SLRCON
	extern	_WPUB
	extern	_IOCB
	extern	_PSTR2CON
	extern	_CCP2AS
	extern	_ECCP2AS
	extern	_PWM2CON
	extern	_CCP2CON
	extern	_CCPR2
	extern	_CCPR2L
	extern	_CCPR2H
	extern	_SSP2CON3
	extern	_SSP2MSK
	extern	_SSP2CON2
	extern	_SSP2CON1
	extern	_SSP2STAT
	extern	_SSP2ADD
	extern	_SSP2BUF
	extern	_BAUD2CON
	extern	_BAUDCON2
	extern	_RC2STA
	extern	_RCSTA2
	extern	_TX2STA
	extern	_TXSTA2
	extern	_TX2REG
	extern	_TXREG2
	extern	_RC2REG
	extern	_RCREG2
	extern	_SP2BRG
	extern	_SPBRG2
	extern	_SP2BRGH
	extern	_SPBRGH2
	extern	_CM12CON
	extern	_CM2CON1
	extern	_CM2CON
	extern	_CM2CON0
	extern	_CM1CON
	extern	_CM1CON0
	extern	_PIE4
	extern	_PIR4
	extern	_IPR4
	extern	_PIE5
	extern	_PIR5
	extern	_IPR5
	extern	_PORTA
	extern	_PORTB
	extern	_PORTC
	extern	_PORTD
	extern	_PORTE
	extern	_LATA
	extern	_LATB
	extern	_LATC
	extern	_LATD
	extern	_LATE
	extern	_DDRA
	extern	_TRISA
	extern	_DDRB
	extern	_TRISB
	extern	_DDRC
	extern	_TRISC
	extern	_DDRD
	extern	_TRISD
	extern	_DDRE
	extern	_TRISE
	extern	_OSCTUNE
	extern	_HLVDCON
	extern	_LVDCON
	extern	_PIE1
	extern	_PIR1
	extern	_IPR1
	extern	_PIE2
	extern	_PIR2
	extern	_IPR2
	extern	_PIE3
	extern	_PIR3
	extern	_IPR3
	extern	_EECON1
	extern	_EECON2
	extern	_EEDATA
	extern	_EEADR
	extern	_EEADRH
	extern	_RC1STA
	extern	_RCSTA
	extern	_RCSTA1
	extern	_TX1STA
	extern	_TXSTA
	extern	_TXSTA1
	extern	_TX1REG
	extern	_TXREG
	extern	_TXREG1
	extern	_RC1REG
	extern	_RCREG
	extern	_RCREG1
	extern	_SP1BRG
	extern	_SPBRG
	extern	_SPBRG1
	extern	_SP1BRGH
	extern	_SPBRGH
	extern	_SPBRGH1
	extern	_T3CON
	extern	_TMR3
	extern	_TMR3L
	extern	_TMR3H
	extern	_T3GCON
	extern	_ECCP1AS
	extern	_ECCPAS
	extern	_PWM1CON
	extern	_PWMCON
	extern	_BAUD1CON
	extern	_BAUDCON
	extern	_BAUDCON1
	extern	_BAUDCTL
	extern	_PSTR1CON
	extern	_PSTRCON
	extern	_T2CON
	extern	_PR2
	extern	_TMR2
	extern	_CCP1CON
	extern	_CCPR1
	extern	_CCPR1L
	extern	_CCPR1H
	extern	_ADCON2
	extern	_ADCON1
	extern	_ADCON0
	extern	_ADRES
	extern	_ADRESL
	extern	_ADRESH
	extern	_SSP1CON2
	extern	_SSPCON2
	extern	_SSP1CON1
	extern	_SSPCON1
	extern	_SSP1STAT
	extern	_SSPSTAT
	extern	_SSP1ADD
	extern	_SSPADD
	extern	_SSP1BUF
	extern	_SSPBUF
	extern	_SSP1MSK
	extern	_SSPMSK
	extern	_SSP1CON3
	extern	_SSPCON3
	extern	_T1GCON
	extern	_T1CON
	extern	_TMR1
	extern	_TMR1L
	extern	_TMR1H
	extern	_RCON
	extern	_WDTCON
	extern	_OSCCON2
	extern	_OSCCON
	extern	_T0CON
	extern	_TMR0
	extern	_TMR0L
	extern	_TMR0H
	extern	_STATUS
	extern	_FSR2L
	extern	_FSR2H
	extern	_PLUSW2
	extern	_PREINC2
	extern	_POSTDEC2
	extern	_POSTINC2
	extern	_INDF2
	extern	_BSR
	extern	_FSR1L
	extern	_FSR1H
	extern	_PLUSW1
	extern	_PREINC1
	extern	_POSTDEC1
	extern	_POSTINC1
	extern	_INDF1
	extern	_WREG
	extern	_FSR0L
	extern	_FSR0H
	extern	_PLUSW0
	extern	_PREINC0
	extern	_POSTDEC0
	extern	_POSTINC0
	extern	_INDF0
	extern	_INTCON3
	extern	_INTCON2
	extern	_INTCON
	extern	_PROD
	extern	_PRODL
	extern	_PRODH
	extern	_TABLAT
	extern	_TBLPTR
	extern	_TBLPTRL
	extern	_TBLPTRH
	extern	_TBLPTRU
	extern	_PC
	extern	_PCL
	extern	_PCLATH
	extern	_PCLATU
	extern	_STKPTR
	extern	_TOS
	extern	_TOSL
	extern	_TOSH
	extern	_TOSU
	extern	_delay100tcy
	extern	_cinit

;--------------------------------------------------------
;	Equates to used internal registers
;--------------------------------------------------------
STATUS	equ	0xfd8
PCLATH	equ	0xffa
PCLATU	equ	0xffb
WREG	equ	0xfe8
FSR0L	equ	0xfe9
FSR0H	equ	0xfea
FSR1L	equ	0xfe1
FSR2L	equ	0xfd9
INDF0	equ	0xfef
POSTINC0	equ	0xfee
POSTINC1	equ	0xfe6
POSTDEC1	equ	0xfe5
PREINC1	equ	0xfe4
PLUSW2	equ	0xfdb
PRODL	equ	0xff3
PRODH	equ	0xff4


	idata
___uflags	db	0x00


; Internal registers
.registers	udata_ovr	0x0000
r0x00	res	1
r0x01	res	1
r0x02	res	1
r0x03	res	1
r0x04	res	1
r0x05	res	1
r0x06	res	1

udata_main_0	udata
_tmr0_isr_countdown	res	2

udata_main_1	udata
_tmr0_load	res	1

;--------------------------------------------------------
; interrupt vector
;--------------------------------------------------------

;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
; ; Starting pCode block
S_main___entry	code	0X000300
__entry:
	goto	__startup
	
; ; Starting pCode block
S_main__high_isr_proxy	code	0X000308
_high_isr_proxy:
	goto	_high_isr 
; ; Starting pCode block
S_main__low_isr_proxy	code	0X000318
_low_isr_proxy:
	goto	_low_isr 
; ; Starting pCode block
S_main__high_isr	code
_high_isr:
;	.line	35; main.c	void high_isr(void) __shadowregs  __interrupt {
	MOVFF	PRODL, POSTDEC1
	MOVFF	PRODH, POSTDEC1
	MOVFF	FSR0L, POSTDEC1
	MOVFF	FSR0H, POSTDEC1
	MOVFF	PCLATH, POSTDEC1
	MOVFF	PCLATU, POSTDEC1
;	.line	37; main.c	tmr0_isr_intr();
	CALL	_tmr0_isr_intr
	MOVFF	PREINC1, PCLATU
	MOVFF	PREINC1, PCLATH
	MOVFF	PREINC1, FSR0H
	MOVFF	PREINC1, FSR0L
	MOVFF	PREINC1, PRODH
	MOVFF	PREINC1, PRODL
	RETFIE	0x01

; ; Starting pCode block
S_main__low_isr	code
_low_isr:
;	.line	40; main.c	void low_isr(void) __shadowregs  __interrupt {
	MOVFF	PRODL, POSTDEC1
	MOVFF	PRODH, POSTDEC1
	MOVFF	FSR0L, POSTDEC1
	MOVFF	FSR0H, POSTDEC1
	MOVFF	PCLATH, POSTDEC1
	MOVFF	PCLATU, POSTDEC1
;	.line	42; main.c	}
	MOVFF	PREINC1, PCLATU
	MOVFF	PREINC1, PCLATH
	MOVFF	PREINC1, FSR0H
	MOVFF	PREINC1, FSR0L
	MOVFF	PREINC1, PRODH
	MOVFF	PREINC1, PRODL
	RETFIE	0x01

; I code from now on!
; ; Starting pCode block
S_main__main	code
_main:
	BANKSEL	_ANSELA
;	.line	45; main.c	AllDigital();
	CLRF	_ANSELA, B
	BANKSEL	_ANSELB
	CLRF	_ANSELB, B
	BANKSEL	_ANSELC
	CLRF	_ANSELC, B
	BANKSEL	_ANSELD
	CLRF	_ANSELD, B
	BANKSEL	_ANSELE
	CLRF	_ANSELE, B
	CLRF	_ADCON0
	CLRF	_ADCON1
	CLRF	_ADCON2
	CLRF	_CM1CON0
	CLRF	_CM2CON0
	CLRF	_CM2CON1
;	.line	47; main.c	OnBoardLED_dir = OUTPUT; 
	BCF	_TRISCbits, 2
;	.line	48; main.c	OnBoardButton_dir = INPUT; 
	BSF	_TRISAbits, 4
;	.line	49; main.c	OnBoardLED = 0;
	BCF	_LATCbits, 2
;	.line	51; main.c	sei(); /* enable global interrupts */
	CLRF	_RCON
	BSF	_RCONbits, 7
	BSF	_INTCONbits, 6
	BSF	_INTCONbits, 7
;	.line	52; main.c	tmr0_isr_init();
	CALL	_tmr0_isr_init
;	.line	53; main.c	tmr0_set_delay(0, 250);
	CLRF	POSTDEC1
	MOVLW	0xfa
	MOVWF	POSTDEC1
	MOVLW	0x00
	CLRF	POSTDEC1
	CALL	_tmr0_set_delay
	MOVLW	0x03
	ADDWF	FSR1L, F
_00250_DS_:
;	.line	55; main.c	if (tmr0_check_delay(0) == TRUE) {
	MOVLW	0x00
	CLRF	POSTDEC1
	CALL	_tmr0_check_delay
	MOVF	POSTINC1, F
	XORLW	0x01
	BNZ	_00250_DS_
;	.line	56; main.c	tmr0_set_delay(0, 250);
	CLRF	POSTDEC1
	MOVLW	0xfa
	MOVWF	POSTDEC1
	MOVLW	0x00
	CLRF	POSTDEC1
	CALL	_tmr0_set_delay
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	57; main.c	OnBoardLED = !OnBoardLED;
	CLRF	r0x00
	BTFSC	_LATCbits, 2
	INCF	r0x00, F
	MOVF	r0x00, W
	BSF	STATUS, 0
	TSTFSZ	WREG
	BCF	STATUS, 0
	CLRF	r0x00
	RLCF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	RLNCF	WREG, W
	RLNCF	WREG, W
	MOVWF	PRODH
	MOVF	_LATCbits, W
	ANDLW	0xfb
	IORWF	PRODH, W
	MOVWF	_LATCbits
	BRA	_00250_DS_
	RETURN	

; ; Starting pCode block
S_main___startup	code
__startup:
	;	Initialize the stack pointer
	lfsr	1, _stack_end
	lfsr	2, _stack_end
	;	1st silicon does not do this on POR
	clrf	_TBLPTRU, 0
	;	Initialize the flash memory access configuration.
	;	This is harmless for non-flash devices, so we do it on all parts.
	bsf	0xa6, 7, 0 ; EECON1.EEPGD = 1, TBLPTR accesses program memory
	bcf	0xa6, 6, 0 ; EECON1.CFGS = 0, TBLPTR accesses program memory
	;	TBLPTR = &cinit
	movlw	low(_cinit)
	movwf	_TBLPTRL, 0
	movlw	high(_cinit)
	movwf	_TBLPTRH, 0
	movlw	upper(_cinit)
	movwf	_TBLPTRU, 0
	;	0x05 = cinit.num_init
	tblrd*+
	movff	_TABLAT, 0x05
	tblrd*+
	movff	_TABLAT, (0x05 + 1)
	;	while (0x05)
	bra	entry_loop_dec
entry_loop:
	;	Count down so we only have to look up the data in _cinit once.
	;	At this point we know that TBLPTR points to the top of the current
	;	entry in _cinit, so we can just start reading the from, to, and
	;	size values.
	;	Read the source address low.
	;	0x00 = 0x07 ->from;
	tblrd*+
	movff	_TABLAT, 0x00
	;	source address high
	tblrd*+
	movff	_TABLAT, (0x00 + 1)
	;	source address upper
	tblrd*+
	movff	_TABLAT, (0x00 + 2)
	;	Skip 0 byte since it is stored as 0 32bit int.
	tblrd*+
	;	Read the destination address directly into FSR0
	;	destination address low.
	;	FSR0 = (unsigned short)0x07 ->to;
	tblrd*+
	movff	_TABLAT, _FSR0L
	;	destination address high
	tblrd*+
	movff	_TABLAT, _FSR0H
	;	Skip two bytes since it is stored as 0 32bit int.
	tblrd*+
	tblrd*+
	;	Read the size of data to transfer to destination address.
	;	0x03 = (unsigned short)0x07 ->size;
	tblrd*+
	movff	_TABLAT, 0x03
	tblrd*+
	movff	_TABLAT, (0x03 + 1)
	;	Skip two bytes since it is stored as 0 32bit int.
	tblrd*+
	tblrd*+
	;	0x00 = 0x07 ->from;
	;	FSR0 = (unsigned short)0x07 ->to;
	;	0x03 = (unsigned short)0x07 ->size;
	;	The table pointer now points to the next entry. Save it
	;	off since we will be using the table pointer to do the copying
	;	for the entry.
	;	0x07 = TBLPTR
	movff	_TBLPTRL, 0x07
	movff	_TBLPTRH, (0x07 + 1)
	movff	_TBLPTRU, (0x07 + 2)
	;	Now assign the source address to the table pointer.
	;	TBLPTR = 0x00
	movff	0x00, _TBLPTRL
	movff	(0x00 + 1), _TBLPTRH
	movff	(0x00 + 2), _TBLPTRU
	bra	copy_loop_dec
copy_loop:
	tblrd*+
	movff	_TABLAT, _POSTINC0
copy_loop_dec:
	;	while (--0x03);
	;	Decrement and test the byte counter.
	;	The cycle ends when the value of counter reaches the -1.
	decf	0x03, f, 0
	bc	copy_loop
	decf	(0x03 + 1), f, 0
	bc	copy_loop
	;	Restore the table pointer for the next entry.
	;	TBLPTR = 0x07
	movff	0x07, _TBLPTRL
	movff	(0x07 + 1), _TBLPTRH
	movff	(0x07 + 2), _TBLPTRU
entry_loop_dec:
	;	while (--0x05);
	;	Decrement and test the entry counter.
	;	The cycle ends when the value of counter reaches the -1.
	decf	0x05, f, 0
	bc	entry_loop
	decf	(0x05 + 1), f, 0
	bc	entry_loop
	
;	.line	249; ../my_sdcc_lib/crt0i.c	main ();
	CALL	_main
lockup:
	;	Returning from main will lock up.
	bra	lockup
	
; ; Starting pCode block
S_main__tmr0_isr_intr	code
_tmr0_isr_intr:
;	.line	113; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	void tmr0_isr_intr(void) {
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
;	.line	116; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	if (INTCONbits.TMR0IF) {
	BTFSS	_INTCONbits, 2
	BRA	_00196_DS_
;	.line	117; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	TMR0L = tmr0_load;
	MOVFF	_tmr0_load, _TMR0L
;	.line	123; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	for (index = 0; index < TMR0_DELAY_SLOTS; index++) {
	CLRF	r0x00
	CLRF	r0x01
	CLRF	r0x02
_00194_DS_:
;	.line	124; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	if (tmr0_isr_countdown[index] > 0)
	MOVLW	LOW(_tmr0_isr_countdown)
	ADDWF	r0x01, W
	MOVWF	r0x03
	MOVLW	HIGH(_tmr0_isr_countdown)
	ADDWFC	r0x02, W
	MOVWF	r0x04
	MOVFF	r0x03, FSR0L
	MOVFF	r0x04, FSR0H
	MOVFF	POSTINC0, r0x05
	MOVFF	INDF0, r0x06
	MOVF	r0x06, W
	ADDLW	0x80
	ADDLW	0x80
	BNZ	_00211_DS_
	MOVLW	0x01
	SUBWF	r0x05, W
_00211_DS_:
	BNC	_00195_DS_
;	.line	125; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	tmr0_isr_countdown[index] = tmr0_isr_countdown[index] - 1;
	MOVLW	0xff
	ADDWF	r0x05, F
	ADDWFC	r0x06, F
	MOVFF	r0x03, FSR0L
	MOVFF	r0x04, FSR0H
	MOVFF	r0x05, POSTINC0
	MOVFF	r0x06, INDF0
_00195_DS_:
;	.line	123; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	for (index = 0; index < TMR0_DELAY_SLOTS; index++) {
	MOVLW	0x02
	ADDWF	r0x01, F
	BTFSC	STATUS, 0
	INCF	r0x02, F
	INCF	r0x00, F
	MOVLW	0x01
	SUBWF	r0x00, W
	BNC	_00194_DS_
;	.line	128; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
_00196_DS_:
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	RETURN	

; ; Starting pCode block
S_main__tmr0_isr_init	code
_tmr0_isr_init:
;	.line	61; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	void tmr0_isr_init(void) {
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
;	.line	75; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	T0CONbits.T0PS = 5; // prescaler 64
	MOVF	_T0CONbits, W
	ANDLW	0xf8
	IORLW	0x05
	MOVWF	_T0CONbits
;	.line	76; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	tmr0_load = 255 - (uint8_t) (tmr0_div / 64);
	MOVLW	0x06
	BANKSEL	_tmr0_load
	MOVWF	_tmr0_load, B
;	.line	98; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	T0CONbits.T08BIT = 1;
	BSF	_T0CONbits, 6
;	.line	99; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	T0CONbits.T0CS = 0; // internal clock
	BCF	_T0CONbits, 5
;	.line	100; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	T0CONbits.PSA = 0; // assign prescaler to timer0
	BCF	_T0CONbits, 3
;	.line	101; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	INTCONbits.RBIF = 0;
	BCF	_INTCONbits, 0
;	.line	102; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	103; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	INTCONbits.TMR0IE = 1;
	BSF	_INTCONbits, 5
;	.line	108; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	for (i = 0; i < TMR0_DELAY_SLOTS; i++) tmr0_isr_countdown[i] = 0;
	CLRF	r0x00
	CLRF	r0x01
	CLRF	r0x02
_00176_DS_:
	MOVLW	LOW(_tmr0_isr_countdown)
	ADDWF	r0x01, W
	MOVWF	r0x03
	MOVLW	HIGH(_tmr0_isr_countdown)
	ADDWFC	r0x02, W
	MOVWF	r0x04
	MOVFF	r0x03, FSR0L
	MOVFF	r0x04, FSR0H
	CLRF	POSTINC0
	CLRF	INDF0
	MOVLW	0x02
	ADDWF	r0x01, F
	BTFSC	STATUS, 0
	INCF	r0x02, F
	INCF	r0x00, F
	MOVLW	0x01
	SUBWF	r0x00, W
	BNC	_00176_DS_
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	RETURN	

; ; Starting pCode block
S_main__tmr0_check_delay	code
_tmr0_check_delay:
;	.line	48; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	bool_t tmr0_check_delay(uint8_t slot) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	49; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	if (slot >= TMR0_DELAY_SLOTS) return (TRUE);
	MOVLW	0x01
	SUBWF	r0x00, W
	BNC	_00154_DS_
	MOVLW	0x01
	BRA	_00159_DS_
; ;multiply lit val:0x02 by variable r0x00 and store in r0x00
_00154_DS_:
;	.line	50; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	if (tmr0_isr_countdown[slot] == 0) {
	MOVF	r0x00, W
	MULLW	0x02
	MOVF	PRODH, W
	MOVWF	r0x01
	MOVFF	PRODL, r0x00
	MOVLW	LOW(_tmr0_isr_countdown)
	ADDWF	r0x00, F
	MOVLW	HIGH(_tmr0_isr_countdown)
	ADDWFC	r0x01, F
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, FSR0H
	MOVFF	POSTINC0, r0x00
	MOVFF	INDF0, r0x01
	MOVF	r0x00, W
	IORWF	r0x01, W
	BNZ	_00158_DS_
;	.line	51; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	if (tmr0_isr_countdown[slot] == 0) {
	MOVF	r0x00, W
	IORWF	r0x01, W
	BNZ	_00158_DS_
;	.line	54; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	return (TRUE); //-- delay passed
	MOVLW	0x01
	BRA	_00159_DS_
_00158_DS_:
;	.line	57; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	return (FALSE); //-- still waiting
	CLRF	WREG
_00159_DS_:
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__tmr0_set_delay	code
_tmr0_set_delay:
;	.line	41; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	void tmr0_set_delay(uint8_t slot, uint16_t ticks) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
;	.line	42; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	if (slot >= TMR0_DELAY_SLOTS) slot = 0;
	MOVLW	0x01
; #	SUBWF	r0x00, W
; #	BTFSS	STATUS, 0
; #	GOTO	_00142_DS_
; #	CLRF	r0x00
; #	BCF	_INTCONbits, 5
; #;;multiply lit val:0x02 by variable r0x00 and store in r0x00
;	.line	43; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	INTCONbits.TMR0IE = 0;
	SUBWF	r0x00, W
	BTFSC	STATUS, 0
	CLRF	r0x00
	BCF	_INTCONbits, 5
;	.line	44; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	tmr0_isr_countdown[slot] = ticks;
	MOVF	r0x00, W
	MULLW	0x02
	MOVF	PRODH, W
	MOVWF	r0x03
	MOVFF	PRODL, r0x00
	MOVLW	LOW(_tmr0_isr_countdown)
	ADDWF	r0x00, F
	MOVLW	HIGH(_tmr0_isr_countdown)
	ADDWFC	r0x03, F
	MOVFF	r0x00, FSR0L
	MOVFF	r0x03, FSR0H
	MOVFF	r0x01, POSTINC0
	MOVFF	r0x02, INDF0
;	.line	45; ../my_sdcc_lib/rosso_sdcc_isr_delay.h	INTCONbits.TMR0IE = 1;
	BSF	_INTCONbits, 5
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main___delay_ms	code
__delay_ms:
;	.line	174; ../my_sdcc_lib/rosso_sdcc.h	void _delay_ms(uint16_t x){
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
;	.line	176; ../my_sdcc_lib/rosso_sdcc.h	for(i=0; i<x; i++){
	CLRF	r0x02
	CLRF	r0x03
_00125_DS_:
	MOVF	r0x01, W
	SUBWF	r0x03, W
	BNZ	_00136_DS_
	MOVF	r0x00, W
	SUBWF	r0x02, W
_00136_DS_:
	BC	_00127_DS_
;	.line	177; ../my_sdcc_lib/rosso_sdcc.h	delay_1ms();
	MOVLW	0xa0
	CALL	_delay100tcy
;	.line	176; ../my_sdcc_lib/rosso_sdcc.h	for(i=0; i<x; i++){
	INFSNZ	r0x02, F
	INCF	r0x03, F
	BRA	_00125_DS_
_00127_DS_:
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main___delay_us	code
__delay_us:
;	.line	167; ../my_sdcc_lib/rosso_sdcc.h	void _delay_us(uint16_t x){
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
;	.line	169; ../my_sdcc_lib/rosso_sdcc.h	for(i=0; i<x; i++){
	CLRF	r0x02
	CLRF	r0x03
_00107_DS_:
	MOVF	r0x01, W
	SUBWF	r0x03, W
	BNZ	_00118_DS_
	MOVF	r0x00, W
	SUBWF	r0x02, W
_00118_DS_:
	BC	_00109_DS_
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
;	.line	169; ../my_sdcc_lib/rosso_sdcc.h	for(i=0; i<x; i++){
	INFSNZ	r0x02, F
	INCF	r0x03, F
	BRA	_00107_DS_
_00109_DS_:
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	



; Statistics:
; code size:	 1144 (0x0478) bytes ( 0.87%)
;           	  572 (0x023c) words
; udata size:	    3 (0x0003) bytes ( 0.08%)
; access size:	    7 (0x0007) bytes


	end
