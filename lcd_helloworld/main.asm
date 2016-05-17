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
	global	_LCD_POS
	global	_s
	global	_sr
	global	__delay_us
	global	__delay_ms
	global	___lcd_write_nibble
	global	___lcd_write
	global	__lcd_write_data
	global	__lcd_write_command
	global	__lcd_line2index
	global	__lcd_restore_cursor
	global	_lcd_write_char
	global	_lcd_write_str
	global	_lcd_write_strF
	global	_lcd_cursor_position
	global	_lcd_shift_left
	global	_lcd_shift_right
	global	_lcd_cursor_shift_left
	global	_lcd_cursor_shift_right
	global	_lcd_clear_screen
	global	_lcd_cursor_blink_display
	global	_lcd_home
	global	_lcd_clear_line
	global	_lcd_progress
	global	_lcd_init
	global	_dectobcd
	global	_bcdtodec
	global	_nibble2hex
	global	_byte2dec
	global	_word2dec
	global	_double2dec
	global	_double2hex
	global	_word2hex
	global	_byte2hex
	global	_sf

;--------------------------------------------------------
; extern variables in this module
;--------------------------------------------------------
	extern	__gptrget1
	extern	__divuchar
	extern	__moduchar
	extern	__gptrput1
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
	extern	_delay10tcy
	extern	_delay100tcy
	extern	_delay1ktcy
	extern	_delay10ktcy
	extern	__moduint
	extern	__divuint
	extern	__modulong
	extern	__divulong
	extern	_cinit

;--------------------------------------------------------
;	Equates to used internal registers
;--------------------------------------------------------
STATUS	equ	0xfd8
WREG	equ	0xfe8
FSR0L	equ	0xfe9
FSR0H	equ	0xfea
FSR1L	equ	0xfe1
FSR2L	equ	0xfd9
INDF0	equ	0xfef
POSTINC1	equ	0xfe6
POSTDEC1	equ	0xfe5
PREINC1	equ	0xfe4
PLUSW2	equ	0xfdb
PRODL	equ	0xff3
PRODH	equ	0xff4


	idata
___uflags	db	0x00
_sr	db	0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20, 0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21
	db	0x00, 0x00


; Internal registers
.registers	udata_ovr	0x0000
r0x00	res	1
r0x01	res	1
r0x02	res	1
r0x03	res	1
r0x04	res	1
r0x05	res	1
r0x06	res	1
r0x07	res	1
r0x08	res	1
r0x09	res	1
r0x0a	res	1

udata_main_0	udata
___lcd_write_nibble_nibble_1_14	res	1

udata_main_1	udata
_LCD_POS	res	1

udata_main_2	udata
_s	res	4

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
	
; I code from now on!
; ; Starting pCode block
S_main__main	code
_main:
	BANKSEL	_ANSELA
;	.line	49; main.c	AllDigital(); /* all pins digital */
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
;	.line	51; main.c	OnBoardLED_dir = 0; /* output */
	BCF	_TRISCbits, 2
;	.line	52; main.c	OnBoardButton_dir = 1; /* input */
	BSF	_TRISAbits, 4
;	.line	53; main.c	OnBoardLED = 0;
	BCF	_LATCbits, 2
;	.line	57; main.c	lcd_init(LCD_HD44780);
	MOVLW	0x00
	CLRF	POSTDEC1
	CALL	_lcd_init
	MOVF	POSTINC1, F
;	.line	59; main.c	for (i = 0; i < 4; i++) {
	CLRF	r0x00
_00606_DS_:
;	.line	60; main.c	OnBoardLED = 1;
	BSF	_LATCbits, 2
;	.line	61; main.c	delay_100ms();
	MOVLW	0xa0
	CALL	_delay10ktcy
;	.line	62; main.c	delay_100ms();
	MOVLW	0xa0
	CALL	_delay10ktcy
;	.line	63; main.c	OnBoardLED = 0;
	BCF	_LATCbits, 2
;	.line	64; main.c	delay_100ms();
	MOVLW	0xa0
	CALL	_delay10ktcy
;	.line	65; main.c	delay_100ms();
	MOVLW	0xa0
	CALL	_delay10ktcy
;	.line	59; main.c	for (i = 0; i < 4; i++) {
	INCF	r0x00, F
	MOVLW	0x04
	SUBWF	r0x00, W
	BNC	_00606_DS_
;	.line	67; main.c	lcd_cursor_position(0, 0);
	MOVLW	0x00
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CALL	_lcd_cursor_position
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	68; main.c	lcd_write_str(sr);  /* reading the string from the RAM */
	MOVLW	0x80
; #	MOVWF	r0x02
; #	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVLW	HIGH(_sr)
	MOVWF	POSTDEC1
	MOVLW	LOW(_sr)
	MOVWF	POSTDEC1
	CALL	_lcd_write_str
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	69; main.c	lcd_write_strF(sf); /* reading the string from the FLASH */
	MOVLW	UPPER(_sf)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_sf)
	MOVWF	POSTDEC1
	MOVLW	LOW(_sf)
	MOVWF	POSTDEC1
	CALL	_lcd_write_strF
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	70; main.c	while (1) {
	CLRF	r0x00
_00604_DS_:
;	.line	72; main.c	counter += 1; /* count up to 255 and start again from zer0 */
	INCF	r0x00, F
;	.line	73; main.c	lcd_cursor_position(1, 0);
	CLRF	POSTDEC1
	MOVLW	0x01
	MOVWF	POSTDEC1
	CALL	_lcd_cursor_position
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	74; main.c	byte2dec(counter, s);
	MOVLW	0x80
; #	MOVWF	r0x03
; #	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVLW	HIGH(_s)
	MOVWF	POSTDEC1
	MOVLW	LOW(_s)
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_byte2dec
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	75; main.c	for (i = 0; i < 3; i++) _lcd_write_data(s[i]);
	CLRF	r0x01
_00608_DS_:
	MOVLW	LOW(_s)
	ADDWF	r0x01, W
	MOVWF	r0x02
	CLRF	r0x03
	MOVLW	HIGH(_s)
	ADDWFC	r0x03, F
	MOVFF	r0x02, FSR0L
	MOVFF	r0x03, FSR0H
	MOVFF	INDF0, r0x02
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	CALL	__lcd_write_data
	MOVF	POSTINC1, F
	INCF	r0x01, F
	MOVLW	0x03
	SUBWF	r0x01, W
	BNC	_00608_DS_
;	.line	76; main.c	delay_150ms();
	MOVLW	0xf0
	CALL	_delay10ktcy
;	.line	77; main.c	delay_150ms();
	MOVLW	0xf0
	CALL	_delay10ktcy
	BRA	_00604_DS_
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
S_main__byte2hex	code
_byte2hex:
;	.line	287; ../my_sdcc_lib/rosso_sdcc_conversion.h	void byte2hex(uint8_t val, uint8_t *s) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
;	.line	288; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = nibble2hex(val >> 4);
	SWAPF	r0x00, W
	ANDLW	0x0f
; #	MOVWF	r0x04
; #	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x04
	MOVF	POSTINC1, F
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, PRODL
	MOVF	r0x03, W
	CALL	__gptrput1
;	.line	289; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = nibble2hex(val);
	MOVF	r0x01, W
	ADDLW	0x01
	MOVWF	r0x04
	MOVLW	0x00
	ADDWFC	r0x02, W
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x00
	MOVF	POSTINC1, F
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	290; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = 0;
	MOVLW	0x02
	ADDWF	r0x01, F
	MOVLW	0x00
	ADDWFC	r0x02, F
	ADDWFC	r0x03, F
	CLRF	POSTDEC1
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, PRODL
	MOVF	r0x03, W
	CALL	__gptrput1
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__word2hex	code
_word2hex:
;	.line	279; ../my_sdcc_lib/rosso_sdcc_conversion.h	void word2hex(uint16_t val, uint8_t *s) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x08, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
	MOVLW	0x06
	MOVFF	PLUSW2, r0x04
;	.line	280; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = nibble2hex(val >> 12);
	SWAPF	r0x01, W
	ANDLW	0x0f
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x05
	MOVF	POSTINC1, F
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x02, FSR0L
	MOVFF	r0x03, PRODL
	MOVF	r0x04, W
	CALL	__gptrput1
;	.line	281; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = nibble2hex(val >> 8);
	MOVF	r0x02, W
	ADDLW	0x01
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x08
	MOVF	POSTINC1, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	282; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = nibble2hex(val >> 4);
	MOVF	r0x02, W
	ADDLW	0x02
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	SWAPF	r0x00, W
	ANDLW	0x0f
	MOVWF	r0x08
	SWAPF	r0x01, W
	ANDLW	0xf0
	ADDWF	r0x08, F
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x08
	MOVF	POSTINC1, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	283; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = nibble2hex(val);
	MOVF	r0x02, W
	ADDLW	0x03
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x00
	MOVF	POSTINC1, F
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	284; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = 0;
	MOVLW	0x04
	ADDWF	r0x02, F
	MOVLW	0x00
	ADDWFC	r0x03, F
	ADDWFC	r0x04, F
	CLRF	POSTDEC1
	MOVFF	r0x02, FSR0L
	MOVFF	r0x03, PRODL
	MOVF	r0x04, W
	CALL	__gptrput1
	MOVFF	PREINC1, r0x08
	MOVFF	PREINC1, r0x07
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__double2hex	code
_double2hex:
;	.line	267; ../my_sdcc_lib/rosso_sdcc_conversion.h	void double2hex(uint32_t val, uint8_t *s) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x09, POSTDEC1
	MOVFF	r0x0a, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
	MOVLW	0x06
	MOVFF	PLUSW2, r0x04
	MOVLW	0x07
	MOVFF	PLUSW2, r0x05
	MOVLW	0x08
	MOVFF	PLUSW2, r0x06
;	.line	268; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = nibble2hex(val >> 28);
	SWAPF	r0x03, W
	ANDLW	0x0f
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x07
	MOVF	POSTINC1, F
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	269; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = nibble2hex(val >> 24);
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x0a
	MOVF	POSTINC1, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	270; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = nibble2hex(val >> 20);
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	SWAPF	r0x02, W
	ANDLW	0x0f
	MOVWF	r0x0a
	SWAPF	r0x03, W
	ANDLW	0xf0
	ADDWF	r0x0a, F
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x0a
	MOVF	POSTINC1, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	271; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = nibble2hex(val >> 16);
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x0a
	MOVF	POSTINC1, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	272; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = nibble2hex(val >> 12);
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	SWAPF	r0x01, W
	ANDLW	0x0f
	MOVWF	r0x0a
	SWAPF	r0x02, W
	ANDLW	0xf0
	ADDWF	r0x0a, F
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x0a
	MOVF	POSTINC1, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	273; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = nibble2hex(val >> 8);
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x0a
	MOVF	POSTINC1, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	274; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = nibble2hex(val >> 4);
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	SWAPF	r0x00, W
	ANDLW	0x0f
	MOVWF	r0x0a
	SWAPF	r0x01, W
	ANDLW	0xf0
	ADDWF	r0x0a, F
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x0a
	MOVF	POSTINC1, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	275; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = nibble2hex(val);
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_nibble2hex
	MOVWF	r0x00
	MOVF	POSTINC1, F
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	276; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = 0;
	MOVLW	0x08
	ADDWF	r0x04, F
	MOVLW	0x00
	ADDWFC	r0x05, F
	ADDWFC	r0x06, F
	CLRF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
	MOVFF	PREINC1, r0x0a
	MOVFF	PREINC1, r0x09
	MOVFF	PREINC1, r0x08
	MOVFF	PREINC1, r0x07
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__double2dec	code
_double2dec:
;	.line	98; ../my_sdcc_lib/rosso_sdcc_conversion.h	void double2dec(uint32_t val, uint8_t *s) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x09, POSTDEC1
	MOVFF	r0x0a, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
	MOVLW	0x06
	MOVFF	PLUSW2, r0x04
	MOVLW	0x07
	MOVFF	PLUSW2, r0x05
	MOVLW	0x08
	MOVFF	PLUSW2, r0x06
;	.line	99; ../my_sdcc_lib/rosso_sdcc_conversion.h	if (val > 999999999) {
	MOVLW	0x3b
	SUBWF	r0x03, W
	BNZ	_00563_DS_
	MOVLW	0x9a
	SUBWF	r0x02, W
	BNZ	_00563_DS_
	MOVLW	0xca
	SUBWF	r0x01, W
	BNZ	_00563_DS_
	MOVLW	0x00
	SUBWF	r0x00, W
_00563_DS_:
	BTFSS	STATUS, 0
	GOTO	_00532_DS_
;	.line	100; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	101; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	102; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	103; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	104; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	105; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	106; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	107; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	108; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	109; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	110; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	111; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	112; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	113; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	114; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	115; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	116; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	117; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	118; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = '0' + val;
	MOVF	r0x00, W
	MOVWF	r0x07
	MOVLW	0x30
	ADDWF	r0x07, F
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	119; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVF	r0x04, W
	ADDLW	0x0a
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	CLRF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	CALL	__gptrput1
	GOTO	_00534_DS_
_00532_DS_:
;	.line	120; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 99999999) {
	MOVLW	0x05
	SUBWF	r0x03, W
	BNZ	_00564_DS_
	MOVLW	0xf5
	SUBWF	r0x02, W
	BNZ	_00564_DS_
	MOVLW	0xe1
	SUBWF	r0x01, W
	BNZ	_00564_DS_
	MOVLW	0x00
	SUBWF	r0x00, W
_00564_DS_:
	BTFSS	STATUS, 0
	GOTO	_00529_DS_
;	.line	121; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	122; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	123; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	124; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	125; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	126; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	127; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	128; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	129; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	130; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	131; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	132; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	133; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	134; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	135; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	136; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	137; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = '0' + val;
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x00, W
	MOVWF	r0x0a
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	138; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	139; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVF	r0x04, W
	ADDLW	0x0a
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	CLRF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	CALL	__gptrput1
	GOTO	_00534_DS_
_00529_DS_:
;	.line	140; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 9999999) {
	MOVLW	0x00
	SUBWF	r0x03, W
	BNZ	_00565_DS_
	MOVLW	0x98
	SUBWF	r0x02, W
	BNZ	_00565_DS_
	MOVLW	0x96
	SUBWF	r0x01, W
	BNZ	_00565_DS_
	MOVLW	0x80
	SUBWF	r0x00, W
_00565_DS_:
	BTFSS	STATUS, 0
	BRA	_00526_DS_
;	.line	141; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	142; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	143; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	144; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	145; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	146; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	147; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	148; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	149; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	150; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	151; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	152; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	153; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	154; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	155; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = '0' + val;
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x00, W
	MOVWF	r0x0a
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	156; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	157; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	158; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVF	r0x04, W
	ADDLW	0x0a
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	CLRF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	CALL	__gptrput1
	GOTO	_00534_DS_
_00526_DS_:
;	.line	159; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 999999) {
	MOVLW	0x00
	SUBWF	r0x03, W
	BNZ	_00566_DS_
	MOVLW	0x0f
	SUBWF	r0x02, W
	BNZ	_00566_DS_
	MOVLW	0x42
	SUBWF	r0x01, W
	BNZ	_00566_DS_
	MOVLW	0x40
	SUBWF	r0x00, W
_00566_DS_:
	BTFSS	STATUS, 0
	BRA	_00523_DS_
;	.line	160; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	161; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	162; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	163; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	164; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	165; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	166; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	167; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	168; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	169; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	170; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	171; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	172; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = '0' + val;
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x00, W
	MOVWF	r0x0a
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	173; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = ' ';
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	174; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	175; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	176; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVF	r0x04, W
	ADDLW	0x0a
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	CLRF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	CALL	__gptrput1
	GOTO	_00534_DS_
_00523_DS_:
;	.line	177; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 99999) {
	MOVLW	0x00
	SUBWF	r0x03, W
	BNZ	_00567_DS_
	MOVLW	0x01
	SUBWF	r0x02, W
	BNZ	_00567_DS_
	MOVLW	0x86
	SUBWF	r0x01, W
	BNZ	_00567_DS_
	MOVLW	0xa0
	SUBWF	r0x00, W
_00567_DS_:
	BTFSS	STATUS, 0
	BRA	_00520_DS_
;	.line	178; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	179; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	180; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	181; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	182; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	183; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	184; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	185; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	186; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	187; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	188; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + val;
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x00, W
	MOVWF	r0x0a
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	189; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = ' ';
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	190; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = ' ';
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	191; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	192; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	193; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVF	r0x04, W
	ADDLW	0x0a
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	CLRF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	CALL	__gptrput1
	GOTO	_00534_DS_
_00520_DS_:
;	.line	194; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 9999) {
	MOVLW	0x00
	SUBWF	r0x03, W
	BNZ	_00568_DS_
	MOVLW	0x00
	SUBWF	r0x02, W
	BNZ	_00568_DS_
	MOVLW	0x27
	SUBWF	r0x01, W
	BNZ	_00568_DS_
	MOVLW	0x10
	SUBWF	r0x00, W
_00568_DS_:
	BTFSS	STATUS, 0
	BRA	_00517_DS_
;	.line	195; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	196; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	197; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	198; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	199; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	200; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	201; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	202; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	203; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = '0' + val;
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x00, W
	MOVWF	r0x0a
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	204; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = ' ';
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	205; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = ' ';
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	206; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = ' ';
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	207; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	208; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	209; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVF	r0x04, W
	ADDLW	0x0a
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	CLRF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	CALL	__gptrput1
	GOTO	_00534_DS_
_00517_DS_:
;	.line	210; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 999) {
	MOVLW	0x00
	SUBWF	r0x03, W
	BNZ	_00569_DS_
	MOVLW	0x00
	SUBWF	r0x02, W
	BNZ	_00569_DS_
	MOVLW	0x03
	SUBWF	r0x01, W
	BNZ	_00569_DS_
	MOVLW	0xe8
	SUBWF	r0x00, W
_00569_DS_:
	BTFSS	STATUS, 0
	BRA	_00514_DS_
;	.line	211; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	212; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	213; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	214; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	215; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	216; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	217; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = '0' + val;
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x00, W
	MOVWF	r0x0a
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	218; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = ' ';
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	219; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = ' ';
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	220; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = ' ';
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	221; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = ' ';
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	222; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	223; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	224; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVF	r0x04, W
	ADDLW	0x0a
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	CLRF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	CALL	__gptrput1
	GOTO	_00534_DS_
_00514_DS_:
;	.line	225; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 99) {
	MOVLW	0x00
	SUBWF	r0x03, W
	BNZ	_00570_DS_
	MOVLW	0x00
	SUBWF	r0x02, W
	BNZ	_00570_DS_
	MOVLW	0x00
	SUBWF	r0x01, W
	BNZ	_00570_DS_
	MOVLW	0x64
	SUBWF	r0x00, W
_00570_DS_:
	BTFSS	STATUS, 0
	BRA	_00511_DS_
;	.line	226; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	227; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	228; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	229; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	230; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = '0' + val;
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x00, W
	MOVWF	r0x0a
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	231; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = ' ';
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	232; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = ' ';
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	233; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = ' ';
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	234; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = ' ';
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	235; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = ' ';
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	236; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	237; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	238; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVF	r0x04, W
	ADDLW	0x0a
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	CLRF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	CALL	__gptrput1
	BRA	_00534_DS_
_00511_DS_:
;	.line	239; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 9) {
	MOVLW	0x00
	SUBWF	r0x03, W
	BNZ	_00571_DS_
	MOVLW	0x00
	SUBWF	r0x02, W
	BNZ	_00571_DS_
	MOVLW	0x00
	SUBWF	r0x01, W
	BNZ	_00571_DS_
	MOVLW	0x0a
	SUBWF	r0x00, W
_00571_DS_:
	BTFSS	STATUS, 0
	BRA	_00508_DS_
;	.line	240; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + (val % 10);
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x0a
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	241; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	242; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = '0' + val;
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVF	r0x00, W
	MOVWF	r0x0a
	MOVLW	0x30
	ADDWF	r0x0a, F
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	243; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = ' ';
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	244; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = ' ';
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	245; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = ' ';
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	246; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = ' ';
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	247; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = ' ';
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	248; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = ' ';
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	249; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	250; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	251; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVF	r0x04, W
	ADDLW	0x0a
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	CLRF	POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	CALL	__gptrput1
	BRA	_00534_DS_
_00508_DS_:
;	.line	253; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[9] = '0' + val;
	MOVF	r0x04, W
	ADDLW	0x09
	MOVWF	r0x07
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x08
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x09
	MOVLW	0x30
	ADDWF	r0x00, F
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x07, FSR0L
	MOVFF	r0x08, PRODL
	MOVF	r0x09, W
	CALL	__gptrput1
;	.line	254; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[8] = ' ';
	MOVF	r0x04, W
	ADDLW	0x08
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x02
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	255; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[7] = ' ';
	MOVF	r0x04, W
	ADDLW	0x07
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x02
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	256; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[6] = ' ';
	MOVF	r0x04, W
	ADDLW	0x06
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x02
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	257; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = ' ';
	MOVF	r0x04, W
	ADDLW	0x05
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x02
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	258; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = ' ';
	MOVF	r0x04, W
	ADDLW	0x04
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x02
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	259; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = ' ';
	MOVF	r0x04, W
	ADDLW	0x03
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x02
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	260; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = ' ';
	MOVF	r0x04, W
	ADDLW	0x02
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x02
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	261; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x04, W
	ADDLW	0x01
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x05, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x06, W
	MOVWF	r0x02
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	262; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	263; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[10] = 0;
	MOVLW	0x0a
	ADDWF	r0x04, F
	MOVLW	0x00
	ADDWFC	r0x05, F
	ADDWFC	r0x06, F
	CLRF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
_00534_DS_:
	MOVFF	PREINC1, r0x0a
	MOVFF	PREINC1, r0x09
	MOVFF	PREINC1, r0x08
	MOVFF	PREINC1, r0x07
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__word2dec	code
_word2dec:
;	.line	49; ../my_sdcc_lib/rosso_sdcc_conversion.h	void word2dec(uint16_t val, uint8_t *s) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x08, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
	MOVLW	0x06
	MOVFF	PLUSW2, r0x04
;	.line	50; ../my_sdcc_lib/rosso_sdcc_conversion.h	if (val > 9999) {
	MOVLW	0x27
	SUBWF	r0x01, W
	BNZ	_00499_DS_
	MOVLW	0x10
	SUBWF	r0x00, W
_00499_DS_:
	BTFSS	STATUS, 0
	BRA	_00483_DS_
;	.line	51; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x04
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	52; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	53; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x03
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	54; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	55; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x02
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	56; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	57; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x01
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	58; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	59; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = '0' + val;
	MOVF	r0x00, W
	MOVWF	r0x05
	MOVLW	0x30
	ADDWF	r0x05, F
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x02, FSR0L
	MOVFF	r0x03, PRODL
	MOVF	r0x04, W
	CALL	__gptrput1
;	.line	60; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = 0;
	MOVF	r0x02, W
	ADDLW	0x05
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	CLRF	POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	CALL	__gptrput1
	GOTO	_00485_DS_
_00483_DS_:
;	.line	61; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 999) {
	MOVLW	0x03
	SUBWF	r0x01, W
	BNZ	_00500_DS_
	MOVLW	0xe8
	SUBWF	r0x00, W
_00500_DS_:
	BTFSS	STATUS, 0
	BRA	_00480_DS_
;	.line	62; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x04
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	63; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	64; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x03
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	65; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	66; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x02
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	67; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	68; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = '0' + val;
	MOVF	r0x02, W
	ADDLW	0x01
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	MOVF	r0x00, W
	MOVWF	r0x08
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	69; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x02, FSR0L
	MOVFF	r0x03, PRODL
	MOVF	r0x04, W
	CALL	__gptrput1
;	.line	70; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = 0;
	MOVF	r0x02, W
	ADDLW	0x05
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	CLRF	POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	CALL	__gptrput1
	BRA	_00485_DS_
_00480_DS_:
;	.line	71; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 99) {
	MOVLW	0x00
	SUBWF	r0x01, W
	BNZ	_00501_DS_
	MOVLW	0x64
	SUBWF	r0x00, W
_00501_DS_:
	BTFSS	STATUS, 0
	BRA	_00477_DS_
;	.line	72; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x04
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	73; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	74; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x03
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	75; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	76; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = '0' + val;
	MOVF	r0x02, W
	ADDLW	0x02
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	MOVF	r0x00, W
	MOVWF	r0x08
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	77; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x02, W
	ADDLW	0x01
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	78; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x02, FSR0L
	MOVFF	r0x03, PRODL
	MOVF	r0x04, W
	CALL	__gptrput1
;	.line	79; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = 0;
	MOVF	r0x02, W
	ADDLW	0x05
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	CLRF	POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	CALL	__gptrput1
	BRA	_00485_DS_
_00477_DS_:
;	.line	80; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 9) {
	MOVLW	0x00
	SUBWF	r0x01, W
	BNZ	_00502_DS_
	MOVLW	0x0a
	SUBWF	r0x00, W
_00502_DS_:
	BTFSS	STATUS, 0
	BRA	_00474_DS_
;	.line	81; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + (val % 10);
	MOVF	r0x02, W
	ADDLW	0x04
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__moduint
	MOVWF	r0x08
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	82; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	CLRF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divuint
	MOVWF	r0x00
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	83; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = '0' + val;
	MOVF	r0x02, W
	ADDLW	0x03
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	MOVF	r0x00, W
	MOVWF	r0x08
	MOVLW	0x30
	ADDWF	r0x08, F
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	84; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = ' ';
	MOVF	r0x02, W
	ADDLW	0x02
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	85; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x02, W
	ADDLW	0x01
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	86; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x02, FSR0L
	MOVFF	r0x03, PRODL
	MOVF	r0x04, W
	CALL	__gptrput1
;	.line	87; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = 0;
	MOVF	r0x02, W
	ADDLW	0x05
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	CLRF	POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	CALL	__gptrput1
	BRA	_00485_DS_
_00474_DS_:
;	.line	89; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[4] = '0' + val;
	MOVF	r0x02, W
	ADDLW	0x04
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x07
	MOVLW	0x30
	ADDWF	r0x00, F
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, PRODL
	MOVF	r0x07, W
	CALL	__gptrput1
;	.line	90; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = ' ';
	MOVF	r0x02, W
	ADDLW	0x03
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x05
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x05, W
	CALL	__gptrput1
;	.line	91; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = ' ';
	MOVF	r0x02, W
	ADDLW	0x02
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x05
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x05, W
	CALL	__gptrput1
;	.line	92; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x02, W
	ADDLW	0x01
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x01
	MOVLW	0x00
	ADDWFC	r0x04, W
	MOVWF	r0x05
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x05, W
	CALL	__gptrput1
;	.line	93; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x02, FSR0L
	MOVFF	r0x03, PRODL
	MOVF	r0x04, W
	CALL	__gptrput1
;	.line	94; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[5] = 0;
	MOVLW	0x05
	ADDWF	r0x02, F
	MOVLW	0x00
	ADDWFC	r0x03, F
	ADDWFC	r0x04, F
	CLRF	POSTDEC1
	MOVFF	r0x02, FSR0L
	MOVFF	r0x03, PRODL
	MOVF	r0x04, W
	CALL	__gptrput1
_00485_DS_:
	MOVFF	PREINC1, r0x08
	MOVFF	PREINC1, r0x07
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__byte2dec	code
_byte2dec:
;	.line	27; ../my_sdcc_lib/rosso_sdcc_conversion.h	void byte2dec(uint8_t val, uint8_t *s) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVFF	r0x07, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
;	.line	28; ../my_sdcc_lib/rosso_sdcc_conversion.h	if (val > 99) {
	MOVLW	0x64
	SUBWF	r0x00, W
	BTFSS	STATUS, 0
	BRA	_00457_DS_
;	.line	29; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = '0' + (val % 10);
	MOVF	r0x01, W
	ADDLW	0x02
	MOVWF	r0x04
	MOVLW	0x00
	ADDWFC	r0x02, W
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVFF	r0x00, POSTDEC1
	CALL	__moduchar
	MOVWF	r0x07
	MOVF	PREINC1, W
	MOVF	PREINC1, W
	MOVLW	0x30
	ADDWF	r0x07, F
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	30; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVFF	r0x00, POSTDEC1
	CALL	__divuchar
	MOVWF	r0x00
	MOVF	PREINC1, W
	MOVF	PREINC1, W
;	.line	31; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = '0' + (val % 10);
	MOVF	r0x01, W
	ADDLW	0x01
	MOVWF	r0x04
	MOVLW	0x00
	ADDWFC	r0x02, W
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVFF	r0x00, POSTDEC1
	CALL	__moduchar
	MOVWF	r0x07
	MOVF	PREINC1, W
	MOVF	PREINC1, W
	MOVLW	0x30
	ADDWF	r0x07, F
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	32; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVFF	r0x00, POSTDEC1
	CALL	__divuchar
	MOVWF	r0x00
	MOVF	PREINC1, W
	MOVF	PREINC1, W
;	.line	33; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = '0' + val;
	MOVF	r0x00, W
	MOVWF	r0x04
	MOVLW	0x30
	ADDWF	r0x04, F
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, PRODL
	MOVF	r0x03, W
	CALL	__gptrput1
;	.line	34; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = 0;
	MOVF	r0x01, W
	ADDLW	0x03
	MOVWF	r0x04
	MOVLW	0x00
	ADDWFC	r0x02, W
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	CLRF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	CALL	__gptrput1
	BRA	_00459_DS_
_00457_DS_:
;	.line	35; ../my_sdcc_lib/rosso_sdcc_conversion.h	} else if (val > 9) {
	MOVLW	0x0a
	SUBWF	r0x00, W
	BTFSS	STATUS, 0
	BRA	_00454_DS_
;	.line	36; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = '0' + (val % 10);
	MOVF	r0x01, W
	ADDLW	0x02
	MOVWF	r0x04
	MOVLW	0x00
	ADDWFC	r0x02, W
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVFF	r0x00, POSTDEC1
	CALL	__moduchar
	MOVWF	r0x07
	MOVF	PREINC1, W
	MOVF	PREINC1, W
	MOVLW	0x30
	ADDWF	r0x07, F
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	37; ../my_sdcc_lib/rosso_sdcc_conversion.h	val /= 10;
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVFF	r0x00, POSTDEC1
	CALL	__divuchar
	MOVWF	r0x00
	MOVF	PREINC1, W
	MOVF	PREINC1, W
;	.line	38; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = '0' + val;
	MOVF	r0x01, W
	ADDLW	0x01
	MOVWF	r0x04
	MOVLW	0x00
	ADDWFC	r0x02, W
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVF	r0x00, W
	MOVWF	r0x07
	MOVLW	0x30
	ADDWF	r0x07, F
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	39; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, PRODL
	MOVF	r0x03, W
	CALL	__gptrput1
;	.line	40; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = 0;
	MOVF	r0x01, W
	ADDLW	0x03
	MOVWF	r0x04
	MOVLW	0x00
	ADDWFC	r0x02, W
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	CLRF	POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	CALL	__gptrput1
	BRA	_00459_DS_
_00454_DS_:
;	.line	42; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[2] = '0' + val;
	MOVF	r0x01, W
	ADDLW	0x02
	MOVWF	r0x04
	MOVLW	0x00
	ADDWFC	r0x02, W
	MOVWF	r0x05
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x06
	MOVLW	0x30
	ADDWF	r0x00, F
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x04, FSR0L
	MOVFF	r0x05, PRODL
	MOVF	r0x06, W
	CALL	__gptrput1
;	.line	43; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[1] = ' ';
	MOVF	r0x01, W
	ADDLW	0x01
	MOVWF	r0x00
	MOVLW	0x00
	ADDWFC	r0x02, W
	MOVWF	r0x04
	MOVLW	0x00
	ADDWFC	r0x03, W
	MOVWF	r0x05
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x04, PRODL
	MOVF	r0x05, W
	CALL	__gptrput1
;	.line	44; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[0] = ' ';
	MOVLW	0x20
	MOVWF	POSTDEC1
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, PRODL
	MOVF	r0x03, W
	CALL	__gptrput1
;	.line	45; ../my_sdcc_lib/rosso_sdcc_conversion.h	s[3] = 0;
	MOVLW	0x03
	ADDWF	r0x01, F
	MOVLW	0x00
	ADDWFC	r0x02, F
	ADDWFC	r0x03, F
	CLRF	POSTDEC1
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, PRODL
	MOVF	r0x03, W
	CALL	__gptrput1
_00459_DS_:
	MOVFF	PREINC1, r0x07
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__nibble2hex	code
_nibble2hex:
;	.line	19; ../my_sdcc_lib/rosso_sdcc_conversion.h	uint8_t nibble2hex(uint8_t val) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	21; ../my_sdcc_lib/rosso_sdcc_conversion.h	s = '0' + (val & 0xf);
	MOVLW	0x0f
	ANDWF	r0x00, F
	MOVLW	0x30
	ADDWF	r0x00, F
;	.line	22; ../my_sdcc_lib/rosso_sdcc_conversion.h	if (s > '9')
	MOVLW	0x3a
	SUBWF	r0x00, W
	BNC	_00441_DS_
;	.line	23; ../my_sdcc_lib/rosso_sdcc_conversion.h	s += 'A' - '9' - 1;
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVLW	0x07
	ADDWF	r0x01, W
	MOVWF	r0x00
_00441_DS_:
;	.line	24; ../my_sdcc_lib/rosso_sdcc_conversion.h	return s;
	MOVF	r0x00, W
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__bcdtodec	code
_bcdtodec:
;	.line	15; ../my_sdcc_lib/rosso_sdcc_conversion.h	uint8_t bcdtodec(uint8_t pValue) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	16; ../my_sdcc_lib/rosso_sdcc_conversion.h	return ((pValue >> 4) * 10 + (pValue & 0x0F));
	SWAPF	r0x00, W
	ANDLW	0x0f
; #	MOVWF	r0x01
; #;;multiply lit val:0x0a by variable r0x01 and store in r0x01
; #	MOVF	r0x01, W
	MULLW	0x0a
	MOVFF	PRODL, r0x01
	MOVLW	0x0f
	ANDWF	r0x00, F
	MOVF	r0x00, W
	ADDWF	r0x01, F
	MOVF	r0x01, W
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__dectobcd	code
_dectobcd:
;	.line	11; ../my_sdcc_lib/rosso_sdcc_conversion.h	uint8_t dectobcd(uint8_t pValue) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	12; ../my_sdcc_lib/rosso_sdcc_conversion.h	return (((pValue / 10) << 4) | (pValue % 10));
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVFF	r0x00, POSTDEC1
	CALL	__divuchar
	MOVWF	r0x01
	MOVF	PREINC1, W
	MOVF	PREINC1, W
	SWAPF	r0x01, W
	ANDLW	0xf0
	MOVWF	r0x02
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVFF	r0x00, POSTDEC1
	CALL	__moduchar
	MOVWF	r0x00
	MOVF	PREINC1, W
	MOVF	PREINC1, W
	MOVF	r0x00, W
	IORWF	r0x02, F
	MOVF	r0x02, W
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_init	code
_lcd_init:
;	.line	225; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_init(CHIP chipset) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	244; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_RS_DIR = 0;
	BCF	_TRISEbits, 1
;	.line	245; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_EN_DIR = 0;
	BCF	_TRISEbits, 2
;	.line	246; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_D4_DIR = 0;
	BCF	_TRISBbits, 4
;	.line	247; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_D5_DIR = 0;
	BCF	_TRISBbits, 5
;	.line	248; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_D6_DIR = 0;
	BCF	_TRISBbits, 6
;	.line	249; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_D7_DIR = 0;
	BCF	_TRISBbits, 7
;	.line	251; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_RS = 0; // set to control char mode
	BCF	_LATEbits, 1
;	.line	252; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (chipset == LCD_HD44780) {
	MOVF	r0x00, W
	BTFSS	STATUS, 2
	BRA	_00414_DS_
;	.line	253; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_25ms(); // power-up delay (> 15 ms)
	MOVLW	0x28
	CALL	_delay10ktcy
;	.line	254; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000011); // function set
	MOVLW	0x03
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	255; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_5ms(); // > 4.1 milliseconds
	MOVLW	0x50
	CALL	_delay1ktcy
;	.line	256; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000011); // function set
	MOVLW	0x03
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	257; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_100us(); // > 100 us
	MOVLW	0xa0
	CALL	_delay10tcy
;	.line	258; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000011); // function set
	MOVLW	0x03
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	259; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_35us(); // > 37 us
	MOVLW	0x38
	CALL	_delay10tcy
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
;	.line	261; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000010); // select 4-bits mode
	MOVLW	0x02
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	262; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_35us(); // > 37 us
	MOVLW	0x38
	CALL	_delay10tcy
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
;	.line	264; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_command(0b00101000); // 2 lines, 5x8 dots font
	MOVLW	0x28
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
;	.line	265; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_command(0b00011100); // cursor (not data) move right
	MOVLW	0x1c
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
;	.line	266; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_command(0b00001100); // display on, cursor off, no blink
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
;	.line	267; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_command(0b00000110); // cursor shift right, no data shift
	MOVLW	0x06
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
;	.line	269; ../my_sdcc_lib/rosso_sdcc_lcd4.h	lcd_clear_screen(); // clear display
	CALL	_lcd_clear_screen
	BRA	_00416_DS_
_00414_DS_:
;	.line	270; ../my_sdcc_lib/rosso_sdcc_lcd4.h	} else if (chipset == LCD_ST7066U) {
	MOVF	r0x00, W
	XORLW	0x01
	BZ	_00425_DS_
	BRA	_00416_DS_
_00425_DS_:
;	.line	271; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000011);
	MOVLW	0x03
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	272; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_40us();
	MOVLW	0x40
	CALL	_delay10tcy
;	.line	273; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000010);
	MOVLW	0x02
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	274; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00001100);
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	275; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_40us();
	MOVLW	0x40
	CALL	_delay10tcy
;	.line	276; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000010);
	MOVLW	0x02
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	277; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00001100);
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	278; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_35us(); // > 37 us
	MOVLW	0x38
	CALL	_delay10tcy
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
;	.line	280; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000000); // display on / off
	MOVLW	0x00
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	281; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00001100);
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	282; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_35us(); // > 37 us
	MOVLW	0x38
	CALL	_delay10tcy
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
;	.line	284; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000000); // display clear
	MOVLW	0x00
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	285; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000001);
	MOVLW	0x01
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	286; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_1ms();
	MOVLW	0xa0
	CALL	_delay100tcy
;	.line	287; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_500us();
	MOVLW	0x50
	CALL	_delay100tcy
;	.line	288; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_40us();
	MOVLW	0x40
	CALL	_delay10tcy
;	.line	289; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000000); // entry mode set
	MOVLW	0x00
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	290; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000110);
	MOVLW	0x06
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	291; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_35us(); // > 37 us
	MOVLW	0x38
	CALL	_delay10tcy
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
;	.line	293; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000000); // display clear
	MOVLW	0x00
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	294; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(0b00000001);
	MOVLW	0x01
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	295; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_1ms();
	MOVLW	0xa0
	CALL	_delay100tcy
;	.line	296; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_500us();
	MOVLW	0x50
	CALL	_delay100tcy
;	.line	297; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_40us();
	MOVLW	0x40
	CALL	_delay10tcy
_00416_DS_:
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_progress	code
_lcd_progress:
;	.line	217; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_progress(uint8_t line, uint8_t amount, uint8_t pattern) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
;	.line	219; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_POS = _lcd_line2index(line);
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__lcd_line2index
	BANKSEL	_LCD_POS
	MOVWF	_LCD_POS, B
	MOVF	POSTINC1, F
;	.line	220; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_restore_cursor();
	CALL	__lcd_restore_cursor
;	.line	221; ../my_sdcc_lib/rosso_sdcc_lcd4.h	for (i = 0; i < amount; i++) lcd_write_char(pattern);
	CLRF	r0x00
_00384_DS_:
	MOVF	r0x01, W
	SUBWF	r0x00, W
	BC	_00381_DS_
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	CALL	_lcd_write_char
	MOVF	POSTINC1, F
	INCF	r0x00, F
	BRA	_00384_DS_
_00381_DS_:
;	.line	222; ../my_sdcc_lib/rosso_sdcc_lcd4.h	for (i = 0; i < LCD_NR_CHARS - amount; i++) lcd_write_char(' ');
	CLRF	r0x00
_00387_DS_:
	MOVFF	r0x01, r0x02
	CLRF	r0x03
	MOVF	r0x02, W
	SUBLW	0x10
	MOVWF	r0x02
	MOVLW	0x00
	SUBFWB	r0x03, F
	MOVFF	r0x00, r0x04
	ADDLW	0x80
	MOVWF	PRODL
	MOVF	r0x03, W
	ADDLW	0x80
	SUBWF	PRODL, W
	BNZ	_00406_DS_
	MOVF	r0x02, W
	SUBWF	r0x04, W
_00406_DS_:
	BC	_00389_DS_
	MOVLW	0x20
	MOVWF	POSTDEC1
	CALL	_lcd_write_char
	MOVF	POSTINC1, F
	INCF	r0x00, F
	BRA	_00387_DS_
_00389_DS_:
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_clear_line	code
_lcd_clear_line:
;	.line	206; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_clear_line(uint8_t line) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	209; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_POS = _lcd_line2index(line);
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__lcd_line2index
	BANKSEL	_LCD_POS
	MOVWF	_LCD_POS, B
	MOVF	POSTINC1, F
;	.line	210; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_restore_cursor();
	CALL	__lcd_restore_cursor
;	.line	212; ../my_sdcc_lib/rosso_sdcc_lcd4.h	for (i = 0; i < LCD_NR_CHARS; i++) lcd_write_char(' ');
	CLRF	r0x00
_00365_DS_:
	MOVLW	0x20
	MOVWF	POSTDEC1
	CALL	_lcd_write_char
	MOVF	POSTINC1, F
	INCF	r0x00, F
	MOVLW	0x10
	SUBWF	r0x00, W
	BNC	_00365_DS_
;	.line	214; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_restore_cursor();
	CALL	__lcd_restore_cursor
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_home	code
_lcd_home:
;	.line	200; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_command(LCD_RETURN_HOME);
	MOVLW	0x02
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
;	.line	201; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_1ms();
	MOVLW	0xa0
	CALL	_delay100tcy
;	.line	202; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_500us();
	MOVLW	0x50
	CALL	_delay100tcy
;	.line	203; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_300us();
	MOVLW	0x30
	CALL	_delay100tcy
	RETURN	

; ; Starting pCode block
S_main__lcd_cursor_blink_display	code
_lcd_cursor_blink_display:
;	.line	190; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_cursor_blink_display(bit_t cursor, bit_t blink, bit_t display) {
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
;	.line	192; ../my_sdcc_lib/rosso_sdcc_lcd4.h	reg = LCD_DISPLAY_ONOFF;
	MOVLW	0x08
	MOVWF	r0x03
;	.line	193; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (display) reg = reg + 4;
	MOVF	r0x02, W
	BZ	_00349_DS_
	MOVLW	0x0c
	MOVWF	r0x03
_00349_DS_:
;	.line	194; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (cursor) reg = reg + 2;
	MOVF	r0x00, W
	BZ	_00351_DS_
	INCF	r0x03, F
	INCF	r0x03, F
_00351_DS_:
;	.line	195; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (blink) reg = reg + 1;
	MOVF	r0x01, W
	BZ	_00353_DS_
	INCF	r0x03, F
_00353_DS_:
;	.line	196; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_command(reg);
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_clear_screen	code
_lcd_clear_screen:
;	.line	184; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_command(LCD_CLEAR_DISPLAY);
	MOVLW	0x01
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
;	.line	185; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_1ms();
	MOVLW	0xa0
	CALL	_delay100tcy
;	.line	186; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_500us();
	MOVLW	0x50
	CALL	_delay100tcy
;	.line	187; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_300us();
	MOVLW	0x30
	CALL	_delay100tcy
	RETURN	

; ; Starting pCode block
S_main__lcd_cursor_shift_right	code
_lcd_cursor_shift_right:
;	.line	176; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_cursor_shift_right(uint8_t nr) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	178; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (nr > 0) {
	MOVF	r0x00, W
	BZ	_00326_DS_
;	.line	179; ../my_sdcc_lib/rosso_sdcc_lcd4.h	for (i = 0; i < nr; i++) _lcd_write_command(LCD_CURSOR_SHIFT_R_VAL);
	CLRF	r0x01
_00324_DS_:
	MOVF	r0x00, W
	SUBWF	r0x01, W
	BC	_00326_DS_
	MOVLW	0x14
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
	INCF	r0x01, F
	BRA	_00324_DS_
_00326_DS_:
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_cursor_shift_left	code
_lcd_cursor_shift_left:
;	.line	169; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_cursor_shift_left(uint8_t nr) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	171; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (nr > 0) {
	MOVF	r0x00, W
	BZ	_00303_DS_
;	.line	172; ../my_sdcc_lib/rosso_sdcc_lcd4.h	for (i = 0; i < nr; i++) _lcd_write_command(LCD_CURSOR_SHIFT_L_VAL);
	CLRF	r0x01
_00301_DS_:
	MOVF	r0x00, W
	SUBWF	r0x01, W
	BC	_00303_DS_
	MOVLW	0x10
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
	INCF	r0x01, F
	BRA	_00301_DS_
_00303_DS_:
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_shift_right	code
_lcd_shift_right:
;	.line	162; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_shift_right(uint8_t nr) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	164; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (nr > 0) {
	MOVF	r0x00, W
	BZ	_00280_DS_
;	.line	165; ../my_sdcc_lib/rosso_sdcc_lcd4.h	for (i = 0; i < nr; i++) _lcd_write_command(LCD_DISPLAY_SHIFT_RIGHT);
	CLRF	r0x01
_00278_DS_:
	MOVF	r0x00, W
	SUBWF	r0x01, W
	BC	_00280_DS_
	MOVLW	0x1c
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
	INCF	r0x01, F
	BRA	_00278_DS_
_00280_DS_:
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_shift_left	code
_lcd_shift_left:
;	.line	155; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_shift_left(uint8_t nr) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	157; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (nr > 0) {
	MOVF	r0x00, W
	BZ	_00257_DS_
;	.line	158; ../my_sdcc_lib/rosso_sdcc_lcd4.h	for (i = 0; i < nr; i++) _lcd_write_command(LCD_DISPLAY_SHIFT_LEFT);
	CLRF	r0x01
_00255_DS_:
	MOVF	r0x00, W
	SUBWF	r0x01, W
	BC	_00257_DS_
	MOVLW	0x18
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
	INCF	r0x01, F
	BRA	_00255_DS_
_00257_DS_:
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_cursor_position	code
_lcd_cursor_position:
;	.line	150; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_cursor_position(uint8_t line, uint8_t pos) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
;	.line	151; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_POS = pos + _lcd_line2index(line);
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__lcd_line2index
	MOVF	POSTINC1, F
	ADDWF	r0x01, W
	BANKSEL	_LCD_POS
	MOVWF	_LCD_POS, B
;	.line	152; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_restore_cursor();
	CALL	__lcd_restore_cursor
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_write_strF	code
_lcd_write_strF:
;	.line	143; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_write_strF(const uint8_t *data){
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
_00229_DS_:
;	.line	144; ../my_sdcc_lib/rosso_sdcc_lcd4.h	while(*data){
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrget1
	MOVWF	r0x03
	MOVF	r0x03, W
	BZ	_00232_DS_
;	.line	145; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_data(*data);
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	CALL	__lcd_write_data
	MOVF	POSTINC1, F
;	.line	146; ../my_sdcc_lib/rosso_sdcc_lcd4.h	*data++;
	INCF	r0x00, F
	BNC	_00229_DS_
	INFSNZ	r0x01, F
	INCF	r0x02, F
_00241_DS_:
	BRA	_00229_DS_
_00232_DS_:
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_write_str	code
_lcd_write_str:
;	.line	136; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_write_str(uint8_t *data){
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
_00212_DS_:
;	.line	137; ../my_sdcc_lib/rosso_sdcc_lcd4.h	while(*data){
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrget1
	MOVWF	r0x03
	MOVF	r0x03, W
	BZ	_00215_DS_
;	.line	138; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_data(*data);
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	CALL	__lcd_write_data
	MOVF	POSTINC1, F
;	.line	139; ../my_sdcc_lib/rosso_sdcc_lcd4.h	*data++;
	INCF	r0x00, F
	BNC	_00212_DS_
	INFSNZ	r0x01, F
	INCF	r0x02, F
_00224_DS_:
	BRA	_00212_DS_
_00215_DS_:
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main__lcd_write_char	code
_lcd_write_char:
;	.line	132; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void lcd_write_char(uint8_t data) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	133; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_data(data);
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__lcd_write_data
	MOVF	POSTINC1, F
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main___lcd_restore_cursor	code
__lcd_restore_cursor:
;	.line	129; ../my_sdcc_lib/rosso_sdcc_lcd4.h	_lcd_write_command(LCD_SET_DDRAM_ADDRESS | LCD_POS);
	MOVLW	0x80
	BANKSEL	_LCD_POS
	IORWF	_LCD_POS, W, B
; #	MOVWF	r0x00
; #	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__lcd_write_command
	MOVF	POSTINC1, F
	RETURN	

; ; Starting pCode block
S_main___lcd_line2index	code
__lcd_line2index:
;	.line	117; ../my_sdcc_lib/rosso_sdcc_lcd4.h	uint8_t _lcd_line2index(uint8_t line) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	119; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (line < LCD_NR_LINES) {
	SUBWF	r0x00, W
	BC	_00173_DS_
;	.line	120; ../my_sdcc_lib/rosso_sdcc_lcd4.h	if (line == 0) return 0x00;
	MOVF	r0x00, W
	BNZ	_00170_DS_
	CLRF	WREG
	BRA	_00174_DS_
_00170_DS_:
;	.line	121; ../my_sdcc_lib/rosso_sdcc_lcd4.h	else if (line == 1) return 0x40;
	MOVF	r0x00, W
	XORLW	0x01
	BNZ	_00167_DS_
	MOVLW	0x40
	BRA	_00174_DS_
_00167_DS_:
;	.line	122; ../my_sdcc_lib/rosso_sdcc_lcd4.h	else if (line == 2) return 0x00 + LCD_NR_CHARS;
	MOVF	r0x00, W
	XORLW	0x02
	BNZ	_00164_DS_
	MOVLW	0x10
	BRA	_00174_DS_
_00164_DS_:
;	.line	123; ../my_sdcc_lib/rosso_sdcc_lcd4.h	else if (line == 3) return 0x40 + LCD_NR_CHARS;
	MOVF	r0x00, W
	XORLW	0x03
	BNZ	_00173_DS_
	MOVLW	0x50
	BRA	_00174_DS_
_00173_DS_:
;	.line	125; ../my_sdcc_lib/rosso_sdcc_lcd4.h	return 0x00;
	CLRF	WREG
_00174_DS_:
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main___lcd_write_command	code
__lcd_write_command:
;	.line	112; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void _lcd_write_command(uint8_t value) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	113; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_RS = 0; // select command mode
	BCF	_LATEbits, 1
;	.line	114; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write(value); // write byte
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___lcd_write
	MOVF	POSTINC1, F
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main___lcd_write_data	code
__lcd_write_data:
;	.line	107; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void _lcd_write_data(uint8_t value) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	108; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_RS = 1; // select data mode
	BSF	_LATEbits, 1
;	.line	109; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write(value); // write byte
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___lcd_write
	MOVF	POSTINC1, F
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main____lcd_write	code
___lcd_write:
;	.line	100; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void __lcd_write(uint8_t value) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	101; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(value >> 4); // write high nibble
	SWAPF	r0x00, W
	ANDLW	0x0f
; #	MOVWF	r0x01
; #	MOVF	r0x01, W
	MOVWF	r0x01
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	102; ../my_sdcc_lib/rosso_sdcc_lcd4.h	__lcd_write_nibble(value); // write low nibble
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___lcd_write_nibble
	MOVF	POSTINC1, F
;	.line	103; ../my_sdcc_lib/rosso_sdcc_lcd4.h	delay_35us(); // > 37 us
	MOVLW	0x38
	CALL	_delay10tcy
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
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_main____lcd_write_nibble	code
___lcd_write_nibble:
;	.line	88; ../my_sdcc_lib/rosso_sdcc_lcd4.h	void __lcd_write_nibble(uint8_t value) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	90; ../my_sdcc_lib/rosso_sdcc_lcd4.h	nibble.val = value;
	MOVF	r0x00, W
	BANKSEL	___lcd_write_nibble_nibble_1_14
	MOVWF	___lcd_write_nibble_nibble_1_14, B
;	.line	91; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_D4 = nibble.bits.b0;
	CLRF	r0x00
; removed redundant BANKSEL
	BTFSC	___lcd_write_nibble_nibble_1_14, 0, B
	INCF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	SWAPF	WREG, W
	MOVWF	PRODH
	MOVF	_LATBbits, W
	ANDLW	0xef
	IORWF	PRODH, W
	MOVWF	_LATBbits
;	.line	92; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_D5 = nibble.bits.b1;
	CLRF	r0x00
; removed redundant BANKSEL
	BTFSC	___lcd_write_nibble_nibble_1_14, 1, B
	INCF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	SWAPF	WREG, W
	RLNCF	WREG, W
	MOVWF	PRODH
	MOVF	_LATBbits, W
	ANDLW	0xdf
	IORWF	PRODH, W
	MOVWF	_LATBbits
;	.line	93; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_D6 = nibble.bits.b2;
	CLRF	r0x00
; removed redundant BANKSEL
	BTFSC	___lcd_write_nibble_nibble_1_14, 2, B
	INCF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	RRNCF	WREG, W
	RRNCF	WREG, W
	MOVWF	PRODH
	MOVF	_LATBbits, W
	ANDLW	0xbf
	IORWF	PRODH, W
	MOVWF	_LATBbits
;	.line	94; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_D7 = nibble.bits.b3;
	CLRF	r0x00
; removed redundant BANKSEL
	BTFSC	___lcd_write_nibble_nibble_1_14, 3, B
	INCF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	RRNCF	WREG, W
	MOVWF	PRODH
	MOVF	_LATBbits, W
	ANDLW	0x7f
	IORWF	PRODH, W
	MOVWF	_LATBbits
;	.line	95; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_EN = 1;
	BSF	_LATEbits, 2
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
;	.line	97; ../my_sdcc_lib/rosso_sdcc_lcd4.h	LCD_EN = 0;
	BCF	_LATEbits, 2
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

; ; Starting pCode block for Ival
	code
_sf:
	DB	0x20, 0x45, 0x4e, 0x47, 0x00, 0x00
; ; Starting pCode block
__str_0:
	DB	0x20, 0x45, 0x4e, 0x47, 0x00, 0x00


; Statistics:
; code size:	14648 (0x3938) bytes (11.18%)
;           	 7324 (0x1c9c) words
; udata size:	    6 (0x0006) bytes ( 0.16%)
; access size:	   11 (0x000b) bytes


	end
