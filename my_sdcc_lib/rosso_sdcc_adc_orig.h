/*
 * adc_open's `channel' argument:
 *
 * one of ADC_CHN_*
 */

/* channel selection (CHS field in ADCON0) */
#define ADC_CHN_0               0x00
#define ADC_CHN_1               0x01
#define ADC_CHN_2               0x02
#define ADC_CHN_3               0x03
#define ADC_CHN_4               0x04
#define ADC_CHN_5               0x05
#define ADC_CHN_6               0x06
#define ADC_CHN_7               0x07
#define ADC_CHN_8               0x08
#define ADC_CHN_9               0x09
#define ADC_CHN_10              0x0a
#define ADC_CHN_11              0x0b
#define ADC_CHN_12              0x0c
#define ADC_CHN_13              0x0d
#define ADC_CHN_14              0x0e
#define ADC_CHN_15              0x0f
#define ADC_CHN_16              0x10
#define ADC_CHN_17              0x11
#define ADC_CHN_18              0x12
#define ADC_CHN_19              0x13
#define ADC_CHN_20              0x14
#define ADC_CHN_21              0x15
#define ADC_CHN_22              0x16
#define ADC_CHN_23              0x17
#define ADC_CHN_24              0x18
#define ADC_CHN_25              0x19
#define ADC_CHN_26              0x1a
#define ADC_CHN_27              0x1b
#define ADC_CHN_K_CTMU          0x1d
#define ADC_CHN_K_DAC           0x1e
#define ADC_CHN_K_FVR           0x1f


/*
 * adc_open's `fosc' argument:
 *
 * ADC_FOSC_* | ADC_ACQT_* | ADC_CAL | ADC_TRIGSEL_*
 *
 *    7     6     5     4     3     2     1     0
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 * | TRG | CAL |       ACQT      |    FOSC/ADCS    |
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 */

/* oscillator frequency (ADCS field) */
#define ADC_FOSC_2              0x00
#define ADC_FOSC_4              0x04
#define ADC_FOSC_8              0x01
#define ADC_FOSC_16             0x05
#define ADC_FOSC_32             0x02
#define ADC_FOSC_64             0x06
#define ADC_FOSC_RC             0x07

/* acquisition time (13k50/2220/24j50/65j50-styles only) */
#define ADC_ACQT_0              (0x00 << 3)
#define ADC_ACQT_2              (0x01 << 3)
#define ADC_ACQT_4              (0x02 << 3)
#define ADC_ACQT_6              (0x03 << 3)
#define ADC_ACQT_8              (0x04 << 3)
#define ADC_ACQT_12             (0x05 << 3)
#define ADC_ACQT_16             (0x06 << 3)
#define ADC_ACQT_20             (0x07 << 3)

/* calibration enable (24j50/65j50-style only) */
#define ADC_CAL                 0x40

/* trigger selection (23k22-style only) */
#define ADC_TRIGGER             0x80

/*
 * adc_open's `config' argument:
 *
 * ADC_FRM_* | ADC_INT_* | ADC_VCFG_* | ADC_NVCFG_* | ADC_PVCFG_*
 *
 *    7     6     5     4     3     2     1     0
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 * | FRM | INT |   VCFG    |   PVCFG   |   NVCFG   |
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 */

/* output format */
#define ADC_FRM_LJUST           0x00
#define ADC_FRM_RJUST           0x80

/* interrupt on/off flag */
#define ADC_INT_OFF             0x00
#define ADC_INT_ON              0x40

/* reference voltage configuration (not for 18f242-style ADC) */
#define ADC_VCFG_VDD_VSS        0x00
#define ADC_VCFG_AN3_VSS        0x10
#define ADC_VCFG_VDD_AN2        0x20
#define ADC_VCFG_AN3_AN2        0x30

/* reference voltage configuration (13k50-style) */
#define ADC_NVCFG_VSS           0x00
#define ADC_NVCFG_AN5           0x01

#define ADC_PVCFG_VDD           (0x00 << 2)
#define ADC_PVCFG_AN4           (0x01 << 2)
#define ADC_PVCFG_FVR           (0x02 << 2)

/* reference voltage configuration (23k22-style) */
#define ADC_NVCFG_AN2           0x01
#define ADC_PVCFG_AN3           (0x01 << 2)
#define ADC_TRIGSEL_CCP5        (0x00 << 7)
#define ADC_TRIGSEL_CTMU        (0x01 << 7)


/*
 * parameters are:
 *   channel: one of ADC_CHN_*
 *   fosc:    one of ADC_FOSC_* | ADC_ACQT_* | ADC_CAL
 *   config:  ADC_FRM_* | ADC_INT_* | ADC_VCFG_* | ADC_NVCFG_* | ADC_PVCFG_*
 */

void
adc_open(uint8_t channel, uint8_t fosc, uint8_t config)
{
  ADCON0 = 0;
  /* use ANSELA, ANSELB, ANSELC, ANSELD, ANSELE registers and
     TRISA, TRISB, TRISC, TRISD, TRISE registers to set
     corresponding port to analog mode */
  /* 46k22 supports up to 28 ADC ports */
  ADCON0 = ((channel & 0x1f) << 2);
  /* TRIGSEL — — — PVCFG<1:0> NVCFG<1:0> */
  ADCON1 = (fosc & ADC_TRIGGER) | (config & 0x0f);
  /* ADFM — ACQT<2:0> ADCS<2:0> */
  ADCON2 = (config & ADC_FRM_RJUST) | (fosc & 0x3f);

  if (config & ADC_INT_ON) {
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    INTCONbits.PEIE = 1;
  }

  /* enable the A/D module */
  ADCON0bits.ADON = 1;
}

