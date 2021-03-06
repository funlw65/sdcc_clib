/* 
 * File:   rosso_i2c.h
 * Author: Vasile Guta Ciucur
 *
 * Created on December 9, 2014, 11:36 PM
 */

#ifndef ROSSO_I2C_MASTER_H
#define	ROSSO_I2C_MASTER_H

#define I2C_SCL_DIR TRISCbits.RC3
#define I2C_SDA_DIR TRISCbits.RC4

#ifndef I2C_LEVEL
#define I2C_LEVEL    // this should be user set-able.. is ok for now...

typedef enum{
    I2C_1MHZ    = (_XTAL_FREQ /  1000000 / 4 - 1),
    I2C_400KHZ  = (_XTAL_FREQ /   400000 / 4 - 1),
    I2C_100KHZ  = (_XTAL_FREQ /   100000 / 4 - 1),
    I2C_SLOWEST = 127
} I2CSPEED;
#endif

void i2c_init(I2CSPEED speed) {
    I2C_SCL_DIR = INPUT;
    I2C_SDA_DIR = INPUT;
    SSPCON1 = 0b00101000;
    SSPCON2 = 0b00100000;
    SSPADD = speed;
    #if defined(I2C_LEVEL)
        SSPSTAT = 0b00000000;
    #else
        SSPSTAT = 0b01000000;
    #endif
}

void i2c_start(void) {
    SSPCON2bits.SEN = 1;
    while (SSPCON2bits.SEN == 1);
}

void i2c_restart(void) {
    SSPCON2bits.RSEN = 1;
    while (SSPCON2bits.RSEN == 1);
}

void i2c_stop(void) {
    SSPCON2bits.PEN = 1;
    while (SSPCON2bits.PEN == 1);
}

bool_t i2c_write(uint8_t data) {
    PIR1bits.SSPIF = 0;
    SSPBUF = data;
    while (!PIR1bits.SSPIF);
    if (SSPCON2bits.ACKSTAT == 0)return (TRUE);
    else {
        SSPCON1bits.SSPEN = 0;
        SSPCON1bits.SSPEN = 1;
        return (FALSE);
    }
}

uint8_t i2c_read(bool_t myack) {
    SSPCON2bits.RCEN = 1;
    while (SSPSTATbits.BF == 0);
    SSPCON2bits.ACKDT = !myack;
    SSPCON2bits.ACKEN = 1;
    while (SSPCON2bits.ACKEN == 1);
    return (SSPBUF);
}


#endif	/* ROSSO_I2C_MASTER_H */

