#ifndef __I2C_H__
#define __I2C_H__



/*

Typisches Beispiel Senden:

i2c_init();

i2c_start();
i2c_write((slaveid<<1)+(R/W Bit == 0));
i2c_write(Registeradresse);
i2c_write(Wert);
i2c_stop();

Typisches Beispiel Lesen:

i2c_init();

i2c_start();
i2c_write((slaveid<<1)+(R/W Bit == 0));
i2c_write(Registeradresse);
i2c_start(); //repeated start
i2c_write((slaveid<<1)+(R/W Bit == 1));
i2c_read(NAK==0);
i2c_stop();

*/


#include <avr/io.h>
#include <util/delay.h>


#define SCLPORT	PORTD	//TAKE PORTD as SCL OUTPUT WRITE
#define SCLDDR	DDRD	//TAKE DDRB as SCL INPUT/OUTPUT configure

#define SDAPORT	PORTD	//TAKE PORTD as SDA OUTPUT WRITE
#define SDADDR	DDRD	//TAKE PORTD as SDA INPUT configure

#define SDAPIN	PIND	//TAKE PORTD TO READ DATA
#define SCLPIN	PIND	//TAKE PORTD TO READ DATA

#define SCL	PD1			//PORTD.0 PIN AS SCL PIN
#define SDA	PD0			//PORTD.1 PIN AS SDA PIN


#define SOFT_I2C_SDA_LOW	SDADDR|=((1<<SDA))
#define SOFT_I2C_SDA_HIGH	SDADDR&=(~(1<<SDA))

#define SOFT_I2C_SCL_LOW	SCLDDR|=((1<<SCL))
#define SOFT_I2C_SCL_HIGH	SCLDDR&=(~(1<<SCL))

#define Q_DEL _delay_loop_2(3)
#define H_DEL _delay_loop_2(5)

#define NAK 0
#define ACK 1

void i2c_init();
void i2c_start();
void i2c_stop();
uint8_t i2c_write(uint8_t data);

uint8_t i2c_readRegister(uint8_t slaveid, uint8_t registerAdress);
void i2c_writeRegister(uint8_t slaveid, uint8_t registerAdress, uint8_t value);

// Lesefunktion
// 0 = Letztes Byte, sende NAK
// 1 = Es kommen noch welche, sende ACK
uint8_t i2c_read(uint8_t ack);

#else
#error "double include i2c.h"
#endif //__I2C_H__
