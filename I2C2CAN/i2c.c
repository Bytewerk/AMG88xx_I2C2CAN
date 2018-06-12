#include <stdint.h>
#include "i2c.h"
#include "timer.h"

void i2c_init()
{
	SDAPORT&=(1<<SDA);
	SCLPORT&=(1<<SCL);

	SOFT_I2C_SDA_HIGH;
	SOFT_I2C_SCL_HIGH;

}

void i2c_start()
{
	SOFT_I2C_SCL_HIGH;
	H_DEL;

	SOFT_I2C_SDA_LOW;
	H_DEL;
}

void i2c_stop()
{
	 SOFT_I2C_SDA_LOW;
	 H_DEL;
	 SOFT_I2C_SCL_HIGH;
	 Q_DEL;
	 SOFT_I2C_SDA_HIGH;
	 H_DEL;
}

uint8_t i2c_write(uint8_t data)
{

	 uint8_t i;
	 uint32_t now = timer_getMs();

	 for(i=0;i<8;i++)
	 {
		SOFT_I2C_SCL_LOW;
		Q_DEL;

		if(data & 0x80)
			SOFT_I2C_SDA_HIGH;
		else
			SOFT_I2C_SDA_LOW;

		H_DEL;

		SOFT_I2C_SCL_HIGH;
		H_DEL;

		while((SCLPIN & (1<<SCL)) == 0 ){
			if(timer_getMs() - now > 500){		// TODO: test this if timeout works
				break;
			}
		}									

		data=data<<1;
	}

	//The 9th clock (ACK Phase)
	SOFT_I2C_SCL_LOW;
	Q_DEL;

	SOFT_I2C_SDA_HIGH;
	H_DEL;

	SOFT_I2C_SCL_HIGH;
	H_DEL;

	// ----- ACK des Slaves einlesen
	uint8_t ack =! (SDAPIN & (1<<SDA));

	// --- Bus loslassen
	SOFT_I2C_SCL_LOW;
	H_DEL;

	return ack;
}

// Lesefunktion
// 0 = Letztes Byte, sende NAK
// 1 = Es kommen noch welche, sende ACK
uint8_t i2c_read(uint8_t ack)
{
	uint8_t data=0x00;
	uint8_t i;

	for(i=0;i<8;i++)
	{
		// SCL clocken
		SOFT_I2C_SCL_LOW;
		H_DEL;
		SOFT_I2C_SCL_HIGH;
		H_DEL;

		while((SCLPIN & (1<<SCL)) == 0);

		if(SDAPIN & (1<<SDA))
			data |= (0x80>>i);

	}

	SOFT_I2C_SCL_LOW;
	Q_DEL;						//Soft_I2C_Put_Ack

	if(ack) {					// ACK
		SOFT_I2C_SDA_LOW;
	}
	else {
		SOFT_I2C_SDA_HIGH;		// NACK = Ende
	}
	H_DEL;

	SOFT_I2C_SCL_HIGH;
	H_DEL;

	SOFT_I2C_SCL_LOW;
	H_DEL;

	SOFT_I2C_SDA_HIGH;

	return data;
}

void i2c_writeRegister(uint8_t slaveid, uint8_t registerAdress, uint8_t data)
{
    i2c_start();
    i2c_write(slaveid<<1);
    i2c_write(registerAdress);
    i2c_write(data);
    i2c_stop();
}

uint8_t i2c_readRegister(uint8_t slaveid, uint8_t registerAdress)
{
    uint8_t data=0;
    i2c_start();
    i2c_write(slaveid<<1);
    i2c_write(registerAdress);
    i2c_start(); //repeated start
    i2c_write((slaveid<<1)+1); //1 for read mode
    data=i2c_read(NAK);
    i2c_stop();
    return data;
}
