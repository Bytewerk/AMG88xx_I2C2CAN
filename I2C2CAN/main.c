/*
	This is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this. If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "can/can.h"
#include "timer.h"
#include "i2c.h"

#define CAN_MSG_MAX_LENGTH (8)

#define HW_SERIALNUM      (3)    // 0 - 3
#define SW_VERSION_MAJOR  (0x03) // <minor(8)>
#define SW_VERSION_MINOR  (0x02) // <major(8)>
#define SENSOR_SLAVE_ID   0x69U



enum canMsgIds {
	eCanImgDataBaseId  = 0x0100 | (HW_SERIALNUM<<4), // + 0x0000 to 0x0007
	eCanHeartbeatId    = 0x0108 | (HW_SERIALNUM<<4),
	eCanResetId        = 0x0109 | (HW_SERIALNUM<<4),
	eCanGetThermistor  = 0x010A | (HW_SERIALNUM<<4),
	eCanSendThermistor = 0x010B | (HW_SERIALNUM<<4),
	eCanReadI2C        = 0x010C | (HW_SERIALNUM<<4),
	eCanAnsReadI2C     = 0x010D | (HW_SERIALNUM<<4),
	eCanWriteI2C       = 0x010E | (HW_SERIALNUM<<4)
	};

enum delays {
	eImgDataDelay   = 100,  // ms
	eHeartbeatDelay = 1000  // ms
};


// function prototypes

int my_can_init(void);
void init_amg88(void);



int main( void )
{
	//can
	can_t msg_img, msg_heartbeat, msg_rx;
	uint16_t canStartID = eCanImgDataBaseId;
	msg_img.length = CAN_MSG_MAX_LENGTH;
	msg_img.flags.rtr = 0;
	msg_img.flags.extended = 0;

	msg_heartbeat.id = eCanHeartbeatId;
	msg_heartbeat.length = 2;
	msg_heartbeat.flags.rtr = 0;
	msg_heartbeat.flags.extended = 0;
	msg_heartbeat.data[0] = (SW_VERSION_MAJOR) & 0xFF;
	msg_heartbeat.data[1] = (SW_VERSION_MINOR) & 0xFF;

	//timer
	uint32_t now, t_read_pixels=0;
	uint32_t t_send_heartbeat = 0;

	//i2c
	//uint8_t sensor=0x69;
	//sensor
	uint8_t data[64][2];
	//uint16_t *value=&data;
	
	// thermistor
	uint8_t thermistor[2] = {0, 0};
		
	

	//init
	timer_init( );
	my_can_init( );
	i2c_init();
	sei( );
	wdt_enable(WDTO_250MS);
	wdt_reset();
	init_amg88();

	while( 1 )
	{
		wdt_reset();
		now = timer_getMs( );

		while( can_check_message() ) {
			can_get_message( &msg_rx );
			if( msg_rx.id == eCanResetId ) {
				while( 1 ); // wait for watchdog reset
			}
			if( msg_rx.id == eCanGetThermistor ){
				// read thermistor data
				thermistor[0] = i2c_readRegister(SENSOR_SLAVE_ID, 0x0E); // lower value
				thermistor[1] = i2c_readRegister(SENSOR_SLAVE_ID, 0x0F); // upper value
				// set ID and DLC
				msg_img.id = eCanSendThermistor;
				msg_img.length = 2;
				// MSB is bit 3 in data0, LSB ist bit 0 in data1!!
				msg_img.data[0] = thermistor[1]; 
				msg_img.data[1] = thermistor[0];
				// send can message
				can_send_message( &msg_img );
			}
			// read register and send it over CAN
			if( msg_rx.id == eCanReadI2C ){
				if(msg_rx.length >= 1){
					uint8_t tmp = 0;
					tmp = i2c_readRegister(SENSOR_SLAVE_ID, msg_rx.data[0]);
					msg_img.id = eCanAnsReadI2C;
					msg_img.length = 1;
					msg_img.data[0] = tmp;
					can_send_message( &msg_img );
				}
			}
			// write register
			if( msg_rx.id == eCanWriteI2C ){
				if(msg_rx.length >= 1){
					i2c_writeRegister(SENSOR_SLAVE_ID, msg_rx.data[0], msg_rx.data[1]);				
				}
			}
		}

		//every 100ms
		if( t_read_pixels + eImgDataDelay < now)
		{
			DDRC  |= (1<<PC6); // blue LED
			PORTC |= (1<<PC6);

			t_read_pixels = now;
			//read data from the I2C Sensor
			for(uint8_t pixelCounter=0; pixelCounter<=63; pixelCounter++)
			{
				data[pixelCounter][0]=i2c_readRegister(SENSOR_SLAVE_ID, 0x80+(pixelCounter<<1)); //low
				data[pixelCounter][1]=i2c_readRegister(SENSOR_SLAVE_ID, 0x81+(pixelCounter<<1)); //high
			}


			PORTC &= ~(1<<PC6);
			for(uint8_t canMessageCounter=0; canMessageCounter<8; canMessageCounter++)
			{
				//generate the packet id
				msg_img.id = canStartID+canMessageCounter;
				msg_img.length = 8;

				//fill the packet with data
				for(uint8_t canDataCounter=0; canDataCounter<CAN_MSG_MAX_LENGTH; canDataCounter++)
				{
					msg_img.data[canDataCounter] = data[canMessageCounter*8+canDataCounter][0];
				}
                //and send them over can
				can_send_message( &msg_img );

				_delay_ms(1);
			}

		}

		if( t_send_heartbeat + eHeartbeatDelay < now ) {
			DDRC  |= (1<<PC5); // green LED
			PORTC ^= (1<<PC5);
			t_send_heartbeat = now;
			can_send_message( &msg_heartbeat );
		}

	}
	return 0;
}

int my_can_init(void)
{
	can_init( BITRATE_500_KBPS );

	can_filter_t can_filter = {     .id = 0, .mask = 0, .flags = { .rtr = 0, .extended = 0 } };
	can_set_filter( 0, &can_filter );
	can_set_filter( 1, &can_filter );
	can_set_filter( 2, &can_filter );

	DDRB |= (1<<PB3) | (1<<PB4); // !STB! | EN
	PORTB |= (1<<PB3) | (1<<PB4); // ready for sending data

	return 0;
}

void init_amg88(void){
	
	
	// setting moving Average => application note p. 16
	i2c_writeRegister(SENSOR_SLAVE_ID, 0x1F, 0x50 );
	timer_waitMs(5);
	i2c_writeRegister(SENSOR_SLAVE_ID, 0x1F, 0x45 );
	timer_waitMs(5);
	i2c_writeRegister(SENSOR_SLAVE_ID, 0x1F, 0x57 );
	timer_waitMs(5);
	i2c_writeRegister(SENSOR_SLAVE_ID, 0x07, 0x20 );
	timer_waitMs(5);
	i2c_writeRegister(SENSOR_SLAVE_ID, 0x1F, 0x00 );
	timer_waitMs(5);
	
}
