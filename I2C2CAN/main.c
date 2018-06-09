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

#define CAN_MSG_MAX_LENGTH 8

int my_can_init(void);

int main( void )
{
	//can
	can_t msg_tx;
	uint16_t canStartID = 0x100;
	msg_tx.flags.extended = 0;
	msg_tx.flags.rtr = 0;
	msg_tx.length = CAN_MSG_MAX_LENGTH;
	//timer
	uint32_t now, t_read_pixels=0;
	//i2c
	uint8_t sensor=0x69;
	//sensor
	uint8_t data[64][2];
	//uint16_t *value=&data;

	//init
	timer_init( );
	my_can_init( );
	i2c_init();
	sei( );
	wdt_enable(WDTO_500MS);
	wdt_reset();

	while( 1 )
	{
		wdt_reset();
		now = timer_getMs( );

		//every 100ms
		if( t_read_pixels + 100 < now)
		{
			t_read_pixels = now;
			//read data from the I2C Sensor
			for(uint8_t pixelCounter=0; pixelCounter<=63; pixelCounter++)
			{
				data[pixelCounter][0]=i2c_readRegister(sensor, 0x80+(pixelCounter<<1)); //low
				data[pixelCounter][1]=i2c_readRegister(sensor, 0x81+(pixelCounter<<1)); //high
			}

			DDRC  |= (1<<PC6); //debug blink
			PORTC ^= (1<<PC6);

			for(uint8_t canMessageCounter=0; canMessageCounter<8; canMessageCounter++)
			{
				//generate the packet id
				msg_tx.id = canStartID+canMessageCounter;

				//fill the packet with data
				for(uint8_t canDataCounter=0; canDataCounter<CAN_MSG_MAX_LENGTH; canDataCounter++)
				{
					//msg_tx.data[canDataCounter] = ((uint8_t)((value[canMessageCounter*8+canDataCounter])<<0)); //convert the signed 12bit to unsigned 8bit, just watching at the lower 8 bit (<<0). This is a range of 64°C.
					msg_tx.data[canDataCounter] = data[canMessageCounter*8+canDataCounter][0];
				}
                //and send them over can
				can_send_message( &msg_tx );

				DDRC  |= (1<<PC5);//debug blink
				PORTC ^= (1<<PC5);
				_delay_ms(1);
			}
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
