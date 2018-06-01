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

/*
// Function Pototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init1")));

// Function Implementation
void wdt_init(void) {
	MCUSR = 0;
	wdt_disable();
	return;
}
*/

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "config.h"
#include "can/can.h"
#include "timer.h"
#include "i2c.h"

int my_can_init(void);
uint8_t i2c_readRegister(uint8_t slaveid, uint8_t registerAdress);
void i2c_writeRegister(uint8_t slaveid, uint8_t registerAdress, uint8_t value);

int main( void )
{
    //can
    can_t msg_tx;
    uint16_t canStartID = 0x100;
    msg_tx.flags.extended = 0;
    msg_tx.flags.rtr = 0;
    msg_tx.length = 8;
    //timer
    uint32_t now, t_read_pixels=0;
    //i2c
    uint8_t sensor=0x69;
    //sensor
    uint8_t data[64][2];
    uint8_t value[64];

    //init
    timer_init( );
    my_can_init( );
    i2c_init();
    sei( );
    wdt_enable(WDTO_500MS);
    wdt_reset();

    //sensor configuration
    //i2c_writeRegister(sensor,0x01,0x3F); //initial reset
    //i2c_writeRegister(sensor,0x03,0x03); //Absolute Value Interrupt Mode; Interrupt Output Active

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
                value[pixelCounter]= (uint8_t) (((((uint16_t)data[pixelCounter][1])<<8)+data[pixelCounter][0])>>4); //signed 12bit to signed 8bit conversion

                //if there are enough data
                if ((((pixelCounter+1)%msg_tx.length)==0) && (pixelCounter!=0))
                {
										DDRC  |= (1<<PC6);
										PORTC ^= (1<<PC6);
                    //generate the packet id
                    msg_tx.id = canStartID+(pixelCounter/msg_tx.length);

                    //fill the packet with data
                    for(uint8_t canDataCounter=0; canDataCounter<msg_tx.length; canDataCounter++)
                    {
                        msg_tx.data[canDataCounter] = (value[pixelCounter-msg_tx.length+canDataCounter])&0xff;
                    }
                    //and send them over can
                    can_send_message( &msg_tx );
                }
            }
						DDRC  |= (1<<PC5);
						PORTC ^= (1<<PC5);
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

    // PB3 == STDBY
    // PB4 == EN
		DDRB |= (1<<PB3) | (1<<PB4); // !STB! | EN
		PORTB |= (1<<PB3) | (1<<PB4); // ready for sending data
    //goto_active_mode();

    return 0;
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

/*
void goto_sleep_mode(void);
int  goto_active_mode(void);
void soft_reset(void);

void goto_sleep_mode( void ) {
	// PB3 == STDBY
	// PB4 == EN
	PORTB &= ~(1<<PB3);
	PORTB |=  (1<<PB4);

	wdt_enable(WDTO_500MS);
	while(1) { ; } // wait for sleep or watchdog reset
}



int goto_active_mode( void ) {
	// PB3 == STDBY
	// PB4 == EN
	PORTB |= (1<<PB3)|(1<<PB4);
	return 0;
}


void soft_reset( void )
{
    wdt_enable(WDTO_500MS);
    while(1);
}

//	uint32_t t_sleep = 60000; // stay awake for 1 minute initially

if( now > t_sleep ) {
	uint32_t t_clear = timer_getMs( ) + 100;
	while ( timer_getMs() < t_clear ) {
		led_run( );
		timer_waitMs(1);
	}
	wdt_reset();
	goto_sleep_mode();
}

//	uint8_t i;

			msg_tx.data[0] = now&0xff;
			msg_tx.data[1] = (now>>8)&0xff;
			msg_tx.data[2] = (now>>16)&0xff;
			msg_tx.data[3] = (now>>24)&0xff;
			msg_tx.data[4] = l_mode;



					// receive CAN
					if ( can_check_message() ) {
						can_get_message( &msg_rx );
						//t_sleep = now;

						switch( msg_rx.id ) {
							case RESET_MSG:
							soft_reset( );
							break;

							case SWITCH_MODE_MSG:
							l_mode = msg_rx.data[0];
							break;

							case SET_COLOR_MSG: // [R][G][B]
							break;

							case REQ_DISP_CONTENT_MSG: // [RATE(fps)]
							break;

							case SET_LED_MSG: // [R][G][B]
							break;

							case SET_SLEEPTIME: // Set remaining time to stay awake
							t_sleep = now + ((((msg_rx.data[1])<<8) | ((msg_rx.data[0])<<0))*1000UL);
							break;
						}
					}

*/
