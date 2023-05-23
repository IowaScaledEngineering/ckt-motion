/*************************************************************************
Title:    CKT-MOTION v1.0
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2022 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

/* 
 * Directions:
 * 
 *     ---------  ^ NEGATIVE (left)
 *     |   *   |  |
 *     |-------|  |
 *     ||     ||  dy
 *     ||     ||  |
 *     |-------|  |
 *     ---------  v POSITIVE (right)
 *     <-- dx -->
 *     NEG    POS
 */


#include <stdbool.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define PAT9125_ADDR           0x73
#define INFO_ADDR              0x20

// LOOP_MILLISECS should be a multiple of 10
#define LOOP_MILLISECS         50

#define ON_DEBOUNCE_COUNT      (SET_MILLISECS / LOOP_MILLISECS)
#define OFF_DEBOUNCE_COUNT     (RELEASE_MILLISECS / LOOP_MILLISECS)

#define SDA   PB0
#define SCL   PB2

#define OUTPUT_WHITE_PULLDOWN   PB3
#define OUTPUT_BLUE_PULLDOWN    PB1
#define JUMPER                  PB4

#define PAT9125_PID1            0x00
#define PAT9125_PID2            0x01
#define PAT9125_MOTION          0x02
#define PAT9125_DELTA_XL        0x03
#define PAT9125_DELTA_YL        0x04
#define PAT9125_MODE            0x05
#define PAT9125_CONFIG          0x06
#define PAT9125_WP              0x09
#define PAT9125_SLEEP1          0x0a
#define PAT9125_SLEEP2          0x0b
#define PAT9125_RES_X           0x0d
#define PAT9125_RES_Y           0x0e
#define PAT9125_DELTA_XYH       0x12
#define PAT9125_SHUTTER         0x14
#define PAT9125_FRAME           0x17
#define PAT9125_ORIENTATION     0x19
#define PAT9125_BANK_SELECTION  0x7f
#define PAT9125_MYSTERY         0x4B
#define PAT9125_MAGIC_JUJU_1    0x5D
#define PAT9125_MAGIC_JUJU_2    0x5E

static inline void sda_low() { DDRB |= _BV(SDA); PORTB &= ~_BV(SDA); _delay_us(3); }
static inline void sda_high() { DDRB &= ~_BV(SDA); PORTB |= _BV(SDA); _delay_us(3); }
static inline void scl_low() { PORTB &= ~_BV(SCL); _delay_us(3); }
static inline void scl_high() { PORTB |= _BV(SCL); _delay_us(3); }

volatile uint8_t trigger = 0;

ISR(TIM0_COMPA_vect)
{
	static uint8_t ticks = 0;
	// Called every 10ms
	if (++ticks >= (LOOP_MILLISECS / 10))
	{
		ticks = 0;
		trigger = 1;
	}
}

void i2cStart(void)
{
	scl_high();
	sda_low();
	scl_low();
	sda_high();
}

void i2cStop(void)
{
	scl_low();
	sda_low();
	scl_high();
	sda_high();
}

uint8_t i2cWriteByte(uint8_t byte)
{
	uint8_t i = 0x80, ack;

	do
	{
		if(byte & i)
		{
			sda_high();
		}
		else
		{
			sda_low();
		}
		
		scl_high();
		scl_low();
		
		i >>= 1;
	} while(i);

	sda_high();  // Release SDA
	
	scl_high();
	if(PINB & _BV(SDA))
		ack = 0;
	else
		ack = 1;
	scl_low();

	return ack;
}

uint8_t i2cReadByte(uint8_t ack)
{
	uint8_t i, data = 0;

	for(i=0; i<8; i++)
	{
		data = data << 1;
		scl_high();
		if(PINB & _BV(SDA))
			data |= 0x01;
		scl_low();
	}
	
	if(ack)
		sda_low();
	scl_high();
	scl_low();
	sda_high();

	return data;
}

uint8_t PAT9125_RegWrite(uint8_t reg, uint8_t data) 
{
	uint8_t ack;
	i2cStart();
	i2cWriteByte(PAT9125_ADDR << 1);
	i2cWriteByte(reg);
	ack = i2cWriteByte(data);
	i2cStop();
	_delay_ms(10);
	
	return ack;
}

uint8_t PAT9125_RegRead(uint8_t reg)
{
	uint8_t retval = 0;
	i2cStart();
	i2cWriteByte((PAT9125_ADDR << 1));
	i2cWriteByte(reg);

	i2cStart();
	i2cWriteByte((PAT9125_ADDR << 1) | 0x01);
	retval = i2cReadByte(0);
	i2cStop();

	_delay_ms(1);
	return retval;
}

uint8_t PAT9125_RegWriteRead(uint8_t reg, uint8_t data)
{
  PAT9125_RegWrite(reg, data);
  return(PAT9125_RegRead(reg));
}

void debounce(bool *detect, uint8_t* count, bool isDetecting, const uint8_t onDebounceCount, const uint8_t offDebounceCount)
{
	if(!(*detect) && isDetecting)
	{
		// ON debounce
		(*count)++;
		if(*count > onDebounceCount)
		{
			*detect = true;
			*count = 0;
		}
	}
	else if(!(*detect) && !isDetecting)
	{
		if (*count > 0)
			count--;
	}

	else if(*detect & !isDetecting)
	{
		// OFF debounce
		(*count)++;
		if(*count > offDebounceCount)
		{
			*detect = false;
			*count = 0;
		}
	}
	else if(*detect & isDetecting)
	{
		if (*count > 0)
			count--;
	}
}

uint8_t getMotionThreshold(void)
{
	if(PINB & _BV(JUMPER))
	{
		return MOTION_THRESHOLD_A;
	}
	else
	{
		return MOTION_THRESHOLD_B;
	}
}

uint8_t getNoMotionThreshold(void)
{
	if(PINB & _BV(JUMPER))
	{
		return NO_MOTION_THRESHOLD_A;
	}
	else
	{
		return NO_MOTION_THRESHOLD_B;
	}
}

int main(void)
{
//	int16_t dx=0;
	int16_t dy=0;

	bool leftDetect = false, rightDetect = false;
	uint8_t leftCount = 0, rightCount = 0;
	
	// Application initialization
	MCUSR = 0;
	wdt_reset();
	wdt_enable(WDTO_250MS);
	wdt_reset();

	PORTB = _BV(SDA) | _BV(SCL) | _BV(JUMPER);  // All outputs low except SDA (pull-up enable), SCL, JUMPER (pull-up enable)
	DDRB = _BV(SCL) | _BV(OUTPUT_WHITE_PULLDOWN) | _BV(OUTPUT_BLUE_PULLDOWN);

	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 94;  // 9.6MHz / 1024 / 94 = 100Hz = 10ms
	trigger = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);  // 1024 prescaler
	TIMSK0 |= _BV(OCIE0A);

	sei();

	// BEGIN: PAT9125_init()
	// Read sensor_pid in address 0x00 to check if the serial link is valid, read value should be 0x31.
	if(0x31 != PAT9125_RegRead(PAT9125_PID1))
	{
		PORTB &= ~_BV(OUTPUT_WHITE_PULLDOWN);
		PORTB &= ~_BV(OUTPUT_BLUE_PULLDOWN);
		while(1); // Force WDT reset
	}
	else
	{
		//PAT9125 sensor recommended settings:
		PAT9125_RegWrite(PAT9125_BANK_SELECTION, 0x00);
		// switch to bank0, not allowed to perform OTS_RegWriteRead
		PAT9125_RegWrite(PAT9125_CONFIG, 0x97);
		// software reset (i.e. set bit7 to 1). OTS_RegWriteRead is not allowed
		// because this bit will clear to 0 automatically.

		_delay_ms(1);
		// delay 1ms

		PAT9125_RegWrite(PAT9125_CONFIG, 0x17);
		// ensure the sensor has left the reset state.
		PAT9125_RegWriteRead(PAT9125_WP, 0x5A);// disable write protect
		PAT9125_RegWriteRead(PAT9125_RES_X, 0x65);// set X-axis resolution (depends on application)
		PAT9125_RegWriteRead(PAT9125_RES_Y, 0xFF);// set Y-axis resolution (depends on application)
		PAT9125_RegWriteRead(PAT9125_ORIENTATION, 0x04);// set 12-bit X/Y data format (depends on application)
		PAT9125_RegWriteRead(PAT9125_MYSTERY, 0x04);// ONLY for VDD=VDDA=1.7~1.9V: for power saving

		if(PAT9125_RegRead(PAT9125_MAGIC_JUJU_2) == 0x04)
		{
			PAT9125_RegWriteRead(PAT9125_MAGIC_JUJU_2, 0x08);
			if(PAT9125_RegRead(PAT9125_MAGIC_JUJU_1) == 0x10)
				PAT9125_RegWriteRead(PAT9125_MAGIC_JUJU_1, 0x19);
		}

		PAT9125_RegWriteRead(PAT9125_WP, 0x00);// enable write protect
	}
	// END: PAT9125_init()

	while (1)
	{
		wdt_reset();

		if(trigger)
		{
			trigger = 0;

			// BEGIN: PAT9125_ReadMotion()
//			int16_t deltaX_l=0;
			int16_t deltaY_l=0;
			int16_t deltaXY_h=0;
//			int16_t deltaX_h=0;
			int16_t deltaY_h=0;

//			dx = 0;
			dy = 0;

			// Read sensor_pid in address 0x00 to check if the serial link is valid, read value should be 0x31.
			if (0x31 != PAT9125_RegRead(PAT9125_PID1))
			{
				PORTB &= ~_BV(OUTPUT_WHITE_PULLDOWN);
				PORTB &= ~_BV(OUTPUT_BLUE_PULLDOWN);
				while(1); // Force WDT reset
			}

			if( PAT9125_RegRead(PAT9125_MOTION) & 0x80 ) //check motion bit in bit7
			{
//				deltaX_l = PAT9125_RegRead(PAT9125_DELTA_XL);
				deltaY_l = PAT9125_RegRead(PAT9125_DELTA_YL);
				deltaXY_h = PAT9125_RegRead(PAT9125_DELTA_XYH);

//				deltaX_h = (deltaXY_h << 4) & 0xF00;
//				if(deltaX_h & 0x800)
//					deltaX_h |= 0xf000;

				deltaY_h = (deltaXY_h << 8) & 0xF00;
				if(deltaY_h & 0x800)
					deltaY_h |= 0xf000;
			}

			//inverse X and/or Y if necessary
//			dx = -(deltaX_h | deltaX_l);
			dy = -(deltaY_h | deltaY_l);
			// END: PAT9125_ReadMotion()

			uint8_t leftMotion = 0;
			uint8_t rightMotion = 0;
			
			// Motion is detected when either leftMotion or rightMotion exceeds the debounce count
			// Once direction is detected, it stays until the abs value of the motion falls within the NO_MOTION_THRESHOLD band
			// Direction can only change once no motion is detected - quick changes in direction will be sensed as the original direction
			if(!leftDetect && !rightDetect)
			{
				// Neither is detecting, so accumulate samples toward a detect
				leftMotion = dy < -getMotionThreshold();
				rightMotion = dy > getMotionThreshold();
			}
			else if(leftDetect)
			{
				// Left is detecting, apply hysteresis
				leftMotion = (dy < -getNoMotionThreshold()) || (dy > getNoMotionThreshold());
				rightMotion = 0;
			}
			else if(rightDetect)
			{
				// Right is detecting, apply hysteresis
				rightMotion = (dy < -getNoMotionThreshold()) || (dy > getNoMotionThreshold());
				leftMotion = 0;
			}
			
			debounce(&leftDetect, &leftCount, leftMotion, ON_DEBOUNCE_COUNT, OFF_DEBOUNCE_COUNT);
			debounce(&rightDetect, &rightCount, rightMotion, ON_DEBOUNCE_COUNT, OFF_DEBOUNCE_COUNT);

			if (leftDetect)
				PORTB |= _BV(OUTPUT_WHITE_PULLDOWN);
			else
				PORTB &= ~_BV(OUTPUT_WHITE_PULLDOWN);

			if (rightDetect)
				PORTB |= _BV(OUTPUT_BLUE_PULLDOWN);
			else
				PORTB &= ~_BV(OUTPUT_BLUE_PULLDOWN);

/*
			if (rightDetect || leftDetect)
			{
				PORTB |= _BV(OUTPUT_WHITE_PULLDOWN);
				PORTB &= ~_BV(OUTPUT_BLUE_PULLDOWN);
			}
			else
			{
				PORTB &= ~_BV(OUTPUT_WHITE_PULLDOWN);
				PORTB |= _BV(OUTPUT_BLUE_PULLDOWN);
			}
*/

		}
	}
}

