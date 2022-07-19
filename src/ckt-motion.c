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
 *     ---------  ^ NEGATIVE
 *     |   *   |  |
 *     |-------|  |
 *     ||     ||  dy
 *     ||     ||  |
 *     |-------|  |
 *     ---------  v POSITIVE
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

#define SENSOR_ERROR_THRESHOLD    0
#define RIGHT_MOTION_THRESHOLD  75
#define LEFT_MOTION_THRESHOLD  -75

#define ON_DEBOUNCE_COUNT      SET_DECISECS
#define OFF_DEBOUNCE_COUNT     RELEASE_DECISECS

#define SDA   PB0
#define SCL   PB2

static inline void sda_low() { DDRB |= _BV(SDA); PORTB &= ~_BV(SDA); _delay_us(3); }
static inline void sda_high() { DDRB &= ~_BV(SDA); PORTB |= _BV(SDA); _delay_us(3); }
static inline void scl_low() { PORTB &= ~_BV(SCL); _delay_us(3); }
static inline void scl_high() { PORTB |= _BV(SCL); _delay_us(3); }

volatile uint8_t decisecs = 0;

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 94;  // 9.6MHz / 1024 / 94 = 100Hz
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);  // 1024 prescaler
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIM0_COMPA_vect)
{
	static uint8_t ticks = 0;
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
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

void init(void)
{
	MCUSR = 0;
	wdt_reset();
	wdt_enable(WDTO_250MS);
	wdt_reset();

	// Configure ADC
	ADMUX = 0x02;  // ref = VCC, right-justified, input = PB4 (ADC2)
	ADCSRA = 0x87; // ADC enabled, prescaler = 128
	DIDR0 = _BV(ADC2D);  // Disable ADC2 digital input buffer
	
	PORTB = _BV(SCL) | _BV(PB1);  // All outputs low except SCL and PB1
	DDRB |= _BV(PB1) | _BV(SCL) | _BV(PB3);
}

bool PAT9125_test()
{
	if (0x31 == PAT9125_RegRead(0x00))
		return true;
	return false;
}

uint8_t PAT9125_init()
{
	uint8_t sensor_pid=0, read_id_ok=0;
	// Read sensor_pid in address 0x00 to check if the serial link is valid, read value should be 0x31.
	sensor_pid = PAT9125_RegRead(0x00);

	if(sensor_pid == 0x31)
	{
		read_id_ok =1;
		//PAT9125 sensor recommended settings:
		PAT9125_RegWrite(0x7F, 0x00);
		// switch to bank0, not allowed to perform OTS_RegWriteRead
		PAT9125_RegWrite(0x06, 0x97);
		// software reset (i.e. set bit7 to 1). OTS_RegWriteRead is not allowed
		// because this bit will clear to 0 automatically.

		_delay_ms(1);
		// delay 1ms

		PAT9125_RegWrite(0x06, 0x17);
		// ensure the sensor has left the reset state.
		PAT9125_RegWriteRead(0x09, 0x5A);// disable write protect
		PAT9125_RegWriteRead(0x0D, 0x65);// set X-axis resolution (depends on application)
		PAT9125_RegWriteRead(0x0E, 0xFF);// set Y-axis resolution (depends on application)
		PAT9125_RegWriteRead(0x19, 0x04);// set 12-bit X/Y data format (depends on application)
		PAT9125_RegWriteRead(0x4B, 0x04);// ONLY for VDD=VDDA=1.7~1.9V: for power saving

		if(PAT9125_RegRead(0x5E) == 0x04)
		{
			PAT9125_RegWriteRead(0x5E, 0x08);
			if(PAT9125_RegRead(0x5D) == 0x10)
				PAT9125_RegWriteRead(0x5D, 0x19);
		}

		PAT9125_RegWriteRead(0x09, 0x00);// enable write protect
	}

	return read_id_ok;
}

bool PAT9125_ReadMotion(int16_t *dx, int16_t *dy)
{
	int16_t deltaX_l=0, deltaY_l=0, deltaXY_h=0;
	int16_t deltaX_h=0, deltaY_h=0;

	*dx = 0;
	*dy = 0;

	if (!PAT9125_test())
		return false;

	if( PAT9125_RegRead(0x02) & 0x80 ) //check motion bit in bit7
	{
		deltaX_l = PAT9125_RegRead(0x03);
		deltaY_l = PAT9125_RegRead(0x04);
		deltaXY_h = PAT9125_RegRead(0x12);

		deltaX_h = (deltaXY_h << 4) & 0xF00;
		if(deltaX_h & 0x800)
			deltaX_h |= 0xf000;

		deltaY_h = (deltaXY_h << 8) & 0xF00;
		if(deltaY_h & 0x800)
			deltaY_h |= 0xf000;
	}

	//inverse X and/or Y if necessary
	*dx = -(deltaX_h | deltaX_l);
	*dy = -(deltaY_h | deltaY_l);
	return true;
}

void setOutputs(bool detect)
{
	if(detect)
	{
		PORTB |= _BV(PB3);
		PORTB &= ~_BV(PB1);
	}
	else
	{
		PORTB &= ~_BV(PB3);
		PORTB |= _BV(PB1);
	}
}

void hysteresis(bool *detect, uint8_t* count, bool isDetecting, const uint8_t onDebounceCount, const uint8_t offDebounceCount)
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
//		*count = 0;
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

//		*count = 0;
	}
}


int main(void)
{
	uint8_t sensorError = 0;
//	int16_t adc, adc_filt = 0;
	bool ack = false;
	int16_t dx=0, dy=0;

	bool leftDetect = false, rightDetect = false;
	uint8_t leftCount = 0, rightCount = 0;

	// Application initialization
	init();
	initialize100HzTimer();
	sei();

	PAT9125_init();

	while (1)
	{
		wdt_reset();

		if(decisecs >= 1)
		{
			decisecs = 0;

/*			// Read ADC
			ADCSRA |= _BV(ADSC);  // Trigger conversion
			while(ADCSRA & _BV(ADSC));
			adc = ADC;
			adc_filt = adc_filt + ((adc - adc_filt) / 4);
*/
			if (sensorError)
				PAT9125_init();

			ack = PAT9125_ReadMotion(&dx, &dy);
			if (!ack)
			{
				// Sensor's gone wonky, reset it and try again
				if (sensorError < 255)
					sensorError++;

				if (sensorError > SENSOR_ERROR_THRESHOLD)
				{
					leftDetect = rightDetect = false;
					leftCount = rightCount = 0;
					setOutputs(false);
				}

				continue;

			} else {
				sensorError = 0;
			}

			hysteresis(&leftDetect, &leftCount, dy < LEFT_MOTION_THRESHOLD, ON_DEBOUNCE_COUNT, OFF_DEBOUNCE_COUNT);
			hysteresis(&rightDetect, &rightCount, dy > RIGHT_MOTION_THRESHOLD, ON_DEBOUNCE_COUNT, OFF_DEBOUNCE_COUNT);

#if defined MODE_LEFT_MOTION_ONLY
			if (leftDetect)
				setOutputs(true);
			else
				setOutputs(false);
#elif defined MODE_RIGHT_MOTION_ONLY
			if (rightDetect)
				setOutputs(true);
			else
				setOutputs(false);
#elif defined MODE_DIRECTION_MOTION
			if (rightDetect)
				PORTB |= _BV(PB3);
			else
				PORTB &= ~_BV(PB3);

			if (leftDetect)
				PORTB |= _BV(PB1);
			else
				PORTB &= ~_BV(PB1);
#else
			if (rightDetect || leftDetect)
				setOutputs(true);
			else
				setOutputs(false);
#endif

		}
	}
}

