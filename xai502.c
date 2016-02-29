/* USB Mouse Plus Debug Channel Example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_mouse.html
 * Copyright (c) 2009 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "usb_mouse.h"
#include "srom_3366.h"
#include "uart.h"

#define BAUD_RATE 38400

#define PORT_SPI PORTB
#define DDR_SPI	DDRB
#define DD_SS	0
#define DD_SCK	1
#define DD_MOSI	2
#define DD_MISO	3

#define SS_LOW(void)	(PORT_SPI &= ~(1<<DD_SS))
#define SS_HIGH(void)	(PORT_SPI |= (1<<DD_SS))

#define PORT_BUTTON PORTD
#define DDR_BUTTON DDRD
#define PIN_BUTTON PIND
#define BUTTON_LEFT 	3
#define BUTTON_MIDDLE	1
#define BUTTON_RIGHT	0
#define BUTTON_BACK		4
#define BUTTON_FORWARD	2

#define PORT_WHEEL PORTC
#define DDR_WHEEL DDRC
#define PIN_WHEEL PINC
#define WHEEL_A 	6
#define WHEEL_B		7


static void pins_init(void) {
	// Button Pins
	DDR_BUTTON &= ~(1<<BUTTON_LEFT); PORT_BUTTON |= (1<<BUTTON_LEFT); // Pullup
	DDR_BUTTON &= ~(1<<BUTTON_MIDDLE); PORT_BUTTON |= (1<<BUTTON_MIDDLE); // Pullup
	DDR_BUTTON &= ~(1<<BUTTON_RIGHT); PORT_BUTTON |= (1<<BUTTON_RIGHT); // Pullup
	DDR_BUTTON &= ~(1<<BUTTON_FORWARD); PORTD |= (1<<BUTTON_FORWARD); 
	DDR_BUTTON &= ~(1<<BUTTON_BACK); PORTD |= (1<<BUTTON_BACK);

	// Wheel Pins
	DDR_WHEEL &= ~(1<<WHEEL_A); PORT_WHEEL |= (1<<WHEEL_A); // Wheel A pullup
	DDR_WHEEL &= ~(1<<WHEEL_B); PORT_WHEEL |= (1<<WHEEL_B); // Wheel B pullup

	// FPC Pins
	DDRB |= (1<<4); PORTB |= (1<<4); // FPC 2 to B 4 High Output
    DDRF |= (1<<5); PORTF |= (1<<5); // FPC 12 to F 5 High Output
    DDRF &= ~(1<<4); // FPC 13 to F 4 Normal Input
    DDRF &= ~(1<<1); // FPC 14 to F 1 Normal Input
    DDRF &= ~(1<<0); PORTF |= (1<<0); // FPC 10, F 0 Pullup Input
}

static void spi_init(void) {
	// Initialise SPI comms
	DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS);
	
	// Set MISO to Pullup Input
	DDR_SPI &= ~(1<<DD_MISO); PORT_SPI |= (1<<DD_MISO); 

	// Enable SPI; master mode, mode 3, & Set Clock Rate fck/4 (2MHz)
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);
}

static inline void spi_send(const uint8_t byte) {
	SPDR = (byte);
	while(!(SPSR & (1<<SPIF)));
}

static inline uint8_t spi_receive(void) {
	spi_send(0x00);
	return SPDR;
}

static inline void spi_write(const uint8_t address, const uint8_t data) {
	SS_LOW();
	spi_send((address) | 0x80);
	spi_send(data);
	_delay_us(30);
	SS_HIGH();
}

static inline int8_t spi_read(const uint8_t address) {
	spi_send((address));
	_delay_us(4);
	int8_t data = SPDR;
	return data;
}

static void set_dpi(uint8_t multi) {
	spi_write(0x0f, multi);
}

uint8_t EEMEM stored_dpi;

static void eeprom_init(void) {
	eeprom_write_byte(&stored_dpi, 0x17);
	uint8_t new_dpi = eeprom_read_byte(&stored_dpi);

	set_dpi(new_dpi);
}

static void sensor_init(void) { 
//most of this is just copying the g502 power up procedure "verbatim"
	uint16_t i;
	uint8_t *psrom = srom;

	SS_HIGH();
	_delay_ms(3);

	// Shutdown first
	spi_write(0x3b, 0xb6);
	_delay_ms(300);

	// Drop and raise ncs to reset spi port
	SS_LOW();
	_delay_us(40);
	SS_HIGH();
	_delay_us(40);
	
	// Power up reset
	spi_write(0x3a, 0x5a);
	_delay_ms(50);
	
	// Flip on and off the clock tuning. not sure purpose...
	spi_write(0x3d, 0x95);
	_delay_ms(1);
	spi_write(0x3d, 0x15);

	spi_write(0x10, 0x00); //well i know this disables rest mode. not sure purpose
	spi_write(0x22, 0x00); //???
	
	//srom download
	spi_write(0x13, 0x1d);
	_delay_ms(10);
	spi_write(0x13, 0x18);
	
	SS_LOW();
	spi_send(0x62 | 0x80);
	for (i = 0; i < SROM_LENGTH; i++) {
		_delay_us(16);
		spi_send(pgm_read_byte(psrom++));
	}
	_delay_us(18);
	SS_HIGH();
	_delay_us(200);
	
	// Configuration
	spi_write(0x10, 0x00); //0x20 (g502 default) enables rest mode after ~10s of inactivity
	spi_write(0x14, 0xff); //how long to wait before going to rest mode. 0xff is max (~10 seconds)
	spi_write(0x17, 0xff); //???
	spi_write(0x18, 0x00); //???
	spi_write(0x19, 0x00); //???
	spi_write(0x1b, 0x00); //???
	spi_write(0x1c, 0x00); //???

	// Surface Tuning (default: 0x0a, 0x10)
	_delay_us(18);
	spi_write(0x2c, 0x0a);
	spi_write(0x2b, 0x10);

	// "Manual" Clock Tuning
	_delay_us(500); //arbitrary padding
	spi_write(0x3d, 0x16); //increase this to increase the clock frequency during run mode
	_delay_us(500); //arbitrary padding
	spi_write(0x4f, 0x0e); //increase this to increase the clock frequency during rest mode
	_delay_us(500); //arbitrary padding

	// Motion Burst (not sure if necessary)
	spi_write(0x50, 0x00);
	_delay_us(9);
	SS_LOW();
	spi_send(0x50);
	_delay_us(42);
	spi_send(0x00);
	spi_send(0x00);
	spi_send(0x00);
	spi_send(0x00);
	spi_send(0x00);
	spi_send(0x00);
	_delay_us(4);
	SS_HIGH();
	_delay_us(500); //arbitrary padding

	//dpi
	eeprom_init();
	_delay_us(500); //arbitrary padding
	
	//motion burst
	spi_write(0x50, 0x00);
	_delay_us(9);
	SS_LOW();
	spi_send(0x50);
	_delay_us(42);
	spi_send(0x00);
	spi_send(0x00);
	spi_send(0x00);
	spi_send(0x00);
	spi_send(0x00);
	spi_send(0x00);
	_delay_us(4);
	SS_HIGH();
	_delay_us(500); //arbitrary padding
	//angle snapping
	spi_write(0x42, 0x00);
	_delay_us(500); //arbitrary padding
}

static inline uint8_t button_read(void) {
	uint8_t mask=0;

	mask |= (!(PIN_BUTTON & (1<<BUTTON_LEFT)) << 0);
	mask |= (!(PIN_BUTTON & (1<<BUTTON_MIDDLE)) << 2);
	mask |= (!(PIN_BUTTON & (1<<BUTTON_RIGHT)) << 1);
	mask |= (!(PIN_BUTTON & (1<<BUTTON_BACK)) << 3);
	mask |= (!(PIN_BUTTON & (1<<BUTTON_FORWARD)) << 4);
	
	return mask;
}

static inline int8_t wheel_read(void) {
	uint8_t a, b;
	if ((PIN_WHEEL & (1<<WHEEL_A))) {
		a = 1;
	} else {
		a = 0;
	}

	if ((PIN_WHEEL & (1<<WHEEL_B))) {
		b = 1;
	} else {
		b = 0;
	}
	
	static uint8_t a_b;
	a_b <<= 2;
	a_b |= (((a << 1) | b) & 0x03);

	static int8_t wheel_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
	return wheel_states[(a_b & 0x0f)];
}

static void enter_bootloader_mode(void) {
	// Copied from https://www.pjrc.com/teensy/jump_to_bootloader.html

	cli();
	// disable watchdog, if enabled
	// disable all peripherals
	UDCON = 1;
	USBCON = (1<<FRZCLK);  // disable USB
	UCSR1B = 0;
	_delay_ms(5);
	#if defined(__AVR_AT90USB162__)                // Teensy 1.0
	    EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0;
	    TIMSK0 = 0; TIMSK1 = 0; UCSR1B = 0;
	    DDRB = 0; DDRC = 0; DDRD = 0;
	    PORTB = 0; PORTC = 0; PORTD = 0;
	    asm volatile("jmp 0x3E00");
	#elif defined(__AVR_ATmega32U4__)              // Teensy 2.0
	    EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
	    TIMSK0 = 0; TIMSK1 = 0; TIMSK3 = 0; TIMSK4 = 0; UCSR1B = 0; TWCR = 0;
	    DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0; TWCR = 0;
	    PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
	    asm volatile("jmp 0x7E00");
	#elif defined(__AVR_AT90USB646__)              // Teensy++ 1.0
	    EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
	    TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
	    DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
	    PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
	    asm volatile("jmp 0xFC00");
	#elif defined(__AVR_AT90USB1286__)             // Teensy++ 2.0
	    EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
	    TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
	    DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
	    PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
	    asm volatile("jmp 0x1FC00");
	#endif 


	// Disable Current Timers
	TCCR1B = 0;
	TCNT1 = 0;
}

static void uart_handle_input(void) {
	uint8_t input_byte = uart_getchar();
	switch (input_byte) {
		case 0xFF:
			enter_bootloader_mode();
		default:
			set_dpi(input_byte);
	}
}

int main(void) {
	// Set clock prescaler for 8MHz
	CLKPR = 0x80;
	CLKPR = 0x01;
	
	pins_init();

	usb_init();
	while (!usb_configured());
	_delay_ms(500);

	spi_init();
	
	uart_init(BAUD_RATE);
	//eeprom_init();

	sensor_init();

	// Configures Prescaler & Timer, Increments every 1/8th microsecond
	TCCR1B |= ((1 << CS10));
	TCNT1 = 0; // Reset Timer Count

	// Measure Timer Set & Assignment duration
	uint16_t assignment_duration = TCNT1; 

	uint16_t usb_frame_duration = 8000;
	usb_frame_duration -= assignment_duration;

	// Measure Wheel Polling duration
	TCNT1 = 0; // Reset Timer Count
	int8_t wheel = wheel_read();
	uint16_t wheel_read_duration = TCNT1; // Snapshot
	wheel_read_duration -= assignment_duration;

	// Measure sensor movement duration
	TCNT1 = 0; // Reset Timer Count
	fast_usb_mouse_update(0, 0);

	uint16_t sensor_duration = TCNT1; // Snapshot
	sensor_duration -= assignment_duration; 

	// Offset for while check
	uint16_t pre_sensor_period = ( usb_frame_duration - sensor_duration ) - 32; 

	// Offset for clearance
	uint16_t button_period = ( pre_sensor_period - wheel_read_duration ) - 48; 

	// Main Loop
	while (1) {
		UDINT &= ~(1<<SOFI); // Ack Start of Frame
		while(!(UDINT & (1<<SOFI))); // Wait until Start Of Frame occurs
		TCNT1 = 0; // Reset Timer Count
		SS_HIGH();

		// Serial Comms
		if ( uart_available() ) {
			uart_handle_input();
		}

		// Button Read Loop
		uint8_t button_mask = 0;
		while ( TCNT1 < button_period ) {
			button_mask |= button_read();
		}

		// Wheel Read
		int8_t wheel = wheel_read();

		while ( TCNT1 < pre_sensor_period ); // Wait to align Sensor Read
		fast_usb_mouse_update( button_mask , wheel );
	}
}
