#include "defines.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include "usart.h"
#include "comm.h"

#define LED_RED_ON PORTB |= (1<<7)
#define LED_RED_OFF PORTB &= ~(1<<7)
#define LED_GREEN_ON PORTB |= (1<<6)
#define LED_GREEN_OFF PORTB &= ~(1<<6)
#define INV_A13_HI PORTF |= (1<<6)
#define INV_A13_LOW PORTF &= ~(1<<6)
#define MODE_READ { PORTC = 0xFF; DDRC = 0; }
#define MODE_WRITE { DDRC = 0xFF; }
#define WRITE_HI PORTF |= (1<<4)
#define WRITE_LOW PORTF &= ~(1<<4)
#define READ_HI PORTG |= (1<<3)
#define READ_LOW PORTG &= ~(1<<3)

ISR(USART0_RX_vect)
{
	unsigned char b;
	while (UCSR0A & (1<<RXC0))
	{
		b = UDR0;
		comm_proceed(b);
	}
}

static inline void clock_wait(double clock)
{
	_delay_us(clock * 0.55);
}

static const unsigned char BitReverseTable256[] = 
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};


void set_address(unsigned int address)
{	
	unsigned char l7 = BitReverseTable256[address & 0b01111111];
	unsigned char h7 = address >> 7;
	PORTD = l7;
	PORTA = h7;
	if ((address >> 13) & 1)
	{
		INV_A13_LOW;
	} else {
		INV_A13_HI;
	}
}

unsigned char read_chr_byte(unsigned int address)
{
	unsigned char data;
	MODE_READ;
	set_address(address);
	_delay_us(1);
	READ_LOW;
	_delay_us(1);
	data = PINC;		
	READ_HI;
	set_address(address | 0x2000);
	_delay_us(1);
	return data;
}

static void read_chr_send(unsigned int address, unsigned int len)
{
	LED_GREEN_ON;
	comm_start(COMMAND_CHR_READ_RESULT, len);
	while (len > 0)
	{
		comm_send_byte(read_chr_byte(address));
		len--;
		address++;
	}
	LED_GREEN_OFF;
}

void write_chr_byte(unsigned int address, uint8_t data)
{
	MODE_WRITE;
	PORTC = data;
	set_address(address);
	WRITE_LOW;
	_delay_us(1);
	WRITE_HI;
	MODE_READ;
}

void write_chr(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	while (len > 0)
	{
		write_chr_byte(address, *data);
		len--;
		address++;
		data++;
	}
	LED_RED_OFF;
}

static void write_eprom_prepare()
{
	set_address(0x2000);
	READ_HI;
	WRITE_HI;
	MODE_WRITE;
	LED_RED_ON;
	_delay_ms(1000);
	LED_RED_OFF;
}

static void write_eprom_byte(unsigned int address, uint8_t data)
{
	set_address(address | (1<<13));
	int tries = 0;
	while (tries < 10)
	{
		MODE_WRITE;
		PORTC = data;
		_delay_us(100);
		set_address(address);		
		WRITE_LOW;
		_delay_us(100);
		set_address(address | (1<<13));
		WRITE_HI;
		_delay_us(100);
		tries++;
	}
}

static void write_eprom(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	while (len > 0)
	{
		write_eprom_byte(address, *data);
		len--;
		address++;
		data++;
	}
	MODE_READ;
	LED_RED_OFF;
}

static void write_flash_command(unsigned int address, uint8_t data)
{
	set_address(address | (1<<13));
	WRITE_HI;
	MODE_WRITE;
	PORTC = data;
	_delay_us(1);
	set_address(address);
	WRITE_LOW;
	_delay_us(1);
	set_address(address | (1<<13));
	WRITE_HI;
	_delay_us(1);
}

static int write_flash_byte(unsigned int address, uint8_t data)
{
	write_flash_command(0x0555, 0xAA);
	write_flash_command(0x02AA, 0x55);
	write_flash_command(0x0555, 0xA0);
	write_flash_command(address, data);
	
	int timeout = 0;
	while (read_chr_byte(address) != data && timeout < 10)
	{
		_delay_us(100);
		timeout++;
	}
	return timeout < 10;
}

static int erase_flash()
{
	LED_RED_ON;
	write_flash_command(0x0555, 0xAA);
	write_flash_command(0x02AA, 0x55);
	write_flash_command(0x0555, 0x80);
	write_flash_command(0x0555, 0xAA);
	write_flash_command(0x02AA, 0x55);
	write_flash_command(0x0555, 0x10);
	
	int timeout = 0;
	while ((read_chr_byte(0) != 0xFF) && (timeout < 10000))
	{
		_delay_ms(1);
		timeout++;
	}
	LED_RED_OFF;
	return timeout < 10000;
}

static int write_flash(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	if (address >= 0x8000) address -= 0x8000;
	int ok = 1;
	while (len > 0)
	{
		if (!write_flash_byte(address, *data))
		{
			ok = 0;
			break;
		}
		address++;
		len--;
		data++;
	}
	MODE_READ;
	LED_RED_OFF;
	return ok;
}

uint8_t get_mirroring()
{
	LED_GREEN_ON;
	set_address(1<<10);
//	READ_HI;
	_delay_ms(1);
	char is_vertical = ((PING >> 4) & 1);
	set_address(1<<11);
	_delay_ms(1);
	char is_horizontal = ((PING >> 4) & 1);
	LED_GREEN_OFF;
	if (is_vertical && is_horizontal) return 0xfe;
	if (is_vertical) return 1;
	if (is_horizontal) return 0;
	return 0xff;
}

/*
static void reset()
{
	READ_LOW;
	WRITE_LOW;
	MODE_READ;
	set_address(0);
	_delay_ms(1000);	
	READ_HI;
	WRITE_HI;
}
*/

int main (void)
{
	// Short circuit test
/*
	DDRB |= (1 << 6) | (1 << 7); // LEDS
	DDRF |= (1<<4); // PPU /WR
	WRITE_HI;
	DDRG |= (1<<3); // PPU /RD
	READ_HI;
	while(1) 
	{
		int i;
		for (i = 0; i < 8; i++)
		{
			DDRC = 0;
			PORTC = 0xFF;
			PORTC &= ~(1<<i);
			DDRC |= 1<<i;
			LED_RED_OFF;
			LED_GREEN_OFF;
			_delay_ms(500);
			if ((PINC != PORTC))
			{
				LED_RED_ON;
			} else {
				LED_GREEN_ON;
			}
			_delay_ms(500);
		}

	}
*/

	_delay_ms(500);
	
	USART_init();
	sei();

	DDRB |= (1 << 6) | (1 << 7); // LEDS
	LED_RED_OFF;
	LED_GREEN_OFF;
	
	DDRF |= (1<<4); // PPU /WR
	WRITE_HI;
	DDRG |= (1<<3); // PPU /RD
	READ_HI;
	DDRG &= ~(1<<4); // CIRAM A10 - input
	PORTG |= (1<<4);
	
		
	// Addr
	set_address(0x2000);
	DDRA = 0xFF;
	DDRD = 0xFF;
	DDRF |= (1<<6); // /A13
	
	MODE_READ;
	
	
	comm_init();
	comm_start(COMMAND_CHR_STARTED, 0);
	
	uint16_t address;
	uint16_t length;
	
	unsigned long int t = 0;
	char led_down = 0;
	int led_bright = 0;
	
	while (1)
	{
		TCCR1A |= (1<<COM1C1) | (1<<COM1B1) | (1<<WGM10);
		TCCR1B |= (1<<CS10);
		if (t++ >= 10000)
		{
			if (!led_down)
			{
				led_bright++;
				if (led_bright >= 110) led_down = 1;
			} else {
				led_bright--;
				if (!led_bright) led_down = 0;
			}
			if (led_bright >= 100) OCR1B = led_bright - 100;
			if (led_down)
			{
				int led_bright2 = 110-led_bright;
				if (led_bright2 <= 20)
				{
					if (led_bright2 > 10) led_bright2 = 20 - led_bright2;
					OCR1C = led_bright2*2;
				}
			}
			t = 0;
		}
		
		if (comm_recv_done)
		{
			t = led_down = led_bright = 0;
			TCCR1A = OCR1B = OCR1C = 0;
			
			switch (comm_recv_command)
			{
				case COMMAND_CHR_INIT:
					comm_start(COMMAND_CHR_STARTED, 0);
					break;
					
				case COMMAND_CHR_READ_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					read_chr_send(address, length);
					break;

				case COMMAND_CHR_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					write_chr(address, length, (uint8_t*)&recv_buffer[4]);
					comm_start(COMMAND_CHR_WRITE_DONE, 0);
					break;

				case COMMAND_MIRRORING_REQUEST:
					comm_start(COMMAND_MIRRORING_RESULT, 1);
					comm_send_byte(get_mirroring());
					break;

				case COMMAND_EPROM_PREPARE:
					write_eprom_prepare();
					break;
				
				case COMMAND_CHR_EPROM_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					write_eprom(address, length, (uint8_t*)&recv_buffer[4]);
					comm_start(COMMAND_CHR_WRITE_DONE, 0);
					break;
					
				case COMMAND_CHR_FLASH_ERASE_REQUEST:
					if (erase_flash())
						comm_start(COMMAND_CHR_WRITE_DONE, 0);
					break;

				case COMMAND_CHR_FLASH_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					if (write_flash(address, length, (uint8_t*)&recv_buffer[4]))
						comm_start(COMMAND_CHR_WRITE_DONE, 0);
					break;


				/*
				case COMMAND_RESET:
					reset();
					break;
				*/
			}
			comm_recv_done = 0;
		}		
	}
}

