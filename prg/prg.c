#include "defines.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include "usart.h"
#include "usart2.h"
#include "comm.h"

//#define UPDATE_CHR_MODULE

#define LED_RED_ON PORTB |= (1<<7)
#define LED_RED_OFF PORTB &= ~(1<<7)
#define LED_GREEN_ON PORTB |= (1<<6)
#define LED_GREEN_OFF PORTB &= ~(1<<6)
#define ROMSEL_HI PORTD |= (1<<7)
#define ROMSEL_LOW PORTD &= ~(1<<7)
#define PHI2_HI PORTA |= (1<<4)
#define PHI2_LOW { PORTA &= ~(1<<4); /*ROMSEL_HI;*/ }
#define MODE_READ { PORTC = 0xFF; DDRC = 0; PORTD |= 1<<6; }
#define MODE_WRITE { PORTD &= ~(1<<6); DDRC = 0xFF; }

ISR(USART0_RX_vect)
{
	unsigned char b;
	while (UCSR0A & (1<<RXC0))
	{
		b = UDR0;
		USART_TransmitByte2(b);
#ifndef UPDATE_CHR_MODULE		
		comm_proceed(b);
#endif
	}
}

ISR(USART1_RX_vect)
{
	unsigned char b;
	while (UCSR1A & (1<<RXC1))
	{
		b = UDR1;
		USART_TransmitByte(b);
	}
}

static void phi2_init()
{
	int i = 0x80;
	unsigned char h = PORTA | (1<<4);
	unsigned char l = PORTA & ~(1<<4);
	while(i != 0){
		PORTA = l;
		PORTA = h;
		i--;
	}
}

static inline void set_address_no_phi2(unsigned int address)
{
	unsigned char l1 = address & 0xFF;
	unsigned char h1 = ((address >> 8) & 0b1111);
	h1 |= ((address >> 7) & 0b11100000); // PHI2 is LOW
	
	PORTF = l1;
	PORTA = h1;
}

static inline void set_address(unsigned int address)
{
	unsigned char l1 = address & 0xFF;
	unsigned char l2 = l1;
	unsigned char h1 = ((address >> 8) & 0b1111);
	h1 |= ((address >> 7) & 0b11100000); // PHI2 is LOW
	unsigned char h2 = h1 | (1<<4); // PHI2 is HIGH
	
	PORTF = l2;
	PORTA = h2;

	if (address & 0x8000)
	{
		ROMSEL_LOW;
	} else {
		ROMSEL_HI;
	}
}

static unsigned char read_prg_byte(unsigned int address)
{
	MODE_READ;
	set_address(address);
	_delay_us(1);
	return PINC;
}

static void read_prg_send(unsigned int address, unsigned int len)
{
	LED_GREEN_ON;
	MODE_READ;
	comm_start(COMMAND_PRG_READ_RESULT, len);
	while (len > 0)
	{
		comm_send_byte(read_prg_byte(address));
		len--;
		address++;
	}
	LED_GREEN_OFF;
}

static void write_prg_byte(unsigned int address, uint8_t data)
{
	PHI2_LOW;
	ROMSEL_HI;
	MODE_WRITE;
	PORTC = data;
	set_address(address); // ROMSEL is low if need, PHI2 high
	_delay_us(1); // WRITING
	PHI2_LOW;	
	ROMSEL_HI;
	MODE_READ;
	PHI2_HI;
}

static void write_prg(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	while (len > 0)
	{
		write_prg_byte(address, *data);
		address++;
		len--;
		data++;
	}
	_delay_ms(10);
	LED_RED_OFF;
}

static void write_eprom_prepare()
{
	LED_RED_ON;
	_delay_ms(1000);
	LED_RED_OFF;
}
static void write_eprom_byte(uint32_t address, uint8_t data)
{
	set_address(address);
	int tries = 0;
	while (tries < 10)
	{
		PORTC = data;
		MODE_WRITE;
		_delay_us(100);
		ROMSEL_LOW;
		_delay_us(100);
		ROMSEL_HI;
		_delay_us(100);
		tries++;
	}
}

static void write_eprom(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	if (address >= 0x8000) address -= 0x8000;
	while (len > 0)
	{
		write_eprom_byte(address, *data);
		address++;
		len--;
		data++;
	}
	MODE_READ;
	LED_RED_OFF;
}

static void write_flash_command(unsigned int address, uint8_t data)
{
	ROMSEL_HI;
	MODE_WRITE;
	PORTC = data;
	set_address(address & 0x7fff); // ROMSEL always HIGH, PHI2 high
	_delay_us(1);
	
	ROMSEL_LOW;
	_delay_us(1); // WRITING
	ROMSEL_HI;
	
	_delay_us(1);
	MODE_READ;
	_delay_us(1);
}

static int write_flash_byte(unsigned int address, uint8_t data)
{
	write_flash_command(0x0555, 0xAA);
	write_flash_command(0x02AA, 0x55);
	write_flash_command(0x0555, 0xA0);
	write_flash_command(address, data);

	int timeout = 0;
	while (read_prg_byte(address | 0x8000) != data && timeout < 10)
	{
		ROMSEL_HI;
		PHI2_LOW;
		_delay_us(100);
		timeout++;
	}
	ROMSEL_HI;
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
	while ((read_prg_byte(0x8000) != 0xFF) && (timeout < 10000))
	{
		ROMSEL_HI;
		_delay_ms(1);
		timeout++;
	}
	ROMSEL_HI;
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
	LED_RED_OFF;
	return ok;
}

static void init_ports()
{
	DDRB |= (1 << 6) | (1 << 7); // LEDS
	MODE_READ;
	ROMSEL_HI;

	PORTF = 0;
	DDRF = 0xFF; // ADDR_LOW
	//PORTA = 0;	// PHI2 low
	DDRA = 0xFF; // ADDR_HIGH + PHI2
	DDRD = (1<<7) | (1<<6); // ROMSEL, R/W
	PORTD = (1<<5); // IRQ pull-up
}

static void reset_phi2()
{
	LED_RED_ON;
	LED_GREEN_ON;		
	/*
	DDRF = 0;
	PORTF = 0;
	DDRA = 0;
	PORTA = 0;
	DDRD = 0;
	PORTD = 0;
	*/
	PHI2_LOW;
	_delay_ms(1000);
//	init_ports();
	PHI2_HI;
	LED_RED_OFF;
	LED_GREEN_OFF;
}

static void write_prg_flash_command(unsigned int address, uint8_t data)
{
	write_prg_byte(address | 0x8000, data);
}

static int erase_coolgirl_sector()
{
	LED_RED_ON;
	write_prg_flash_command(0x0000, 0xF0);
	write_prg_flash_command(0x0AAA, 0xAA);
	write_prg_flash_command(0x0555, 0x55);
	write_prg_flash_command(0x0AAA, 0x80);
	write_prg_flash_command(0x0AAA, 0xAA);
	write_prg_flash_command(0x0555, 0x55);
	write_prg_flash_command(0x0000, 0x30);
	
	int timeout = 0;
	uint8_t debug;
	while (((debug = read_prg_byte(0x8000)) != 0xFF) && (timeout < 3000))
	{
		//comm_start(0xFF, 1);
		//comm_send_byte(debug);
		_delay_ms(1);
		timeout++;
	}
	set_address(0);
	PHI2_HI;
	ROMSEL_HI;

	LED_RED_OFF;
	return timeout < 3000;
}

static int write_coolgirl(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	ROMSEL_HI;
	uint8_t ok = 1;
	while (len > 0)
	{
		
		//uint8_t count = len > 16 ? 16 : len;
		uint8_t count = 0;
		uint8_t* d = data;
		unsigned int a = address;
		unsigned int address_base = a & 0xFFE0;
		while (len > 0 && ((a & 0xFFE0) == address_base))
		{
			if (*d != 0xFF) count++;
			a++;
			len--;
			d++;
		}

		if (count)
		{
			//write_prg_flash_command(0x0000, 0xF0);
			write_prg_flash_command(0x0AAA, 0xAA);
			write_prg_flash_command(0x0555, 0x55);
			write_prg_flash_command(0x0000, 0x25);
			write_prg_flash_command(0x0000, count-1);

			while (count > 0)
			{
				if (*data != 0xFF)
				{
					write_prg_flash_command(address, *data);
					count--;
				}
				address++;
				data++;
			}
		
			write_prg_flash_command(0x0000, 0x29);
			_delay_us(10);

			long int timeout = 0;
			uint8_t res, last_res = 0;
			while (timeout < 100000)
			{
				res = read_prg_byte((address-1) | 0x8000);
				ROMSEL_HI;
				if (res == last_res && last_res == *(data-1)) break;
				last_res = res;
				_delay_us(10);
				timeout++;
			}
			if (timeout >= 100000)
			{
				ok = 0;
				break;
			}
		}
		
		address = a;
		data = d;
	}
	ROMSEL_HI;
	LED_RED_OFF;
	return ok;
}

int main (void)
{
	// Short circuit test
	/*
	DDRB |= (1 << 6) | (1 << 7); // LEDS
	DDRD = (1<<7) | (1<<6); // ROMSEL, R/W
	PORTD = (1<<7) | (1<<6); // ROMSEL, R/W	

	while(1) 
	{
		int i;
		for (i = 0; i < 8; i++)
		{
			DDRA = 0;
			PORTA = 0xFF;
			DDRF = 0;
			PORTF = 0xFF;
			PORTA &= ~(1<<i);
			DDRA |= 1<<i;
			LED_RED_OFF;
			LED_GREEN_OFF;
			_delay_ms(500);
			if ((PINA != PORTA) || (PINF != PORTF))
			{
				LED_RED_ON;
			} else {
				LED_GREEN_ON;
			}
			_delay_ms(500);
		}
		for (i = 0; i < 8; i++)
		{
			DDRA = 0;
			PORTA = 0xFF;
			DDRF = 0;
			PORTF = 0xFF;
			PORTF &= ~(1<<i);
			DDRF |= 1<<i;
			LED_RED_OFF;
			LED_GREEN_OFF;
			_delay_ms(500);
			if ((PINA != PORTA) || (PINF != PORTF))
			{
				LED_RED_ON;
			} else {
				LED_GREEN_ON;
			}
			_delay_ms(500);
		}

	}
*/
/*
	DDRB |= (1 << 6) | (1 << 7); // LEDS
	DDRD = (1<<7) | (1<<6); // ROMSEL, R/W
	PORTD = (1<<7) | (1<<6); // ROMSEL, R/W

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

	sei();
	USART_init();
	USART_init2();	
		
#ifdef UPDATE_CHR_MODULE
  unsigned int bd = (F_CPU / (16UL * 19200UL)) - 1;
  UBRR0L = bd & 0xFF;
  UBRR0H = bd >> 8;
  UBRR1L = bd & 0xFF;
  UBRR1H = bd >> 8;
	LED_RED_ON;
	LED_GREEN_ON;		

	while(1)	{	}
#endif

	init_ports();	

	LED_RED_OFF;
	LED_GREEN_OFF;
	
	// MMC1 test
	/*
	set_address(0x8000);
	PHI2_LOW;
	MODE_WRITE;
	PORTC = 0;
	while (1)
	{
		LED_GREEN_OFF;
		ROMSEL_HI;
		_delay_ms(200);
		LED_GREEN_ON;
		ROMSEL_LOW;
		_delay_ms(200);
	}
	*/
	/*
	write_prg_byte(0x8000, 0x80);
	write_prg_byte(0x8000, 0); 
	write_prg_byte(0x8000, 0);
	write_prg_byte(0x8000, 1);
	write_prg_byte(0x8000, 1);
	write_prg_byte(0x8000, 0);
	
	write_prg_byte(0xe000, 0); 
	write_prg_byte(0xe000, 0);
	write_prg_byte(0xe000, 0);
	write_prg_byte(0xe000, 0);
	write_prg_byte(0xe000, 0);
	
	_delay_ms(500);
	
	MODE_READ;
	//set_address(0x8000);
	PORTA = 0;
	while(1);
	*/	
	
	comm_init();
	comm_start(COMMAND_PRG_STARTED, 0);

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
				case COMMAND_PRG_INIT:
					comm_start(COMMAND_PRG_STARTED, 0);
					break;
					
				case COMMAND_PRG_READ_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					read_prg_send(address, length);
					break;

				case COMMAND_PRG_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					write_prg(address, length, (uint8_t*)&recv_buffer[4]);
					comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;

				case COMMAND_PHI2_INIT:
					phi2_init();
					comm_start(COMMAND_PHI2_INIT_DONE, 0);
					break;

				case COMMAND_RESET:
					reset_phi2();
					comm_start(COMMAND_RESET_ACK, 0);
					break;
					
				case COMMAND_EPROM_PREPARE:
					write_eprom_prepare();
					break;

				case COMMAND_PRG_EPROM_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					write_eprom(address, length, (uint8_t*)&recv_buffer[4]);
					comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;

				case COMMAND_PRG_FLASH_ERASE_REQUEST:
					if (erase_flash())
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;

				case COMMAND_PRG_FLASH_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					if (write_flash(address, length, (uint8_t*)&recv_buffer[4]))
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;

				case COMMAND_COOLGIRL_ERASE_SECTOR_REQUEST:
					if (erase_coolgirl_sector())
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;

				case COMMAND_COOLGIRL_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					if (write_coolgirl(address, length, (uint8_t*)&recv_buffer[4]))
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;
			}
			comm_recv_done = 0;
		}		
	}
}

