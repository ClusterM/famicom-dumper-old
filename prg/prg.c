#include "defines.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include "usart.h"
#include "usart2.h"
#include "comm.h"

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
	
	if (address & 0x8000)
	{
		ROMSEL_LOW;
	} else {
		ROMSEL_HI;
	}

	PORTF = l2;
	PORTA = h2;

	_delay_us(1);
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
	set_address_no_phi2(address); // PHI2 low, ROMSEL always HIGH
	_delay_us(1);
	
	set_address(address); // ROMSEL is low if need, PHI2 high
	_delay_us(1); // WRITING
	PHI2_LOW;	
	ROMSEL_HI;
	
	_delay_us(1);
	MODE_READ;
	_delay_us(1);
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

static void reset_phi2()
{
	LED_RED_ON;
	LED_GREEN_ON;		

	PHI2_LOW;
	_delay_ms(1000);
	PHI2_HI;

	LED_RED_OFF;
	LED_GREEN_OFF;
}

int main (void)
{	
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

	DDRB |= (1 << 6) | (1 << 7); // LEDS
	MODE_READ;
	ROMSEL_HI;

	PORTF = 0;
	DDRF = 0xFF; // ADDR_LOW
	//PORTA = 0;	// PHI2 low
	DDRA = 0xFF; // ADDR_HIGH + PHI2
	DDRD = (1<<7) | (1<<6); // ROMSEL, R/W
	PORTD = (1<<5); // IRQ pull-up
	LED_RED_OFF;
	LED_GREEN_OFF;
		
	comm_init();
	comm_start(COMMAND_PRG_STARTED, 0);

	uint16_t address;
	uint16_t length;	
	
	while (1)
	{
		if (comm_recv_done)
		{
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

				case COMMAND_RESET:
					reset_phi2();
					comm_start(COMMAND_RESET_ACK, 0);
					break;
			}
			comm_recv_done = 0;
		}		
	}
}

