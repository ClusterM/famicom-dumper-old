#include "defines.h"
#include <avr/io.h>

void USART_init2(void)
{
  unsigned int bd = (F_CPU / (16UL * UART_BAUD)) - 1;
  UBRR1L = bd & 0xFF;
  UBRR1H = bd >> 8;

  UCSR1B = _BV(TXEN1) | _BV(RXEN1) | _BV(RXCIE1); /* tx/rx enable */
//  UCSRC = 1<<URSEL|1<<UCSZ0|1<<UCSZ1;
  UCSR1C |= /*_BV(UMSEL1)|*/  _BV(UCSZ11) | _BV(UCSZ10);
  //UCSRA = _BV(U2X);
}

void USART_TransmitByte2( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) );
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

void USART_TransmitHex2( unsigned char data )
{
	unsigned char h = data>>4;
	char ho = (h < 10) ? (h+'0') : (h+'A'-10);
	unsigned char l = data & 0xF;
	char lo = (l < 10) ? (l+'0') : (l+'A'-10);
	while ( !( UCSR1A & (1<<UDRE1)) );
	UDR1 = ho;
	while ( !( UCSR1A & (1<<UDRE1)) );
	UDR1 = lo;
}

void USART_TransmitText2(char* data)
{
	while (*data != 0)
	{
		/* Wait for empty transmit buffer */
		while ( !( UCSR1A & (1<<UDRE1)) );
		/* Put data into buffer, sends the data */
		UDR1 = *data;
		data++;
	}
}

void USART_Transmit2(void* p, unsigned long int len)
{
	unsigned char* buff = (unsigned char*)p;
	unsigned long int b;
	for (b = 0; b < len; b++) USART_TransmitByte2(buff[b]);
}
