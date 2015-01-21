#ifndef _USART2_H
#define _USART2_H

void USART_init2(void);
void USART_TransmitByte2( unsigned char data );
void USART_TransmitText2(char* data);
void USART_Transmit2(void* p, unsigned long int len);
void USART_TransmitHex2(unsigned char data);

#endif
