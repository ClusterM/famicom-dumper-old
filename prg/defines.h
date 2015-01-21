#define F_CPU 8000000UL
#define UART_BAUD 250000UL
#define RECV_BUFFER 1050

//#define UPDATE_CHR_MODULE

#define LED_RED_ON PORTB |= (1<<7)
#define LED_RED_OFF PORTB &= ~(1<<7)
#define LED_GREEN_ON PORTB |= (1<<6)
#define LED_GREEN_OFF PORTB &= ~(1<<6)
#define ROMSEL_HI PORTD |= (1<<7)
#define ROMSEL_LOW PORTD &= ~(1<<7)
#define PHI2_HI PORTA |= (1<<4)
#define PHI2_LOW PORTA &= ~(1<<4)
#define MODE_READ { PORTC = 0xFF; DDRC = 0; PORTD |= 1<<6; }
#define MODE_WRITE { PORTD &= ~(1<<6); DDRC = 0xFF; }
