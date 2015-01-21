#define F_CPU 8000000UL
#define UART_BAUD 250000UL
#define RECV_BUFFER 1050
#define SEND_DELAY 10

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
