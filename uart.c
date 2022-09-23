#define	F_CPU 8000000UL

#include <avr/io.h>
#include <stdio.h>

int uart0_putchar(char ch, FILE *stream);
int uart0_getchar(FILE *stream);

FILE uart0_dev = FDEV_SETUP_STREAM(uart0_putchar, uart0_getchar, _FDEV_SETUP_RW);

void uart0_init(uint32_t baudrate)
{
	UCSR0A = 0b00000000; // U2X0=0: No double speed
	UCSR0B = 0b00011000; // Enable Tx, 8 Data bits
	UCSR0C = 0b00000110; // Async mode, No Parity, 1 Stop bit, 8 Data bits
	
	UBRR0 = F_CPU/(baudrate*16UL)-1; // Baud Rate
	
	stdout = &uart0_dev;
	stdin  = &uart0_dev;
}

int uart0_putchar(char ch, FILE *stream)
{
	while (!(UCSR0A & (1 << UDRE0)));	// Wait until Tx Data Register Empty	
	UDR0 = ch;							// Write a character(ch) to be transmitted
	
	return 0;							
}

int uart0_getchar(FILE *stream)
{
	while ((UCSR0A & (1 << 7)) == 0);
	
	return UDR0;
}
