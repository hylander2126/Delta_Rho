#include <math.h>
#include <avr/io.h>


void USART0_Initialize ( long int BAUD_Rate, char Mode, char DataSize, char StopBits, char Parity )
{
		
	
	// Compute the value for ubrr register
	uint16_t ubrr = lrint( F_CPU/(16L*BAUD_Rate) ) - 1;
	
	// Set baud rate
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;
	
	//UBRR0H = 0x00;
	//UBRR0L = 0x0A;
	
	
	// Set the Mode of operation
	switch ( Mode )
	{
		case (0): break;							// Asynchronous USART (Normal)
		case (1): UCSR0A |= (1 << U2X0); break;		// Asynchronous USART (Double Transmission Speed)  
		case (2): UCSR0C |= (1 << UMSEL00); break;  // Synchronous USART (Transmitted Data Changed = Rising XCKn Edge --- Received Data Sampled = Falling XCKn Edge) 
		case (3): UCSR0C |= (1 << UMSEL00);		    // Synchronous USART (Transmitted Data Changed = Falling XCKn Edge --- Received Data Sampled =  Rising XCKn Edge) 
				  UCSR0C |= (1 << UCPOL0); break;
		case (4): UCSR0C |= (3 << UMSEL00); break;	// Master SPI (MSPIM)
		default: break;								// Asynchronous USART
	}

	// Enable the receiver and transmitter
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	
	// Set number of Stop Bits
	if( StopBits == 2 ) UCSR0C |= (1 << USBS0);		// Default is 1, this will change it to two
	
	// Set data length
	switch ( DataSize )
	{
		case (6): UCSR0C |= (1 << UCSZ00); break;						  // 6-bit data length
		case (7): UCSR0C |= (2 << UCSZ00); break;						  // 7-bit data length
		case (8): UCSR0C |= (3 << UCSZ00); break;						  // 8-bit data length
		case (9): UCSR0C |= (3 << UCSZ00); UCSR0B |= (1 << UCSZ02); break; // 9-bit data length
		default:  UCSR0C |= (3 << UCSZ00); break;						  // 8-bit data length
	}
	
	// Set Parity
	switch ( Parity )
	{
		case (0): break;							// Disabled
		case (1): UCSR0C |= (3 << UPM00); break;	// Enabled, Odd Parity
		case (2): UCSR0C |= (1 << UPM01); break;	// Enabled, Even Parity
		default: break;								// Disabled 
	}
	
}


void USART0_EnableInterrupts (char RxInterrupt, char TxInterrupt)
{
	if ( RxInterrupt == 1 ) UCSR0B |= (1 << RXCIE0); 	// Enable RX Complete Interrupt
	else UCSR0B &= ~(1 << RXCIE0);						// Disable RX Complete Interrupt
	
	if( TxInterrupt == 1 ) UCSR0B |= (1 << TXCIE0);		// Enable TX Complete Interrupt
	else UCSR0B &= ~(1 << TXCIE0);						// Disable TX Complete Interrupt
}

void USART0_Transmit ( unsigned char data )
{
	// Wait for empty transmit buffer
	while( !(UCSR0A & (1 << UDRE0)) ) {}
	
	// Put data into buffer, send the data
	UDR0 = data;
}

void USART0_Transmit_9bit ( unsigned int data )
{
	// Wait for empty transmit buffer
	while( !(UCSR0A & (1 << UDRE0)) ) {}
	
	// Copy 9th bit to TXB80
	UCSR0B &= ~(1 << TXB80);
	if( data & 0x0100 )
		UCSR0B |= ( 1<<TXB80 );
	// Put data into buffer, send the data
	UDR0 = data;
}


unsigned char USART0_Receive (void)
{
	// Wait for data to be received
	while( !(UCSR0A & (1<<RXC0)) ) {}
	
	// Get and return received data from buffer
	return UDR0;
		
}

unsigned int USART0_Receive_9bit (void)
{
	unsigned char status, resh, resl;
	
	// Wait for data to be received
	while( !(UCSR0A & (1<<RXC0)) ) {}
	
	
	// Get status and 9th bit, then data from buffer
	status =  UCSR0A;
	resh = UCSR0B;
	resl = UDR0;
	
	// If error, return -1
	if( status & (1<<FE0)|(1<<DOR0)|(1<<UPE0) ) 
		return -1;
		
	// Filter the 9th bit, then return
	resh = (resh >> 1) & 0x01;
	return ((resh << 8) | resl);
	
}


void USART0_Flush ( void )
{
	unsigned char dummy;
	
	while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;

}


void USART0_Transmit_int16 ( int data )
{
	
	USART0_Transmit ( data );
	USART0_Transmit ( data >> 8);
	
	 
}