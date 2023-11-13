#include <math.h>
#include <avr/io.h>


void USART1_Initialize ( long int BAUD_Rate, char Mode, char DataSize, char StopBits, char Parity )
{
		
	
	// Compute the value for ubrr register
	uint16_t ubrr = lrint( F_CPU/(16L*BAUD_Rate) ) - 1;
	
	// Set baud rate
	UBRR1H = (unsigned char) (ubrr >> 8);
	UBRR1L = (unsigned char) ubrr;
	
	
	
	// Set the Mode of operation
	switch ( Mode )
	{
		case (0): break;							// Asynchronous USART (Normal)
		case (1): UCSR1A |= (1 << U2X1); break;		// Asynchronous USART (Double Transmission Speed)  
		case (2): UCSR1C |= (1 << UMSEL10); break;  // Synchronous USART (Transmitted Data Changed = Rising XCKn Edge --- Received Data Sampled = Falling XCKn Edge) 
		case (3): UCSR1C |= (1 << UMSEL10);		    // Synchronous USART (Transmitted Data Changed = Falling XCKn Edge --- Received Data Sampled =  Rising XCKn Edge) 
				  UCSR1C |= (1 << UCPOL1); break;
		case (4): UCSR1C |= (3 << UMSEL10); break;	// Master SPI (MSPIM)
		default: break;								// Asynchronous USART
	}

	// Enable the receiver and transmitter
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	
	// Set number of Stop Bits
	if( StopBits == 2 ) UCSR1C |= (1 << USBS1);		// Default is 1, this will change it to two
	
	// Set data length
	switch ( DataSize )
	{
		case (6): UCSR1C |= (1 << UCSZ10); break;						  // 6-bit data length
		case (7): UCSR1C |= (2 << UCSZ10); break;						  // 7-bit data length
		case (8): UCSR1C |= (3 << UCSZ10); break;						  // 8-bit data length
		case (9): UCSR1C |= (3 << UCSZ10); UCSR1B |= (1 << UCSZ12); break; // 9-bit data length
		default:  UCSR1C |= (3 << UCSZ10); break;						  // 8-bit data length
	}
	
	// Set Parity
	switch ( Parity )
	{
		case (0): break;							// Disabled
		case (1): UCSR1C |= (3 << UPM10); break;	// Enabled, Odd Parity
		case (2): UCSR1C |= (1 << UPM11); break;	// Enabled, Even Parity
		default: break;								// Disabled 
	}
	
}


void USART1_EnableInterrupts (char RxInterrupt, char TxInterrupt)
{
	if ( RxInterrupt == 1 ) UCSR1B |= (1 << RXCIE1); 	// Enable RX Complete Interrupt
	else UCSR1B &= ~(1 << RXCIE1);						// Disable RX Complete Interrupt
	
	if( TxInterrupt == 1 ) UCSR1B |= (1 << TXCIE1);		// Enable TX Complete Interrupt
	else UCSR1B &= ~(1 << TXCIE1);						// Disable TX Complete Interrupt
}

void USART1_Transmit ( unsigned char data )
{
	// Wait for empty transmit buffer
	while( !(UCSR1A & (1 << UDRE1)) ) {}
	
	// Put data into buffer, send the data
	UDR1 = data;
}

void USART1_Transmit_9bit ( unsigned int data )
{
	// Wait for empty transmit buffer
	while( !(UCSR1A & (1 << UDRE1)) ) {}
	
	// Copy 9th bit to TXB80
	UCSR1B &= ~(1 << TXB81);
	if( data & 0x0100 )
		UCSR1B |= ( 1<<TXB81 );
	// Put data into buffer, send the data
	UDR1 = data;
}


unsigned char USART1_Receive (void)
{
	// Wait for data to be received
	while( !(UCSR1A & (1<<RXC1)) ) {}
	
	// Get and return received data from buffer
	return UDR1;
		
}

unsigned int USART1_Receive_9bit (void)
{
	unsigned char status, resh, resl;
	
	// Wait for data to be received
	while( !(UCSR1A & (1<<RXC1)) ) {}
	
	
	// Get status and 9th bit, then data from buffer
	status =  UCSR1A;
	resh = UCSR1B;
	resl = UDR1;
	
	// If error, return -1
	if( status & (1<<FE1)|(1<<DOR1)|(1<<UPE1) ) 
		return -1;
		
	// Filter the 9th bit, then return
	resh = (resh >> 1) & 0x01;
	return ((resh << 8) | resl);
	
}


void USART1_Flush ( void )
{
	unsigned char dummy;
	
	while ( UCSR1A & (1<<RXC1) ) dummy = UDR1;

}


void USART1_Transmit_int16 ( int data )
{

	USART1_Transmit ( data );
	USART1_Transmit ( data >> 8);
	
	 
}

void USART1_SerialSend(void* ptr, char len)
{
	int i;
	for(i=0;i<len*2;i++){
		USART1_Transmit(((char*)ptr)[i]);
	}
}


void USART1_SerialGet(void* ptr, char len)
{
	int i;
	unsigned char regl, regh;
	
	for(i=0;i<len;i++){
		regl = USART1_Receive();
		regh = USART1_Receive();
		((signed int*)ptr)[i] = ((regh << 8) | regl);
	}
}