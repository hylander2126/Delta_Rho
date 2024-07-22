#include "DeltaRho.h"
//#include "I2CSlave.h"
#include <util/twi.h>
#include "TWI_slave.h"

// REMEMBER TO CHANGE ROBOTid FOR EACH ROBOT
#define RobotID				4
#define I2C_ADDR			0x08 // I2C Slave Address
#define PI					3.14159265358979323846
// Enable Power Management (see main loop for execution)
#define POWER_MANAGEMENT_ENABLED

// Testing serial monitor
#define BAUD 115200
#define MYUBRR F_CPU/16/BAUD-1

// Handle TWI (i2c) error
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg );
// I2C Byte Variable
volatile uint8_t receivedI2C = 0x02; // Global var to store latest received i2c data
unsigned char messageBuf[TWI_BUFFER_SIZE];

// Old, potentially unused variables
volatile signed int xd[3] = {0,0,0};
volatile signed int x[3] = {0,0,0};
volatile signed int xO[3] = {0,0,0};
volatile int out[3] = {0,0,0};
volatile int in[3] = {0,0,0};
volatile char start = 0;
volatile unsigned char n_x = 0;
volatile unsigned char n_o = 0;
volatile uint32_t overflowCount = 0; // Hyland added/modified this

// For sending data via XBee
volatile int send_data[3] = {0, 0, 0};
	
// Timer function
double t = 0;
double t_run = 0;


// =======================================================================
// =======================     INTERRUPTS     ============================
// =======================================================================
ISR(USART1_RX_vect)
{
	//char i;
	char data, address;
	unsigned char requestedID, requestedCode;
	
	data = USART1_Receive();
	
	if(data == 0x55){ // Check for "Start" byte (85)
		
		address = USART1_Receive();
		requestedID = address & 0x0F;
		requestedCode = address & 0xf0;
		
		if( requestedID == RobotID || requestedID == 0x00 ){ // Check for correct robot ID
			PORTC &= ~BIT(redLED);
			switch ( requestedCode ){
				
				// NOTE: USART1_Receive() grabs each byte sequentially and 'consumes' it from the buffer. So the next time it 
				// is called, the next byte is retrieved ...
				
				case(0x00): // Desired position -> Read request (0)
					USART1_SerialSend(xd, 3);
				break;
				
				case(0x80): // Desired position -> Write request (128)
					USART1_SerialGet(xd, 3);
				break;
				
				case(0x10): // Current position -> Read request (16)
					USART1_SerialSend(x, 3);
				break;
				
				//case(0x90): // Current position -> Write request (144)
					//updateState(x,dx,&n_x,&t_x);
				//break;
				
				case(0x20): // Object position -> Read request (32)
					USART1_SerialSend(xO, 3);
				break;
				
				//case(0xA0): // Object position -> Write request (160)
					//updateState(xO,dxO,&n_o,&t_o);
				//break;
				
				case(0x30): // Send "out" data (48)
					USART1_SerialSend(out, 3);
				break;
				
				case(0xB0): // Get "in" data (176)
					USART1_SerialGet(in, 3);
				break;
				
				case(0x40): // Actuator values -> Read request (64)
				// USART1_SerialSend();
				break;
				
				case(0xC0): // Actuator values -> Write request (192)
					FLCCW = USART1_Receive();
					FLCW = USART1_Receive();
					RCCW = USART1_Receive();
					RCW = USART1_Receive();
					FRCCW = USART1_Receive();
					FRCW = USART1_Receive();
				break;
				
				
				//////////////////////////////////////////////////////////////////////////
				// Hyland 11-16-22 - Adding sensor read request: SerialCommunication.m says 208, 224, 80, 96 are reserved...

				case(0x50): // Sensor Values -> Read request (80 in decimal)
				// SEND BASE ANGLES (degrees)
					//send_data[0] = (int) (raw_sensor_data.x * 100); // * 180/3.1415);
					//send_data[1] = (int) (raw_sensor_data.y * 100); // * 180/3.1415);
				// SEND EE POSITION (mm)
					//send_data[0] = (int) (EE[0] * 100);
					//send_data[1] = (int) (EE[1] * 100);
				// SEND STATE ESTIMATE (mm)
					//send_data[0] = (int) (curr_X[0]); // * 100000);
					//send_data[1] = (int) (curr_X[1]); // * 100000);
					//send_data[2] = (int) (curr_X[2]); // * 100000);
				// SEND u_r motor command
					//send_data[0] = (int) (u_r[1] * 100);
					//send_data[1] = (int) (u_r[2] * 100);
					//send_data[2] = (int) (u_r[0] * 100);
				// SEND CoM control output
					//send_data[0] = (int) (output1 * 100);
					//send_data[1] = (int) (output2 * 100);
				// SEND I2C DATA from esp-cam
					send_data[0] = (int) receivedI2C;
					send_data[1] = (int) receivedI2C;
					
					
					USART1_SerialSend(send_data, 3);					
				break;
				
				
				case(0xD0): // Switch actuation mode -> Write request (208 in decimal)
					//mode_switch	= !mode_switch; // Toggle from zero-force mode to driving mode.
				
				break;
				//////////////////////////////////////////////////////////////////////////
				
				case(0x70): // Reset (112)
					start = 1;
				break;
				
				case(0xF0): // Stop/Sleep (240)
				
				break;
	
			}	// End of: switch
		}		// End of: if ID match	
	}			// End of: if start
	PORTC |= BIT(redLED);
}

// =======================================================================
// =======================     I2C SETUP     =============================
// =======================================================================

//		Receive I2C Data
//========================================================================
void I2C_received(uint8_t received_data) {
	receivedI2C = received_data;
	PORTC ^= BIT(blueLED);	// Toggle blueLED
}

//		Receive I2C Data
//========================================================================
void I2C_requested() {
	I2C_transmitByte(receivedI2C);
}

//		I2C Error Handling Function
//========================================================================
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.
	
	// This very simple example puts the error code on PORTB and restarts the transceiver with
	// all the same data in the transmission buffers.
	receivedI2C = TWIerrorMsg;
	TWI_Start_Transceiver();
	
	return TWIerrorMsg;
}

//================================================================================================
//                                 Serial Monitor Functions
//================================================================================================
void USART_Init(unsigned int ubrr) {
	UBRR1H = (unsigned char)(ubrr >> 8);  // Set baud rate
	UBRR1L = (unsigned char)ubrr;
	UCSR1B = (1 << RXEN1) | (1 << TXEN1); // Enable receiver and transmitter
	UCSR1C = (3 << UCSZ10); // Set frame format: 8 data bits, no parity, 1 stop bit
}

void USART_Transmit(char data) {
	while (!(UCSR1A & (1 << UDRE1))); // Wait for empty transmit buffer
	UDR1 = data; // Put data into buffer, sends the data
}

void printString(const char *s) {
	while (*s) {
		USART_Transmit(*s++);
	}
}


//================================================================================================
//                                          Main
//================================================================================================
int main(void){
		
	// Initialize micro controller
	mC_init();
	
	// Initialize USART for serial monitoring
	USART_Init(MYUBRR);
	printString("Hello\n");
	
	// Initialize TWI module for slave operation. Include address and/or enable General Call
	TWI_Slave_Initialise((unsigned char)((0x08<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) )); // TRUE
	
	sei();					// Enable global interrupts
	// Start the TWI transceiver to enable reception of the first command from TWI master
	TWI_Start_Transceiver();
	
	TCNT3H = 0;
	TCNT3L = 0;
	TIMSK3 |= (1<<TOIE3);	// Enable Timer3 overflow interrupt - ChatGPT advised
	PORTC |= BIT(redLED);	// Turn ON redLED
	
	// ====== Primary Loop ======
	for(;;){
		
		// Check if the TWI Transceiver has completed an operation.
		if ( ! TWI_Transceiver_Busy() ) {
			// Check if the last operation was successful
			if ( TWI_statusReg.lastTransOK ) {
				// Check if the last operation was a reception
				if ( TWI_statusReg.RxDataInBuf ) {
					PORTC ^= BIT(blueLED);	// Toggle blueLED
					TWI_Get_Data_From_Transceiver(messageBuf, 2);
					receivedI2C = messageBuf[0];
				}
				else { // Ends up here if the last operation was a transmission
					asm("nop");   // Put own code here.
					TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info() );
				}
				// Check if the TWI Transceiver has already been started.
				// If not then restart it to prepare it for new receptions.
				if ( ! TWI_Transceiver_Busy() ) {
					TWI_Start_Transceiver();
				}
			}
			else { // Ends up here if the last operation completed unsuccessfully
				//TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info() );
				int tempt = 1;
			}
		}
		
		PORTC ^= BIT(blueLED);	// Toggle blueLED
		_delay_ms(100);		// Changed from 100 to test which loop is running 11/28/23
	}
	
	return 0;
}