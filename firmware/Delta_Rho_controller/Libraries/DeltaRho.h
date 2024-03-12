#define F_CPU 20000000
#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <avr/sleep.h>

#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


//#include <\\research.wpi.edu\srl\Projects\Ant\Delta_Rho\Code\Libraries\USART0.h>
//#include <\\research.wpi.edu\srl\Projects\Ant\Delta_Rho\Code\Libraries\USART1.h>
//#include <\\research.wpi.edu\srl\Projects\Ant\Delta_Rho\Code\Libraries\ADC.h>
#include <ADC.h>
#include <USART0.h>
#include <USART1.h>


#define BIT(x)		(1<<(x))

#define blueLED		6
#define redLED		7
#define FRCW		OCR2B
#define FRCCW		OCR2A
#define FLCW		OCR1BL
#define FLCCW		OCR1AL
#define RCW			OCR0B
#define RCCW		OCR0A


//==============================================================================================
//                              Micro-Controller Initializations
//==============================================================================================
void mC_init(void)
{

	
	// Crystal Oscillator division factor: 1
	//#pragma optsize-
	CLKPR=(1<<CLKPCE);
	CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
	//#ifdef _OPTIMIZE_SIZE_
	//#pragma optsize+
	//#endif
	
	
	// Input/Output Ports initialization
	// Port A initialization
	DDRA= 0x00;
	PORTA= 0x00;

	// Port B initialization
	DDRB= 0xff;
	PORTB= 0x00;

	// Port C initialization
	DDRC=0xF0;
	PORTC=0x00;

	// Port D initialization
	DDRD=0xFB;
	PORTD=0x00;
	
	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: 2500.000 kHz
	// Mode: Phase correct PWM top=0xFF
	// OC0A output: Non-Inverted PWM
	// OC0B output: Non-Inverted PWM
	// Timer Period: 0.204 ms
	// Output Pulse(s):
	// OC0A Period: 0.204 ms Width: 0 us
	// OC0B Period: 0.204 ms Width: 0 us
	TCCR0A=(1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (1<<WGM00);
	TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
	TCNT0=0x00;
	OCR0A=0x00;
	OCR0B=0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 2500.000 kHz
	// Mode: Ph. correct PWM top=0x00FF
	// OC1A output: Non-Inverted PWM
	// OC1B output: Non-Inverted PWM
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 0.204 ms
	// Output Pulse(s):
	// OC1A Period: 0.204 ms Width: 0 us
	// OC1B Period: 0.204 ms Width: 0 us
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (1<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: 2500.000 kHz
	// Mode: Phase correct PWM top=0xFF
	// OC2A output: Non-Inverted PWM
	// OC2B output: Non-Inverted PWM
	// Timer Period: 0.204 ms
	// Output Pulse(s):
	// OC2A Period: 0.204 ms Width: 0 us
	// OC2B Period: 0.204 ms Width: 0 us
	ASSR=(0<<EXCLK) | (0<<AS2);
	TCCR2A=(1<<COM2A1) | (0<<COM2A0) | (1<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (1<<WGM20);
	TCCR2B=(0<<WGM22) | (0<<CS22) | (1<<CS21) | (0<<CS20);
	TCNT2=0x00;
	OCR2A=0x00;
	OCR2B=0x00;


	// Timer/Counter 3 initialization
	// Clock source: System Clock
	// Clock value: Timer3 Stopped
	// Mode: CTC top=OCR3A
	// OC3A output: Disconnected
	// OC3B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer3 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: On
	// Compare B Match Interrupt: Off
	TCCR3A=(0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<WGM31) | (0<<WGM30);
	TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (1<<WGM32) | (1<<CS32) | (0<<CS31) | (1<<CS30);
	TCNT3H=0x00;
	TCNT3L=0x00;
	ICR3H=0x00;
	ICR3L=0x00;
	OCR3AH=0x00;
	OCR3AL=0x63;
	OCR3BH=0x00;
	OCR3BL=0x00;

	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

	// Timer/Counter 3 Interrupt(s) initialization
	TIMSK3=(0<<ICIE3) | (0<<OCIE3B) | (1<<OCIE3A) | (0<<TOIE3);
	
	
	
	
	// Initializing ADC
	// ADC:					Enabled
	// Analog Inputs:		0, 3
	// Reference Voltage:	AVCC
	// Clock Prescaler:		128
	// Data:				Right adjusted
	// AutoTrigger:			Off
	// AutoTrigger Source:	NA
	// Interrupt:			Disabled
	//ADC_Initialize ( 1, 0x09, 1 , 7, 0, 0, 0, 0);
	
	// Baud Rate = 9600
	// Asynchronous
	// 8 bit data
	// 1 Stop bit
	// Parity disabled
	// Enable RX Interrupt
	USART1_Initialize(57600, 0, 8, 1, 0);
	UCSR1B |= (1 << RXCIE1);
	

}