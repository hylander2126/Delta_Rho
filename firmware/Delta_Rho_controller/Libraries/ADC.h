#include <avr/io.h>



void ADC_Initialize ( char Enable, char ADCInputs, char VoltageRefernce , char PrescalerSelect, char LeftAdjust, char AutoTrigger, char AutoTriggerSource, char InterruptEnable)
{
	
	ADMUX = 0x00;
	ADCSRA = 0x00;
	ADCSRB = 0x00;
	DIDR0 = 0x00;
	
	// Enable ADC
	ADCSRA |= (Enable << ADEN);			
	
	// Disable digital inputs on ADC pins
	DIDR0 = ADCInputs;
	
	// Voltage Reference
	//	0 -> AREF, Internal Vref turned off
	//  1 -> AVCC with external capacitor at AREF pin
	//  2 -> Internal 1.1V Voltage Reference with external capacitor at AREF pin
	//  3 -> Internal 2.56V Voltage Reference with external capacitor at AREF pin
	ADMUX |= (VoltageRefernce << REFS0);
	
	// ADC Left Adjust Result
	//	0 -> Right adjust the result
	//	1 -> Left adjust the result
	ADMUX |= (LeftAdjust << ADLAR);
	
	// ADC Pre-scaler Select Bits
	//	0 -> Division Factor = 2
	//	1 -> Division Factor = 2
	//  2 -> Division Factor = 4
	//  3 -> Division Factor = 8
	//  4 -> Division Factor = 16
	//  5 -> Division Factor = 32
	//  6 -> Division Factor = 64
	//  7 -> Division Factor = 128
	ADCSRA |= PrescalerSelect;
	
	
	// ADC Auto Trigger
	//	0 -> Off
	//	1 -> On
	ADCSRA |= (AutoTrigger << ADATE);
	
	// ADC Auto Trigger Source
	//	0 -> Free Running mode
	//	1 -> Analog Comparator
	//	2 -> External Interrupt Request 0
	//	3 -> Timer/Counter0 Compare Match
	//	4 -> Timer/Counter0 Overflow
	//	5 -> Timer/Counter1 Compare Match B
	//	6 -> Timer/Counter1 Overflow
	//	7 -> Timer/Counter1 Capture Event
	ADCSRB |= AutoTriggerSource;
	
	
	// ADC InterruptEnable
	//	0 -> Disable
	//	1 -> Enable
	ADCSRA |= (InterruptEnable << ADIE);
	
	
}

int ADC_read (char Channel)
{
	
	unsigned char resh, resl;
	
	ADMUX &= 0xE0;				// Set analog channel and gain selection bits to zero
	ADMUX |= Channel;			// Set MUX to the requested channel
	
	ADCSRA |= (1 << ADSC);		// Start conversion
	
	while ((ADCSRA & (1<<ADIF))==0);

	ADCSRA|=(1<<ADIF);
	
	
	resl = ADCL;
	resh = ADCH;
	
	return ((resh << 8) | resl);
	
}