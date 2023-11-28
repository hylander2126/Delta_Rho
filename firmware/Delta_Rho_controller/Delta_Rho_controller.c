#include <\\research.wpi.edu\srl\Projects\Ant\Delta_Rho\Code\Libraries\DeltaRho.h>
// #include "Libraries/DeltaRho.h"

// REMEMBER TO CHANGE ROBOTid FOR EACH ROBOT
#define RobotID 3
#define Kp 10


volatile signed int xd[3] = {0,0,0};
volatile signed int x[3] = {0,0,0};
volatile signed int xO[3] = {0,0,0};
	
volatile signed int dx[3] = {0,0,0};
volatile signed int dxO[3] = {0,0,0};
		

volatile float F_R[3] = {0,0,0};


volatile int out[3] = {0,0,0};
volatile int in[3] = {0,0,0};

volatile char start = 0;

// Hyland - Adding sensor variable
volatile signed int sensor[] = {0,0,0};


volatile unsigned int t_x = 0;
volatile unsigned int t_o = 0;
volatile unsigned char n_x = 0;
volatile unsigned char n_o = 0;



void controller(void);
void f_desired_manipulation(void);
void f_desired_position(void);

// Hyland - Adding sensor read function
//void sensor_read(void);

float calculateTime( unsigned char *n, unsigned int *timerValue){
	
	float time = 0;
	unsigned int CurrentTimer;
	
	CurrentTimer = TCNT3;
	
	time =  *n*(OCR3A + 1) + CurrentTimer - *timerValue;
	
	*n = 0;
	*timerValue = CurrentTimer;
	
	time = time*0.0001024;
	
	return time;
}

void updateState(signed int* X, signed int *dX, unsigned char* n, unsigned int* timerValue){
	
	char i;
	float time;
	signed int X_old;
	unsigned char regl, regh;
	
	time = calculateTime(n, timerValue);
	
	
	for(i=0;i<3;i++){
		X_old = X[i];
		
		regl = USART1_Receive();
		regh = USART1_Receive();
		((signed int*)X)[i] = ((regh << 8) | regl);
		
		((signed int*)dX)[i] = (((int *)X[i]) - X_old);
		}
	
}

ISR(USART1_RX_vect)
{
	char i;
	char data, address;
	unsigned char requestedID, requestedCode;
	
	data = USART1_Receive();
	
	if(data == 0x55){ // "Start" byte (85)
		
		address = USART1_Receive();
		requestedID = address & 0x0F;
		requestedCode = address & 0xf0;
		
		if( requestedID == RobotID || requestedID == 0x00 ){
			PORTC &= ~BIT(redLED);
			switch ( requestedCode ){
				
				case(0x00): // Desired position -> Read request (0)
					USART1_SerialSend(xd, 3);
				break;
				
				case(0x80): // Desired position -> Write request (128)
					USART1_SerialGet(xd, 3);
				break;
				
				case(0x10): // Current position -> Read request (16)
					USART1_SerialSend(x, 3);
				break;
				
				case(0x90): // Current position -> Write request (144)
					updateState(x,dx,&n_x,&t_x);
				break;
				
				case(0x20): // Object position -> Read request (32)
					USART1_SerialSend(xO, 3);
				break;
				
				case(0xA0): // Object position -> Write request (160)
					updateState(xO,dxO,&n_o,&t_o);
				break;
				
				case(0x30): // Send "out" data (48)
					USART1_SerialSend(out, 3);
				break;
				
				case(0xB0): // Get "in" data (176)
					USART1_SerialGet(in, 3);
				break;
				
				case(0xC0): // Actuator values -> Write request (192)
					FLCCW = USART1_Receive();
					FLCW = USART1_Receive();
					RCCW = USART1_Receive();
					RCW = USART1_Receive();
					FRCCW = USART1_Receive();
					FRCW = USART1_Receive();
				break;
				
				case(0x40): // Actuator values -> Read request (64)
					// USART1_SerialSend();
				break;
				
				//////////////////////////////////////////////////////////////////////////
				// Hyland 11-16-22 - Adding sensor read request: in MATLAB, SerialCommunication.m says 208, 224, 80, 96 are reserved...

				case(0x50): // Sensor Values -> Read request (80 in decimal)
					USART1_SerialSend(sensor, 3);
				break;
				//////////////////////////////////////////////////////////////////////////
				
				case(0x70): // Reset (112)
					start = 1;
				break;
				
				case(0xF0): // Stop/Sleep (240)
				
				break;
	
			}	// End of: switch
		} // End of: if ID match	
	}	// End of: if start
	PORTC |= BIT(redLED);
}

void f_desired_manipulation(void){
	
	
	signed int e[3];
	float qr,qd,qO,cq,sq;
	float e_o[3];
	int rx = 72;
	float PD[3];
	float F[2];
	
	//====================================================================================================================================
	// Robot 1
	
	//static float J[6][3] = {{0.3079,-0.0611,-0.0031},{0.0049,0.3452,0.0006},{0.3325,-0.0020,-0.0001},{-0.0394,0.2388,-0.0047},{0.3596,0.0631,0.0032},{0.0345,0.4161,0.0041}};
	static float J_[4][3] = {{0.5,0,-0.0081},{0,0.5,-0.0014},{0.5,0,0.0081},{0,0.5,0.0014}};
	
	//////===================================================================
	////// Robot 2
	////
	//////static float J[6][3] = {{0.3079,-0.0611,-0.0031},{0.0049,0.3452,0.0006},{0.3325,-0.0020,-0.0001},{-0.0394,0.2388,-0.0047},{0.3596,0.0631,0.0032},{0.0345,0.4161,0.0041}};
	//static float J_[4][3] = {{0.5,0,-0.0005},{0,0.5,-0.005},{0.5,0,0.0005},{0,0.5,0.005}};
	////
	////===================================================================
	// Robot 3
	
	//static float J[6][3] = {{0.3079,-0.0611,-0.0031},{0.0049,0.3452,0.0006},{0.3325,-0.0020,-0.0001},{-0.0394,0.2388,-0.0047},{0.3596,0.0631,0.0032},{0.0345,0.4161,0.0041}};
	//static float J_[4][3] = {{0.5,0,0.005},{0,0.5,0.0054},{0.5,0,0.0050},{0,0.5,-0.0054}};
	
	//===================================================================================================================================
	
	qr = 0.0175*x[2];
	qd = 0.0175*xd[2];
	qO = 0.0175*xO[2];
	
	
	e[0] = xd[0] - xO[0];
	e[1] = xd[1] - xO[1];
	
	cq = cos(qO);
	sq = sin(qO);
	
	e_o[0] = e[0]*cq + e[1]*sq;
	e_o[1] = e[1]*cq - e[0]*sq;
	e_o[2] = qd - qO;
	
	PD[0] = Kp*e_o[0];
	PD[1] = Kp*e_o[1];
	PD[2] = Kp*e_o[2];
	
	
	F[0] =  J_[0][0]*PD[0] + J_[0][1]*PD[1] + J_[0][2]*PD[2];
	F[1] =  J_[1][0]*PD[0] + J_[1][1]*PD[1] + J_[1][2]*PD[2];
	
	
	cq = cos(qO - qr);
	sq = sin(qO - qr);
	
	
	F_R[0] = cq*F[0] - sq*F[1];
	F_R[1] = sq*F[0] + cq*F[1];
	F_R[2] = rx*F_R[1];	
}

void f_desired_position(void){

	signed int e[2];
	double qr,qd;
	double cqr,sqr;
	double e_r[3], de_r[3];
	
	//e[0] = xd[0] - x[0];
	//e[1] =xd[1] - x[1];
	//e[2]=xd[2]-x[2];
	//
	//qr = 0.0175*x[2];
	//qd = 0.0175*xd[2];
	//
	//cqr = cos(qr);
	//sqr = sin(qr);
	//
	//
	//e_r[0] = cqr*e[0]+sqr*e[1];
	////e_r[0] += sqr*e[1];
	//
	//e_r[1] = -sqr*e[0]+cqr*e[1];
	////e_r[1] += cqr*e[1];
	//
	//e_r[2] = xd[2] - x[2];
//
	//
	//
	//de_r[0] = -cqr*(dx[0])- sqr*(dx[1]);
	//de_r[1] = sqr*(dx[0])- cqr*(dx[1]);
	//de_r[2] = -dx[2];
	//
	//
	//F_R[0] = 10*e_r[0]; //+ 2*de_r[0];
	//F_R[1] = 10*e_r[1]; //+ 2*de_r[1];
	//F_R[2] = 10*e_r[2]; //+ 2*de_r[2];
	
	F_R[0]=in[0];
	F_R[1]=in[1];
	F_R[2]=in[2];
	
	out[0] = (signed int)(F_R[0]);
	out[1] = (signed int)(F_R[1]);
	out[2] = (signed int)(F_R[2]);	
	
}

void controller (void){
	
	
	int i;
	
	unsigned char u[3][2];
	float f[3] = {0,0,0};
	signed int temp;
		
	//0.01788*
	f[0] = -1.732*F_R[0] + F_R[1] -F_R[2];
	f[1] = -2.0*F_R[1] -F_R[2];
	f[2] =  1.732*F_R[0] + F_R[1] -F_R[2];
	
	
	for (i = 0; i < 3; i++){
		
		temp = (int)roundf(f[i]);
		
		if( temp > 255  ) {
			temp = 255;
		}
		else if(temp < -255){
			temp = -255;
		}
		
		if(temp >= 0){
			u[i][0] = 0;
			u[i][1] = abs(temp);
		}
		
		else{
			u[i][0] = abs(temp);
			u[i][1] = 0;
		}
		
		
	}
	// Front Left
	FLCW = u[0][1];
	FLCCW = u[0][0];
	// Rear
	RCCW = u[1][0];
	RCW = u[1][1];
	//Front Right
	FRCCW = u[2][0];
	FRCW = u[2][1];
	
	
}

void controller_old(void){
	int i;
	
	
	unsigned char u[3][2];
	
	
	double e[3];
	
	float f[3] = {0,0,0};
	
	
	float eq, sq, cq, C1, C2, C3, C4;
	
	
	signed int temp;

	
	
	
	for(i=0; i<3 ; i++){
		
		e[i] = (double) (xd[i] - x[i]);
	}
	
	
	
	sq = sin(0.01745*x[2]);
	
	cq = cos(0.01745*x[2]);
	
	
	eq = 0.01745*e[2];
	
	

	C1 = (sq + (1.7321*cq));
	
	C2 = (cq - (1.7321*sq));
	
	C3 = (sq - (1.7321*cq));
	
	C4 = (cq + (1.7321*sq));

	
	

	f[0] = eq - e[0]*C1 + e[1]*C2;

	f[1] = eq - 2*e[1]*cq + 2*e[0]*sq;

	f[2] = eq - e[0]*C3 + e[1]*C4;
	

	
	for (i = 0; i < 3; i++){
		
		temp = (int)roundf(f[i]);
		
		if( temp > 255  ) {
			temp = 255;
		}
		else if(temp < -255){
			temp = -255;
		}
		
		if(temp >= 0){
			u[i][0] = 0;
			u[i][1] = abs(temp);
		}
		
		else{
			u[i][0] = abs(temp);
			u[i][1] = 0;
		}
		
		
	}
	// Front Left
	OCR1BL =u[0][1];
	OCR1AL = u[0][0];
	// Rear
	OCR0A = u[1][0];
	OCR0B = u[1][1];
	//Front Right
	OCR2B = u[2][1];
	OCR2A = u[2][0];
}


ISR(TIMER3_COMPA_vect)
{
	n_x++;
	n_o++;
	
	//f_desired_position();
	//controller();
}


//////////////////////////////////////////////////////////////////////////
///     Sensor Read Function
//////////////////////////////////////////////////////////////////////////
void update_sensor(int *array)
{
	signed int a = ADC_read(0);
	signed int b = ADC_read(3);

	array[0] = a;
	array[1] = b;
}

//================================================================================================
//                                          Main
//================================================================================================
int main(void){

	int i;
	float time;
	
	
	mC_init();
	
	
	PORTC &= ~BIT(redLED);

	PORTC |= BIT(blueLED);
	PORTC |= BIT(redLED);
	
	n_x = 15;
	t_x = 22;
	
	
	TCCR3B = (0<<CS32) | (0<<CS31) | (0<<CS30);
	sei();
	while(start == 0){
		update_sensor(sensor);
		PORTC  ^= BIT(blueLED);
		_delay_ms(100); // Changed from 100 to test which loop is running 12/5/22
	}
	PORTC |= BIT(blueLED);
	TCCR3B = (1<<CS32) | (0<<CS31) | (1<<CS30);
	
	while (1){
		// controller_old();
		//f_desired_position();
		//controller();

		PORTC ^= BIT(blueLED);
		_delay_ms(500); // Changed from 500 07/31/23
	}

	return 0;
}