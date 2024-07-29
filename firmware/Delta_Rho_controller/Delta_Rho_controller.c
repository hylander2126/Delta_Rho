//#include <\\research.wpi.edu\srl\Projects\Ant\Delta_Rho\Code\Libraries\DeltaRho.h>
#include "DeltaRho.h"
#include "config.h"

#include <math.h>
#include <time.h>
#include <util/twi.h>
#include "TWI_slave.h"
#include "avr/sleep.h"

// ================= IMPORTANT: CHANGE RobotID =================
#define RobotID				6
// =============================================================
#define I2C_ADDR			0x08 // I2C Slave Address *** Pin 1 is SCL***
#define POWER_MANAGEMENT_ENABLED // Enable power management for TWI


// Legacy variables
volatile signed int xd[3], x[3], xO[3]; // = {0,0,0};
volatile int out[3], in[3]; // = {0,0,0};
volatile char start = 0;

// Global variable for whatevs
float daGlobal;
int mode_switch = 0;							// Mode Toggle (binary for now)
volatile int send_data[3] = {0, 0, 0};			// For sending data wirelessly via XBee
	
// I2C & TWI setup
volatile int i2c_data = 180;					// Global var to store latest received i2c data: 0x02 default value
unsigned char message_buf[TWI_BUFFER_SIZE];
int payl_orient_0 = 180;						// Initial payload orientation for attitude control (camera is inverted currently)

// Robot Jacobian
float J_r[3][3] = {{0.8192, 0.5736, -0.1621}, {0.0, -1.0, -1.36}, {-0.8192, 0.5736, -0.1621}}; // Default value
float J[3][3];

float u_r[3] = {0, 0, 0};						// Control Input
	
	
// --- FORWARD DECLARATIONS ---
float medianFilter(float arr[], int n);
Vector2D rotateVector(Vector2D v_in, float angle);
unsigned char TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg);



// ===========================================================================
// =======================     INTERRUPT     =================================
// ===========================================================================
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
					
					thisVoltage[0] = USART1_Receive();
					thisVoltage[1] = USART1_Receive();
					thisVoltage[2] = USART1_Receive();
					thisVoltage[3] = USART1_Receive();
					thisVoltage[4] = USART1_Receive();
					thisVoltage[5] = USART1_Receive();					
					applyPulse();
					
					// Apply amplified (or non-amplified) voltages
					FLCCW	= tempVoltage[0];
					FLCW	= tempVoltage[1];
					RCCW	= tempVoltage[2];
					RCW		= tempVoltage[3];
					FRCCW	= tempVoltage[4];
					FRCW	= tempVoltage[5];
					
					//FLCCW = USART1_Receive();
					//FLCW = USART1_Receive();
					//RCCW = USART1_Receive();
					//RCW = USART1_Receive();
					//FRCCW = USART1_Receive();
					//FRCW = USART1_Receive();
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
					//send_data[0] = (int) (u_r[0] * 100);
					//send_data[1] = (int) (u_r[1] * 100);
					//send_data[2] = (int) (u_r[2] * 100);
				
				// SEND CoM control output
				
				// SEND I2C DATA from esp-cam
					//send_data[0] = i2c_data;
					//send_data[1] = i2c_data;
				// SEND WHATEVER GLOBAL DATA YOU WANT YO
					send_data[0] = daGlobal;
					send_data[1] = daGlobal;
					
					
					USART1_SerialSend(send_data, 3);					
				break;
				
				
				case(0xD0): // Switch actuation mode -> Write request (208 in decimal)
					mode_switch	= !mode_switch; // Toggle from zero-force mode to driving mode.
				
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


// ===========================================================================
// =====================     I2C & MOTOR COMMS     ===========================
// ===========================================================================

//		I2C Receive from Master
//========================================================================
void runTWI (void){
	
	if ( ! TWI_Transceiver_Busy() ) {			// Check if transceiver has completed operation
		if ( TWI_statusReg.lastTransOK ) {		// Check if last operation was success
			if ( TWI_statusReg.RxDataInBuf ) {	// Check if last operation was reception
				TWI_Get_Data_From_Transceiver(message_buf, 4);
				message_buf[4] = '\0';			// Null-terminate the string
				i2c_data = atoi((char*)message_buf); // TODO: Unwrap to -180 to +180
			}
			
			if ( ! TWI_Transceiver_Busy() ) {	// Check if transceiver already started
				TWI_Start_Transceiver();		// If not, restart it to prepare for new receptions
			}
		}
		else {									// Ends up here if last operation unsuccessful
			TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info() );
		}
	}
}

//		Send current signal to motors
//========================================================================
void sendMotor(void){
	// Send u_r after converting to motor currents
	unsigned char u[3][2];
	float f[3] = {0,0,0};
	signed int temp;
	
	// Update Jacobian given new EE position (actual attachment to payload)
	calculateJacobian(EE[0], EE[1], J);
	
	// Perform the matrix multiplication J_r * u_r
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			f[i] += J[i][j] * u_r[j]; // NEVERMIND --> * 100; // 3-14-24 ADDED *100 WITH NEW JACOBIAN WHICH REQUIRES HIGHER GAINS
		}
	}
	
	// Assign to wheel array
	for (int i = 0; i < 3; i++){
		temp = (int)roundf(f[i]);
		if(temp > 255){
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
	
	// Apply stiction pulse if needed
	thisVoltage[0] = u[0][0];
	thisVoltage[1] = u[0][1];
	thisVoltage[2] = u[1][0];
	thisVoltage[3] = u[1][1];
	thisVoltage[4] = u[2][0];
	thisVoltage[5] = u[2][1];
	//applyPulse();
	
	//// Front Left
	//FLCCW = tempVoltage[0];
	//FLCW = tempVoltage[1];
	//// Rear
	//RCCW = tempVoltage[2];
	//RCW = tempVoltage[3];
	////Front Right
	//FRCCW = tempVoltage[4];
	//FRCW = tempVoltage[5];
	
	
	// Checking to see if this is messing me up
	
	// Front Left
	FLCCW = u[0][0];
	FLCW = u[0][1];
	// Rear
	RCCW = u[1][0];
	RCW = u[1][1];
	//Front Right
	FRCCW = u[2][0];
	FRCW = u[2][1];
}


// ===========================================================================
// ======================     ALGORITHM FUNCTIONS     ========================
// ===========================================================================

//		Cancel out force measured by sensor
//========================================================================
void nullSpaceControl (void) {
	
	// u_r[0] is X motion (Forw+/Backw-)
	// u_r[1] is Y motion (Right+/Left-)
	// u_r[2] is Z motion (CCW+/CW-)
		
	float gainX; // gain in ROBOT's X direction
	float gainY; // gain in ROBOT's Y direction
	
	gainY = 20;
	gainX = gainY;
	// Sensor displacement 'away' from robot has dampened so increase the 'fwd/back' gain
	if (EE[1] > 3) {
		gainX += 10;
	}

	// Sensor 'x' is robot 'y' and vice-versa
	u_r[0] = EE[1]*gainX;
	u_r[1] = EE[0]*gainY;
	u_r[2] = 0;
	
	sendMotor();
}

//		Correct the attitude of the robot using force sensor
//========================================================================
void correctAttitude (PIDController *pid_attitude) {

	// u_r[0] is Z motion (CCW+/CW-)
	// u_r[1] is X motion (Fwd+/Bkwd-)
	// u_r[2] is Y motion (Right+/Left-)

	// ----- PID Controller -----
	float error = pid_attitude->setPoint - i2c_data;
	pid_attitude->integral += error;
	float derivative = (error - pid_attitude->prevError);
	float output = pid_attitude->Kp * error + pid_attitude->Ki * pid_attitude->integral + pid_attitude->Kd * derivative;
	pid_attitude->prevError = error;

	u_r[0] = output;
	daGlobal = error;
}

//		CoM estimate VERSION 2 - Use angle method
//========================================================================
void estimateCoM (PIDController *pid_com_angle) {
	// Now move IN the direction of force (force amplification)
	// u_r[0] is Z motion (CCW+/CW-)
	// u_r[1] is X motion (Fwd+/Bkwd-)
	// u_r[2] is Y motion (Right+/Left-)
	
	Vector2D sensor_direc	= {-EE[0], -EE[1]}; // Opposite of sensor's reading. Flip direction of the EE vector
	Vector2D robot_heading	= {u_r[1], u_r[2]}; // Robot's command input TODO update this with ACTUAL robot heading from IMU or mocap or encoders
	Vector2D ref_axis		= {1, 0};			// +x axis for now
	
	// Catch when force sensor 'thinks' 
	
	// Catch when force sensor is 'near zero' and sets u_r to 0:
	float mag_robot_heading = sqrt(pow(robot_heading.x, 2) + pow(robot_heading.y, 2));
	if (abs(mag_robot_heading) < 0.5) {
		robot_heading.x = 0;
		robot_heading.y = 1; }
	
	// ----- PID Controller -----
	// Error is angle between heading and force measurement
	float e_1 = getAngle(ref_axis, robot_heading);
	float e_2 = getAngle(ref_axis, sensor_direc);
	float error = e_2 - e_1; // do i want to also account for the AMOUNT of displacement of sensor EE?
	
	// Integral Term
	pid_com_angle->integral  += error;
	// Derivative Term
	float derivative = (error - pid_com_angle->prevError);
	// PID Output
	float output1  =  pid_com_angle->Kp  * error  +   pid_com_angle->Ki  * pid_com_angle->integral   +   pid_com_angle->Kd  * derivative;
	// Update prev error to current error
	pid_com_angle->prevError  = error;
	
	Vector2D rotated_output = rotateVector(robot_heading, output1);
	
	// Assign to control input array
	float mag_sensor_direc  = sqrt(pow(sensor_direc.x, 2) + pow(sensor_direc.y, 2));
	if (abs(mag_sensor_direc) < -9) {
		u_r[1] = 0;
		u_r[2] = 0;
	}
	else {
		u_r[1] = 0; // rotated_output.x;
		u_r[2] = 0; // rotated_output.y;
	}
	
}



//================================================================================================
//************************************       MAIN       ******************************************
//================================================================================================
int main(void){
	mC_init();					// Initialize micro controller

	// Initialize TWI module for slave operation. Include address and/or enable General Call
	TWI_Slave_Initialise((unsigned char)((0x08<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) )); // TRUE
	sei();						// Enable global interrupts
	TWI_Start_Transceiver();	// Enable reception of first command from master
		
	// Start timer3 (unused?)
	TCNT3H = 0;
	TCNT3L = 0;
	TIMSK3 |= (1<<TOIE3);		// Enable Timer3 overflow interrupt - ChatGPT advised
	PORTC |= BIT(redLED);		// Turn ON redLED
	
	// PID INITIALIZATIONS (Kp, Ki, Kd, setPoint)
	PIDController pid_com, pid_att;
	PID_Init(&pid_com, 1, 0, 0, 0);
	PID_Init(&pid_att, 1.4, 0.14, 0.2, 180); // , .005, .004); // 1.4, 0, 0 works perfectly for robot 6
		
	getHome();					// Get force sensor home position and assign to 'home'
	int iii = 0;				// iterator



	// ========== PRIMARY LOOP ==========
	while(start == 0){
		
		runTWI();				// Run all i2c comms
							
		sensorKinematics();		// Force feedback, assigned to EE var
		
		if (mode_switch) {		// Defaults to FALSE. Switch mode via MATLAB
			while (iii < 1000){ // Induce motion in payload for CoM estimate
				iii ++;
				u_r[2] = 2500;
				sendMotor();
			}

			//nullSpaceControl();			// Cancel out force detected by sensor
			estimateCoM(&pid_com);		// CoM Estimate
			correctAttitude(&pid_att);	// Perform attitude correction
			//u_r[1] = 0;
			//u_r[2] = 0;
			
			sendMotor(); // Send all motor commands
		}	
		else {
			// When not in on-board mode, send initial 'stop' command
			iii = 0;
			if (iii == 0) {
				u_r[0] = 0;
				u_r[1] = 0;
				u_r[2] = 0;
				sendMotor();
				iii ++;
			}
		}
		
		PORTC ^= BIT(blueLED);	// Toggle blueLED
		_delay_ms(100);			// Changed from 100 to test which loop is running 11/28/23
	}
	
	//PORTC |= BIT(blueLED);		// Turn ON blueLED
	TCCR3B = (1<<CS32) | (0<<CS31) | (1<<CS30);
		
	return 0;
}
