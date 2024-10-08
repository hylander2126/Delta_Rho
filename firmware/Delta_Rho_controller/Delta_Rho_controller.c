//#include <\\research.wpi.edu\srl\Projects\Ant\Delta_Rho\Code\Libraries\DeltaRho.h>
#include "DeltaRho.h"
#include "config.h"

#include <math.h>
#include <time.h>
#include <util/twi.h>
#include "TWI_slave.h" 
#include "avr/sleep.h"

// ================= IMPORTANT: CHANGE RobotID =================
#define RobotID				5
// =============================================================
#define I2C_ADDR			0x08	// I2C Slave Address *** Pin 1 is SCL***
#define POWER_MANAGEMENT_ENABLED	// Enable power management for TWI
#define STOP_UPPER			184		// UPPER acceptable payload rotation for stop condition
#define STOP_LOWER			176		// LOWER acceptable payload rotation for stop condition


// Legacy variables
volatile char start			= 0;
volatile int out[3], in[3];
volatile signed int xd[3], x[3], xO[3];

float daGlobal;									// Global variable for whatevs
int mode_switch				= 0;				// Mode Toggle (binary for now)
volatile int send_data[3]	= {0, 0, 0};		// For sending data wirelessly via XBee
	
// I2C & TWI setup
volatile int i2c_data		= 180;				// Global var to store latest received i2c data: 0x02 default value
unsigned char message_buf[TWI_BUFFER_SIZE];

// Robot Jacobian
float J_r[3][3]				= {{0.8192, 0.5736, -0.1621}, {0.0, -1.0, -1.36}, {-0.8192, 0.5736, -0.1621}}; // Default value
float J[3][3];

// Primary Algorithm Variables
float u_r[3]				= {0, 0, 0};	// Control Input
volatile int stop_counter	= 0;			// STOP counter for Line of Action estimate (when minimal rotation observed)
volatile int new_data_rec	= 0;			// Flag for I2C reception
float u_r_mem[2]			= {0, 0};		// Memory for switching between CoM and attitude correction
int att_mem					= 0;			// Memory for attitude control and stop condition
Vector2D f_mem				= {0, 0};		// Memory for force sensor and stop condition
int oob_counter				= 0;			// Count for payload rotation out of bounds (o.o.b.)

// NEW binary search variables
Vector2D neg_bound			= {1, 0.0001};		// Negative-most possibility for LoA
Vector2D pos_bound			= {-1, 0.0001};		// Positive-most possibility for LoA
int desired_vel				= 2500;				// Desired velocity of robot
	
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
				break;
				
				
				//////////////////////////////////////////////////////////////////////////
				// Hyland 11-16-22 - Adding sensor read request: SerialCommunication.m says 208, 224, 80, 96 are reserved...

				case(0x50): // Sensor Values -> Read request (80 in decimal)
				// SEND BASE ANGLES (degrees)
					//send_data[0] = (int) (raw_sensor_data.x * 100); // * 180/3.1415);
					//send_data[1] = (int) (raw_sensor_data.y * 100); // * 180/3.1415);
				// SEND EE POSITION (mm)
					send_data[0] = (int) (EE[0] * 100);
					send_data[1] = (int) (EE[1] * 100);
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
					//send_data[0] = daGlobal;
					//send_data[1] = daGlobal;
					
					
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
	
			} // End of: switch
		} // End of: if ID match	
	} // End of: if start
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
				i2c_data = atoi((char*)message_buf);
				
				new_data_rec = 1;			// Set i2c reception flag to TRUE
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
	// u_r[1] is X motion (Right+/Left-)
	// u_r[2] is Y motion (Fwd+/Bwd-)
	
	// STOP CONDITION CHECK 
	//if (i2c_data >= STOP_LOWER && i2c_data <= STOP_UPPER){
		//PORTC ^= BIT(blueLED);	// Toggle blueLED
		//stop_counter++;}
	//else
		//stop_counter		= 0;
		//
	//// If payload in acceptable range for short while, stop correcting attitude. THIS HAS to go after the stop condition check...
	//if (stop_counter >= 6) {
		//u_r[0] = 0;
		//return;
	//}
	
	// Basically if payload rot ~unchanged wrt robot, increment stop counter, and don't bother correcting attitude
	//if (i2c_data >= att_mem-3 && i2c_data <= att_mem+3) {
		//PORTC ^= BIT(blueLED);	// Toggle blueLED
		//stop_counter++;
		//return; 
	//}
	//else { // Otherwise, reset/update the attitude memory
		//stop_counter = 0;
		//att_mem = i2c_data;
	//}
	
	
	
	// Check to see if payload out of bounds
	if (i2c_data >= 165 && i2c_data <= 195) { // If in bounds, continue CoM search STOP_LOWER and STOP_UPPER
		u_r[0] = 0;
		return;
	}
	//else {					// If out of bounds, stop CoM search and continue att.corr.
		//u_r[1] = 0;
		//u_r[2] = 0;
	//}
	
	
	// ----- PID Controller -----
	float error				= pid_attitude->setPoint - i2c_data;
	pid_attitude->integral	+= error;
	float derivative		= (error - pid_attitude->prevError);
	float output			= pid_attitude->Kp  * error   +   pid_attitude->Ki  *  pid_attitude->integral   +   pid_attitude->Kd  * derivative;
	pid_attitude->prevError = error;

	u_r[0]					= output;	// Assign z-axis motor command
	
	// Set I2C reception flag to FALSE now that the data has been utilized.
	new_data_rec			= 0;
}

//		CoM Estimate - 'Force Amplification'
//========================================================================
void estimateCoM (PIDController *pid_CoM) {
	// Move in the direction of detected force
	// u_r[0] is Z motion (CCW+/CW-)
	// u_r[1] is X motion (Right+/Left-)
	// u_r[2] is Y motion (Fwd+/Bwd-)
	
	Vector2D f_s			= {-EE[0], -EE[1]};		// Force sensor 'direction' - opposite of reading (flip direction of EE vect)
	Vector2D f_r			= {u_r_mem[0], u_r_mem[1]}; // {u_r[1], u_r[2]};		// Robot heading from input - TODO update with ACTUAL robot heading from IMU or mocap or encoders
	
	// Catch when force reading is 'away' from robot (pulling)	--> retain heading
	if (f_s.y < -3)			// in mm
		return;
	
	// Get angle of each vector w.r.t. +x axis
	float angle_f_r			= atan2f(f_r.y, f_r.x);
	float angle_f_s			= atan2f(f_s.y, f_s.x);
	
	// STOP condition check:
	//	Catch if force reading unchanged within margin. If so, increment stop counter
	float angle_f_mem		= atan2f(f_mem.y, f_mem.x);
	if (abs(angle_f_s - angle_f_mem) < 5) // FIX DEGREES
		stop_counter++;
	
	// ----- PID Controller -----
	// Error is angle between heading (f_r) and force measurement (f_s)
	float error				= angle_f_s - angle_f_r;		// TODO DO I ALSO WANT TO ACCOUNT FOR * AMOUNT * OF SENS DISPLACEMENT?
	pid_CoM->integral		+= error;
	float derivative		= (error - pid_CoM->prevError);
	float output			= pid_CoM->Kp  * error  +   pid_CoM->Ki  * pid_CoM->integral   +   pid_CoM->Kd  * derivative;
	pid_CoM->prevError		= error;
	
	// Now use the output angular *change* to rotate our robot heading by a small margin. Same velocity magnitude
	Vector2D output_rotated = rotateVector(f_r, output);
	
	// Assign to control input array
	u_r[1]					= output_rotated.x;
	u_r[2]					= output_rotated.y;
	
	// Assign motion memory to new motion command
	u_r_mem[0]				= u_r[1];
	u_r_mem[1]				= u_r[2];
	// Update f_mem
	f_mem = f_s;
}


//		LoA Search - Binary Search Algorithm
//========================================================================
void loaSearch (void) {
	// u_r[0] is Z motion (CCW+/CW-)
	// u_r[1] is X motion (Right+/Left-)
	// u_r[2] is Y motion (Fwd+/Bwd-)
	
	// Bounds bisection method. Recall bounds initialized to negBound=(1, 0.1) posBound=(-1, 0.1)
	
	// Determine payload rotation sense with force feedback
	Vector2D f_s			= {-EE[0],		-EE[1]};	 // Force sensor 'direction' - opposite of reading (flip direction of EE vect)
	Vector2D f_r			= {u_r_mem[0],	u_r_mem[1]}; // {u_r[1], u_r[2]};		// Robot heading from input - TODO update with ACTUAL robot heading from IMU or mocap or encoders
	
	// Catch when force reading is 'away' from robot (pulling)	--> retain heading
	if (f_s.y < -3)			// in mm
		return;
		
	// Get angle of force and heading vectors w.r.t. +x axis
	float angle_f_r			= atan2f(f_r.y, f_r.x);
	float angle_f_s			= atan2f(f_s.y, f_s.x);
	// Get rotation sense based on angle between motion and force feedback (-1 or 1 using RHR)
	float rot_sense			= (angle_f_s - angle_f_r) / abs(angle_f_s - angle_f_r);
	
	// Catch when angle between motion and force is minimal. Continue pushing. (Sensor uncertainty)
	if (abs(angle_f_s - angle_f_r) < 0.09) // 5 degrees
		return;
	
	// Now set the new bounds based on rotation sense (make it a unit vector)
	if (rot_sense >= 0) {
		neg_bound.x = u_r_mem[0] / sqrt(pow(u_r_mem[0], 2) + pow(u_r_mem[1], 2));
		neg_bound.y = u_r_mem[1] / sqrt(pow(u_r_mem[0], 2) + pow(u_r_mem[1], 2));
	}
	else {
		pos_bound.x = u_r_mem[0] / sqrt(pow(u_r_mem[0], 2) + pow(u_r_mem[1], 2));
		pos_bound.y = u_r_mem[1] / sqrt(pow(u_r_mem[0], 2) + pow(u_r_mem[1], 2));
	}
	
	// Get new push direction by summing bounds and making unit vector
	float output_x = pos_bound.x + neg_bound.x;
	float output_y = pos_bound.y + neg_bound.y;
	float output_norm = sqrt(pow(output_x, 2) + pow(output_y, 2));
	
	output_x *= ((desired_vel+150)/output_norm);
	output_y *= ((desired_vel+150)/output_norm);
	
	Vector2D output = {output_x, output_y};
	// Assign to control input array
	u_r[1]					= output.x;
	u_r[2]					= output.y;
	
	// Assign motion memory to new motion command
	if (u_r[0] != 0 || u_r[1] != 0) {
		u_r_mem[0]				= u_r[1];
		u_r_mem[1]				= u_r[2];
	}
	
	// Update f_mem
	f_mem = f_s;
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
	PORTC &= ~BIT(blueLED);		// Turn OFF blueLED
	
	getHome();					// Get force sensor home position and assign to 'home'
	int iii = 0;				// iterator
	int jjj = 0;				// iterator
		
	// ==== PID INITIALIZATIONS (Kp, Ki, Kd, setPoint) ====
	PIDController pid_com, pid_att;
	PID_Init(&pid_com, 1, 0, 0.08, 0); // 0.8, 0, 0.08 for 6 // 1, 0, 0 works OK but maybe too aggressive (it's immediately changing direction towards force feedback)
	PID_Init(&pid_att, 1.25, 0.05, 0.2, 180); // 1.25, 0.05, 0.2 is critically damped for robot 5


	// ========== PRIMARY LOOP ==========
	while(start == 0){
		
		runTWI();				// Run all i2c communications
		sensorKinematics();		// Force feedback, assigned to EE var
		
		if (mode_switch) {		// Defaults to FALSE. Switch mode via MATLAB
			while (iii < 1100){ // Induce motion in payload for CoM estimate
				iii ++;
				u_r[2]			= desired_vel;
				sendMotor(); 
				u_r_mem[1]		= desired_vel;	// Set initial 'memory' as 'forward' upon entering this mode. 2750 is a good speed
				att_mem			= 180;
				stop_counter	= 0;	// Reset stop counter for multiple trials w/o restarting robot
				f_mem.x			= 0;
				f_mem.y			= 1;	// Reset f_mem to 'forward'
				neg_bound.x		= 1; // Reset bounds
				neg_bound.y		= 0.0001;
				pos_bound.x		= -1; // Reset bounds
				pos_bound.y		= 0.0001;
			}

			correctAttitude(&pid_att);	// Perform Attitude Correction
			if (jjj % 10 == 0)
				loaSearch();			// Search for LoA
			
			jjj ++;						// Increment secondary iterator for LoA execution
				
			sendMotor();				// Send all motor commands
			PORTC ^= BIT(blueLED);	// Toggle blueLED
			
			if (stop_counter >= 30)		// Check for STOP condition. If achieved, switch modes back to passive.
				mode_switch = 0;
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
		
		_delay_ms(100);			// Changed from 100 to test which loop is running 11/28/23
	}
	
	//PORTC |= BIT(blueLED);		// Turn ON blueLED
	TCCR3B = (1<<CS32) | (0<<CS31) | (1<<CS30);
		
	return 0;
}
