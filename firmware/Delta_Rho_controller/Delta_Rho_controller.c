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
#define I2C_ADDR			0x08				// I2C Slave Address *** Square Pin 1 is SCL is Blue is pin15 on esp ***
#define POWER_MANAGEMENT_ENABLED				// Enable power management for TWI
#define STOP_UPPER			194					// UPPER acceptable payload rotation for att. corr.
#define STOP_LOWER			166					// LOWER acceptable payload rotation for att. corr.


// Legacy variables
volatile char start			= 0;
volatile int out[3], in[3];
volatile signed int xd[3], x[3], xO[3];

// Meta global vars
float daGlobal;								// Global variable for whatevs
int mode_switch				= 0;			// Mode Toggle (binary for now)
volatile int send_data[3]	= {0, 0, 0};	// For sending data wirelessly via XBee
	
// I2C & TWI setup
volatile int i2c_data		= 180;			// Global var to store latest received i2c data: 0x02 default value
unsigned char message_buf[TWI_BUFFER_SIZE];

// Robot Jacobian
float J_r[3][3]				= {{-0.9874, 0.0263, 0.0456}, {-7.8542, -0.0526, 0}, {-0.9874, 0.0263, -0.0456}}; // Default value
float J[3][3];

// Control input and input memory
float u_r[3]				= {0, 0, 0};	// Control Input
float u_r_mem[2]			= {0, 0};		// Memory for switching between CoM and attitude correction
		
// Primary Algorithm Variables
Vector2D neg_bound			= {1, 0.0001};	// Negative-most possibility for LoA (using right-hand-rule)
Vector2D pos_bound			= {-1, 0.0001};	// Positive-most possibility for LoA (using right-hand-rule)
float rot_mem				= 0;			// Payload rotation memory for stop condition
volatile int stop_counter	= 0;			// STOP counter for Line of Action estimate (when minimal rotation observed)
volatile int new_data_rec	= 0;			// Flag for I2C reception
int desired_vel				= 2650;			// Desired velocity of robot
int step					= 0;			// iterator
int att_switch				= 0;			// attitude correction switch
int STOP_SWITCH				= 0;			// STOP switch for ending experiment
float temp_mem				= 180;
	
// --- FORWARD DECLARATIONS ---
float			medianFilter(float arr[], int n);
Vector2D		rotateVector(Vector2D v_in, float angle);
unsigned char	TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg);


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
					
					//thisVoltage[0] = USART1_Receive();
					//thisVoltage[1] = USART1_Receive();
					//thisVoltage[2] = USART1_Receive();
					//thisVoltage[3] = USART1_Receive();
					//thisVoltage[4] = USART1_Receive();
					//thisVoltage[5] = USART1_Receive();					
					//applyPulse();
					
					// Apply amplified (or non-amplified) voltages
					//FLCCW	= tempVoltage[0];
					//FLCW	= tempVoltage[1];
					//RCCW	= tempVoltage[2];
					//RCW		= tempVoltage[3];
					//FRCCW	= tempVoltage[4];
					//FRCW	= tempVoltage[5];
					
					FLCCW	= USART1_Receive();
					FLCW	= USART1_Receive();
					RCCW	= USART1_Receive();
					RCW		= USART1_Receive();
					FRCCW	= USART1_Receive();
					FRCW	= USART1_Receive();
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
				// SEND u_r motor command
					//send_data[0] = (int) (u_r[0] * 100);
					//send_data[1] = (int) (u_r[1] * 100);
					//send_data[2] = (int) (u_r[2] * 100);
								
				// SEND I2C DATA from esp-cam
					send_data[0] = (int) i2c_data;
					send_data[1] = (int) new_data_rec;
				// SEND WHATEVER GLOBAL DATA YOU WANT YO
					//send_data[0] = daGlobal;
					//send_data[1] = daGlobal;
					
					
					USART1_SerialSend(send_data, 3);					
				break;
				
				
				case(0xD0): // Switch actuation mode -> Write request (208 in decimal)
					mode_switch	= !mode_switch; // Toggle from zero-force mode to driving mode.
				break;
				
				case(0xE0) : // Switch actuation to mode 3 -> Write request (224 in decimal)
					mode_switch = 2; // Switch to mode 3 (temporary, need to instead send a serial packet with the mode num)
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

//		Send current signal to motors ************* TODO FIX THIS *************
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
			f[i] += J_r[i][j] * u_r[j]; // 09-10-24 TEMP CHANGING TO J_r for default value
		}
	}
	
	// 09-10-24 REAR WHEEL SEEMS WEAK. AMPLIFYING A LITTLE BIT TO GET STRAIGHT 
	// LINE DIAGONAL MOTION
	f[1] *= 1.75;
	
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


//		Get random angle for perturbation of LoA Search
//========================================================================
void perturbation (void) {
	Vector2D temp_u = {u_r[1], u_r[2]};
	Vector2D rot_u;
	
	float perturbation_angle = ((float)rand() / (float)RAND_MAX - 0.5) * M_PI / 12.0;  // +/- 15 degrees
	
	rot_u = rotateVector(temp_u, perturbation_angle);
	u_r[1] = rot_u.x;
	u_r[2] = rot_u.y;
}


//		Correct the attitude of the robot using force sensor
//========================================================================
void correctAttitude (PIDController *pid_att) {
	// u_r = [Z X Y] (Clockwise, Right, Forward)
	
	// ----- PID Controller -----
	float error			= pid_att->setPoint - i2c_data;
	pid_att->integral	+= error;
	float derivative	= (error - pid_att->prevError);
	float output		= pid_att->Kp  * error   +   pid_att->Ki  *  pid_att->integral   +   pid_att->Kd  * derivative;
	pid_att->prevError	= error;

	// ==== Assign to Control Input Array ====
	u_r[0]				= output;	// Assign z-axis motor command
}

//		LoA Search - Binary Search Algorithm
//========================================================================
void loaSearch (void) {
	// u_r = [Z X Y] (Clockwise, Right, Forward)
		
	// Determine payload rotation sense with force feedback
	Vector2D f_s			= {-EE[0],		-EE[1]};	 // Force sensor 'direction' - opposite of reading (flip EE vector)
	Vector2D f_r			= {u_r_mem[0],	u_r_mem[1]}; // Robot heading from input - TODO update with IMU / mocap / encoders
	
	// Get angle of force and heading vectors w.r.t. +x axis
	float angle_f_r			= atan2f(f_r.y, f_r.x);
	float angle_f_s			= atan2f(f_s.y, f_s.x);
	// Get rotation sense based on angle between motion and force feedback (-1 or 1 using RHR)
	float rot_amt			= (angle_f_s - 0.01) - angle_f_r; // -.02 TEMPORARY: SUBTRACT A SMALL ERROR CAUSE OF THE SENSOR BIAS TOWARDS THE RIGHT
	float rot_sense			= rot_amt / abs(rot_amt);
	
	//IDEA: CREATE AN ARRAY OR HISTORY OF 'SENSE' OR EVEN JUST ADD HISTORY (-1 + 1 + -1 + -1)=NEG SENSE
	
	
	// ALGORITHM EDGE CASE CATCHING
	if (f_s.y < -1.5)			// (mm) Retain heading when force is 'away' from robot (pulling payload)
		return;
	if (sqrt(pow(f_s.x, 2) + pow(f_s.y, 2)) < 4) // Catch when force is minimal (for testing mainly)
		return;
	if (abs(rot_amt) < 0.03)	// 0.052=3deg (0.09 = 5deg) Catch when angle b/w motion & force minimal. Continue pushing (Sensor uncertainty)
		return;
	step++;						// Increment step counter 
	if (step % 5 != 0)			// Step counter to check stop condition every time step, but only update bounds every nth step
		return;
	
	// Set new bounds based on rotation sense (make it a unit vector)
	float u_r_mag			= sqrt(pow(u_r_mem[0], 2) + pow(u_r_mem[1], 2));
	if (rot_sense >= 0) {
		neg_bound.x			= u_r_mem[0] / u_r_mag;
		neg_bound.y			= u_r_mem[1] / u_r_mag;
	}
	else {
		pos_bound.x			= u_r_mem[0] / u_r_mag;
		pos_bound.y			= u_r_mem[1] / u_r_mag;
	}
	
	// Get new push direction by summing bounds (same as bisecting)
	float output_x			= pos_bound.x + neg_bound.x;
	float output_y			= pos_bound.y + neg_bound.y;
	// Make unit vector and apply desired velocity simultaneously
	float output_mag		= sqrt(pow(output_x, 2) + pow(output_y, 2));
	output_x				*= (desired_vel/output_mag);
	output_y				*= (desired_vel/output_mag);
	
	// ==== Assign to Control Input Array ====
	u_r[1]					= output_x;
	u_r[2]					= output_y;
	
	// == TEMP - Add a perturbation to u_r ==
	perturbation();
	
	// Assign motion memory to new motion command
	u_r_mem[0]			= u_r[1];
	u_r_mem[1]			= u_r[2];
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
	srand(time(NULL));			// Set the seed for rng
	int iii = 0;				// iterator
			
	// ===== PID INITIALIZATIONS (Kp, Ki, Kd, setPoint) ====
	PIDController pid_att; // pid_com, pid_att;
	//PID_Init(&pid_com, 1, 0, 0.08, 0); // 0.8, 0, 0.08 for 6 // 1, 0, 0 works OK but maybe too aggressive (it's immediately changing direction towards force feedback)
	PID_Init(&pid_att, 1.25, 0.05, 0.2, 180); // 1.25, 0.05, 0.2 is critically damped for robot 5


	// ========== PRIMARY LOOP ==========
	while(start == 0){
		runTWI();				// Run all i2c communications (receive camera info)
		sensorKinematics();		// Force feedback, assigned to EE var
		
		// Defaults to case 0. Change modes via MATLAB
		switch (mode_switch) {
			// ---------------------------------------------------------------------------------------------------
			case 0:
			
			// ============================================================
				// When not in on-board mode, send initial 'stop' command
				//if (iii != 0){
					//u_r[0] = u_r[1] = u_r[2] = 0;
					//sendMotor();
				//}
				//iii = 0;
			// ============================================================
			
				PORTC ^= BIT(blueLED);				// Toggle blueLED
				_delay_ms(100);						// Rapid heartbeat
				break;

			case 1:
				correctAttitude(&pid_att);
				//u_r[1] = 2900 * -0.2;
				//u_r[2] = 2900 * 0.98;
				sendMotor();
				break;

			case 2:
				while (iii <= 800){
					iii ++;
					u_r[2]			= u_r_mem[1] = desired_vel;	// Induce motion and set initial 'memory' as 'forward'
					u_r_mem[0]		= 0;
					sendMotor();
					stop_counter	= 0;			// Reset stop counter for multiple trials w/o restarting robot
					neg_bound.x		= 1;
					pos_bound.x		= -1;
					neg_bound.y		= pos_bound.y = 0.0001;		// Reset bounds
					rot_mem			= 0;
					STOP_SWITCH		= 0;			// Reset global stop
					att_switch		= 0;
					temp_mem		= 180;
					step			= 0;
				}


				//  === Want to do EITHER LoA search OR correct attitude ===
				
				// If wheel about to collide, activate attitude control
				if (i2c_data <= STOP_LOWER || i2c_data >= STOP_UPPER) {
					att_switch = 1; // Turn ON attitude control, stop LoA search
					u_r[1] = 0;
					u_r[2] = 0;
				}
				if (i2c_data >= 170  && i2c_data <= 190){ // 179 and 181 If attitude control is done turn OFF attitude control switch
					att_switch = 0;
					u_r[0] = 0;
					u_r[1] = u_r_mem[0]; // Resume prior motion
					u_r[2] = u_r_mem[1];
				}
				
				// Attitude control switching
				if (att_switch) {
					correctAttitude(&pid_att);
					// STOP CONDITION RESET
					stop_counter = 0;
				}
				else {
					loaSearch();
					correctAttitude(&pid_att);
					// STOP CONDITION CHECK
					if (abs(i2c_data - rot_mem) < 0.052) //10 deg // 0.09=5 degrees  // .052 = 3 degrees // Increment if payload vs robot orientation small rate of change
					stop_counter ++;
				}
				
				rot_mem = i2c_data;
				
				// END EXPERIMENT
				if (stop_counter >= 40 || STOP_SWITCH == 1){			// Check for STOP condition
					STOP_SWITCH	= 1;				// Enable global stop
					EE[0] = EE[1] = 0.69;			// SET EE to 'stop value' so MATLAB can change mode. Easier for data collection.
					u_r[0] = u_r[1] = u_r[2] = 0;	// And stop robot motion (untested)
				}
				
				sendMotor();						// Send all motor commands
				
				PORTC ^= BIT(blueLED);				// Toggle blueLED
				_delay_ms(100);						// Rapid heartbeat
				
				break;
		}
	}
	
	//PORTC |= BIT(blueLED);		// Turn ON blueLED
	TCCR3B = (1<<CS32) | (0<<CS31) | (1<<CS30);
		
	return 0;
}
