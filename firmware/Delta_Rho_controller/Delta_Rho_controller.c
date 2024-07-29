//#include <\\research.wpi.edu\srl\Projects\Ant\Delta_Rho\Code\Libraries\DeltaRho.h>
#include "DeltaRho.h"

#include <math.h>
#include <time.h>
#include <util/twi.h>
#include "TWI_slave.h"
#include "avr/sleep.h"

// REMEMBER TO CHANGE ROBOTid FOR EACH ROBOT
#define RobotID				4
#define I2C_ADDR			0x08 // I2C Slave Address *** Pin 1 is SCL***
#define PI					3.14159265358979323846
#define NUM_SAMPLES			5	 // Number of sensor samples for mean filtering
#define PULSE_VOLTAGE		255  // or 100?
#define THRESHOLD_VOLTAGE	80	 // Below this voltage, motors may not move
#define POWER_MANAGEMENT_ENABLED // Enable power management for TWI

// PID structure
typedef struct {
	double Kp; // Proportional gain
	double Ki; // Integral gain
	double Kd; // Derivative gain

	double integral;
	double prevError;
	double setPoint; // Desired value of the process variable
} PIDController;

// 2D Vector structure
typedef struct {
	float x;
	float y;
} Vector2D;

// Legacy variables
volatile signed int xd[3], x[3], xO[3]; // = {0,0,0};
volatile int out[3], in[3]; // = {0,0,0};
volatile char start = 0;
volatile unsigned char n_x, n_o; // = 0;
volatile uint32_t overflowCount = 0;

int mode_switch = 0;							// Mode Toggle (binary for now)
volatile int send_data[3] = {0, 0, 0};			// For sending data wirelessly via XBee
	
// I2C & TWI setup
volatile int i2c_data = 180;					// Global var to store latest received i2c data: 0x02 default value
unsigned char message_buf[TWI_BUFFER_SIZE];
int payl_orient_0 = 180;						// Initial payload orientation for attitude control (camera is inverted currently)

float u_r[3] = {0, 0, 0};						// Control Input
float output1, output2;

// Force Sensor
float Links[5] = {25.0, 25.0, 40.0, 40.0, 22.84};
float EE[2] = {0, 0};
Vector2D home = {0, 0};							// Force sensor home EE position
Vector2D raw_sensor_data = {729.211, 418.9086}; // Hard-coded voltages representing 120 and 60 degs

// Robot Jacobian
float J_r[3][3] = {{0.8192, 0.5736, -0.1621}, {0.0, -1.0, -1.36}, {-0.8192, 0.5736, -0.1621}}; // Default value
float J[3][3];

// Add 'pulse' to overcome stiction
int thisVoltage[6], tempVoltage[6], lastVoltage[6]; // = {0, 0, 0, 0, 0, 0};


// Global variable for whatevs
float daGlobal;

// --- FORWARD DECLARATIONS ---
float medianFilter(float arr[], int n);
Vector2D rotateVector(Vector2D v_in, float angle);
unsigned char TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg);



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
					//send_data[0] = (int) (output1 * 100);
					//send_data[1] = (int) (output2 * 100);
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


ISR(TIMER3_COMPA_vect) {
	n_x ++; // Increment on every Compare Match interrupt
	n_o ++;
	
	overflowCount ++; // Increment overflow count when timer reaches max value
}


// ===========================================================================
// =========================     I2C SETUP     ===============================
// ===========================================================================

//		I2C Error Handling Function
//========================================================================
unsigned char TWI_Act_On_Failure_In_Last_Transmission (unsigned char TWIerrorMsg)
{
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.
	
	// This very simple example puts the error code on PORTB and restarts the transceiver with
	// all the same data in the transmission buffers.
	//receivedI2C = TWIerrorMsg;
	TWI_Start_Transceiver();
	
	return TWIerrorMsg;
}

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



// ===========================================================================
// ========================     MOTOR FUNCTIONS     ==========================
// ===========================================================================

//		Stiction overcome function
//========================================================================
void applyPulse(void){
	// Takes 'thisVoltage' and compares it to a threshold AND to the 'lastVoltage' to determine
	// if a pulse is needed to kickstart the motor motion.
	
	for (int i=0; i<6; i++){
		if (thisVoltage[i] != 0) {
			if (thisVoltage[i] < THRESHOLD_VOLTAGE && thisVoltage[i] != lastVoltage[i]) {
				tempVoltage[i] = PULSE_VOLTAGE;   // Apply the pulse voltage
			}
			else {
				tempVoltage[i] = thisVoltage[i];
			}
		}
		else {
			tempVoltage[i] = thisVoltage[i];
		}
	}
	
	// Set the previous voltage to the current voltage
	lastVoltage[0] = thisVoltage[0];
	lastVoltage[1] = thisVoltage[1];
	lastVoltage[2] = thisVoltage[2];
	lastVoltage[3] = thisVoltage[3];
	lastVoltage[4] = thisVoltage[4];
	lastVoltage[5] = thisVoltage[5];
	
}

//		Reconstruct Robot Jacobian Based on Sensor
//========================================================================
void calculateJacobian(float sx, float sy, float J[3][3]) {
	// Hardware values
	float beta[3] = {-1.0472, 1.0472, 3.1415};
	float w = 107.822;
	float h1 = 31.1235;
	float h2 = 62.25;
	float wheel_r = 19;
	
	// sx,sy inputs are actually EE pos in {sens} frame (9.7, 50.1) by default
	
	// Add to EE pos the distance to the robot frame {b}.
	sy += (36.88 + 10); // 36.88 is {sens} to {b}, 2nd num is empirical correction factor
	
	// C is the distance from center of rotation {EE for now} to wheel (FR, FL, R)
	sx = 9.7;
	sy = 50.1 + 36.88 + 10;
	double C[3][2] = {{(w/2 - sx), h1-sy}, {(-w/2 - sx), h1-sy}, {-sx, (-h2-sy)}};
	
	float J_temp[3][3];
	// Construct the Jacobian
	for (int i = 0; i < 3; i++) {
		J_temp[i][0] = (C[i][0]*sin(beta[i]) - C[i][1]*cos(beta[i])) / wheel_r;
		J_temp[i][1] = cos(beta[i]) / wheel_r;
		J_temp[i][2] = sin(beta[i]) / wheel_r;
	}
	
	// Swap the order for correct wheel assignment in sendMotor()
	J[0][0] = J_temp[1][0];
	J[0][1] = J_temp[1][1];
	J[0][2] = J_temp[1][2];
	
	J[1][0] = J_temp[2][0];
	J[1][1] = J_temp[2][1];
	J[1][2] = J_temp[2][2];
	
	J[2][0] = J_temp[0][0];
	J[2][1] = J_temp[0][1];
	J[2][2] = J_temp[0][2];
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
// ========================     SENSOR FUNCTIONS     =========================
// ===========================================================================

//		Get mean-filtered sensor voltage data
//========================================================================
void getSensorData(void) {
	
	// Read voltage from registers. Potentiometers numbered L --> R when pots are exposed (label up).
	//     Joint1 --> pin1   and  Joint2 --> pin2
	//       pin1 --> Sens2  and    pin2 --> Sens1 on sensor port
	//      Sens2 --> ADC3   and   Sens1 --> ADC0
	// *** Joint1 --> ADC3   AND  Joint2 --> ADC0 ***
	
	float sensor_data_samples[2][NUM_SAMPLES]; // [joint1; joint2] x NUM_SAMPLES
	
	for (int i=0; i<NUM_SAMPLES; i++)	{
		sensor_data_samples[0][i] = ADC_read(3); // joint 1
		sensor_data_samples[1][i] = ADC_read(0); // joint 2
		_delay_ms(5);
	}
	
	// Apply median filter and return raw_sensor_data
	raw_sensor_data.x = medianFilter(sensor_data_samples[0], NUM_SAMPLES);
	raw_sensor_data.y = medianFilter(sensor_data_samples[1], NUM_SAMPLES);
}

//		Apply median filter to raw sensor data
//========================================================================
float medianFilter(float arr[], int n) {
	// Simple bubble sort
	for (int i = 0; i < n-1; i++) {
		for (int j = 0; j < n-i-1; j++) {
			if (arr[j] > arr[j+1]) {
				float temp = arr[j];
				arr[j] = arr[j+1];
				arr[j+1] = temp;
			}
		}
	}
	// If number of elements are even
	if (n % 2 == 0) {
		return (arr[n/2 - 1] + arr[n/2]) / 2.0;
	}
	// If number of elements are odd
	else {
		return arr[n/2];
	}
}

//		Calculate EE position based on current base angles
//========================================================================
void sensorKinematics(void) {
	// All values in radians and mm where applicable
	
	// Get median-filtered raw sensor data
	getSensorData();
	
	// Map voltage to radians, then subtract offset to align '0 volts' to '0 degrees' (which is -75 deg)
	Vector2D alpha;
	alpha.x = (float) ((raw_sensor_data.x / 1024.0) * 5.7596) - 1.309; // 1.309 rad ~ 75 deg (nominal offset)
	alpha.y = (float) ((raw_sensor_data.y / 1024.0) * 5.7596) - 1.309; // 1.474 rad ~ 84.5 deg (nominal offset)
	
	Vector2D p1 = {0, 0};
	Vector2D p2 = {22.84, 0};
	Vector2D p3, p4, lambda, vec_along_lambda, vec_along_l3;
	float lambda_mag, xi;
	
	// Joint 3 and 4 coordinates
	p3.x = p1.x + Links[0]*cos(alpha.x);
	p3.y = p1.y + Links[0]*sin(alpha.x);
	p4.x = p2.x + Links[1]*cos(alpha.y);
	p4.y = p2.y + Links[1]*sin(alpha.y);
	// Distance between 3 and 4
	lambda.x = p4.x - p3.x;
	lambda.y = p4.y - p3.y;
	lambda_mag = sqrt(pow(lambda.x,2) + pow(lambda.y,2));
	// Angle between lambda and l3 using law of cosines
	xi = (float) acos((pow(Links[2],2) + pow(lambda_mag,2) - pow(Links[3],2)) / (2 * Links[2] * lambda_mag));
	
	// Find EE (joint 5): multiply unit vector along lambda by l3, rotate by xi, then shift origin by p3
	vec_along_lambda.x = Links[2] * lambda.x/lambda_mag;
	vec_along_lambda.y = Links[2] * lambda.y/lambda_mag;
	vec_along_l3 = rotateVector(vec_along_lambda, xi);
	
	Vector2D p5;
	p5.x = p3.x + vec_along_l3.x;
	p5.y = p3.y + vec_along_l3.y;
	
	EE[0] = -(p5.x - home.x); // Must flip x-axis sign with new board design with flipped sensor (label down)
	EE[1] = p5.y - home.y;
}



// ===========================================================================
// =======================     HELPER FUNCTIONS     ==========================
// ===========================================================================

//		PID initialization function
//========================================================================
void PID_Init(PIDController* pid, double Kp, double Ki, double Kd) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0.0;
	pid->prevError = 0.0;
}

//		Rotate 2D vector by angle in radians
//========================================================================
Vector2D rotateVector(Vector2D v_in, float angle) {
	Vector2D rotatedVec;
	
	// Calculate new x and y using rotation matrix
	rotatedVec.x = v_in.x * cos(angle) - v_in.y * sin(angle);
	rotatedVec.y = v_in.x * sin(angle) + v_in.y * cos(angle);
	
	return rotatedVec;
}

//		Get angle between two vectors
//========================================================================
float getAngle(Vector2D v1, Vector2D v2){
	// Catch zero-length sensor measurement
	if (v2.x == 0 && v2.y ==0){
		return 0; }
	return atan2f(v1.x * v2.y - v1.y * v2.x , v1.x * v2.x + v1.y * v2.y);
}

//		Get home position of force sensor
//========================================================================
void getHome(void){
	// Basically doing sensorKinematics ONCE at startup. All values in radians and mm where applicable
	
	// Get median-filtered raw sensor data
	getSensorData();
	
	// Map voltage to radians, then subtract offset to align '0 volts' to '0 degrees' (which is -75 deg)
	Vector2D alpha;
	alpha.x = (float) ((raw_sensor_data.x / 1024.0) * 5.7596) - 1.309; // 1.309 rad ~ 75 deg (nominal offset)
	alpha.y = (float) ((raw_sensor_data.y / 1024.0) * 5.7596) - 1.309; // 1.474 rad ~ 84.5 deg (nominal offset)
	
	Vector2D p1 = {0, 0};
	Vector2D p2 = {22.84, 0};
	Vector2D p3, p4, lambda, vec_along_lambda, vec_along_l3;
	float lambda_mag, xi;
	
	// Joint 3 and 4 coordinates
	p3.x = p1.x + Links[0]*cos(alpha.x);
	p3.y = p1.y + Links[0]*sin(alpha.x);
	p4.x = p2.x + Links[1]*cos(alpha.y);
	p4.y = p2.y + Links[1]*sin(alpha.y);
	// Distance between 3 and 4
	lambda.x = p4.x - p3.x;
	lambda.y = p4.y - p3.y;
	lambda_mag = sqrt(pow(lambda.x,2) + pow(lambda.y,2));
	// Angle between lambda and l3 using law of cosines
	xi = (float) acos((pow(Links[2],2) + pow(lambda_mag,2) - pow(Links[3],2)) / (2 * Links[2] * lambda_mag));
	
	// Find EE (joint 5): multiply unit vector along lambda by l3, rotate by xi, then shift origin by p3
	vec_along_lambda.x = Links[2] * lambda.x/lambda_mag;
	vec_along_lambda.y = Links[2] * lambda.y/lambda_mag;
	vec_along_l3 = rotateVector(vec_along_lambda, xi);
	
	// Get sensor resting position
	home.x = p3.x + vec_along_l3.x;
	home.y = p3.y + vec_along_l3.y;
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
void correctAttitude (PIDController *pid_attitude) { //, float error_metric) {

	// u_r[0] is Z motion (CCW+/CW-)
	// u_r[1] is X motion (Fwd+/Bkwd-)
	// u_r[2] is Y motion (Right+/Left-)

	// ----- PID Controller -----
	float error = payl_orient_0 - i2c_data;
	
	pid_attitude->integral += error;
	float derivative = (error - pid_attitude->prevError);
	float output = pid_attitude->Kp * error + pid_attitude->Ki * pid_attitude->integral + pid_attitude->Kd * derivative;
	pid_attitude->prevError = error;

	u_r[0] = output;
	daGlobal = error;
}


//		CoM estimate VERSION 2 - Use angle method
//========================================================================
void comEstimate2 (PIDController *pid_com_angle) {
	// Now move IN the direction of force (force amplification)
	// u_r[0] is Z motion (CCW+/CW-)
	// u_r[1] is X motion (Fwd+/Bkwd-)
	// u_r[2] is Y motion (Right+/Left-)
	
	Vector2D sensor_direc	= {-EE[0], -EE[1]}; // Opposite of sensor's reading. Flip direction of the EE vector
	Vector2D robot_heading	= {u_r[1], u_r[2]}; // Robot's command input TODO update this with ACTUAL robot heading from IMU or mocap or encoders
	Vector2D ref_axis		= {1, 0};			// +x axis for now
	

	
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
	output1  =  pid_com_angle->Kp  * error  +   pid_com_angle->Ki  * pid_com_angle->integral   +   pid_com_angle->Kd  * derivative;
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
	
	// PID INITIALIZATIONS (Kp, Ki, Kd)
	PIDController pid_com_angle, pid_att;
	PID_Init(&pid_com_angle, 1, 0, 0);
	PID_Init(&pid_att, 1, .005, .004); // 1, 0.005, .004);
		
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

			// nullSpaceControl(); // Cancel out force detected by sensor
			//comEstimate2(&pid_com_angle, &pid_att);				// CoM Estimate
			
			correctAttitude(&pid_att); // Perform attitude correction
			u_r[1] = 0;
			u_r[2] = 0;
			
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