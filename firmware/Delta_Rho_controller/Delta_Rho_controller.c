#include <\\research.wpi.edu\srl\Projects\Ant\Delta_Rho\Code\Libraries\DeltaRho.h>

#include <math.h>
#include <time.h>
//#include <DeltaRho.h>

// REMEMBER TO CHANGE ROBOTid FOR EACH ROBOT
#define RobotID 5
#define PI 3.14159265358979323846


volatile signed int xd[3] = {0,0,0};
volatile signed int x[3] = {0,0,0};
volatile signed int xO[3] = {0,0,0};
	
volatile signed int dx[3] = {0,0,0};
volatile signed int dxO[3] = {0,0,0};
		
volatile float F_R[3] = {0,0,0};

volatile int out[3] = {0,0,0};
volatile int in[3] = {0,0,0};

volatile char start = 0;

volatile unsigned int t_x = 0;
volatile unsigned int t_o = 0;
volatile unsigned char n_x = 0;
volatile unsigned char n_o = 0;

// Hyland added variables
// Force Sensor
float sensor_data[2] = {0.0, 0.0};
const float Links[5] = {25.0, 25.0, 40.0, 40.0, 22.84};
float p1[2] = {0, 0};
float p2[2] = {22.84, 0};
float p3[2] = {0, 0};
float p4[2] = {0, 0};

float EE[2] = {0, 0};
float home[2] = {0, 0};

volatile int send_data[3] = {0, 0, 0};
	
// Mode Toggle (binary for now)
int mode_switch = 0;

// Control Input
float u_r[3] = {0, 0, 0};

// State estimate
float curr_X[3] = {0, 0, 0};
	
// Timer function
double t = 0;
double t_run = 0;
volatile uint32_t overflowCount = 0;

// Dynamic Jacobian Construction
float J_r[3][3] = {{0.8192, 0.5736, -0.1621}, {0.0, -1.0, -1.36}, {-0.8192, 0.5736, -0.1621}}; // Default value
float J[3][3];
float beta[3] = {-1.0472, 1.0472, 3.1415};
float w = 107.8226;
float h1 = 31.13;
float h2 = 62.25;
float wheel_r = 19;


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


ISR(USART1_RX_vect)
{
	//char i;
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
					//send_data[0] = (int) (sensor_data[0] * 180/3.1415);
					//send_data[1] = (int) (sensor_data[1] * 180/3.1415);
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
					//send_data[2] = (int) (u_r[2] *100);
					
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

//		PID initialization function
//========================================================================
void PID_Init(PIDController* pid, double Kp, double Ki, double Kd) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0.0;
	pid->prevError = 0.0;
}


//		Return dt from previous call of getDt()
//========================================================================
double getDt(void){
	// Get elapsed MCU ticks. Overflow # is 65536 -> count nOverflows. TCNT3 is current timer
	uint32_t elapsedTicks = (overflowCount * 65536) + TCNT3;
	// Divide numTicks by CPU Freq 2 MHz
	double current_t = (elapsedTicks / (double) 20000000);
	// Get difference from previous call
	double diff_t = current_t - t_run;
	// Update total run time
	t_run = current_t;
	
	return diff_t;
	
}


//		Estimate robot state at every time step
//========================================================================
void stateEstimator( char* n, int* timerValue){
	
	float u_r_mag;
	float true_v[3];
	
	// Get magnitude of u_r. Stiction prevents movement under some threshold
	u_r_mag = sqrt(pow(u_r[0], 2) + pow(u_r[1], 2) + pow(u_r[2], 2));
	if (u_r_mag < 80){
		return;
	}

	// Get true velocity given u_r using empirical data
	true_v[0] = 0.59135*u_r[0]*.125;
	true_v[1] = 0.575*u_r[1] - 24.55;
	true_v[2] = 0;
	
	double deltaT = getDt();

	float dx[3];	
	
	dx[0] = true_v[0] * (float) deltaT;
	dx[1] = u_r[1] * (float) deltaT;
	dx[2] = u_r[2] * (float) deltaT;
	
	curr_X[0] += dx[0];
	curr_X[1] += dx[1];
	curr_X[2] += dx[2];
}


//		Calculate EE position based on current base angles
//========================================================================
void sensorKinematics(void) { 
	
	// Read from registers 0 and 3 for potentiometers 1 and 2 respectively
	signed int a = ADC_read(0); //  'left' pot (joint 1)
	signed int b = ADC_read(3); // 'right' pot (joint 2)
	
	// Map voltage to radians, then apply offset to make '0 degrees' == +x axis
	float alpha[2];
	alpha[0] = ((a / 1024.0) * 5.76) - 1.3; // 1.3 rad ~ 74.5 deg (nominal offset)
	alpha[1] = ((b / 1024.0) * 5.76) - 1.474; // 1.474 rad ~ 84.5 deg (nominal offset)
	
	
	float p5[2];
	float lambda[2];
	float lambda_mag;
	float xi;
	float vec_along_l3[2];
	
	// Joint 3 coords
	p3[0] = p1[0] + Links[0]*cos(alpha[0]);
	p3[1] = p1[1] + Links[0]*sin(alpha[0]);
	// Joint 4 coords
	p4[0] = p2[0] + Links[1]*cos(alpha[1]);
	p4[1] = p2[1] + Links[1]*sin(alpha[1]);
	// Distance between 3 and 4
	lambda[0] = p4[0] - p3[0];
	lambda[1] = p4[1] - p3[1];	
	//lambda_mag = norm2D(lambda);
	lambda_mag = sqrt(pow(lambda[0],2) + pow(lambda[1],2));
	// Calculate angle between lambda and l3 using law of cosines	
	xi = acos((pow(Links[2],2) + pow(lambda_mag,2) - pow(Links[3],2)) / (2 * Links[2] * lambda_mag));
	// Find EE (joint 5): multiply unit vector along lambda by l3, rotate by xi, then shift origin by p3
	vec_along_l3[0] = Links[2] * lambda[0]/lambda_mag;
	vec_along_l3[1] = Links[2] * lambda[1]/lambda_mag;
	
	rotateVector2D(vec_along_l3, xi);
	
	p5[0] = p3[0] + vec_along_l3[0];
	p5[1] = p3[1] + vec_along_l3[1];
	
	EE[0] = p5[0] - home[0];
	EE[1] = p5[1] - home[1];
}


//		Rotate 2D vector by angle (RADIANS)
//========================================================================
void rotateVector2D(float in_vector[2], float angle) {
	// Calculate new x and y using rotation matrix
	float tempX = in_vector[0] * cos(angle) - in_vector[1] * sin(angle);
	float tempY = in_vector[0] * sin(angle) + in_vector[1] * cos(angle);

	in_vector[0] = tempX;
	in_vector[1] = tempY;
}


//		ALTERNATE ALTERNATE ALTERNATE Rotate 2D vector
//========================================================================
Vector2D ALTrotateVector2D(Vector2D v, float angle) {
	Vector2D rotatedVec;
	
	// Calculate new x and y using rotation matrix
	rotatedVec.x = v.x * cos(angle) - v.y * sin(angle);
	rotatedVec.y = v.x * sin(angle) + v.y * cos(angle);
	
	return rotatedVec;
}


//		Get HOME CONFIGURATION of 5bar sensor
//========================================================================
void getHome(void){
	//float resting_angles[2] = {2.094, 1.047}; // Resting base angles in RADIANS (120 and 60 degs)
	//sensorKinematics(resting_angles); // Run kinematics on resting base angles
	//home[0] = EE[0];
	//home[1] = EE[1];
	
	// TEMPORARILY HARD-CODING VALUES FROM GEOMETRIC ANALYSIS 02-07-2024
	home[0] = 9.7; // 11.42;
	home[1] = 50.1; // 53.71;
}


//		Get angle between two vectors
//========================================================================
float getAngle(Vector2D v1, Vector2D v2){
	// Catch zero-length sensor measurement
	if (v2.x == 0 && v2.y ==0){
		return 0; }
	return atan2f(v1.x * v2.y - v1.y * v2.x , v1.x * v2.x + v1.y * v2.y);
}


//		Reconstruct Robot Jacobian Based on Sensor
//========================================================================
void calculateJacobian(float sx, float sy, float J[3][3]) {
	// sx,sy inputs are actually EE pos in {sens} frame
	
	// Add to EE pos the distance to the robot frame {b}.
	sy += (36.88 + 10); // 36.88 is {sens} to {b}, 2nd num is empirical correction factor
	double p[3][2] = {{(w/2 - sx), h1-sy}, {(-w/2 - sx), h1-sy}, {-sx, (-h2-sy)}};
	
	float J_temp[3][3];
	// Construct the Jacobian
	for (int i = 0; i < 3; i++) {
		J_temp[i][0] = (p[i][0]*sin(beta[i]) - p[i][1]*cos(beta[i])) / wheel_r;
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
			f[i] += J[i][j] * u_r[j] * 100; // 3-14-24 ADDED *100 WITH NEW JACOBIAN WHICH REQUIRES HIGHER GAINS
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
void correctAttitude (PIDController *pid) {
	
	// u_r[0] is X motion (Forw+/Backw-)
	// u_r[1] is Y motion (Right+/Left-)
	// u_r[2] is Z motion (CCW+/CW-)
		
	// ---- PID Controller ----
	// Error is angle between heading and force measurement
	
	float error = EE[0];
	pid->integral += error;
	float derivative = (error - pid->prevError);
	float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
	pid->prevError = error;
	
	// Update ONLY X (lateral) component of robot motion (FOR TESTING)
	u_r[0] = output;
}


//		CoM estimate - push in straight line
//========================================================================
void comEstimate (PIDController *pid) {
	// Now move IN the direction of force (force amplification)
		// u_r[0] is X motion (Fwd+/Bkwd-)
		// u_r[1] is Y motion (Right+/Left-)
		// u_r[2] is Z motion (CCW+/CW-)
		
	//double dt = getDt();
	Vector2D curr_sensor_direc = {-EE[0], -EE[1]};
	Vector2D curr_heading = {0,1}; //{u_r[0], u_r[1]};
	Vector2D x_axis = {0, 1};

	// ---- PID Controller ----
	// Error is angle between heading and force measurement
	float e_1 = getAngle(x_axis, curr_heading);
	float e_2 = getAngle(x_axis, curr_sensor_direc);
	float error = e_1 - e_2; // do i want to also account for the AMOUNT of displacement of sensor EE?
	pid->integral += error; // * dt;
	float derivative = (error - pid->prevError); // / dt;
	float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
	pid->prevError = error;
	
	Vector2D rotated_output = ALTrotateVector2D(curr_heading, output);
	
	u_r[1] = 35*rotated_output.x;
	u_r[2] = 35*rotated_output.y;
	// temp
	//u_r[0] = error;
}


//================================================================================================
//                                          Main
//================================================================================================
int main(void){
		
	// Initialize micro controller
	mC_init();
	
	// Start the timer
	TCNT3H = 0;
	TCNT3L = 0;
	TIMSK3 |= (1<<TOIE3);	// Enable Timer3 overflow interrupt - ChatGPT advised
	sei();					// Enable global interrupts
	PORTC |= BIT(redLED);	// Turn ON redLED	
	//TCCR3B = (0<<CS32) | (0<<CS31) | (0<<CS30); // IDK what this does
		
	// PID INITIALIZATIONS (Kp, Ki, Kd)
	PIDController pid_com;
	PID_Init(&pid_com, 20, 0, 0);
	
	PIDController pid_attitude;
	PID_Init(&pid_attitude, .1, 0, .01); // 0.005, .004);
	
	int rest_period = 1500; // initial resting period
	int iii = 0;			// iterator
 
	getHome();				// Get sensor resting state home config	

	while (iii < rest_period){
		iii ++;
		u_r[2] = 0; //35;
		sendMotor();
	}
	u_r[2] = 0;
	sendMotor();
	
	// ===== Primary Loop =====
	while(start == 0){

		// Run state estimator (TODO implement this with IMU)
		//stateEstimator(&n_x, &t_o);
							
		// Calculate force and direction of sensor, assign to 'EE' global variable
		sensorKinematics();
		
		// Default mode_switch value is 0. Switch modes via MATLAB
		if (!mode_switch) {
			//comEstimate(&pid);
			//nullSpaceControl();
			
			//correctAttitude(&pid_attitude);
		}
		else {
			//nullSpaceControl();
			
			comEstimate(&pid_com);
			// SEND ALL calculated motor commands to motors
			sendMotor();
		}
		
		PORTC ^= BIT(blueLED);	// Toggle blueLED
		_delay_ms(100);		// Changed from 100 to test which loop is running 11/28/23
	}
	
	PORTC |= BIT(blueLED);		// Turn ON blueLED
	TCCR3B = (1<<CS32) | (0<<CS31) | (1<<CS30);
		
	return 0;
}