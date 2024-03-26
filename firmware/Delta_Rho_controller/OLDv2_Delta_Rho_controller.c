#include <\\research.wpi.edu\srl\Projects\Ant\Delta_Rho\Code\Libraries\DeltaRho.h>

#include <math.h>
#include <time.h>
//#include <DeltaRho.h>

// REMEMBER TO CHANGE ROBOTid FOR EACH ROBOT
#define RobotID 3
//#define Kp 10


volatile signed int xd[3] = {0,0,0};
volatile signed int x[3] = {0,0,0};
volatile signed int xO[3] = {0,0,0};
	
volatile signed int dx[3] = {0,0,0};
volatile signed int dxO[3] = {0,0,0};
		
volatile float F_R[3] = {0,0,0};

volatile int out[3] = {0,0,0};
volatile int in[3] = {0,0,0};

volatile char start = 0;


// Hyland - FORCE SENSOR Variables
float sensor_data[2] = {0.0, 0.0};
const float Links[5] = {25.0, 25.0, 40.0, 40.0, 22.84};
float p1[2] = {0, 0};
float p2[2] = {22.84, 0};
float p3[2] = {0, 0};
float p4[2] = {0, 0};
	
float EE[3] = {0, 0};
float home[2] = {0, 0};

volatile int send_EE[3] = {0, 0, 0};
volatile int send_sensor[3] = {0, 0, 0};
	
// Hyland - Mode Toggle (binary for now)
int mode_switch = 0;
float u_r[3] = {0, 0, 0};

// Hyland - Jacobian
float J_r[3][3] = {{0.8192, 0.5736, -0.1621}, {0.0, -1.0, -1.36}, {-0.8192, 0.5736, -0.1621}};

// Hyland - State estimate
float curr_X[3] = {0, 0, 0};
volatile int send_X[3] = {0, 0, 0};
	
// Hyland - Fixing Timer function
double t = 0;
double t_run = 0;
volatile uint32_t overflowCount = 0;

volatile unsigned int t_x = 0;
volatile unsigned int t_o = 0;
volatile unsigned char n_x = 0;
volatile unsigned char n_o = 0;

// Hyland - PID structure
typedef struct {
	double Kp; // Proportional gain
	double Ki; // Integral gain
	double Kd; // Derivative gain

	double integral;
	double prevError;
	double setPoint; // Desired value of the process variable
} PIDController;

// Hyland - Vec structure
// Define a struct for a 2D vector
typedef struct {
	double x;
	double y;
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
					//send_sensor[0] = (int) (sensor_data[0] * 180/3.1415);
					//send_sensor[1] = (int) (sensor_data[1] * 180/3.1415);
					//USART1_SerialSend(send_sensor, 3);
					
					// SEND EE POSITION (mm)
					//send_EE[0] = (int) (u_r[0]); // * 100);
					//send_EE[1] = (int) (u_r[1]); // * 100);
					//USART1_SerialSend(send_EE, 3);
					
					// SEND STATE ESTIMATE (mm)
					send_X[0] = (int) (curr_X[0]); // * 100000);
					send_X[1] = (int) (curr_X[1]); // * 100000);
					send_X[2] = (int) (curr_X[2]); // * 100000);
					USART1_SerialSend(send_X, 3);
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

// PID initialization function
void PID_Init(PIDController* pid, double Kp, double Ki, double Kd, double setPoint) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0.0;
	pid->prevError = 0.0;
	pid->setPoint = setPoint;
}

float calculateTime(unsigned char *n, unsigned int *timerValue){
	
	float time = 0;
	unsigned int CurrentTimer;
	
	// Get current timer time from MCU
	CurrentTimer = TCNT3;
	
	// Get change in time from previous timerValue
	time =  *n*(OCR3A + 1) + CurrentTimer - *timerValue;
	
	// Reset n and set timerValue to currentTimer
	*n = 0;
	*timerValue = CurrentTimer;
	
	// Apply timer scaling factor (clocks per sec)
	time = time * 0.0001024;
	
	return time;
}

double getDt(){
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


//////////////////////////////////////////////////////////////////////////
///     Estimate robot state at every time step
//////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////
///     Read sensor and process to radians (or degrees)
//////////////////////////////////////////////////////////////////////////
void updateSensor(void) {
	// Read from registers 0 and 3 for potentiometers 1 and 2 respectively
	signed int a = ADC_read(0); //  'left' pot (joint 1)
	signed int b = ADC_read(3); // 'right' pot (joint 2)
		
	// Map voltage to radians, then apply offset to make '0 degrees' == +x axis
	sensor_data[0] = ((a / 1024.0) * 5.76) - 1.3; // 1.3 rad ~ 74.5 deg (nominal offset)
	sensor_data[1] = ((b / 1024.0) * 5.76) - 1.474; // 1.474 rad ~ 84.5 deg (nominal offset)
}


//////////////////////////////////////////////////////////////////////////
///     Calculate EE position based on current base angles
//////////////////////////////////////////////////////////////////////////
void sensorKinematics(float alpha[2]) { 
	
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
	
	float l3_vec[2] = rotateVector2D(vec_along_l3, xi);
	
	p5[0] = p3[0] + l3_vec[0];
	p5[1] = p3[1] + l3_vec[1];
	
	EE[0] = p5[0] - home[0];
	EE[1] = p5[1] - home[1];
}


//////////////////////////////////////////////////////////////////////////
///     Rotate 2D vector by angle (RADIANS)
//////////////////////////////////////////////////////////////////////////
float rotateVector2D(float in_vector[2], float angle) {
	float tempX;
	float tempY;
	
	// Calculate new x and y using rotation matrix
	tempX = in_vector[0] * cos(angle) - in_vector[1] * sin(angle);
	tempY = in_vector[0] * sin(angle) + in_vector[1] * cos(angle);

	in_vector[0] = tempX;
	in_vector[1] = tempY;
}

//////////////////////////////////////////////////////////////////////////
///     ALTERNATE ALTERNATE ALTERNATE Rotate 2D vector
//////////////////////////////////////////////////////////////////////////
float ALTrotateVector2D(Vector2D v, float angle) {
	Vector2D rotatedVec;
	
	// Calculate new x and y using rotation matrix
	rotatedVec.x = v.x * cos(angle) - v.y * sin(angle);
	rotatedVec.y = v.x * sin(angle) + v.y * cos(angle);
	
	return rotatedVec;
}

//////////////////////////////////////////////////////////////////////////
///     Get HOME CONFIGURATION of 5bar sensor
//////////////////////////////////////////////////////////////////////////
void getHome(void){
	//float resting_angles[2] = {2.094, 1.047}; // Resting base angles in RADIANS (120 and 60 degs)
	//sensorKinematics(resting_angles); // Run kinematics on resting base angles
	//home[0] = EE[0];
	//home[1] = EE[1];
	
	// TEMPORARILY HARD-CODING VALUES FROM GEOMETRIC ANALYSIS 02-07-2024
	home[0] = 9.7; // 11.42;
	home[1] = 50.1; // 53.71;
}


//////////////////////////////////////////////////////////////////////////
///     Get angle between two vectors
//////////////////////////////////////////////////////////////////////////
double getAngle(Vector2D v1, Vector2D v2){
	return atan2(v1.x * v2.y - v1.y * v2.x , v1.x * v2.x + v1.y * v2.y);
}

//////////////////////////////////////////////////////////////////////////
///     Cancel out force measured by sensor
//////////////////////////////////////////////////////////////////////////
void nullSpaceControl (void) {
	
	// u_r[0] is X motion (Forw+/Backw-)
	// u_r[1] is Y motion (Right+/Left-)
	// u_r[2] is Z motion (CCW+/CW-)
	
	unsigned char u[3][2];
	float f[3] = {0,0,0};
	signed int temp;
	
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
	
	// Perform the matrix multiplication J_r * u_r
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			f[i] += J_r[i][j] * u_r[j];
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


//////////////////////////////////////////////////////////////////////////
///     Correct the attitude of the robot using force sensor
//////////////////////////////////////////////////////////////////////////
void correctAttitude (void) {
	
	// u_r[0] is X motion (Forw+/Backw-)
	// u_r[1] is Y motion (Right+/Left-)
	// u_r[2] is Z motion (CCW+/CW-)
	
	unsigned char u[3][2];
	float f[3] = {0,0,0};
	signed int temp;
	
	float gainR = 20; // gain in ROBOT's ROTATIONAL direction
	
	// Sensor 'x' is robot 'y' and vice-versa
	u_r[0] = 0;
	u_r[1] = 0;
	u_r[2] = EE[0]*gainR;
	
	// Perform the matrix multiplication J_r * u_r
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			f[i] += J_r[i][j] * u_r[j];
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


//////////////////////////////////////////////////////////////////////////
///     CoM estimate - push in straight line
//////////////////////////////////////////////////////////////////////////
void comEstimate (PIDController *pid) {
	// Now move IN the direction of force (force amplification)
	
	// u_r[0] is X motion (Fwd+/Bkwd-)
	// u_r[1] is Y motion (Right+/Left-)
	// u_r[2] is Z motion (CCW+/CW-)
	
	double dt = getDt();
	Vector2D curr_direc = {EE[1], EE[0]}; // must flip x and y
	Vector2D curr_heading = {u_r[0], u_r[1]}; 
	
	// Sensor displacement 'away' from robot has dampened so increase the 'fwd/back' gain
	//if (EE[1] > 3)
		//gainX *= 1.3;
		
	// Get unit direction of sensor displacement
	float EE_mag = sqrt(pow(EE[0],2) + pow(EE[1],2));
	curr_direc.x /= EE_mag;
	curr_direc.y /= EE_mag; 

	// ---- PID Controller ----
		
	// Error is angle between heading and force measurement
	double error = 0 - getAngle(curr_heading, curr_direc); //do i want to also account for the AMOUNT of displacement of sensor EE?
	pid->integral += error * dt;
	double derivative = (error - pid->prevError) / dt;
	double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
	pid->prevError = error;
	
	Vector2D temp = ALTrotateVector2D(curr_heading, output);
	
	u_r = temp;	
	
	unsigned char u[3][2];
	float f[3] = {0,0,0};
	signed int temp;
	// Perform the matrix multiplication J_r * u_r
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			f[i] += J_r[i][j] * u_r[j];
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

//================================================================================================
//                                          Main
//================================================================================================
int main(void){
	
	// Initialize microcontroller fns
	mC_init();
	
	// Start the timer
	TCNT3H = 0;
	TCNT3L = 0;
	
	PORTC &= ~BIT(redLED); // Turn OFF redLED
	PORTC |= BIT(blueLED); // Turn ON blueLED
	PORTC |= BIT(redLED);  // Turn ON redLED
	
	//TCCR3B = (0<<CS32) | (0<<CS31) | (0<<CS30);
	//TCCR3B |= (1<<CS30); // No prescaling - ChatGPT advised
	TIMSK3 |= (1<<TOIE3); // Enable Timer3 overflow interrupt - ChatGPT advised
	sei(); // Enable global interrupts
	
	//float time;
	int rest_period = 0; // initial resting period
	getHome(); // Get sensor resting state home config
	
	// PID controller init
	PIDController pid;
	double Kp = 0.1, Ki = 0.01, Kd = 0.05;
	PID_Init(&pid, Kp, Ki, Kd);
	
	
	// Primary Loop
	while(start == 0){
		
		if (rest_period < 2){ // was 40
			rest_period += 1;
			continue;
		}

		// Run state estimator
		stateEstimator(&n_x, &t_o);						
		// ADC read and assign to 'sensor_data'
		updateSensor();
		// Calculate force and direction of sensor, assign to 'EE'
		sensorKinematics(sensor_data);
		
		// Default is null-space control mode
		if (!mode_switch) {
			nullSpaceControl();
			//correctAttitude();
		}
		else {
			comEstimate(&pid);
		}
		
		PORTC  ^= BIT(blueLED);	// Toggle blueLED
		_delay_ms(100); // Changed from 100 to test which loop is running 11/28/23
	}
	
	PORTC |= BIT(blueLED);		// Turn ON blueLED
	TCCR3B = (1<<CS32) | (0<<CS31) | (1<<CS30);
		
	// IT DOESNT APPEAR THIS LOOP EVER RUNS... HAVEN'T TESTED IF IT RUNS WHEN SENDING MOTOR COMMANDS.
	// Temp commenting out just the first two lines of the loop and the loop itself.
	//while (1){
		//PORTC ^= BIT(blueLED);		// Toggle blueLED
		//_delay_ms(100); // Changed from 500 07/31/23
		
		// controller_old();
		//f_desired_position();
		//controller();				// NOT SURE WHAT THIS DOES ... DOESN'T SEEM TO BE NEEDED WITH CENTRALIZED (MATLAB) CONTROL
	//}

	return 0;
}