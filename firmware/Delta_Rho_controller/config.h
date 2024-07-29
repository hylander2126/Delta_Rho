// Configuration and helper fns for Robot Controllers

#define NUM_SAMPLES			5	 // Number of sensor samples for mean filtering
#define THRESHOLD_VOLTAGE	80	 // Below this voltage, motors may not move

volatile unsigned char n_x, n_o; // = 0;
volatile uint32_t overflowCount = 0;

// Add 'pulse' to overcome stiction
int thisVoltage[6], tempVoltage[6], lastVoltage[6]; // = {0, 0, 0, 0, 0, 0};

// 2D Vector structure
typedef struct {
	float x;
	float y;
} Vector2D;

// PID structure
typedef struct {
	double Kp; // Proportional gain
	double Ki; // Integral gain
	double Kd; // Derivative gain

	double integral;
	double prevError;
	double setPoint; // Desired value of the process variable
} PIDController;


// Force Sensor
float Links[5] = {25.0, 25.0, 40.0, 40.0, 22.84};
float EE[2] = {0, 0};
Vector2D home = {0, 0};							// Force sensor home EE position
Vector2D raw_sensor_data = {729.211, 418.9086}; // Hard-coded voltages representing 120 and 60 degs


//		ISR for Timer3
//========================================================================
ISR(TIMER3_COMPA_vect) {
	n_x ++; // Increment on every Compare Match interrupt
	n_o ++;
	
	overflowCount ++; // Increment overflow count when timer reaches max value
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
				tempVoltage[i] = 255;   // Apply the pulse voltage (255 is max)
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

// ===========================================================================
// =======================     HELPER FUNCTIONS     ==========================
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

//		PID initialization function
//========================================================================
void PID_Init(PIDController* pid, double Kp, double Ki, double Kd, double setPoint) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0.0;
	pid->prevError = 0.0;
	pid->setPoint = setPoint;
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
	if (v1.x == 0 || v1.y == 0 || v2.x == 0 || v2.y == 0)
		return 0;
		
	
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
// ========================     SENSOR FUNCTIONS     =========================
// ===========================================================================

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

