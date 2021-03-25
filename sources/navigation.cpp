//#include "pch.h"
#include "../includes/navigation.h"


Navigation::Navigation(MotorControl * motor_control_) : motor_control{ motor_control_ }
{
}

Navigation::~Navigation()
{
}

double Navigation::activation(double input) {
	//return 50 / (1 + exp(-3*input));			//Sigmoid or Logistic						[0,1]
	//return 30 / (1 + exp(-10 * input)) + 10;	//Sigmoid or Logistic v2					[0,1]
	//return 20 / (1 + exp(-10 * input)) + 20;	//Sigmoid or Logistic v2					[0,1]
	//return 30 * tanh(3*input);				//Hyperbolic tangent (tanh)					[-1,1]
	//return 30 * atan(5 * input);				//Inverse Hyperbolic Tangent (arctanh)		[-pi/2,pi/2]
	//return 20 * 2 * atan(tanh(5x));			//Gudermannian								[-pi/2,pi/2]

	//Test of braitenberg vehicle test activation functions:
	return 20 / (1 + exp(-10 * input)) + 20;	//Sigmoid or Logistic				//Default
	//return 10 * tanh(5 * input) + 30;			//Hyperbolic tangent (tanh)
	//return 8 * atan(5 * input) + 30;			//Inverse Hyperbolic Tangent (arctanh)
	//return 6 * 2 * atan(tanh(5 * input)) + 30;		//Gudermannian
}

void Navigation::setBraitenbergLEDs(int direction) {
	switch (direction) {
	case FORWARD:
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, MAX_BRIGHTNESS, 0);		//Center LED
		//motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, MAX_BRIGHTNESS, 0);		//Center LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_2, 0, 0, 0);					//Right LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_L_8, 0, 0, 0);					//Left LED
		break;
	case LEFT:
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, 0, 0);					//Center LED
		//motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, 0, 0);					//Center LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_2, 0, 0, 0);					//Right LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_L_8, 0, MAX_BRIGHTNESS, 0);		//Left LED
		break;
	case RIGHT:
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, 0, 0);					//Center LED
		//motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, 0, 0);					//Center LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_2, 0, MAX_BRIGHTNESS, 0);		//Right LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_L_8, 0, 0, 0);					//Left LED
		break;
	case STOP:
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, 0, 0);					//Center LED
		//motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, 0, 0);					//Center LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_2, 0, 0, 0);					//Right LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_L_8, 0, 0, 0);					//Left LED
		break;
	default:
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, 0, 0);					//Center LED
		//motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, 0, 0);					//Center LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_2, 0, 0, 0);					//Right LED
		motor_control->setMatrixVoiceLED(MATRIX_LED_L_8, 0, 0, 0);					//Left LED
	}

}

void Navigation::braitenberg(double angle, std::ofstream& output_stream, double avoidance_left = 0, double avoidance_right = 0) { //Braitenberg aggression vehicle
	if (angle >= 170 && angle <= 190) //Object is on CENTER
	{
		//Set center LEDs
		//motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, MAX_BRIGHTNESS, 0);
		//motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, MAX_BRIGHTNESS, 0);
		setBraitenbergLEDs(FORWARD);
	}
	if (angle < 180) { //Object is on RIGHT side
		//Set right LED
		//motor_control->setMatrixVoiceLED(MATRIX_LED_R_2, 0, MAX_BRIGHTNESS, 0);
		setBraitenbergLEDs(RIGHT);
	}
	else { // angle >= 180 //object is on LEFT side
		//motor_control->setMatrixVoiceLED(MATRIX_LED_L_8, 0, MAX_BRIGHTNESS, 0);
		setBraitenbergLEDs(LEFT);
	}

	// Update sensor signals
	double angleL = (((360 - angle) - 180) / 180); // Normalize
	double angleR = (angle - 180) / 180; // Normalize

	//motor_control->setRightMotorSpeedDirection(activation(angleR) /*+ VELOCITY_OFFSET*/, 1);
	//motor_control->setLeftMotorSpeedDirection(activation(angleL) /*+ VELOCITY_OFFSET*/, 1);
	motor_control->setRightMotorSpeedOnly(activation(angleR) /*+ VELOCITY_OFFSET*/ + avoidance_right);
	motor_control->setLeftMotorSpeedOnly(activation(angleL) /*+ VELOCITY_OFFSET*/ + avoidance_left);
	//TEST - Print motor values
	std::cout << "Left speed: " << (activation(angleL) /*+ VELOCITY_OFFSET*/) << " - Right speed: " << (activation(angleR) /*+ VELOCITY_OFFSET*/) << std::endl;
	output_stream << (activation(angleL)) << "," << (activation(angleR)) << "," << angleL << "," << angleR << "," << avoidance_left << "," << avoidance_right << std::endl;
}

void Navigation::navigationICO(double angle, double w_A) {
	if (angle < 180) { //Object is on RIGHT side
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, 255, 0);
	}
	else { // angle >= 180 //object is on LEFT side
		motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, 255, 0);
	}

	// Update sensor signals
	double angleL = (((360 - angle) - 180) / 180); // Normalize
	double angleR = (angle - 180) / 180; // Normalize

	motor_control->setRightMotorSpeedDirection(activation(angleR)*w_A + VELOCITY_OFFSET, 1);
	motor_control->setLeftMotorSpeedDirection(activation(angleL)*w_A + VELOCITY_OFFSET, 1);
	//TEST - Print motor values
	std::cout << "Left speed: " << (activation(angleL) + VELOCITY_OFFSET) << " - Right speed: " << (activation(angleR) + VELOCITY_OFFSET) << std::endl;
}
