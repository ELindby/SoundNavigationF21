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
	//output_stream << (activation(angleL)) << "," << (activation(angleR)) << "," << angleL << "," << angleR << "," << avoidance_left << "," << avoidance_right << std::endl;
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

//This is used for debugging and test purposes
void Navigation::manualInputSteering(Vision * vision_, std::ofstream& output_stream){
    std::cout << "Manual steering enabled - Type 'r' to resume" << std::endl;
    bool run_bool = true;
    bool print_ico = true;
    while(run_bool){
        switch(vision_->k){
            case 'w':
                motor_control->setMotorDirection(FORWARD);
                break;
            case 's':
                motor_control->setMotorDirection(BACKWARD);
                break;
            case 'a':
                motor_control->setMotorDirection(LEFT);
                break;
            case 'd':
                motor_control->setMotorDirection(RIGHT);
                break;
            case 'i':
                if(print_ico){
                    std::cout << "w_reflex_var: " << w_reflex_var << " -- v_learning rate: " << v_learning << " -- Reflexcounter: " << reflexcounter << std::endl;
                    output_stream << w_reflex_var << "," << v_learning << "," << reflexcounter << std::endl;
                    print_ico = false;
                }
            case 27: //27 = 'ESC'
            case 'r':
                run_bool = false;
                break;
            default:
                motor_control->setMotorDirection(STOP);
                break;
        }
        usleep(50000);

    }
    std::cout << "Manual steering disabled - control loop resumed" << std::endl;
}

void Navigation::obstacleReflex(double angle_to_obst, double dist_to_obst_current, double dist_to_obst_prev, double dist_to_obst_prev_prev)
{
	//LEFT OR RIGHT REFLEX DODGING
	if (angle_to_obst <= 180) { //RIGHT SIDE OBSTACLE
		double angle_norm = (angle_to_obst - 90) / 90;
		motor_control->setRightMotorSpeedOnly(activation(angle_norm));
		motor_control->setLeftMotorSpeedOnly(activation(-angle_norm));

	}
	else { // angle_to_obst > 180 //LEFT SIDE OBSTACLE
		double angle_norm = (90 - (angle_to_obst - 180)) / 90;
		motor_control->setRightMotorSpeedOnly(activation(-angle_norm));
		motor_control->setLeftMotorSpeedOnly(activation(angle_norm));
	}
	//Update weight used for v_learning
	w_reflex_var = w_reflex_var + reflex_learning_rate * (dist_to_obst_current / REFLEX_THRESHOLD) * (dist_to_obst_prev_prev - dist_to_obst_prev) / REFLEX_THRESHOLD;
	reflexcounter += 1;
}

void Navigation::obstacleAvoidance(double angle_to_obst, double dist_to_obst_current, double dist_to_obst_prev, double angle_to_sound, std::ofstream& output_stream)
{
	v_learning = ((AVOIDANCE_THRESHOLD - dist_to_obst_current) / (AVOIDANCE_THRESHOLD - REFLEX_THRESHOLD)) * w_reflex_var + ((AVOIDANCE_THRESHOLD - dist_to_obst_prev) / (AVOIDANCE_THRESHOLD - REFLEX_THRESHOLD)) * w_reflex_novar;

	if (angle_to_obst <= 180) {  //RIGHT SIDE OBSTACLE
		braitenberg(angle_to_sound, output_stream, 0, v_learning);
	}
	else { // angle_to_obst > 180 //LEFT SIDE OBSTACLE
		braitenberg(angle_to_sound, output_stream, v_learning, 0);
	}
}

void Navigation::updateState(states & current_state, double dist_to_obst_current, int sound_energy_level)
{
    if (current_state == TARGET_FOUND){
        return;
    }
    //Check for obstacle within reflex threshold
	if (dist_to_obst_current < REFLEX_THRESHOLD)
	{
		current_state = REFLEX;
	}
	//Check for active sound source, reactive sound navigation
	else if (sound_energy_level > ENERGY_THRESHOLD) {
		if (dist_to_obst_current < AVOIDANCE_THRESHOLD) {
			current_state = AVOIDANCE;
		}
		else {
			current_state = NAVIGATION;
		}

	}
	//If neccessary behaviour has been learned, proactive navigation towards inactive sound source
	else if (proactive_nav_ready == true){
        current_state = PROACTIVE_NAVIGATION;
	}
	else {
		current_state = WAIT;
	}
}
