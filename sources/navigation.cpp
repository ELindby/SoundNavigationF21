#include "../includes/navigation.h"

Navigation::Navigation(MotorControl * motor_control_, LearnedPathHandler * learned_path_handler_)
: motor_control{ motor_control_ }, learned_path_handler{ learned_path_handler_ } {}

Navigation::~Navigation(){}

double Navigation::activation(double input) {
	return 20 / (1 + exp(-10 * input)) + 20;	//Sigmoid or Logistic				
}

double Navigation::activationAvoidance(double input) {
	return 2 / (1 + exp(-5 * input)) - 1;	//Sigmoid or Logistic
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

void Navigation::reactiveSoundNavigation(double angle, std::ofstream& output_stream, double avoidance_left = 0, double avoidance_right = 0) { //Braitenberg aggression vehicle
//	if (angle >= 170 && angle <= 190) //Object is on CENTER
//	{
//		//Set center LEDs
//		//motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, MAX_BRIGHTNESS, 0);
//		//motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, MAX_BRIGHTNESS, 0);
//		setBraitenbergLEDs(FORWARD);
//	}
//	if (angle < 180) { //Object is on RIGHT side
//		//Set right LED
//		//motor_control->setMatrixVoiceLED(MATRIX_LED_R_2, 0, MAX_BRIGHTNESS, 0);
//		setBraitenbergLEDs(RIGHT);
//	}
//	else { // angle >= 180 //object is on LEFT side
//		//motor_control->setMatrixVoiceLED(MATRIX_LED_L_8, 0, MAX_BRIGHTNESS, 0);
//		setBraitenbergLEDs(LEFT);
//	}

	// Update sensor signals
	double angle_norm_l = (((360 - angle) - 180) / 180); // Normalize
	double angle_norm_r = (angle - 180) / 180; // Normalize

	//motor_control->setRightMotorSpeedDirection(activation(angleR) /*+ VELOCITY_OFFSET*/, 1);
	//motor_control->setLeftMotorSpeedDirection(activation(angleL) /*+ VELOCITY_OFFSET*/, 1);
	motor_control->setRightMotorSpeedOnly(activation(angle_norm_r) /*+ VELOCITY_OFFSET*/ + avoidance_right + 4.0f);
	motor_control->setLeftMotorSpeedOnly(activation(angle_norm_l) /*+ VELOCITY_OFFSET*/ + avoidance_left);
	//TEST - Print motor values
	//std::cout << "Left speed: " << (activation(angleL) /*+ VELOCITY_OFFSET*/) << " - Right speed: " << (activation(angleR) /*+ VELOCITY_OFFSET*/) << std::endl;
	//output_stream << (activation(angleL)) << "," << (activation(angleR)) << "," << angleL << "," << angleR << "," << avoidance_left << "," << avoidance_right << std::endl;

	//Store last issued motor commands, for path learning
	left_motor_command	= activation(angle_norm_l);
	right_motor_command	= activation(angle_norm_r) + 4.0f;
}

void Navigation::printICOValues(std::ofstream& output_stream){
    std::cout << "w_reflex_var: " << w_reflex_var << " -- v_learning rate: " << v_learning << " -- Reflexcounter: " << reflexcounter << std::endl;
    output_stream << w_reflex_var << "," << v_learning << "," << reflexcounter << std::endl;
}

//This is used for debugging and test purposes
void Navigation::consoleControl(Vision * vision_, std::ofstream& output_stream){
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
                    std::cout << "w_reflex_var: " << w_reflex_var << " -- v_learning: " << v_learning << " -- Reflexcounter: " << reflexcounter << std::endl;
                    output_stream << w_reflex_var << "," << v_learning << "," << reflexcounter << std::endl;
                    print_ico = false;
                }
                break;
            case 27: //27 = 'ESC'
                run_bool = false;
                break;
            case 'r':
                run_bool = false;
                break;
			case 'l':
				proactive_nav_ready = true;
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
	//Reflex only looks for nodes in a 90 degree cone towards the front of the robot
	//Instead of the 180 degree half circle avoidance looks in. This is because collision almost only happens in front,
	//and avoidance learns values to make tight turns around obstacles.
	//Otherwise reflex would trigger on otherwise successful avoidance

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
	w_reflex_var = w_reflex_var + reflex_learning_rate * ((REFLEX_THRESHOLD - dist_to_obst_current) / REFLEX_THRESHOLD) * (dist_to_obst_prev_prev - dist_to_obst_prev) / REFLEX_THRESHOLD;
	reflexcounter += 1;

	//Store no motor commands, for path learning
	setMotorCommandsForTrackingNone();
}

void Navigation::obstacleAvoidance(double angle_to_obst, double dist_to_obst_current, double dist_to_obst_prev, double& avoidance_left_o, double& avoidance_right_o)
{
	v_learning = ((AVOIDANCE_THRESHOLD - dist_to_obst_current) / (AVOIDANCE_THRESHOLD - REFLEX_THRESHOLD)) * w_reflex_var +
				 ((AVOIDANCE_THRESHOLD - dist_to_obst_prev) / (AVOIDANCE_THRESHOLD - REFLEX_THRESHOLD)) * w_reflex_novar;

	if (angle_to_obst <= 180) {  //RIGHT SIDE OBSTACLE
		double angle_norm = (angle_to_obst - 90) / 90;
		avoidance_left_o	= v_learning * activationAvoidance(-angle_norm);
		avoidance_right_o	= v_learning * activationAvoidance( angle_norm);
	}
	else { // angle_to_obst > 180 //LEFT SIDE OBSTACLE
		double angle_norm = (90 - (angle_to_obst - 180)) / 90;
		avoidance_left_o	= v_learning * activationAvoidance( angle_norm);
		avoidance_right_o	= v_learning * activationAvoidance(-angle_norm);
	}
}

void Navigation::updateState(states & current_state, int sound_energy_level, double dist_to_obst_current, double narrow_dist_to_obst_current, bool target_found)
{
	//Check if target has been found
    if (target_found ||  current_state == TARGET_FOUND){
		current_state = TARGET_FOUND;
        return;
    }
    //Check for obstacle within reflex threshold
	if (narrow_dist_to_obst_current < REFLEX_THRESHOLD)
	{
		current_state = REFLEX;
	}
	//Check for active sound source, reactive sound navigation towards active sound source
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
        if (dist_to_obst_current < AVOIDANCE_THRESHOLD) {
			current_state = PROACTIVE_NAV_AVOIDANCE;
		}
		else {
			current_state = PROACTIVE_NAVIGATION;
		}
	}
	else {
		current_state = WAIT;
	}
}

void Navigation::displayStateLED(const states current_state)
{
    switch (current_state)
    {
    case WAIT:                      //BLUE
        motor_control->setMatrixVoiceLED(MATRIX_LED_CONTROL, 0, 0, MAX_BRIGHTNESS);
        break;
    case NAVIGATION:                //GREEN
        motor_control->setMatrixVoiceLED(MATRIX_LED_CONTROL, 0, MAX_BRIGHTNESS, 0);
        break;
    case AVOIDANCE:                 //YELLOW
        motor_control->setMatrixVoiceLED(MATRIX_LED_CONTROL, MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0);
        break;
    case REFLEX:                    //RED
        motor_control->setMatrixVoiceLED(MATRIX_LED_CONTROL, MAX_BRIGHTNESS, 0, 0);
        break;
    case TARGET_FOUND:              //WHITE
        motor_control->setMatrixVoiceLED(MATRIX_LED_CONTROL, MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
        break;
    case PROACTIVE_NAVIGATION:      //CYAN
        motor_control->setMatrixVoiceLED(MATRIX_LED_CONTROL, 0, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
        break;
    case PROACTIVE_NAV_AVOIDANCE:   //PURPLE
        motor_control->setMatrixVoiceLED(MATRIX_LED_CONTROL, MAX_BRIGHTNESS, 0, MAX_BRIGHTNESS);
        break;
    default:                        //NONE
        motor_control->setMatrixVoiceLED(MATRIX_LED_CONTROL, 0, 0, 0);
        break;
    }
}

void Navigation::setMotorCommandsForTrackingNone(){
    //Store no motor commands, for path learning
	left_motor_command	= NO_COMMAND_FOR_TRACKING;
	right_motor_command = NO_COMMAND_FOR_TRACKING;
}
