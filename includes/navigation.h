#pragma once
/*
 * Description:     Navigation class
 *                  Handles navigation methods and information.
 *
 * Author:			Erik Lindby
 *					University of Southern Denmark
 * Creation date:   12-02-2021
 */

#include "motorcontrol.h"
#include "learnedpath.h"
#include "vision.h"
#include "defines.h"
#include <math.h>
#include <fstream>

#define VELOCITY_OFFSET		12

//Navigation states, as defined in state machine
enum states { WAIT = 0, NAVIGATION = 1, AVOIDANCE = 2, REFLEX = 3, TARGET_FOUND = 4, PROACTIVE_NAVIGATION = 5, PROACTIVE_NAV_AVOIDANCE = 6};

class Navigation
{
private:
	MotorControl * motor_control; //Pointer to motor control, used so navigation can pass motor commands to the motor control object created in main
	LearnedPathHandler * learned_path_handler; //Pointer to learned path handler (Where learned paths are stored)

	//double * angle_pointer;
	double w_reflex_var = 9.26925;//9.0;		// Standard weight that needs to be multiplied with distance to current Obstacle
	double w_reflex_novar = 9.0;		//

	double reflex_learning_rate = 10;	// Learning rate for reflex µ
	double v_learning = 0.0; 		// Velocity to add to the initial velocity
	int reflexcounter = 0;//0;

	double activation(double input);		//Activation function for reactive sound navigation, and obstacle reflex
	double activationAvoidance(double input);		//Activation function for obstacle avoidance
public:
	Navigation(MotorControl * motor_control_, LearnedPathHandler * learned_path_handler_);
	~Navigation();

	bool proactive_nav_ready = false; //If the necessary behaviour needed to use proactive nagivation, this is set to true. Used for updateState

	void reactiveSoundNavigation(double angle, std::ofstream& output_stream, double avoidance_left, double avoidance_right); //Braitenberg-like reactive navigation
	void setBraitenbergLEDs(int direction);

	void printICOValues(std::ofstream& output_stream);

	void consoleControl(Vision * vision_, std::ofstream& output_stream);

	void obstacleReflex(double angle_to_obst, double dist_to_obst_current, double dist_to_obst_prev, double dist_to_obst_prev_prev, double& reflex_left, double& reflex_right);
	void obstacleAvoidance(double angle_to_obst, double dist_to_obst_current, double dist_to_obst_prev, double& avoidance_left_o, double& avoidance_right_o);

	void updateState(states & current_state, int sound_energy_level, double dist_to_obst_current, double narrow_dist_to_obst_current, bool target_found);

	void displayStateLED(const states current_state);

	//Store last issued motor commands, for path learning
	double left_motor_command = 0;
	double right_motor_command = 0;

	void setMotorCommandsForTrackingNone();
	void setMotorCommandsForTracking(double left_motor_command_navigation, double right_motor_command_navigation)
};
