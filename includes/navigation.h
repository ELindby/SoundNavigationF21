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
#include "vision.h"
#include "defines.h"
#include <math.h>
#include <fstream>

#define VELOCITY_OFFSET		12

//Navigation states, as defined in state machine
enum states { WAIT = 0, NAVIGATION = 1, AVOIDANCE = 2, REFLEX = 3, TARGET_FOUND = 4, PROACTIVE_NAVIGATION = 5 };

class Navigation
{
private:
	MotorControl * motor_control;


	//double * angle_pointer;
	double w_reflex_var = 1.0;		// Standard weight that needs to be multiplied with distance to current Obstacle
	double w_reflex_novar = 1.0;		//

	double reflex_learning_rate = 10;	// Learning rate for reflex µ
	double v_learning = 0.0; 		// Velocity to add to the initial velocity
	int reflexcounter = 0;


public:
    bool proactive_nav_ready = false; //If the necessary behaviour needed to use proactive nagivation, this is set to true. Used for updateState

	double activation(double input);		//Activation function
	Navigation(MotorControl * motor_control_);
	~Navigation();

	void braitenberg(double angle, std::ofstream& output_stream, double avoidance_left, double avoidance_right); //Braitenberg vehicle
	void setBraitenbergLEDs(int direction);

	void navigationICO(double angle, double w_A);

	void manualInputSteering(Vision * vision_, std::ofstream& output_stream);

	void obstacleReflex(double angle_to_obst, double dist_to_obst_current, double dist_to_obst_prev, double dist_to_obst_prev_prev);
	void obstacleAvoidance(double angle_to_obst, double dist_to_obst_current, double dist_to_obst_prev, double angle_to_sound, std::ofstream& output_stream);
	void updateState(states & current_state, double dist_to_obst_current, int sound_energy_level);


};
