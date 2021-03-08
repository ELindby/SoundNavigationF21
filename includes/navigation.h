#pragma once
/*
 * Description:     Navigation class
 *
 * Author:			Erik Lindby
 *					University of Southern Denmark
 * Creation date:   12-02-2021
 */

#include "includes/motorcontrol.h"
#include <math.h>

#define VELOCITY_OFFSET		12


class Navigation
{
private:
	MotorControl * motor_control;

	double activation(double input);		//Activation function
	//double * angle_pointer;

	
public:
	Navigation(MotorControl * motor_control_);
	~Navigation();

	void braitenberg(double angle, std::ofstream& output_stream); //Braitenberg vehicle
	void setBraitenbergLEDs(int direction)

	void navigationICO(double angle, double w_A)
};
