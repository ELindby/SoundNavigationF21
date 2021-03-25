#pragma once
/*
 * Description:     Navigation class
 *
 * Author:			Erik Lindby
 *					University of Southern Denmark
 * Creation date:   12-02-2021
 */

#include "motorcontrol.h"
#include "vision.h"
#include <math.h>
#include <fstream>

#define VELOCITY_OFFSET		12


class Navigation
{
private:
	MotorControl * motor_control;


	//double * angle_pointer;


public:
	double activation(double input);		//Activation function
	Navigation(MotorControl * motor_control_);
	~Navigation();

	void braitenberg(double angle, std::ofstream& output_stream, double avoidance_left, double avoidance_right); //Braitenberg vehicle
	void setBraitenbergLEDs(int direction);

	void navigationICO(double angle, double w_A);

	void manualInputSteering(Vision * vision_);
};
