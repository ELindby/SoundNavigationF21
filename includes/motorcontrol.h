#pragma once
/*
 * Description:     Motor and LED control
 *
 * Author:			Erik Lindby
 *					University of Southern Denmark
 * Creation date:   12-02-2021
 */
#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <unistd.h>

#include "defines.h"


enum DIRECTIONS { STOP = 0, FORWARD = 1, BACKWARD = 2, LEFT = 3, RIGHT = 4};

class MotorControl {
private:
	matrix_hal::MatrixIOBus* bus;				// Create MatrixIOBus object for hardware communication
	int ledCount;
	matrix_hal::EverloopImage* everloop_image;	// Create EverloopImage object, with size of ledCount
	matrix_hal::Everloop* everloop;				// Create Everloop object
	matrix_hal::GPIOControl* gpio;				// Create GPIOControl object - General Purpose Input Output
public:
	MotorControl(matrix_hal::MatrixIOBus* bus_, matrix_hal::Everloop* everloop_, matrix_hal::EverloopImage* everloop_image_, matrix_hal::GPIOControl* gpio_);
	~MotorControl();


	void initGPIOPins();
	void setLeftMotorSpeedDirection(int speed, int dir);
	void setRightMotorSpeedDirection(int speed, int dir);
	void startupShowLEDRainbow();
	void setMatrixVoiceLED(int ledn, int r, int g, int b);
	void resetMatrixVoiceLEDs();
	void setMotorDirection(int direction);
};
