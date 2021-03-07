/*
 * Description:     The main class of Environment navigation using limited sound and machine learning
 *
 * Author:			Erik Lindby
 *					University of Southern Denmark
 * Creation date:   01-02-2021
 */

 /*******************************************************************************
  *******************************************************************************
  * MAIN CLASS
  *******************************************************************************
  ******************************************************************************/
//INCLUDE DEFINES
#include "includes/defines.h"


//INCLUDE CLASSES
#include "includes/soundlocalization.h"
#include "includes/motorcontrol.h"
#include "includes/lidar.h"
#include "includes/icolearning.h"
#include "includes/navigation.h"
#include "includes/odas.h"
#include "includes/vision.h"


//Odas test includes
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <array>
#include <vector>

#include <cmath>
#include <math.h>
#include <ctime>
#include <fstream>
#include <iostream>
//#include <string>
#include <unistd.h>
#include <raspicam/raspicam.h>
//#include <pigpio.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>

#define REFLEX_THRESHOLD	80
#define AVOIDANCE_THRESHOLD	65

#define VELOCITY_OFFSET		12

#define ENERGY_THRESHOLD 30




using namespace std;
//using namespace cv;


double test_angle = 270;
 //odas.updateODAS(); &//dummy: getOdasAngle(); returns double angle
double getODASAngle() {
	double res = test_angle;
	if (test_angle > 180) {
		test_angle -= 10;
	}

	return res;
}

double activation(double input) {
	return 50 / (1 + exp(-input));
}

void braitenberg(double angle, MotorControl * motor_control) { //Braitenberg aggression vehicle
	if (angle < 180) { //Object is on RIGHT side
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, 255, 0);
	}
	else { // angle >= 180 //object is on LEFT side
		motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, 255, 0);
	}

	// Update sensor signals
	double angleL = (((360 - angle) - 180) / 180); // Normalize
	double angleR = (angle-180) / 180; // Normalize

	motor_control->setRightMotorSpeedDirection(activation(angleR) + VELOCITY_OFFSET, 1);
	motor_control->setLeftMotorSpeedDirection(activation(angleL) + VELOCITY_OFFSET, 1);
	//TEST - Print motor values
	std::cout << "Left speed: " << (activation(angleL) + VELOCITY_OFFSET) << " - Right speed: " << (activation(angleR) + VELOCITY_OFFSET) << std::endl;
}

void navigationICO(double angle, MotorControl * motor_control, double w_A) {
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




int main (int argc, char** argv)
{
/*****************************************************************************
**********************   INITIALISE CONTROL OBJECTS   ************************
*****************************************************************************/
	matrix_hal::MatrixIOBus bus;									// Create MatrixIOBus object for hardware communication
	if (!bus.Init())
		throw("Bus Init failed");
	matrix_hal::EverloopImage everloop_image(bus.MatrixLeds());		// Create EverloopImage object "image1d", with size of ledCount
	matrix_hal::Everloop everloop;									// Create Everloop object
	everloop.Setup(&bus);											// Set everloop to use MatrixIOBus bus
	matrix_hal::GPIOControl gpio;									// Create GPIOControl object - General Purpose Input Output
	gpio.Setup(&bus);												// Set gpio to use MatrixIOBus bus

	//Initialise control class instances
	ODAS odas = ODAS(&bus, &everloop, &everloop_image);				//Initialise ODAS, class that handles MATRIX Voice
	MotorControl motor_control = MotorControl(&bus, &everloop, 
		&everloop_image, &gpio);									//Initialise Motor Control
	//Vision vision;

/*****************************************************************************
************************   TEST IMPLEMENTATIONS   ****************************
*****************************************************************************/

	// Wait 3 seconds for camera image to stabilise
	//cout << "Waiting for camera to stabilise...";
	//usleep(3000000);
	//cout << "done." << endl;

	//Create .csv output stream
	//std::ofstream output_stream;
	//output_stream.open("./testdata/ODASbugTest2_class.csv");



/*****************************************************************************
************************   ICO LEARNING   *********************************
*****************************************************************************/
	/*double angle_current = 270.0;
	double angle_prev = 0.0;
	double w_A = 1.0; //weight
	double v_learning = 0.0;
	double S_L = 0.3;*/


/*****************************************************************************
************************   CONTROLLER LOOP   *********************************
*****************************************************************************/
    
	//while(true){
	for(int i = 0; i < 1000;i++){
		odas.updateODAS(/*output_stream*/);
        //vision.updateCamera();

		/*if (odas.getSoundEnergy() > ENERGY_THRESHOLD) {
			braitenberg(odas.getSoundAngle(), &motor_control);
		} else {
			motor_control.setMotorDirection(NONE); //STOPS ALL MOTORS
		}*/

		/*
		w_A = (abs(angle_current - 180) - abs(angle_prev - 180))/180 * S_L + (1 - S_L) * w_A;
        */

	} // End of while loop
/***********************   END OF CONTROLLER LOOP   *************************/

	
	//motor_control.setMotorDirection(NONE);	//STOP ALL MOTORS
	//motor_control.resetMatrixVoiceLEDs();		//RESET ALL LEDS
    //vision.camera->release();					//Release camera resources

	//Test flag
	std::cout << "End of main -------" << std::endl;

	//Close outputstream
	//output_stream.close();





	return 0;
}
