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



//#include <cmath>
#include <math.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
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
************************   INITIALISE MOTOR CONTROL   ************************
*****************************************************************************/

	//MotorControl motor_control;
	ODAS odas = ODAS();
	//Vision vision;

/*********************************   DONE   *********************************/


	// Wait 3 seconds for camera image to stabilise
	//cout << "Waiting for camera to stabilise...";
	//usleep(3000000);
	//cout << "done." << endl;


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
    //odas.updateODAS(); &//dummy: getOdasAngle(); returns double angle
	//vision.updateCamera();
    //double soundAngle = 0;
    //std::vector<int> energy_array;
	//while(true)
	for(int i = 0; i < 15;i++)
	{
		odas.updateODAS();
        //if(odas.getSoundAngle() != soundAngle){
        //    soundAngle = odas.getSoundAngle();
        //    std::cout << "Angle: " << soundAngle << std::endl;
		//}
		/*energy_array = odas.getEnergyArray();
		for(int i = 0; i < 16;i++){
            printf("%*d",4,energy_array[i]);
		}
		std::cout << std::endl;*/

        //vision.updateCamera();
		/*angle_prev = angle_current;
		angle_current = getODASAngle();

		braitenberg(angle_current,&motor_control);
		usleep(500000);

		w_A = (abs(angle_current - 180) - abs(angle_prev - 180))/180 * S_L + (1 - S_L) * w_A;
        */




	} // End of while loop
/*********************************   END OF CONTROLLER LOOP   *********************************/

	// Stop all motors
	//motor_control.setRightMotorSpeedDirection(0,1);
	//motor_control.setLeftMotorSpeedDirection(0,1);
	//motor_control.resetMatrixVoiceLEDs();
    //Release camera resources
    //vision.camera->release();

	//Test flag
	std::cout << "End of main -------" << std::endl;




	return 0;
}
