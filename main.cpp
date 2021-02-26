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
//#include "includes/motorcontrol.h"
#include "includes/lidar.h"
#include "includes/icolearning.h"
#include "includes/navigation.h"
#include "includes/odas.h"
#include "includes/vision.h"



#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
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






using namespace std;
//using namespace cv;





int main (int argc, char** argv)
{
/*****************************************************************************
************************   INITIALISE MOTOR CONTROL   ************************
*****************************************************************************/

	//MotorControl motor_control;
	ODAS odas;
	//Vision vision;

/*********************************   DONE   *********************************/


	// Wait 3 seconds for camera image to stabilise
	//cout << "Waiting for camera to stabilise...";
	//usleep(3000000);
	//cout << "done." << endl;


/*****************************************************************************
************************   CONTROLLER LOOP   *********************************
*****************************************************************************/
    odas.updateODAS();
	//while(true)
	//{
		//odas.updateODAS();
        //vision.updateCamera();



	//} // End of while loop
/*********************************   END OF CONTROLLER LOOP   *********************************/

	// Stop all motors
	//motor_control.setRightMotorSpeedDirection(0,1);
	//motor_control.setLeftMotorSpeedDirection(0,1);

	//Test flag
	std::cout << "End of main -------" << std::endl;




	return 0;
}
