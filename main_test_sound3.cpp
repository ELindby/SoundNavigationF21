/*
 * Description:     
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




using namespace std;
//using namespace cv;



int main(int argc, char** argv)
{
	/*****************************************************************************
	************************   INITIALISE MOTOR CONTROL   ************************
	*****************************************************************************/

	//MotorControl motor_control;
	//ODAS odas = ODAS();
	//Vision vision;

	/*********************************   DONE   *********************************/

	matrix_hal::MatrixIOBus bus;				// Create MatrixIOBus object for hardware communication
	if (!bus.Init())
		throw("Bus Init failed");
	//return false;


	matrix_hal::EverloopImage image1d(bus.MatrixLeds());			// Create EverloopImage object "image1d", with size of ledCount
	matrix_hal::Everloop everloop;				// Create Everloop object
	everloop.Setup(&bus);

	ODAS odas = ODAS(&everloop,&image1d);
	


	/*****************************************************************************
	************************   CONTROLLER LOOP   *********************************
	*****************************************************************************/
	//odas.updateODAS(); &//dummy: getOdasAngle(); returns double angle
	//vision.updateCamera();
	//double soundAngle = 0;
	//std::vector<int> energy_array;
	odas.updateODAS(&bus, &everloop, &image1d);
	//while(true)
	
	//} // End of while loop
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
