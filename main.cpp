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
//#include "includes/defines.h"


//INCLUDE CLASSES
#include "includes/soundlocalization.h"
#include "includes/motorcontrol.h"
#include "includes/lidar.h"
#include "includes/icolearning.h"
#include "includes/navigation.h"
#include "includes/odas.h"
#include "includes/vision.h"
#include "includes/lidar.h"


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
#include <string>
#include <sstream>
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

#define ENERGY_THRESHOLD 30


using namespace std;
//using namespace cv;

int main (int argc, char** argv)
{
	/*****************************************************************************
	************************   INITIALISE MATRIXIO  ************************
	*****************************************************************************/
	matrix_hal::MatrixIOBus bus;									// Create MatrixIOBus object for hardware communication
	if (!bus.Init())												// Set gpio to use MatrixIOBus bus
		throw("Bus Init failed");
	matrix_hal::EverloopImage everloop_image(bus.MatrixLeds());		// Create EverloopImage object "image1d", with size of ledCount
	matrix_hal::Everloop everloop;									// Create Everloop object
	everloop.Setup(&bus);											// Set everloop to use MatrixIOBus bus
	matrix_hal::GPIOControl gpio;									// Create GPIOControl object - General Purpose Input Output
	gpio.Setup(&bus);
	/*****************************************************************************
	************************   INITIALISE CLASSES  ************************
	*****************************************************************************/
	MotorControl motor_control(&bus, &everloop, &everloop_image, &gpio);
	//ODAS soundLocalization(&bus, &everloop, &everloop_image);
	navigation navigation(&motor_control);


	

	LIDAR lidar;
	std::thread thread_LIDAR(&LIDAR::scanLIDAR,
		&lidar);




	//Vision vision;

	char k;



	//while(true){
	for (int i = 0; i < 1000; i++) {
		rplidar_response_measurement_node_hq_t closestNode = lidar.readScan();
		std::cout << "Nearest distance to obstacle: " << closestNode.dist_mm_q2 / 4.0f << " Angle: " << closestNode.angle_z_q14 * 90.f / (1 << 14) << std::endl;

		usleep(100000);

		//	//odas.updateODAS();
		//	//motor_control.setMatrixVoiceLED(MATRIX_LED_L_9, MAX_BRIGHTNESS, 0, 0);


		//	if (soundLocalization.getEnergy() > ENERGY_THRESHOLD) {
		//		navigation.braitenberg(soundLocalization.getSoundAngle(), outputStream);
		//	}
		//	else {
		//		motorControl.changeMotorCommand(STOP); //STOPS ALL MOTORS
		//	}

		//	vision.updateCamera();
		//	k = cv::waitKey(10);
		//	if (k == 27) //27 = 'ESC'
		//		break;
	}

	/*********************************   END OF CONTROLLER LOOP   *********************************/

	motor_control.changeMotorCommand(STOP);		//STOP ALL MOTORS
	motor_control.resetMatrixVoiceLEDs();		//RESET ALL LEDS
	//vision.releaseCamera();						//Release camera resources

	//Test flag
	std::cout << "End of main -------" << std::endl;

	//threadOdas.join();
	//std::cout << "Odas thread joined" << std::endl;
	lidar.ctrlc();
	threadLIDAR.~thread();

	std::cout << "LIDAR thread terminated!" << std::endl;

	motor_control.resetMatrixVoiceLEDs();		//RESET ALL LEDS

	//outputStream.close();
	//vision.releaseCamera();
	std::cout << "End of main -------" << std::endl;

	return 0;
}
