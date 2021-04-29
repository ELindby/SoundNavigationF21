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
// INCLUDE DEFINES
#include "includes/defines.h"

// INCLUDE CLASSES
#include "includes/motorcontrol.h"
#include "includes/lidar.h"
#include "includes/icolearning.h"
#include "includes/navigation.h"
#include "includes/odas.h"
#include "includes/vision.h"

// ODAS includes
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <array>
#include <vector>

// OpenCV includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <raspicam/raspicam.h>

// MATRIX HAL includes
#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>


#include <cmath>
#include <math.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>
//#include <pigpio.h>
#include <thread>
#include <mutex>


using namespace std;
//using namespace cv;

int main (int argc, char** argv)
{
	/*****************************************************************************
	**********************   INITIALISE CONTROL OBJECTS   ************************
	*****************************************************************************/
	matrix_hal::MatrixIOBus bus;									// Create MatrixIOBus object for hardware communication, aborts (terminate char const) if matrix device isnt connected properly
	if (!bus.Init())
		throw("Bus Init failed");
	matrix_hal::EverloopImage everloop_image(bus.MatrixLeds());		// Create EverloopImage object "image1d", with size of ledCount
	matrix_hal::Everloop everloop;									// Create Everloop object
	everloop.Setup(&bus);											// Set everloop to use MatrixIOBus bus
	matrix_hal::GPIOControl gpio;									// Create GPIOControl object - General Purpose Input Output
	gpio.Setup(&bus);												// Set gpio to use MatrixIOBus bus

	//Initialise control class instances
	MotorControl motor_control = MotorControl(&bus, &everloop,
		&everloop_image, &gpio);									//Initialise Motor Control - OBS: This constructor has to be called BEFORE the ODAS constructor, initGPIO
    Navigation navigation = Navigation(&motor_control);				//Initialise Navigation

	ODAS odas(&bus, &everloop, &everloop_image);				    //Initialise ODAS, class that handles MATRIX Voice
	std::thread thread_odas(&ODAS::updateODAS, &odas);				//Start ODAS thread

	LIDAR lidar;                                                    //Initialise LIDAR, class that handles sensor data from RP LIDAR
	std::thread thread_lidar(&LIDAR::scanLIDAR, &lidar);            //Start LIDAR thread
	/*  Lidar needs ~.8 seconds before it can start giving stable readings,
        handled by calling vision constructor afterwards (3s delay to stabilize camera). */

	Vision vision;                                                  //Initialise Vision, class that handles camera, and input
	std::thread thread_vision(&Vision::updateCamera, &vision);      //Start Vision thread




	/*****************************************************************************
	************************   TEST IMPLEMENTATIONS   ****************************
	*****************************************************************************/
	//Turn on tracking LED (Red) for video tracking of tests
	motor_control.setMatrixVoiceLED(MATRIX_LED_TRACKING, MAX_BRIGHTNESS, 0, 0);

	//Create .csv output stream
	std::ofstream output_stream;
	//output_stream.open("./testdata/ODASbugTest2_class.csv");
	output_stream.open("./testdata/icolearningvalues_obstacleavoidance_test1.csv", std::ofstream::out | std::ofstream::trunc); //Truncate argument deletes previous contents of file

	//Obstacle avoidance / ICO Learning
	double dist_to_obst_current = 1000;		// Distance to closest obstacle on the track
	double angle_to_obst = 0;				// Angle to closest obstacle for dist_to_obst_current
	double dist_to_obst_prev;				// Previous Distance to closest obstacle on the track
	double dist_to_obst_prev_prev = 35.0;	// Previous Previus Distance to closest obstacle on the track


    rplidar_response_measurement_node_hq_t closest_node = lidar.readScan();
    std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
	while(closest_node.dist_mm_q2 == 0){ //0 is default value of faulty LIDAR readings
            closest_node = lidar.readScan();
            std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
    }


	/*****************************************************************************
	************************   CONTROL LOOP   *********************************
	*****************************************************************************/

	states current_state = WAIT;

	while(true){
	//for (int i = 0; i < 1000; i++) {
        closest_node = lidar.readScan();
		std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << std::endl;

		dist_to_obst_prev_prev	= dist_to_obst_prev;
		dist_to_obst_prev		= dist_to_obst_current;
		dist_to_obst_current	= closest_node.dist_mm_q2 / 4.0f;
		angle_to_obst			= lidar.getCorrectedAngle(closest_node);

		navigation.updateState(current_state, dist_to_obst_current, odas.getSoundEnergy());

		switch (current_state)
		{
		case WAIT:
			motor_control.setMatrixVoiceLED(MATRIX_LED_CONTROL, 0, 0, MAX_BRIGHTNESS); //BLUE
			motor_control.setMotorDirection(STOP);		//STOP ALL MOTORS
			break;
		case NAVIGATION:
			motor_control.setMatrixVoiceLED(MATRIX_LED_CONTROL, 0, MAX_BRIGHTNESS, 0); //GREEN
			navigation.braitenberg(odas.getSoundAngle(), output_stream, 0, 0);
			break;
		case AVOIDANCE:
			motor_control.setMatrixVoiceLED(MATRIX_LED_CONTROL, MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0); //YELLOW
			std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << std::endl;
			navigation.obstacleAvoidance(angle_to_obst, dist_to_obst_current, dist_to_obst_prev, odas.getSoundAngle(), output_stream);
			break;
		case REFLEX:
			motor_control.setMatrixVoiceLED(MATRIX_LED_CONTROL, MAX_BRIGHTNESS, 0, 0); //RED
			std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << std::endl;
			navigation.obstacleReflex(angle_to_obst, dist_to_obst_current, dist_to_obst_prev, dist_to_obst_prev_prev);
			break;
		case TARGET_FOUND:
			motor_control.setMatrixVoiceLED(MATRIX_LED_CONTROL, MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS); //WHITE
			motor_control.setMotorDirection(STOP);		//STOP ALL MOTORS
			//validate target with sound information
			// Reset state (Start new iteration)
			if(vision.k == 115){ //115 = 's'
                current_state = WAIT;
            }
			break;
        case PROACTIVE_NAVIGATION:
            //Todo: Add proactive navigation based on learned paths here.
            motor_control.setMatrixVoiceLED(MATRIX_LED_CONTROL, 0, MAX_BRIGHTNESS, MAX_BRIGHTNESS); //CYAN
            break;
		default:
			break;
		}

		usleep(100000); //Todo: Clock this to be remainder of timestep since last wait, to effectively clock the process.

		//Check Vision thread waitkey - exit or manual steering
		if(vision.k == 112){ //112 = 'p'
            navigation.manualInputSteering(&vision, output_stream);
		}
		if(vision.k == 27){ //27 = 'ESC'
            std::cout << "Vision thread joining...";
            thread_vision.join();
            std::cout << "done"  << std::endl;
            break;
		}
	}

	/*********************************   END OF CONTROL LOOP   *********************************/

	motor_control.setMotorDirection(STOP);		//STOP ALL MOTORS
	motor_control.resetMatrixVoiceLEDs();		//RESET ALL LEDS
	//vision.releaseCamera();					//Release camera resources, should be handled in vision thread

	//Close outputstream
	output_stream.close();

	//Test flag
	std::cout << "End of main -------" << std::endl;

	lidar.ctrlc(0);
    thread_lidar.join();
	std::cout << "LIDAR thread terminated!" << std::endl;

	thread_odas.join();
	std::cout << "ODAS thread joined" << std::endl;
	motor_control.resetMatrixVoiceLEDs();		//RESET ALL LEDS

	return 0;
}
