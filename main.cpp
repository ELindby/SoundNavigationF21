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
#include <thread>
#include <mutex>

//Sound navigation threshold
#define ENERGY_THRESHOLD 30

//Obstacle avoidance
#define REFLEX_THRESHOLD 400
#define AVOIDANCE_THRESHOLD 600


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

	Vision vision;                                                  //Initialise Vision, class that handles camera, and input
	std::thread thread_vision(&Vision::updateCamera, &vision);      //Start Vision thread

	LIDAR lidar;
	std::thread thread_lidar(&LIDAR::scanLIDAR, &lidar);
	//Lidar needs ~.8 seconds before it can start giving readings. This is handled by calling vision constructor afterwards (With 3s delay to stabilize camera)


	/*****************************************************************************
	************************   TEST IMPLEMENTATIONS   ****************************
	*****************************************************************************/
	//Turn on tracking LED (Red) for video tracking of tests
	motor_control.setMatrixVoiceLED(MATRIX_LED_L_9, MAX_BRIGHTNESS, 0, 0);

	//Create .csv output stream
	std::ofstream output_stream;
	//output_stream.open("./testdata/ODASbugTest2_class.csv");
	output_stream.open("./testdata/no_test_dummy.csv", std::ofstream::out | std::ofstream::trunc); //Truncate argument deletes previous contents of file

	//Obstacle avoidance / ICO Learning
	double dist_to_obst_current = 1000;		// Distance to closest obstacle on the track
	double angle_to_obst = 0;
	double dist_to_obst_prev;				// Previous Distance to closest obstacle on the track

	double dist_to_obst_prev_prev = 35.0;	// Previous Previus Distance to closest obstacle on the track

	double w_reflex_var = 1.0;		// Standard weight that needs to be multiplied with distance to current Obstacle
	double w_reflex_novar = 1.0;		//

	double reflex_learning_rate = 10;	// Learning rate for reflex µ
	double v_learning = 0.0; 		// Velocity to add to the initial velocity
	int reflexcounter = 0;


    rplidar_response_measurement_node_hq_t closest_node = lidar.readScan();
    std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
	while(closest_node.dist_mm_q2 == 0){
            closest_node = lidar.readScan();
            std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
    }


	/*****************************************************************************
	************************   CONTROLLER LOOP   *********************************
	*****************************************************************************/


	while(true){
	//for (int i = 0; i < 1000; i++) {
        closest_node = lidar.readScan();
		std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << std::endl;



		dist_to_obst_prev_prev	= dist_to_obst_prev;
		dist_to_obst_prev		= dist_to_obst_current;
		dist_to_obst_current	= closest_node.dist_mm_q2 / 4.0f;
		angle_to_obst			= lidar.getCorrectedAngle(closest_node);

		motor_control.setMatrixVoiceLED(MATRIX_LED_L_9, MAX_BRIGHTNESS, 0, 0);


		//REFLEX
		if (dist_to_obst_current < REFLEX_THRESHOLD)
		{
			//LEFT OR RIGHT REFLEX DODGING
			if (angle_to_obst <= 180){ //RIGHT SIDE OBSTACLE
				double angle_norm = (angle_to_obst - 90) / 90;
				motor_control.setRightMotorSpeedOnly(navigation.activation(angle_norm));
				motor_control.setLeftMotorSpeedOnly(navigation.activation(-angle_norm));

			} else { // angle_to_obst > 180 //LEFT SIDE OBSTACLE
				double angle_norm = (90 - (angle_to_obst - 180)) / 90;
				motor_control.setRightMotorSpeedOnly(navigation.activation(-angle_norm));
				motor_control.setLeftMotorSpeedOnly(navigation.activation(angle_norm));
			}
			//Update weight used for v_learning
			w_reflex_var = w_reflex_var + reflex_learning_rate * (dist_to_obst_current / REFLEX_THRESHOLD) * (dist_to_obst_prev - dist_to_obst_prev_prev) / REFLEX_THRESHOLD;
			reflexcounter += 1;
		}
		else if (odas.getSoundEnergy() > ENERGY_THRESHOLD) {
			if (dist_to_obst_current < AVOIDANCE_THRESHOLD){
				v_learning = (dist_to_obst_current / REFLEX_THRESHOLD) * w_reflex_var + (dist_to_obst_prev / REFLEX_THRESHOLD) * w_reflex_novar;
				if (angle_to_obst <= 180){  //RIGHT SIDE OBSTACLE
					navigation.braitenberg(odas.getSoundAngle(), output_stream, 0, v_learning);
				}else { // angle_to_obst > 180 //LEFT SIDE OBSTACLE
					navigation.braitenberg(odas.getSoundAngle(), output_stream, v_learning, 0);
				}
			} else{
				navigation.braitenberg(odas.getSoundAngle(), output_stream, 0, 0);
			}

		}
		else {
			motor_control.setMotorDirection(STOP);		//STOP ALL MOTORS
		}

		usleep(100000);

		//Check Vision thread waitkey - exit or manual steering
		if(vision.k == 112){ //112 = 'p'
            navigation.manualInputSteering(&vision);
		}
		if(vision.k == 27){ //27 = 'ESC'
            std::cout << "Vision thread joining...";
            thread_vision.join();
            std::cout << "done"  << std::endl;
            break;
		}
	}

	/*********************************   END OF CONTROLLER LOOP   *********************************/

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
