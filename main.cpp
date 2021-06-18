/*
 * Description:     The main class of Environment navigation using limited sound and machine learning
 *
 * Author:			Erik Lindby
 *					University of Southern Denmark
 * Creation date:   01-02-2021
 */

// INCLUDE DEFINES
#include "includes/defines.h"

// INCLUDE CLASSES
#include "includes/motorcontrol.h"
#include "includes/lidar.h"
#include "includes/icolearning.h"
#include "includes/navigation.h"
#include "includes/odas.h"
#include "includes/vision.h"
#include "includes/learnedpath.h"

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

//timed includes:
#include <stdio.h>
#include <stdlib.h>
#include <time.h>


using namespace std;
//using namespace cv;

void logData(std::ofstream& output_stream, int current_timestep, int current_iteration, states current_state, bool proactive_nav_ready, int angle_to_sound, int sound_energy, int last_known_angle_to_sound, double angle_to_obst, double dist_to_obst_current, double narrow_angle_to_obst, double narrow_dist_to_obst_current, double left_motor_command_navigation, double right_motor_command_navigation, double avoidance_left, double avoidance_right, double reflex_left, double reflex_right) {
	//Information about controller
	output_stream << current_timestep << "," << current_iteration << "," << current_state << "," << proactive_nav_ready << ",";
	//Sound sensor data
	output_stream << angle_to_sound << "," << sound_energy << "," << last_known_angle_to_sound << ",";
	//Obstacle sensor data
	output_stream << angle_to_obst << "," << dist_to_obst_current << "," << narrow_angle_to_obst << "," << narrow_dist_to_obst_current << ",";
	//Output motor commands
	output_stream << left_motor_command_navigation << "," << right_motor_command_navigation << "," << avoidance_left << "," << avoidance_right << "," << reflex_left << std::endl;

	//LogEverythingPossible 
			//current_timestep
			//current_state
			//current_iteration
			//angle_to_sound
			//sound_energy
			//angle_to_obst
			//dist_to_obst_current
			//narrow_dist_to_obst_current
			//narrow_angle_to_obst
			//Navigation   Indeholder: reactiveNav/proactiveNav/Reflex //wait
			//avoidance
}

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

	//Initialise class instances
	MotorControl motor_control = MotorControl(&bus, &everloop,
		&everloop_image, &gpio);									//Initialise Motor Control - OBS: This constructor has to be called BEFORE the ODAS constructor, initGPIO

    LearnedPathHandler learned_path_handler;

    Navigation navigation = Navigation(&motor_control, &learned_path_handler);	//Initialise Navigation

	ODAS odas(&bus, &everloop, &everloop_image);				    //Initialise ODAS, class that handles MATRIX Voice
	std::thread thread_odas(&ODAS::updateODAS, &odas);				//Start ODAS thread

	LIDAR lidar;                                                    //Initialise LIDAR, class that handles sensor data from RP LIDAR
	std::thread thread_lidar(&LIDAR::scanLIDAR, &lidar);            //Start LIDAR thread
	/*  Lidar needs ~.8 seconds before it can start giving stable readings,
        handled by calling vision constructor afterwards (3s delay to stabilize camera). */

	Vision vision;                                                  //Initialise Vision, class that handles camera, and input
	std::thread thread_vision(&Vision::updateCamera, &vision);      //Start Vision thread

	/*****************************************************************************
	***************************   SETUP VARIABLES   ******************************
	*****************************************************************************/
	//Turn on tracking LED (Red) for video tracking of tests
	motor_control.setMatrixVoiceLED(MATRIX_LED_TRACKING, MAX_BRIGHTNESS, 0, 0);

	//Create .csv output stream
	std::ofstream output_stream;
	std::ofstream output_stream_ICO;
	std::ofstream output_data;
	//output_stream.open("./testdata/ODASbugTest2_class.csv");
	output_stream.open("./testdata/no_test_dummy.csv", std::ofstream::out | std::ofstream::trunc); //Truncate argument deletes previous contents of file

	output_stream_ICO.open("./testdata/ico_dummy.csv", std::ofstream::out | std::ofstream::trunc); //Truncate argument deletes previous contents of file

	output_data.open("./testdata/logdata_dummy.csv", std::ofstream::out | std::ofstream::trunc); //Truncate argument deletes previous contents of file

	//Front 180 degree cone - LIDAR sensor readings for obstacle avoidance
	rplidar_response_measurement_node_hq_t closest_node;
	double dist_to_obst_current		= 1000;	// Distance to closest obstacle on the track
	double angle_to_obst			= 0;	// Angle to closest obstacle for dist_to_obst_current
	double dist_to_obst_prev		= 1000;	// Previous Distance to closest obstacle on the track

	//Front 90 degree cone  - LIDAR sensor readings for obstacle avoidance
	rplidar_response_measurement_node_hq_t narrow_closest_node;
	double narrow_dist_to_obst_current		= 1000;	// Distance to closest obstacle on the track
	double narrow_angle_to_obst				= 0;	// Angle to closest obstacle for dist_to_obst_current
	double narrow_dist_to_obst_prev			= 1000;	// Previous Distance to closest obstacle on the track
	double narrow_dist_to_obst_prev_prev	= 1000;	// Previous Previus Distance to closest obstacle on the track

	//MATRIX Voice sensor readings
	int angle_to_sound			    = 180;
	int sound_energy			    = 0;
	int last_known_angle_to_sound   = 180;

	//Store read motor commands from learned path
	double left_motor_command_learned		= 0;
	double right_motor_command_learned		= 0;
	//Store motor commands for tracking
	double left_motor_command_navigation	= 0;
	double right_motor_command_navigation	= 0;
	double reflex_left						= 0;
	double reflex_right						= 0;

	//Store avoidance motor values, calculated from Navigation::obstacleAvoidance
	double avoidance_left = 0, avoidance_right = 0;

    std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
	while(closest_node.dist_mm_q2 == 0){ //0 is default value of faulty LIDAR readings
            closest_node = lidar.readScan();
            std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
    }


	/*****************************************************************************
	************************   CONTROL LOOP   *********************************
	*****************************************************************************/

	states current_state = WAIT;
	int current_timestep = 0;
    struct timespec start_time, end_time;
    long elapsed_time_us, time_to_sleep_us;
	int current_iteration = 0;

	while(true)
	{
		clock_gettime(CLOCK_MONOTONIC, &start_time); //Start time for timestep

		//Read LIDAR sensor data
		closest_node		= lidar.readScan();			//Reads closest node in the front 180 degrees of robot
		narrow_closest_node = lidar.readScanNarrow();	//Reads closest node in the front 90 degrees of robot
		dist_to_obst_prev		= dist_to_obst_current;
		dist_to_obst_current	= closest_node.dist_mm_q2 / 4.0f;
		angle_to_obst			= lidar.getCorrectedAngle(closest_node);
		narrow_dist_to_obst_prev_prev	= narrow_dist_to_obst_prev;
		narrow_dist_to_obst_prev		= narrow_dist_to_obst_current;
		narrow_dist_to_obst_current		= narrow_closest_node.dist_mm_q2 / 4.0f;
		narrow_angle_to_obst			= lidar.getCorrectedAngle(narrow_closest_node);

		//Read MATRIX Voice sensor data
		angle_to_sound	= odas.getSoundAngle();
		sound_energy	= odas.getSoundEnergy();
		if(sound_energy > ENERGY_THRESHOLD)
            last_known_angle_to_sound = angle_to_sound;

        if(vision.k == 98){ //98 = 'b'
            current_state = TARGET_FOUND;
		}

		//Reset avoidance (for tracking)
		avoidance_left = 0, avoidance_right = 0, reflex_left = 0, reflex_right = 0;

		//Update state
		navigation.updateState(current_state, sound_energy, dist_to_obst_current, narrow_dist_to_obst_current, vision.getTargetFound());

		switch (current_state)
		{
		case WAIT:
			motor_control.setMotorDirection(STOP);		//STOP ALL MOTORS
			navigation.setMotorCommandsForTrackingNone(); //Set commands for tracking for current timestep to none
			break;
		case NAVIGATION:
			navigation.reactiveSoundNavigation(angle_to_sound, output_stream, 0, 0);
			break;
		case AVOIDANCE:
			navigation.obstacleAvoidance(angle_to_obst, dist_to_obst_current, dist_to_obst_prev, avoidance_left, avoidance_right);
			navigation.reactiveSoundNavigation(angle_to_sound, output_stream, avoidance_left, avoidance_right);
			break;
		case REFLEX:
			navigation.obstacleReflex(narrow_angle_to_obst, narrow_dist_to_obst_current, narrow_dist_to_obst_prev, narrow_dist_to_obst_prev_prev, reflex_left, reflex_right);
			break;
		case TARGET_FOUND:
			motor_control.setMotorDirection(STOP);		//STOP ALL MOTORS
			navigation.setMotorCommandsForTrackingNone(); //Set commands for tracking for current timestep to none
			// Start new ICO Learning iteration
			if(vision.k == 115){ //115 = 's'
				learned_path_handler.startNewPath();
				current_timestep = 0;
				vision.resetTargetFound();
				current_state = WAIT;//REMOVE THIS AFTER DEBUGGING
				navigation.printICOValues(output_stream_ICO);
				current_iteration++;
            }
			// Start new proactive navigation iteration
			if (vision.k == 116) { //116 = 't'
				learned_path_handler.startNewPath();
				current_timestep = 0;
				vision.resetTargetFound();
				current_state = WAIT;//REMOVE THIS AFTER DEBUGGING
				navigation.printICOValues(output_stream_ICO);
				navigation.proactive_nav_ready = true;
				current_iteration++;
			}
			break;
        case PROACTIVE_NAVIGATION:
//			if (learned_path_handler.learned_paths[0].timesteps_tracked < current_timestep)
//			{
//				current_state = TARGET_FOUND;
//				break;
//			}
			learned_path_handler.getLearnedCommands(current_timestep, left_motor_command_learned, right_motor_command_learned,
                                                    last_known_angle_to_sound, angle_to_obst, dist_to_obst_current);
			motor_control.setLeftMotorSpeedOnly(left_motor_command_learned);
			motor_control.setRightMotorSpeedOnly(right_motor_command_learned);
			navigation.setMotorCommandsForTracking(left_command_learned, right_command_learned); //For tracking
            break;
        case PROACTIVE_NAV_AVOIDANCE:
//			if (learned_path_handler.learned_paths[0].timesteps_tracked < current_timestep)
//			{
//				current_state = TARGET_FOUND;
//				break;
//			}
			navigation.obstacleAvoidance(angle_to_obst, dist_to_obst_current, dist_to_obst_prev, avoidance_left, avoidance_right);
			learned_path_handler.getLearnedCommands(current_timestep, left_motor_command_learned, right_motor_command_learned,
                                                    last_known_angle_to_sound, angle_to_obst, dist_to_obst_current);
			motor_control.setLeftMotorSpeedOnly(left_motor_command_learned + avoidance_left);
			motor_control.setRightMotorSpeedOnly(right_motor_command_learned + avoidance_right);
			navigation.setMotorCommandsForTracking(left_command_learned, right_command_learned); //For tracking
            break;
		default:
			break;
		}

		navigation.displayStateLED(current_state);

		if ((current_state != TARGET_FOUND) && (current_state != WAIT) && (current_state != REFLEX) && navigation.proactive_nav_ready != true)
		{
			learned_path_handler.handlerTrackPath(navigation.left_motor_command, navigation.right_motor_command, angle_to_sound, angle_to_obst, dist_to_obst_current);
            current_timestep++;
		}

		//Track data for experiment examination
		logData(output_data, current_timestep, current_iteration, current_state, navigation.proactive_nav_ready, angle_to_sound, sound_energy, last_known_angle_to_sound, angle_to_obst, dist_to_obst_current, narrow_angle_to_obst, narrow_dist_to_obst_current, navigation.left_motor_command, navigation.right_motor_command, avoidance_left, avoidance_right, reflex_left, reflex_right);

		//std::cout << "90deg dist/angle: " << narrow_dist_to_obst_current << " | " << narrow_angle_to_obst << std::endl;
		//std::cout << "180deg dist/angle: " << dist_to_obst_current << " | " << angle_to_obst << std::endl;

        clock_gettime(CLOCK_MONOTONIC, &end_time); //End time for timestep (Before wait)
        elapsed_time_us = ((long)end_time.tv_sec - (long)start_time.tv_sec) * (long)1e+6 + ((long)end_time.tv_nsec - (long)start_time.tv_nsec) / 1000; //Compute elapsed time in microsec [us]
        time_to_sleep_us = 100000 - elapsed_time_us; //Calculate how long to sleep, remainder of timestep interval
		if (time_to_sleep_us > 0) //Skip sleep if timestep has been exceeded
		{
			usleep(time_to_sleep_us); //[microsec]
		}
		else
		{
			std::cout << "Timestep exceeded: " << -time_to_sleep_us << " microsec overtime." << std::endl;
		}

		//std::cout << "Sound energy " << odas.getSoundEnergy() << "\nWAIT = 0, NAVIGATE = 1, AVOID = 2, REFLEX = 3, TARGET_FOUND = 4. ";
		//std::cout << "Current state:  " << current_state << "  Timestep: "<< current_timestep << " Elapsed_time: " << elapsed_time_us << std::endl;

		//Check Vision thread waitkey - exit or manual steering
		if(vision.k == 112){ //112 = 'p'
            navigation.consoleControl(&vision, output_stream);
			current_iteration++;
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
	output_stream_ICO.close();

	//Test flag
	std::cout << "End of main -------" << std::endl;

	lidar.ctrlc(0);
    thread_lidar.join();
	std::cout << "LIDAR thread terminated!" << std::endl;

	thread_odas.join();
	std::cout << "ODAS thread joined" << std::endl;
	motor_control.resetMatrixVoiceLEDs();		//RESET ALL LEDS, repeated because ODAS thread might set an LED value.

	return 0;
}
