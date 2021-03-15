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

#define ENERGY_THRESHOLD 30


using namespace std;
//using namespace cv;

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
	//Vision vision;
	MotorControl motor_control = MotorControl(&bus, &everloop,
		&everloop_image, &gpio);									//Initialise Motor Control - OBS: This constructor has to be called BEFORE the ODAS constructor, initGPIO
    ODAS odas = ODAS(&bus, &everloop, &everloop_image);				//Initialise ODAS, class that handles MATRIX Voice
	Navigation navigation = Navigation(&motor_control);				//Initialise Navigation


/*****************************************************************************
************************   TEST IMPLEMENTATIONS   ****************************
*****************************************************************************/

	// Wait 3 seconds for camera image to stabilise
	//cout << "Waiting for camera to stabilise...";
	//usleep(3000000);
	//cout << "done." << endl;
	//char k;

	//Turn on tracking LED (Red) for video tracking of tests
	motor_control.setMatrixVoiceLED(MATRIX_LED_L_9, MAX_BRIGHTNESS, 0, 0);

	//Create .csv output stream
	std::ofstream output_stream;
	//output_stream.open("./testdata/ODASbugTest2_class.csv");
	output_stream.open("./testdata/no_test_dummy.csv", std::ofstream::out | std::ofstream::trunc); //Truncate argument deletes previous contents of file

	//Read motor commands from .csv file
	/*std::ifstream input_stream;
	input_stream.open("./testdata/Test1pos_13.csv");
	if (!input_stream.is_open()){
		std::cout << "ERROR OPENING MOTOR COMMANDS .csv FILE" << std::endl;
	}

	std::vector<double> motorcommands_left;
	std::vector<double> motorcommands_right;

	std::vector<std::string> string_vec;
	std::string line;
	while (std::getline(input_stream, line)){
		std::istringstream iss(line);
		std::string line_stream;
		std::string::size_type sz;
		std::vector<double> row;
		while (std::getline(iss, line_stream, ',')){
			row.push_back(stod(line_stream, &sz)); // convert to double
		}
		motorcommands_left.push_back(row[0]);
		motorcommands_right.push_back(row[1]);
	}*/



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
	for(int i = 0; i < 100000;i++){
		odas.updateODAS();
		motor_control.setMatrixVoiceLED(MATRIX_LED_L_9, MAX_BRIGHTNESS, 0, 0);


		if (odas.getSoundEnergy() > ENERGY_THRESHOLD) {
			navigation.braitenberg(odas.getSoundAngle(), output_stream);
		} else {
			motor_control.setMotorDirection(STOP); //STOPS ALL MOTORS
		}

		//vision.updateCamera();
		//k = cv::waitKey(100);
        //if(k == 27) //27 = 'ESC'
            //break;





	} // End of while loop

	//Replay motor commands
	/*
	for (size_t i = 0; i < motorcommands_left.size(); i++)
	{
		odas.updateODAS(); //Otherwise no time is spent, TODO: clock the control loop instead
		motor_control.setRightMotorSpeedOnly(motorcommands_right[i]);
		motor_control.setLeftMotorSpeedOnly(motorcommands_left[i]);
	}*/
/***********************   END OF CONTROLLER LOOP   *************************/


	motor_control.setMotorDirection(STOP);		//STOP ALL MOTORS
	motor_control.resetMatrixVoiceLEDs();		//RESET ALL LEDS
    //vision.camera->release();					//Release camera resources

	//Test flag
	std::cout << "End of main -------" << std::endl;

	//Close outputstream
	output_stream.close();





	return 0;
}
