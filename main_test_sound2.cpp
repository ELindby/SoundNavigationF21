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

#include "json-c/json.h"
#include <netinet/in.h>
#include <sys/socket.h>





using namespace std;
//using namespace cv;






int main(int argc, char** argv)
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
	while ((odas.messageSize = recv(odas.connection_id, odas.message, odas.nBytes, 0)) > 0) {
		//if((messageSize = recv(connection_id, message, nBytes, 0)) > 0){
			odas.message[odas.messageSize] = 0x00;

			// printf("message: %s\n\n", message);
		json_object *jobj = json_tokener_parse(message);
		odas.json_parse(jobj);

		for (int i = 0; i < odas.bus.MatrixLeds(); i++) {
			// led index to angle
			int led_angle = odas.bus.MatrixName() == matrix_hal::kMatrixCreator
				? odas.leds_angle_mcreator[i]
				: odas.led_angles_mvoice[i];
			//int led_angle = led_angles_mvoice[i];
			// Convert from angle to pots index
			int index_pots = led_angle * ENERGY_COUNT / 360;
			// Mapping from pots values to color
			int color = odas.energy_array[index_pots] * MAX_BRIGHTNESS / MAX_VALUE;
			// Removing colors below the threshold
			color = (color < MIN_THRESHOLD) ? 0 : color;

			odas.image1d->leds[i].red = 0;
			odas.image1d->leds[i].green = 0;//0;
			odas.image1d->leds[i].blue = color;//color;
			odas.image1d->leds[i].white = 0;
		}
		odas.everloop->Write(odas.image1d);
	}
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
