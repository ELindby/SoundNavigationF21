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

	// Everloop Initialization
	// Initialize bus and exit program if error occurs
	if (!odas.bus.Init())
		throw("Bus Init failed");
	//return false;

// Holds the number of LEDs on MATRIX device
	odas.ledCount = odas.bus.MatrixLeds();
	// Create EverloopImage object, with size of ledCount
	odas.image1d = new matrix_hal::EverloopImage(odas.ledCount);

	// Create Everloop object
	odas.everloop = new matrix_hal::Everloop;
	// Set everloop to use MatrixIOBus bus
	odas.everloop->Setup(&odas.bus);

	// Clear all LEDs
	for (matrix_hal::LedValue &led : odas.image1d->leds) {
		led.red = 0;
		led.green = 0;
		led.blue = 0;
		led.white = 0;
	}
	odas.everloop->Write(image1d);

	//Test values - 25/02 problems with not all matrix voice LEDs lighting up as expected
	//printf("\nDefines: ENERGY_COUNT:%d - MAX_BRIGHTNESS:%d - MAX_VALUE%d - MIN_THRESHOLD:%d - INCREMENT:%d",ENERGY_COUNT, MAX_BRIGHTNESS, MAX_VALUE, MIN_THRESHOLD,INCREMENT);
	//printf("\nbus.MatrixLeds(): %d --------------\n", odas.bus.MatrixLeds());


	odas.server_id = socket(AF_INET, SOCK_STREAM, 0);

	odas.server_address.sin_family = AF_INET;
	odas.server_address.sin_addr.s_addr = htonl(INADDR_ANY);
	odas.server_address.sin_port = htons(odas.portNumber);

	printf("Binding socket........... ");
	fflush(stdout);
	bind(odas.server_id, (struct sockaddr *)&odas.server_address, sizeof(odas.server_address));
	printf("[OK]\n");

	printf("Listening socket......... ");
	fflush(stdout);
	listen(odas.server_id, 1);
	printf("[OK]\n");

	printf("Waiting for connection in port %d ... ", odas.portNumber);
	fflush(stdout);
	connection_id = accept(odas.server_id, (struct sockaddr *)NULL, NULL);
	printf("[OK]\n");

	odas.message = (char *)malloc(sizeof(char) * odas.nBytes);

	printf("Receiving data........... \n\n");


	/*****************************************************************************
	************************   CONTROLLER LOOP   *********************************
	*****************************************************************************/
	while ((odas.messageSize = recv(odas.connection_id, odas.message, odas.nBytes, 0)) > 0) {
		//if((messageSize = recv(connection_id, message, nBytes, 0)) > 0){
			odas.message[odas.messageSize] = 0x00;

			// printf("message: %s\n\n", message);
		json_object *jobj = json_tokener_parse(odas.message);
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
