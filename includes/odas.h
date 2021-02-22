#pragma once
/*
 * Description:     ODAS class - implementation of ODAS for MATRIX VOICE.
 *							   - Listens to odas live (has to be run from same terminal before this class is instantiated)
 *
 * Author:			Erik Lindby & Jacob Fløe Jeppesen
 *					Derivative of MATRIX Labs ODAS
 *					University of Southern Denmark
 * Creation date:   22-02-2021
 */
#include <json.h>
#include <math.h>
#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/matrixio_bus.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <array>
#include <iostream>


#define ENERGY_COUNT 36		// ENERGY_COUNT : Number of sound energy slots to maintain.
#define MAX_VALUE 200		// MAX_VALUE : controls smoothness
#define INCREMENT 20		// INCREMENT : controls sensitivity
#define DECREMENT 1			// DECREMENT : controls delay in the dimming
#define MIN_THRESHOLD 10	// MAX_BRIGHTNESS: Filters out low energy
#define MAX_BRIGHTNESS 50	// MAX_BRIGHTNESS: 0 - 255

class ODAS
{
private:
	double x, y, z, E;
	int energy_array[ENERGY_COUNT];
	const double led_angles_mvoice[18] = { 170, 150, 130, 110, 90,  70,
									  50,  30,  10,  350, 330, 310,
									  290, 270, 250, 230, 210, 190 }; //LED angles for MATRIX Voice

	int ledCount;
	matrix_hal::MatrixIOBus bus;				// Create MatrixIOBus object for hardware communication
	matrix_hal::EverloopImage* image1d;			// Create EverloopImage object "image1d", with size of ledCount
	matrix_hal::Everloop* everloop;				// Create Everloop object
	
	//Connection variables
	char verbose = 0x00;
	int server_id;
	struct sockaddr_in server_address;
	int connection_id;
	char *message;
	int messageSize;
	int c;
	unsigned int portNumber = 9001;
	const unsigned int nBytes = 10240;


	void increase_pots();
	void decrease_pots();
	void json_parse_array(json_object *jobj, char *key);
	void json_parse(json_object *jobj);
public:
	ODAS();
	~ODAS();

	void ODAS::updateODAS();
};
