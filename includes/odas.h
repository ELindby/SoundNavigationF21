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
#include "json-c/json.h"
#include <math.h>
#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/matrixio_bus.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <array>
#include <iostream>
#include <vector>
#include <fstream>

#include "includes/defines.h"



class ODAS
{
private:
	double x, y, z, E;
	int energy_array[ENERGY_COUNT] = {0};
	const double led_angles_mvoice[18] = { 170, 150, 130, 110, 90,  70,
									  50,  30,  10,  350, 330, 310,
									  290, 270, 250, 230, 210, 190 }; //LED angles for MATRIX Voice
    const double leds_angle_mcreator[35] = {170, 159, 149, 139, 129, 118, 108, 98,  87,  77,  67,  57,
                                            46,  36,  26,  15,  5,   355, 345, 334, 324, 314, 303, 293,
                                            283, 273, 262, 252, 242, 231, 221, 211, 201, 190, 180};

	int ledCount;
	matrix_hal::MatrixIOBus *bus;				// Create MatrixIOBus object for hardware communication
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

	//Sound information - loudest sound source angle and energy level
	int angle = -2;
	int angle_prev = -2;
	int energy = -2;


	void increase_pots();
	void decrease_pots();

	void json_parse_array(json_object *jobj, char *key);
	void json_parse(json_object *jobj);
public:
	ODAS(matrix_hal::MatrixIOBus* bus_, matrix_hal::Everloop* everloop_, matrix_hal::EverloopImage* image1d_);
	~ODAS();

	void updateODAS(/*matrix_hal::MatrixIOBus* bus, matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* image1d*//*std::ofstream& output_stream*/);
	//void updateODAS(std::ofstream& output_stream); //Update odas, and print data to a csv file
	void updateSoundInformation(); //Updates angle, angle_prev and energy
	
	int getSoundAngle();
	int getSoundEnergy();
};
