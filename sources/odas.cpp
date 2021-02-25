//#include "pch.h"
#include "../includes/odas.h"


ODAS::ODAS(){

	// Everloop Initialization
	// Initialize bus and exit program if error occurs
	if (!bus.Init())
		throw("Bus Init failed");
		//return false;

	// Holds the number of LEDs on MATRIX device
	ledCount = bus.MatrixLeds();
	// Create EverloopImage object, with size of ledCount
	image1d = new matrix_hal::EverloopImage(ledCount);

	// Create Everloop object
	everloop = new matrix_hal::Everloop;
	// Set everloop to use MatrixIOBus bus
	everloop->Setup(&bus);

	// Clear all LEDs
	for (matrix_hal::LedValue &led : image1d->leds) {
		led.red = 0;
		led.green = 0;
		led.blue = 0;
		led.white = 0;
	}
	everloop->Write(image1d);

	//Test values - 25/02 problems with not all matrix voice LEDs lighting up as expected
    //printf("\nDefines: ENERGY_COUNT:%d - MAX_BRIGHTNESS:%d - MAX_VALUE%d - MIN_THRESHOLD:%d - INCREMENT:%d",ENERGY_COUNT, MAX_BRIGHTNESS, MAX_VALUE, MIN_THRESHOLD,INCREMENT);
    printf("\nbus.MatrixLeds(): %d --------------\n",bus.MatrixLeds());


	server_id = socket(AF_INET, SOCK_STREAM, 0);

	server_address.sin_family = AF_INET;
	server_address.sin_addr.s_addr = htonl(INADDR_ANY);
	server_address.sin_port = htons(portNumber);

	printf("Binding socket........... ");
	fflush(stdout);
	bind(server_id, (struct sockaddr *)&server_address, sizeof(server_address));
	printf("[OK]\n");

	printf("Listening socket......... ");
	fflush(stdout);
	listen(server_id, 1);
	printf("[OK]\n");

	printf("Waiting for connection in port %d ... ", portNumber);
	fflush(stdout);
	connection_id = accept(server_id, (struct sockaddr *)NULL, NULL);
	printf("[OK]\n");

	message = (char *)malloc(sizeof(char) * nBytes);

	printf("Receiving data........... \n\n");

}

ODAS::~ODAS(){}

void ODAS::updateODAS() {
	while ((messageSize = recv(connection_id, message, nBytes, 0)) > 0) {
		message[messageSize] = 0x00;

		// printf("message: %s\n\n", message);
		json_object *jobj = json_tokener_parse(message);
		json_parse(jobj);

		for (int i = 0; i < bus.MatrixLeds(); i++) {
			// led index to angle
			int led_angle = bus.MatrixName() == matrix_hal::kMatrixCreator
				? leds_angle_mcreator[i]
				: led_angles_mvoice[i];
			//int led_angle = led_angles_mvoice[i];
			// Convert from angle to pots index
			int index_pots = led_angle * ENERGY_COUNT / 360;
			// Mapping from pots values to color
			int color = energy_array[index_pots] * MAX_BRIGHTNESS / MAX_VALUE;
			// Removing colors below the threshold
			color = (color < MIN_THRESHOLD) ? 0 : color;

			image1d->leds[i].red = 0;
			image1d->leds[i].green = 0;
			image1d->leds[i].blue = color;
			image1d->leds[i].white = 0;
		}
		everloop->Write(image1d);
	}
}

void ODAS::increase_pots() {
	// Convert x,y to angle. TODO: See why x axis from ODAS is inverted
	double angle_xy = fmodf((atan2(y, x) * (180.0 / M_PI)) + 360, 360);
	// Convert angle to index
	int i_angle = angle_xy / 360 * ENERGY_COUNT;  // convert degrees to index
	// Set energy for this angle
	energy_array[i_angle] += INCREMENT * E;
	// Set limit at MAX_VALUE
	energy_array[i_angle] =
		energy_array[i_angle] > MAX_VALUE ? MAX_VALUE : energy_array[i_angle];
}

void ODAS::decrease_pots() {
	for (int i = 0; i < ENERGY_COUNT; i++) {
		energy_array[i] -= (energy_array[i] > 0) ? DECREMENT : 0;
	}
}

void ODAS::json_parse_array(json_object *jobj, char *key) {
	// Forward Declaration
	//void ODAS::json_parse(json_object * jobj);
	enum json_type type;
	json_object *jarray = jobj;
	if (key) {
		if (json_object_object_get_ex(jobj, key, &jarray) == false) {
			printf("Error parsing json object\n");
			return;
		}
	}

	int arraylen = json_object_array_length(jarray);
	int i;
	json_object *jvalue;

	for (i = 0; i < arraylen; i++) {
		jvalue = json_object_array_get_idx(jarray, i);
		type = json_object_get_type(jvalue);

		if (type == json_type_array) {
			json_parse_array(jvalue, NULL);
		}
		else if (type != json_type_object) {
		}
		else {
			json_parse(jvalue);
		}
	}
}

void ODAS::json_parse(json_object * jobj)
{
	enum json_type type;
	unsigned int count = 0;
	decrease_pots();
	json_object_object_foreach(jobj, key, val) {
		type = json_object_get_type(val);
		switch (type) {
		case json_type_boolean:
			break;
		case json_type_double:
			if (!strcmp(key, "x")) {
				x = json_object_get_double(val);
			}
			else if (!strcmp(key, "y")) {
				y = json_object_get_double(val);
			}
			else if (!strcmp(key, "z")) {
				z = json_object_get_double(val);
			}
			else if (!strcmp(key, "E")) {
				E = json_object_get_double(val);
			}
			increase_pots();
			count++;
			break;
		case json_type_int:
			break;
		case json_type_string:
			break;
		case json_type_object:
			if (json_object_object_get_ex(jobj, key, &jobj) == false) {
				printf("Error parsing json object\n");
				return;
			}
			json_parse(jobj);
			break;
		case json_type_array:
			json_parse_array(jobj, key);
			break;
		}
	}
}
