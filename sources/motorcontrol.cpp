//#include "pch.h"
#include "../includes/motorcontrol.h"

//Constructor
MotorControl::MotorControl(){
	/*****************************************************************************
	*********************   INITIALISE MATRIX VOICE DEVICE   *********************
	*****************************************************************************/
	// Create MatrixIOBus object for hardware communication
	//matrix_hal::MatrixIOBus bus;
	// Initialize bus and exit program if error occurs
	if (!bus.Init())
		throw("Bus Init failed");
		//return false;

	//DEBUG: Confirm bus init
	std::cout << "Bus Init successful" << std::endl;

	// Holds the number of LEDs on MATRIX device
	ledCount = bus.MatrixLeds();
	// Create EverloopImage object, with size of ledCount
	everloop_image = new matrix_hal::EverloopImage(ledCount);

	// Create Everloop object
	everloop = new matrix_hal::Everloop;
	// Set everloop to use MatrixIOBus bus
    everloop->Setup(&bus);
	// Create GPIOControl object - General Purpose Input Output
	gpio = new matrix_hal::GPIOControl;
	// Set gpio to use MatrixIOBus bus
	gpio->Setup(&bus);

	// Display rainbow animation
	startupShowLEDRainbow(/*&everloop, &everloop_image*/);

	/*****************************************************************************
	************************   INITIALISE MOTOR CONTROL   ************************
	*****************************************************************************/
	// Initialise Matrix Voice GPIO pins
	initGPIOPins(/*&gpio*/);
}

//Deconstructor
MotorControl::~MotorControl()
{
}



// Initialise Matrix Voice GPIO pins
void MotorControl::initGPIOPins(/*matrix_hal::GPIOControl* gpio*/)
{

	gpio->SetMode(TB6612_RIGHT_MOTOR_PWMA, 1); //Pin mode as output
	gpio->SetFunction(TB6612_RIGHT_MOTOR_PWMA, 1); // Pin function as PWM
	gpio->SetMode(TB6612_RIGHT_MOTOR_AIN1, 1);
	gpio->SetMode(TB6612_RIGHT_MOTOR_AIN2, 1);

	gpio->SetMode(TB6612_LEFT_MOTOR_PWMB, 1); //Pin mode as output
	gpio->SetFunction(TB6612_LEFT_MOTOR_PWMB, 1); // Pin function as PWM
	gpio->SetMode(TB6612_LEFT_MOTOR_BIN1, 1);
	gpio->SetMode(TB6612_LEFT_MOTOR_BIN2, 1);
}

// Set speed and direction of LEFT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void MotorControl::setLeftMotorSpeedDirection(/*matrix_hal::GPIOControl* gpio,*/ int speed, int dir)
{
	if (dir <= 0) // Reverse
	{
		gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1, 0); // Rotate left motor clockwise
		gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2, 1);
	}
	if ((dir > 0) || (dir >= 1)) // Forward
	{
		gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1, 1); // Rotate left motor counter-clockwise
		gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2, 0);
	}

	// Set motor speed via PWM signal (min. = 0, max. = 100)
	if (speed > 100)
		speed = 100;
	if (speed < 0)
		speed = 0;

	gpio->SetPWM(1000, speed, TB6612_LEFT_MOTOR_PWMB);
}

// Set speed and direction of RIGHT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void MotorControl::setRightMotorSpeedDirection(/*matrix_hal::GPIOControl* gpio,*/ int speed, int dir)
{
	if (dir <= 0) // Reverse
	{
		gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1, 0); // Rotate right motor counter-clockwise
		gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2, 1);
	}
	if ((dir > 0) || (dir >= 1)) // Forward
	{
		gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1, 1); // Rotate right motor clockwise
		gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2, 0);
	}

	// Set motor speed via PWM signal (min. = 0, max. = 100)
	if (speed > 100)
		speed = 100;
	if (speed < 0)
		speed = 0;

	gpio->SetPWM(1000, speed, TB6612_RIGHT_MOTOR_PWMA);
}

// Display a rainbow animation on the Matrix Voice LEDs
void MotorControl::startupShowLEDRainbow(/*matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image*/)
{
	// Variables used for sine wave rainbow logic
	float counter = 0;
	const float freq = 0.375;

	// For each led in everloop_image.leds, set led value
	for (matrix_hal::LedValue &led : everloop_image->leds)
	{
		// Sine waves 120 degrees out of phase for rainbow
		led.red = (std::sin(freq * counter + (M_PI / 180 * 240)) * 155 + 100) / 10;
		led.green = (std::sin(freq * counter + (M_PI / 180 * 120)) * 155 + 100) / 10;
		led.blue = (std::sin(freq * counter + 0) * 155 + 100) / 10;
		counter = counter + 1.01;
		// Updates the LEDs
		everloop->Write(everloop_image);
		usleep(40000);
	}

	usleep(460000);

	// For each led in everloop_image.leds, set led value to 0
	for (matrix_hal::LedValue &led : everloop_image->leds)
	{
		// Turn off Everloop
		led.red = 0;
		led.green = 0;
		led.blue = 0;
		led.white = 0;
	}

	// Updates the Everloop on the MATRIX device
	everloop->Write(everloop_image);
}

// Set individual LEDs on the Matrix Voice
void MotorControl::setMatrixVoiceLED(/*matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image,*/ int ledn, int r, int g, int b)
{
	for (int i = 0; i < 18; i++)
	{
		if (i == ledn)
		{
			everloop_image->leds[ledn].red = r;
			everloop_image->leds[ledn].green = g;
			everloop_image->leds[ledn].blue = b;
			everloop_image->leds[ledn].white = 0;
		}
	}
	everloop->Write(everloop_image);
}

//Turn off all matrix voice LEDS
void MotorControl::resetMatrixVoiceLEDs(){
	setMatrixVoiceLED(MATRIX_LED_R_1, 0, 0, 0);
	setMatrixVoiceLED(MATRIX_LED_L_9, 0, 0, 0);
	setMatrixVoiceLED(MATRIX_LED_R_3, 0, 0, 0);
	setMatrixVoiceLED(MATRIX_LED_L_7, 0, 0, 0);
	setMatrixVoiceLED(MATRIX_LED_R_4, 0, 0, 0);
	setMatrixVoiceLED(MATRIX_LED_L_6, 0, 0, 0);
	setMatrixVoiceLED(MATRIX_LED_R_5, 0, 0, 0);
	setMatrixVoiceLED(MATRIX_LED_L_5, 0, 0, 0);
	setMatrixVoiceLED(MATRIX_LED_R_9, 0, 0, 0);
	setMatrixVoiceLED(MATRIX_LED_L_1, 0, 0, 0);
}
