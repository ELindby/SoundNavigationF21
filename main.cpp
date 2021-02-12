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
//INCLUDE CLASSES
#include "soundlocalization.h"


#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
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


/* Without Matrix Voice
#define TB6612_RIGHT_MOTOR_PWMA		12 // (Orange)
#define TB6612_LEFT_MOTOR_PWMB		13 // (Green)
#define TB6612_RIGHT_MOTOR_AIN1		16 // (Blue)
#define TB6612_RIGHT_MOTOR_AIN2		26 // (Brown)
#define TB6612_LEFT_MOTOR_BIN1		5  // (Grey)
#define TB6612_LEFT_MOTOR_BIN2		6  // (Pink)


// With Matrix Voice but using Raspberry Pi GPIO
#define  TB6612_RIGHT_MOTOR_PWMA        13 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         16 // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        19 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        26 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         20 // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         21 // (Pink)
*/

// GPIO via Matrix Voice
#define  TB6612_RIGHT_MOTOR_PWMA        14 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         8  // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        12 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        10 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         6  // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         4  // (Pink)

// Matrix Voice LEDs
#define MATRIX_LED_R_1          0
#define MATRIX_LED_R_2          1
#define MATRIX_LED_R_3          2
#define MATRIX_LED_R_4          3
#define MATRIX_LED_R_5          4
#define MATRIX_LED_R_6          5
#define MATRIX_LED_R_7          6
#define MATRIX_LED_R_8          7
#define MATRIX_LED_R_9          8

#define MATRIX_LED_L_1          9
#define MATRIX_LED_L_2          10
#define MATRIX_LED_L_3          11
#define MATRIX_LED_L_4          12
#define MATRIX_LED_L_5          13
#define MATRIX_LED_L_6          14
#define MATRIX_LED_L_7          15
#define MATRIX_LED_L_8          16
#define MATRIX_LED_L_9          17

#define REFLEX_THRESHOLD	80
#define AVOIDANCE_THRESHOLD	65

#define VELOCITY_OFFSET		12



using namespace std;
//using namespace cv;

// Initialise Matrix Voice GPIO pins
void initGPIOPins(matrix_hal::GPIOControl* gpio)
{

        gpio->SetMode(TB6612_RIGHT_MOTOR_PWMA,1); //Pin mode as output
        gpio->SetFunction(TB6612_RIGHT_MOTOR_PWMA,1); // Pin function as PWM
        gpio->SetMode(TB6612_RIGHT_MOTOR_AIN1,1);
        gpio->SetMode(TB6612_RIGHT_MOTOR_AIN2,1);

        gpio->SetMode(TB6612_LEFT_MOTOR_PWMB,1); //Pin mode as output
        gpio->SetFunction(TB6612_LEFT_MOTOR_PWMB,1); // Pin function as PWM
        gpio->SetMode(TB6612_LEFT_MOTOR_BIN1,1);
        gpio->SetMode(TB6612_LEFT_MOTOR_BIN2,1);
}

// Set speed and direction of LEFT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void setLeftMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir)
{
        if (dir <= 0) // Reverse
        {
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1,0); // Rotate left motor clockwise
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2,1);
        }
        if ( (dir > 0)  || (dir >= 1) ) // Forward
        {
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1,1); // Rotate left motor counter-clockwise
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2,0);
        }

        // Set motor speed via PWM signal (min. = 0, max. = 100)
       if (speed > 100)
                speed = 100;
        if (speed < 0)
                speed = 0;

        gpio->SetPWM(1000,speed,TB6612_LEFT_MOTOR_PWMB);
}

// Set speed and direction of RIGHT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void setRightMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir)
{
        if (dir <= 0) // Reverse
        {
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1,0); // Rotate right motor counter-clockwise
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2,1);
        }
        if ( (dir > 0)  || (dir >= 1) ) // Forward
        {
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1,1); // Rotate right motor clockwise
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2,0);
        }

        // Set motor speed via PWM signal (min. = 0, max. = 100)
        if (speed > 100)
                speed = 100;
        if (speed < 0)
                speed = 0;

        gpio->SetPWM(1000,speed,TB6612_RIGHT_MOTOR_PWMA);
}

// Display a rainbow animation on the Matrix Voice LEDs
void startupShowLEDRainbow(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image)
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
void setMatrixVoiceLED(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image,int ledn, int r, int g, int b)
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


double activation(double input){
	return 50/(1 + exp(-input));
}





int main (int argc, char** argv)
{

/*****************************************************************************
*********************   INITIALISE MATRIX VOICE DEVICE   *********************
*****************************************************************************/

        // Create MatrixIOBus object for hardware communication
        matrix_hal::MatrixIOBus bus;
        // Initialize bus and exit program if error occurs
        if (!bus.Init())
		return false;

        // Holds the number of LEDs on MATRIX device
        int ledCount = bus.MatrixLeds();
        // Create EverloopImage object, with size of ledCount
        matrix_hal::EverloopImage everloop_image(ledCount);
        // Create Everloop object
        matrix_hal::Everloop everloop;
        // Set everloop to use MatrixIOBus bus
        everloop.Setup(&bus);
        // Create GPIOControl object
        matrix_hal::GPIOControl gpio;
        // Set gpio to use MatrixIOBus bus
        gpio.Setup(&bus);

	// Display rainbow animation
	startupShowLEDRainbow(&everloop, &everloop_image);

/*********************************   DONE   *********************************/


/*****************************************************************************
************************   INITIALISE MOTOR CONTROL   ************************
*****************************************************************************/

	// Initialise Matrix Voice GPIO pins
        initGPIOPins(&gpio);

/*********************************   DONE   *********************************/


/*****************************************************************************
*********************   INITIALISE RASPBERRY PI CAMERA   *********************
*****************************************************************************/

	// Create Raspberry Pi camera object
	raspicam::RaspiCam Camera;
	// Create OpenCV SimpleBlobDetector parameter objects (One for white object detection (red), one for black object detection).
	cv::SimpleBlobDetector::Params sbdPar_red, sbdPar_black;

	// Set blob detection parameters
	// Change thresholds
	sbdPar_red.minThreshold = 1;
	sbdPar_red.maxThreshold = 1000;
	sbdPar_black.minThreshold = 1;
	sbdPar_black.maxThreshold = 1000;
	// Filter by colour
	sbdPar_red.filterByColor = true;
	sbdPar_black.filterByColor = true;
	// Look for colours that match grayscale value of 255 (white) or 0 (black)
	sbdPar_red.blobColor = 255;
	sbdPar_black.blobColor = 0;
	// Filter by area
	sbdPar_red.filterByArea = true;
	sbdPar_red.minArea = 100; // 10x10 pixels
	sbdPar_red.maxArea = 160000; // 400x400 pixels
	sbdPar_black.filterByArea = true;
	sbdPar_black.minArea = 1225; // 100x100 pixels
	sbdPar_black.maxArea = 160000; // 400x400 pixels

	// Create OpenCV SimpleBlobDetector object based on assigned parameters
	cv::Ptr<cv::SimpleBlobDetector> sbd_red = cv::SimpleBlobDetector::create(sbdPar_red);
	cv::Ptr<cv::SimpleBlobDetector> sbd_black = cv::SimpleBlobDetector::create(sbdPar_black);
	vector<cv::KeyPoint> keypts_red, keypts_black;

	// Set Raspberry Pi camera parameters
	// Set camera image format to BGR as used by  OpenCV
	Camera.setFormat(raspicam::RASPICAM_FORMAT_BGR);
	// Set image resolution
	Camera.setWidth(640);
	Camera.setHeight(480);
        // Flip camera image vertically and horizontally
        // because camera is mounted upside down
        Camera.setVerticalFlip(true);
        Camera.setHorizontalFlip(true);

	// Display current camera parameters
        cout << "Format: " << Camera.getFormat() << endl;
        cout << "Width: " << Camera.getWidth() << endl;
        cout << "Height: " << Camera.getHeight() << endl;
        cout << "Brightness: " << Camera.getBrightness() << endl;
        cout << "Rotation: " << Camera.getRotation() << endl;
        cout << "ISO: " << Camera.getISO() << endl;
        cout << "Sharrpness: " << Camera.getSharpness() << endl;
        cout << "Contrast: " << Camera.getContrast() << endl;
        cout << "Saturation: " << Camera.getSaturation() << endl;
        cout << "ShutterSpeed: " << Camera.getShutterSpeed() << endl;
        cout << "Exopsure: " << Camera.getExposure() << endl;
        cout << "AWB: " << Camera.getAWB() << endl;
        cout << "Image effect: " << Camera.getImageEffect() << endl;
        cout << "Metering: " << Camera.getMetering() << endl;
	cout << "Format:" << Camera.getFormat() << endl;
	cout << "Body: " << "Ready" << endl;

	// Open camera
	if (!Camera.open())
	{
		cerr << "Error opening camera." << endl;
		return -1;
	}

	// Wait 3 seconds for camera image to stabilise
	cout << "Waiting for camera to stabilise...";
	usleep(3000000);
	cout << "done." << endl;

/*********************************   DONE   *********************************/


/*****************************************************************************
************************   INITIALISE VISUALISATION   ************************
*****************************************************************************/
	
	
	// Black threshold values
		int iLowH_black = 0;
		int iHighH_black = 179;

		int iLowS_black = 0;
		int iHighS_black = 255;

		int iLowV_black = 30;
		int iHighV_black = 255;

	// Red threshold values Remember blobcolor = 255
		int iLowH_red = 0;
		int iHighH_red = 179;

		int iLowS_red = 74;
		int iHighS_red = 255;

		int iLowV_red = 60;
		int iHighV_red = 255;

	//~ // Create a window for displaying HSV
	//~ cv::namedWindow("HSV controls",cv::WINDOW_NORMAL);

	//~ // Create trackbars for H, S and V in the window
	//~ cv::createTrackbar("LowH", "HSV controls", &iLowH, 179); //Hue (0 - 179)
 	//~ cv::createTrackbar("HighH", "HSV controls", &iHighH, 179);
 	//~ cv::createTrackbar("LowS", "HSV controls", &iLowS, 255); //Saturation (0 - 255)
 	//~ cv::createTrackbar("HighS", "HSV controls", &iHighS, 255);
 	//~ cv::createTrackbar("LowV", "HSV controls", &iLowV, 255); //Value (0 - 255)
 	//~ cv::createTrackbar("HighV", "HSV controls", &iHighV, 255);

	// Create a bunch of windows for displaying image processing steps
	// Create window to display original HSV image
	//cv::namedWindow("HSV image",cv::WINDOW_AUTOSIZE);
	// Create window to display thresholded image
	//cv::namedWindow("Thresholded image - Red",cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Thresholded image - Black",cv::WINDOW_AUTOSIZE);
	// Create window to display blob image
	cv::namedWindow("Blobs - Red",cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Blobs - Black",cv::WINDOW_AUTOSIZE);

/*********************************   DONE   *********************************/


/*****************************************************************************
*******************************   CONTROLLER   *******************************
*****************************************************************************/

	// Create buffer of correct size to store image data
	int img_buf_len = Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_BGR);
	unsigned char *img_buf=new unsigned char[img_buf_len];

	// Initialise OpenCV image Mat
	cv::Mat imageMat = cv::Mat(Camera.getHeight(),Camera.getWidth(),CV_8UC3,img_buf);

	// OpeCV image Mat that holds the thresholded image data
	cv::Mat imageThreshold_red, imageThreshold_black;

	// OpenCV image Mat to store image with detected blobs
	cv::Mat imageKeypoints_red, imageKeypoints_black;

	// Vector storing [x,y] co-ordinates of detected blobs
	vector<cv::Point2f> keyptXY_red, keyptXY_black;
	
	
	// ICO Learning
	
	double dist_to_obst_prev_prev = 35.0;	// Previous Previus Distance to closest obstacle on the track
	double dist_to_obst_prev = 35.0;		// Previous Distance to closest obstacle on the track
	double dist_to_obst_current = 0.0;	// Distance to closest obstacle on the track
	
	double w_reflex_var = 1.0;		// Standard weight that needs to be multiplied with distance to current Obstacle
	double w_reflex_novar = 1.0;		// 
	
	double reflex_learning_rate = 10;	// Learning rate for reflex µ 
	double v_learning = 0.0; 		// Velocity to add to the initial velocity
	int reflexcounter = 0;
	
	double red_size = 0;
	//~ double w_reflex_new = 0.0;
	//~ double w_reflex_delta = 0.0;
	
	int bio_timer = 0;
	int pause_timer = 0;
	
	

	


	// Controller loop starts here
	while(true)
	{
		// Grab image into internal buffer
		Camera.grab();

		// Copy latest camera buffer into our defined buffer
		Camera.retrieve(img_buf);

		// Copy image buffer data into OpenCV Mat image
		imageMat = cv::Mat(Camera.getHeight(),Camera.getWidth(),CV_8UC3,img_buf);

		// Exit if there is no image data in OpenCV image Mat
		if (!imageMat.data)
		{
			cout << "No data in Mat imageMat." << endl;

			// Release Raspberry Pi camera resources
	                Camera.release();

			return -1;
		}

		// Convert image from BGR to HSV
		cv::cvtColor(imageMat,imageMat,cv::COLOR_BGR2HSV);

		// Threshold image
		cv::inRange(imageMat,cv::Scalar(iLowH_red,iLowS_red,iLowV_red),cv::Scalar(iHighH_red,iHighS_red,iHighV_red),imageThreshold_red);
		cv::inRange(imageMat,cv::Scalar(iLowH_black,iLowS_black,iLowV_black),cv::Scalar(iHighH_black,iHighS_black,iHighV_black),imageThreshold_black);

		//morphological opening (remove small objects from the foreground)
		erode(imageThreshold_red, imageThreshold_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		dilate(imageThreshold_red, imageThreshold_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		erode(imageThreshold_black, imageThreshold_black, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		dilate(imageThreshold_black, imageThreshold_black, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

		//morphological closing (fill small holes in the foreground)
		dilate(imageThreshold_red, imageThreshold_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	 	erode(imageThreshold_red, imageThreshold_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		dilate(imageThreshold_black, imageThreshold_black, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	 	erode(imageThreshold_black, imageThreshold_black, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

		// Display HSV image
		//cv::imshow("HSV image", imageMat);

		// Display thresholded imnage
		//cv::imshow("Thresholded image - Red",imageThreshold_red);
		//cv::imshow("Thresholded image - Black",imageThreshold_black);

		// Detect keypoints in thresholded image using SimpleBlobDetector object
		sbd_red->detect(imageThreshold_red,keypts_red);
		sbd_black->detect(imageThreshold_black,keypts_black);

		// Draw detected keypoints as red/rblue(black) circles around detected blobs and store new image in OpenCV image Mat
		cv::drawKeypoints(imageThreshold_red,keypts_red,imageKeypoints_red,cv::Scalar(0,0,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::drawKeypoints(imageThreshold_black,keypts_black,imageKeypoints_black,cv::Scalar(255,0,0),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		// Display image with detected blobs
		cv::imshow("Blobs - Red",imageKeypoints_red);
		cv::imshow("Blobs - Black",imageKeypoints_black);

		// Extract [x,y] co-ordinates of blobs from keypoints
		cv::KeyPoint::convert(keypts_red,keyptXY_red);
		cv::KeyPoint::convert(keypts_black,keyptXY_black);

		// If at least one blob has been detected (set to 1 here), print out the [x,y] co-ordinates
		//if (keyptXY_red.size() >= 1)
			//cout << "R:[x,y] = " << "[" << keyptXY_red.front().x << "," << keyptXY_red.front().y << "]" << endl;
		//if (keyptXY_black.size() >= 1)
			//cout << "B:[x,y] = " << "[" << keyptXY_black.front().x << "," << keyptXY_black.front().y << "]" << endl;

		/**************************
		*** YOUR CODE GOES HERE ***
		//~ **************************/
		if (keyptXY_black.size() >= 1){
			dist_to_obst_prev_prev = dist_to_obst_prev;
			dist_to_obst_prev = dist_to_obst_current;
			dist_to_obst_current = keypts_black.front().size;
			bio_timer = 0;
		}
		if (keyptXY_red.size() >= 1){
		if (keyptXY_red.size() >= 1){
			red_size = keypts_red.front().size;
			bio_timer = 0;
		}
		
		// -------------------------------------- Target Found -------------------------------------------------
		if ( (keyptXY_red.size() >= 1) && (red_size > 120))
		{
			cout << "STOP" << endl;
			setRightMotorSpeedDirection(&gpio, 0, 1);
			setLeftMotorSpeedDirection(&gpio, 0, 1);
			while(true)
			{
				if (cv::waitKey(30) == '0')
				{
				break;
				}
			}
				
		}
		// --------------------------------------- Reflex -------------------------------------------------
		else if ( (keyptXY_black.size() >= 1) && (dist_to_obst_current >= REFLEX_THRESHOLD) ){ //Reflex mode
			//cout << "Object ahead: " << keyptXY_black.front().x << endl;
			
			
			
			
			if(keyptXY_black.front().x >= 320){ //Object is on RIGHT side
				setRightMotorSpeedDirection(&gpio, 35, 1);
				setLeftMotorSpeedDirection(&gpio, 25, 1);
				setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_5, 0, 0, 255);
			}else{ //object is on LEFT side
				setRightMotorSpeedDirection(&gpio, 25, 1);
				setLeftMotorSpeedDirection(&gpio, 35, 1);
				setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_5, 0, 0, 255);
			}
			
			//cout << "Size of Obst: " << keypts_black.front().size << endl;
			w_reflex_var = w_reflex_var + reflex_learning_rate * (dist_to_obst_current/70) * ( dist_to_obst_prev - dist_to_obst_prev_prev )/70;
			reflexcounter += 1;
			cout << " w_reflex: " << w_reflex_var << " Reflexcounter: " << reflexcounter << endl;
			//w_new = w_old + µ * D(t) *(D(t-1)+D(t-2))/1
			
		}
		// --------------------------------------- Braitenberg Vehicle ------------------------------------
		else if (keyptXY_red.size() >= 1){ //Braitenberg Vehicle mode (AGGRESSION)
			if(keyptXY_red.front().x >= 320){ //Object is on RIGHT side
				setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 255, 0);
			}else{ //object is on LEFT side
				setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 255, 0);
			}
			
			// Update sensor signals
			double distanceL = (( keyptXY_red.front().x - 320) / 320.0); // Normalize the shit
			double distanceR = (( (640 - keyptXY_red.front().x) - 320 ) / 320.0); // Normalize
			// AVOIDANCE, OR REGULAR BRAITENBERG
			if ( (keyptXY_black.size() >= 1) && (dist_to_obst_current >= AVOIDANCE_THRESHOLD) && (dist_to_obst_current <= REFLEX_THRESHOLD)) { //Braitenberg control WITH avoidance added
				v_learning = (dist_to_obst_current/70) * w_reflex_var + (dist_to_obst_prev/70) * w_reflex_novar;
				// v_learning = D(t)*w_1 + D(t-1)*w_2
				if(keyptXY_black.front().x >= 320){ //Object is on RIGHT side
					setRightMotorSpeedDirection(&gpio, 	(activation(distanceR ))+ VELOCITY_OFFSET + v_learning, 1);
					setLeftMotorSpeedDirection(&gpio, 	(activation(distanceL ))+ VELOCITY_OFFSET, 1);
					setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_4, 140, 140, 140);
				}else{ //object is on LEFT side
					setRightMotorSpeedDirection(&gpio, 	(activation(distanceR ))+ VELOCITY_OFFSET, 1);
					setLeftMotorSpeedDirection(&gpio, 	(activation(distanceL ))+ VELOCITY_OFFSET + v_learning, 1);
					setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_6, 140, 140, 140);
				}
			} else{ // Normal Braitenberg Control
				setRightMotorSpeedDirection(&gpio, 		(activation(distanceR )) + VELOCITY_OFFSET, 1);
				setLeftMotorSpeedDirection(&gpio, 		(activation(distanceL )) + VELOCITY_OFFSET, 1);
			}
		} 
		// --------------------------------------- Avoidance ----------------------------------------------
		else if ( (keyptXY_black.size() >= 1) && (dist_to_obst_current >= AVOIDANCE_THRESHOLD) && (dist_to_obst_current <= REFLEX_THRESHOLD)) { //Avoidance
			v_learning = (dist_to_obst_current/70) * w_reflex_var + (dist_to_obst_prev/70) * w_reflex_novar;
			// v_learning = D(t)*w_1 + D(t-1)*w_2
			cout << "v_learning is: " << v_learning <<  endl;
			//cout << " w_reflex: " << w_reflex_var << " Reflexcounter: " << reflexcounter << endl;
			
			if(keyptXY_black.front().x >= 320){ //Object is on RIGHT side
				setRightMotorSpeedDirection(&gpio, 25 + v_learning, 1);
				setLeftMotorSpeedDirection(&gpio, 25, 1);
				setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 255, 255, 0);
			}else{ //object is on LEFT side
				setRightMotorSpeedDirection(&gpio, 25, 1);
				setLeftMotorSpeedDirection(&gpio, 25 + v_learning, 1);
				setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 255, 255, 0);
			}
		}
		else {	//If no blob is in sight
			//setRightMotorSpeedDirection(&gpio, 0, 1);
			//ssetLeftMotorSpeedDirection(&gpio, 0, 1);
			bio_timer++; // Increment the timer by 1 = 30ms (roughly)
			//cout << bio_timer << endl;
			if (bio_timer >= 5)
			{
				if(pause_timer >= 2)
				{
					setRightMotorSpeedDirection(&gpio, 0, 1);
					setLeftMotorSpeedDirection(&gpio, 0, 0);
					pause_timer = 0;
				}
				else{
				bio_timer = 5;
				pause_timer++;
				setRightMotorSpeedDirection(&gpio, 20, 1);
				setLeftMotorSpeedDirection(&gpio, 20, 0);
				setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_9, 200, 0, 0);
				setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_1, 200, 0, 0);
				}
			}
			

		}
		
		
		//End 
		
		
// Wait 30ms for keypress and exit if ESC (ASCII code 27) is pressed
  int key_input = cv::waitKey(30);
  if (key_input == 27)
  {
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_4, 0, 0, 0);
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_6, 0, 0, 0);
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_5, 0, 0, 0);
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_5, 0, 0, 0);
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_9, 0, 0, 0);
    setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_1, 0, 0, 0);
    break;
  }
  else if (key_input == 'p')
  {
    cout << "PAUSE - manual steering enabled" << endl;
    setRightMotorSpeedDirection(&gpio, 0, 1);
    setLeftMotorSpeedDirection(&gpio, 0, 1);
    while(true)
    {
      key_input = cv::waitKey(30);
      switch (key_input){
        case 'w':
        setRightMotorSpeedDirection(&gpio, 30, 1);
        setLeftMotorSpeedDirection(&gpio, 30, 1);
        break;
        case 'a':
        setRightMotorSpeedDirection(&gpio, 25, 1);
        setLeftMotorSpeedDirection(&gpio, 25, 0);
        break;
        case 's':
        setRightMotorSpeedDirection(&gpio, 25, 0);
        setLeftMotorSpeedDirection(&gpio, 25, 0);
        break;
        case 'd':
        setRightMotorSpeedDirection(&gpio, 25, 0);
        setLeftMotorSpeedDirection(&gpio, 25, 1);
        break;
        case 'r':
        cout << "RESUME" << endl;
        break;break;
        default:
        setRightMotorSpeedDirection(&gpio, 0, 1);
        setLeftMotorSpeedDirection(&gpio, 0, 1);
      }
    }
  }
		
		
				

	} // End of while loop

/*********************************   DONE   *********************************/

	// Stop all motors
        setRightMotorSpeedDirection(&gpio,0,1);
        setLeftMotorSpeedDirection(&gpio,0,1);


	// Release camera resources
	Camera.release();

	return 0;
}
