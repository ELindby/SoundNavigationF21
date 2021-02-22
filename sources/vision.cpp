//#include "pch.h"
#include "../includes/vision.h"


Vision::Vision(){
/*****************************************************************************
*********************   INITIALISE RASPBERRY PI CAMERA   *********************
*****************************************************************************/
	// Create Raspberry Pi camera object
	//raspicam::RaspiCam Camera;

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
	std::cout << "Format: " << Camera.getFormat() << std::endl;
	std::cout << "Width: " << Camera.getWidth() << std::endl;
	std::cout << "Height: " << Camera.getHeight() << std::endl;
	std::cout << "Brightness: " << Camera.getBrightness() << std::endl;
	std::cout << "Rotation: " << Camera.getRotation() << std::endl;
	std::cout << "ISO: " << Camera.getISO() << std::endl;
	std::cout << "Sharrpness: " << Camera.getSharpness() << std::endl;
	std::cout << "Contrast: " << Camera.getContrast() << std::endl;
	std::cout << "Saturation: " << Camera.getSaturation() << std::endl;
	std::cout << "ShutterSpeed: " << Camera.getShutterSpeed() << std::endl;
	std::cout << "Exopsure: " << Camera.getExposure() << std::endl;
	std::cout << "AWB: " << Camera.getAWB() << std::endl;
	std::cout << "Image effect: " << Camera.getImageEffect() << std::endl;
	std::cout << "Metering: " << Camera.getMetering() << std::endl;
	std::cout << "Format:" << Camera.getFormat() << std::endl;
	std::cout << "Body: " << "Ready" << std::endl;

	// Open camera
	if (!Camera.open())
	{
		std::cerr << "Error opening camera." << std::endl;
		throw("Error opening camera.");
	}

	// Wait 3 seconds for camera image to stabilise
	std::cout << "Waiting for camera to stabilise...";
	usleep(3000000);
	std::cout << "done." << std::endl;


	setupSimpleBlobDetector();
	/*****************************************************************************
	*******************************   CONTROLLER   *******************************
	*****************************************************************************/

	// Create buffer of correct size to store image data
	img_buf_len = Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_BGR);
	img_buf = new unsigned char[img_buf_len];

	// Initialise OpenCV image Mat
	imageMat = cv::Mat(Camera.getHeight(), Camera.getWidth(), CV_8UC3, img_buf);

	// Create window to display original image
	cv::namedWindow("Image",cv::WINDOW_AUTOSIZE);
}


Vision::~Vision()
{
}

void Vision::setupSimpleBlobDetector()
{
/*****************************************************************************
*********************  SETUP SIMPLE BLOB DETECTOR   **************************
*****************************************************************************/
	/*// Create OpenCV SimpleBlobDetector parameter objects (One for white object detection (red), one for black object detection).
	//cv::SimpleBlobDetector::Params sbdPar_red, sbdPar_black;

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
	sbd_red = cv::SimpleBlobDetector::create(sbdPar_red);
	sbd_black = cv::SimpleBlobDetector::create(sbdPar_black);
	vector<cv::KeyPoint> keypts_red, keypts_black;*/

	/*****************************************************************************
	************************   INITIALISE VISUALISATION   ************************
	*****************************************************************************/
	/*// Black threshold values
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
	cv::namedWindow("Blobs - Red", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Blobs - Black", cv::WINDOW_AUTOSIZE);*/
	/*********************************   DONE   *********************************/
}

void Vision::updateCamera(){
	// Grab image into internal buffer
	Camera.grab();

	// Copy latest camera buffer into our defined buffer
	Camera.retrieve(img_buf);

	// Copy image buffer data into OpenCV Mat image
	imageMat = cv::Mat(Camera.getHeight(), Camera.getWidth(), CV_8UC3, img_buf);

	// Exit if there is no image data in OpenCV image Mat
	if (!imageMat.data)
	{
		std::cout << "No data in Mat imageMat." << std::endl;

		// Release Raspberry Pi camera resources
		Camera.release();

		return;
	}

	// Display Image
	cv::imshow("Image", imageMat);
}
