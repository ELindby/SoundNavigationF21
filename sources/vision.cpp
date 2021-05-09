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
	camera.setFormat(raspicam::RASPICAM_FORMAT_BGR);
	// Set image resolution
	camera.setWidth(640);
	camera.setHeight(480);
	// Flip camera image vertically and horizontally
	// because camera is mounted upside down
	camera.setVerticalFlip(true);
	camera.setHorizontalFlip(true);

	// Display current camera parameters
	/*std::cout << "Format: " << camera.getFormat() << std::endl;
	std::cout << "Width: " << camera.getWidth() << std::endl;
	std::cout << "Height: " << camera.getHeight() << std::endl;
	std::cout << "Brightness: " << camera.getBrightness() << std::endl;
	std::cout << "Rotation: " << camera.getRotation() << std::endl;
	std::cout << "ISO: " << camera.getISO() << std::endl;
	std::cout << "Sharrpness: " << camera.getSharpness() << std::endl;
	std::cout << "Contrast: " << camera.getContrast() << std::endl;
	std::cout << "Saturation: " << camera.getSaturation() << std::endl;
	std::cout << "ShutterSpeed: " << camera.getShutterSpeed() << std::endl;
	std::cout << "Exopsure: " << camera.getExposure() << std::endl;
	std::cout << "AWB: " << camera.getAWB() << std::endl;
	std::cout << "Image effect: " << camera.getImageEffect() << std::endl;
	std::cout << "Metering: " << camera.getMetering() << std::endl;
	std::cout << "Format:" << camera.getFormat() << std::endl;
	std::cout << "Body: " << "Ready" << std::endl;*/

	// Open camera
	if (!camera.open())
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
	img_buf_len = camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_BGR);
	img_buf = new unsigned char[img_buf_len];

	// Initialise OpenCV image Mat
	imageMat = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC3, img_buf);

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
    //char k;
    while(true){
        // Grab image into internal buffer
        camera.grab();

        // Copy latest camera buffer into our defined buffer
        camera.retrieve(img_buf);

        // Copy image buffer data into OpenCV Mat image
        imageMat = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC3, img_buf);

        // Exit if there is no image data in OpenCV image Mat
        if (!imageMat.data)
        {
            std::cout << "No data in Mat imageMat." << std::endl;

            // Release Raspberry Pi camera resources
            camera.release();

            break;
        }

        // Display Image
        cv::imshow("Image", imageMat);

        //cv::waitKey(30);
        k = cv::waitKey(100);
        if(k == 27) //27 = 'ESC'
            break;
    }
    releaseCamera();
}

void Vision::releaseCamera()
{
	camera.release();
	std::cout << "Camera resources released." << std::endl;
}
