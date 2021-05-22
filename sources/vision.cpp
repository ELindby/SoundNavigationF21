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

    // Create a bunch of windows for displaying image processing steps
	// Create window to display original image
	//cv::namedWindow("Image",cv::WINDOW_AUTOSIZE);
	// Create window to display original HSV image
	//cv::namedWindow("HSV image",cv::WINDOW_AUTOSIZE);
	// Create window to display thresholded image
	//cv::namedWindow("Thresholded image",cv::WINDOW_AUTOSIZE);
	// Create window to display blob image
	cv::namedWindow("Blobs", cv::WINDOW_AUTOSIZE);
}


Vision::~Vision()
{
}

void Vision::setupSimpleBlobDetector()
{
/*****************************************************************************
*********************  SETUP SIMPLE BLOB DETECTOR   **************************
*****************************************************************************/
	// Create OpenCV SimpleBlobDetector parameter object (Stored as member variable instead)
	//cv::SimpleBlobDetector::Params sbdPar;

	// Set blob detection parameters
	// Change thresholds
	sbdPar.minThreshold = 1;
	sbdPar.maxThreshold = 1000;

	// Filter by colour
	sbdPar.filterByColor = true;

	// Look for colours that match grayscale value of 255 (white) or 0 (black)
	sbdPar.blobColor = 0;

	// Filter by area
	sbdPar.filterByArea = true;
	sbdPar.minArea = 100; // 10x10 pixels
	sbdPar.maxArea = 160000; // 400x400 pixels

	// Create OpenCV SimpleBlobDetector object based on assigned parameters
	sbd = cv::SimpleBlobDetector::create(sbdPar);
	//std::vector<cv::KeyPoint> keypts;

	/*****************************************************************************
	************************   INITIALISE VISUALISATION   ************************
	*****************************************************************************/
	//~ // Create a window for displaying HSV
	//~ cv::namedWindow("HSV controls",cv::WINDOW_NORMAL);

	//~ // Create trackbars for H, S and V in the window
	//~ cv::createTrackbar("LowH", "HSV controls", &iLowH, 179); //Hue (0 - 179)
	//~ cv::createTrackbar("HighH", "HSV controls", &iHighH, 179);
	//~ cv::createTrackbar("LowS", "HSV controls", &iLowS, 255); //Saturation (0 - 255)
	//~ cv::createTrackbar("HighS", "HSV controls", &iHighS, 255);
	//~ cv::createTrackbar("LowV", "HSV controls", &iLowV, 255); //Value (0 - 255)
	//~ cv::createTrackbar("HighV", "HSV controls", &iHighV, 255);
	/*********************************   DONE   *********************************/
}

void Vision::simpleBlobDetector() {
	// Convert image from BGR to HSV
	cv::cvtColor(imageMat, imageMatHSV, cv::COLOR_BGR2HSV);

	//Threshold image
	cv::inRange(imageMatHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imageThreshold);

	//Perform morphological opening and closing to smooth out object (Repeated once)
	erode (imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))); //morphological opening (remove small objects from the foreground)
	dilate(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))); //morphological closing (fill small holes in the foreground)
	erode (imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	dilate(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	// Display HSV image
	//cv::imshow("HSV image", imageMatHSV);

	// Display thresholded imnage
	//cv::imshow("Thresholded image",imageThreshold);

	// Detect keypoints in thresholded image using SimpleBlobDetector object
	sbd->detect(imageThreshold, keypts);

	// Draw detected keypoints as red/rblue(black) circles around detected blobs and store new image in OpenCV image Mat
	//cv::drawKeypoints(imageThreshold, keypts, imageKeypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cv::drawKeypoints(imageMat, keypts, imageKeypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	// Display image with detected blobs
	cv::imshow("Blobs", imageKeypoints);

	// Extract [x,y] co-ordinates of blobs from keypoints
	cv::KeyPoint::convert(keypts, keyptXY);

	//Handle detected object, if target is within center and within close proximity, give that information to main thread
	if (keyptXY.size() >= 1) { //If there is a detected target
		if (keypts.front().size >= TARGET_SIZE_THRESHOLD) { //If detected target is close enough (Blob is within size)
			//Target has been found.
			target_found = true;
		}
	}
}

bool Vision::getTargetFound() {
	//Target found is atomic bool (non copyable)
	if (target_found.load()) { //If target_found == true
		return true;
	}
	else
	{
		return false;
	}
}

void Vision::resetTargetFound() {
	target_found.store(false);
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
        //cv::imshow("Image", imageMat);
        //NOTE: Image with detected blobs drawn is shown in SBD instead

		//Perform blob detection
		simpleBlobDetector();

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
