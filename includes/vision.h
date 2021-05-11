#pragma once
/*
 * Description:     Vision (camera and detection) class
 *
 * Author:			Erik Lindby
 *					University of Southern Denmark
 * Creation date:   21-02-2021
 */
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
#include <atomic>

class Vision
{
private:
	//Camera
	raspicam::RaspiCam camera;	// Create Raspberry Pi camera object
	int img_buf_len;			// Create buffer of correct size to store image data
	unsigned char *img_buf;		// Create buffer of correct size to store image data
	cv::Mat imageMat;			// Initialise OpenCV image Mat that holds original image

	//Blob Detector
	cv::SimpleBlobDetector::Params sbdPar;	// Create OpenCV SimpleBlobDetector parameter object
	cv::Ptr<cv::SimpleBlobDetector> sbd;	// Create OpenCV SimpleBlobDetector object based on assigned parameters
	cv::Mat imageMatHSV;					// OpenCV image Mat that holds HSV converted image data
	cv::Mat imageThreshold;					// OpenCV image Mat that holds the thresholded image data
	cv::Mat imageKeypoints;					// OpenCV image Mat to store image with detected blobs
	std::vector<cv::KeyPoint> keypts;
	std::vector<cv::Point2f> keyptXY;		// Vector storing [x,y] co-ordinates of detected blobs

	// SBD threshold values - Remember blobcolor = 255
	int iLowH	= 0;
	int iHighH	= 179;

	int iLowS	= 74;
	int iHighS	= 255;

	int iLowV	= 60;
	int iHighV	= 255;

public:
    //waitkey, written to by vision thread, read by main, terminates control loop.
	std::atomic<char> k;

	//if target has been found this is set to true
	std::atomic<bool> target_found = false;

	Vision();
	~Vision();
	void setupSimpleBlobDetector();
	void simpleBlobDetector();
	void updateCamera();
	void releaseCamera();

	bool getTargetFound();
	void resetTargetFound();
};
