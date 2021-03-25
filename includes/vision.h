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
	raspicam::RaspiCam camera;									// Create Raspberry Pi camera object
	int img_buf_len;											// Create buffer of correct size to store image data
	unsigned char *img_buf;										// Create buffer of correct size to store image data
	cv::Mat imageMat;											// Initialise OpenCV image Mat



	//Blob Detector
	cv::SimpleBlobDetector::Params sbdPar_red, sbdPar_black;	// Create OpenCV SimpleBlobDetector parameter objects (One for white object detection (red), one for black object detection).
	cv::Ptr<cv::SimpleBlobDetector> sbd_red, sbd_black;			// Create OpenCV SimpleBlobDetector object based on assigned parameters
	cv::Mat imageThreshold_red, imageThreshold_black;			// OpenCV image Mat that holds the thresholded image data
	cv::Mat imageKeypoints_red, imageKeypoints_black;			// OpenCV image Mat to store image with detected blobs
	std::vector<cv::KeyPoint> keypts_red, keypts_black;
	std::vector<cv::Point2f> keyptXY_red, keyptXY_black;				// Vector storing [x,y] co-ordinates of detected blobs

public:
    //waitkey, written to by vision thread, read by main, terminates control loop.
	std::atomic<char> k;

	Vision();
	~Vision();
	void setupSimpleBlobDetector();
	void updateCamera();
	void releaseCamera();
};
