#pragma once

class Marker
{
public:

	// Main constructor for the simple ArUco marker module
	// Parameter dictionary specifies what pre-defined list of markers that this module will generate and detect
	Marker(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary);

	// Generates an ArUco marker from user-specified parameters
	// Parameter markerID specifies which pre-generated aruco marker to use
	// Parameter markerSize Specifies the dimensions of the image in pixels. For example the value of 200 will create a 200x200 image
	// Returns the created marker image in cv::matrix form
	cv::Mat generateMarker(int markerID, int markerSize);

	// Detects a marker from an inputted frame (in the cv matrix form) and returns another frame with data written on it
	// Parameter image is the frame to be detecting markers from
	// Returns the annotated marker image in cv::matrix form
	cv::Mat detectMarker(cv::Mat image);

	// Detects a marker from an inputted frame (in the cv matrix form) and returns another frame with data written on it, including pose estimation
	// Parameter image is the frame to be detecting markers from
	// Parameter cameraMatrix is the cameraMatrix from calibration parameters
	// Parameter distortionCoeff is the distortion coefficients from calibration parameters
	// Returns the annotated marker image in cv::matrix form
	cv::Mat detectMarker(cv::Mat image, cv::Mat cameraMatrix, cv::Mat distortionCoeff);
private:
	cv::Ptr<cv::aruco::Dictionary> dictionary; //user-defined specified dictionary for markers
};

