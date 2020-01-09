#include "stdafx.h"
#include "./aruco/ArucoMarker.h"

// Constructor for Marker class; user can define what pre-defined index of markers this object will create/detect
ArucoMarker::ArucoMarker(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary)
{
	this->dictionary = cv::aruco::getPredefinedDictionary(dictionary);
}

// This function creates a marker from the object-specific dictionary using user inputted ID and size of image
cv::Mat ArucoMarker::generateMarker(int markerID, int markerSize)
{
	cv::Mat markerImage;
	cv::aruco::drawMarker(dictionary, markerID, markerSize, markerImage, 1);
	return markerImage;
}

// Returns the annotated frame where marker was detected; returns original if nothing was detected
cv::Mat ArucoMarker::detectMarker(cv::Mat image)
{
	std::vector<int> ids; // Vector containing ids of markers detected from the image
	std::vector<std::vector<cv::Point2f>> corners; // Vector containing the corner coordinates of markers detected from the image

	cv::aruco::detectMarkers(image, dictionary, corners, ids); //detect markers from image

	// if at least one marker detected
	if (ids.size() > 0)
	{
		cv::aruco::drawDetectedMarkers(image, corners, ids);
	}
	return image;
}

// Returns the annotated frame where marker was detected, including pose estimation; returns original if nothing was detected
cv::Mat ArucoMarker::detectMarker(cv::Mat image, cv::Mat cameraMatrix, cv::Mat distortionCoeff)
{
	std::vector<int> ids; // Vector containing ids of markers detected from the image
	std::vector<std::vector<cv::Point2f>> corners; // Vector containing the corner coordinates of markers detected from the image

	cv::aruco::detectMarkers(image, dictionary, corners, ids); //detect markers from image

	// if at least one marker detected
	if (ids.size() > 0)
	{
		cv::aruco::drawDetectedMarkers(image, corners, ids);
		std::vector<cv::Vec3d> rvecs, tvecs; // pose parameters
		cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distortionCoeff, rvecs, tvecs); // 0.05 is the arbitrary length for the axes to be drawn
		// draw axis for each marker
		for (int i = 0; i<ids.size(); i++)
			cv::aruco::drawAxis(image, cameraMatrix, distortionCoeff, rvecs[i], tvecs[i], 0.05);
	}
	return image;
}