#pragma once
#include "stdafx.h"
#include "Utils.h"
#include "activity\RoomActivityUserInterface.h"
#include "activity\RoomActivity.h"

RoomActivityUserInterface::RoomActivityUserInterface
(
	room_activity_dimensions roomActivityDimensions,
	charuco_dimensions charucoSettings,
	yolo_files_paths yoloFilesPaths,
	room_activity_settings roomActivitySettings
) :
	roomActivityDimensions_(roomActivityDimensions),
	charucoSettings_(charucoSettings),
	yoloFilesPaths_(yoloFilesPaths),
	roomActivitySettings_(roomActivitySettings)
{
	frameSize_ = roomActivityDimensions.frameSize;
	detector_ = new ObjectDetector(yoloFilesPaths, 0.5);
}

// Begin the room activity
void RoomActivityUserInterface::beginRoomActivitySession()
{
	// Initialize settings
	initializeWindows();
	initializeDevices();
	initializeCharucoFromSettings();

	// Construct calibration matrices
	calibrationMatrices_ = getCalibrationMatricesFromCharucoboard();

	// Get initial snapshot of the room at the very beginning
	connectedDevices_.pollFrames();
	auto colorFrames = connectedDevices_.getRGBFrames();
	auto depthFrames = connectedDevices_.getDepthFrames();
	filterAndStoreColorAndDepthFramesToCurrentInitialMats(colorFrames, depthFrames);

	runMainRoomActivityLoop();
}

// Create main window for viewing the frames from the connected cameras
void RoomActivityUserInterface::initializeWindows()
{
	//cv::namedWindow(initialWindow, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(mainWindow_, cv::WINDOW_AUTOSIZE);
}

// Initialize the connected devices for streaming
void RoomActivityUserInterface::initializeDevices()
{
	// Initialize the streams that you want to start
	stream_profile_detail stream_depth =
	{
		frameSize_.width,
		frameSize_.height,
		RS2_STREAM_DEPTH,
		rs2_format::RS2_FORMAT_Z16,
		E_FRAME_RATE::FPS_30,
	};
	stream_profile_detail stream_color =
	{
		frameSize_.width,
		frameSize_.height,
		RS2_STREAM_COLOR,
		rs2_format::RS2_FORMAT_BGR8,
		E_FRAME_RATE::FPS_30,
	};

	std::vector<stream_profile_detail> streams = { stream_depth, stream_color };

	rs2::context ctx;    // Create librealsense context for managing devices
	ctx.set_devices_changed_callback([&](rs2::event_information& info)
	{
		connectedDevices_.removeAllDevices(info);
		for (auto&& dev : info.get_new_devices())
		{
			connectedDevices_.enableDeviceToProfiles(dev, streams);
		}
	});

	// Initial population of the device list
	for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
	{
		connectedDevices_.enableDeviceToProfiles(dev, streams);

	}
}

void RoomActivityUserInterface::initializeCharucoFromSettings()
{
	//cv::Ptr<cv::aruco::CharucoBoard> charucoboard = 
	charucoBoard_ =
		cv::aruco::CharucoBoard::create(
			charucoSettings_.cellCountX,
			charucoSettings_.cellCountY,
			charucoSettings_.squareLength,
			charucoSettings_.markerLength,
			dictionary_
		);

	//charucoBoard_ = charucoboard.staticCast<cv::aruco::Board>();
}

// Opens up a window for the user to manually detect and construct calibration matrices from a charucoboard
std::map<std::string, Eigen::Matrix4d> RoomActivityUserInterface::getCalibrationMatricesFromCharucoboard()
{

	// Get camera properties required for constructing calibration matrices
	cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << 0, 0, 0, 0, 0);
	auto cameraMatrices = connectedDevices_.getCameraMatricesMap();

	// Rotatetion and translation vectors that will make up the calibration matrices
	std::map<std::string, cv::Vec3d> rotationVectors, translationVectors;

	while (1) {

		connectedDevices_.pollFrames();

		cv::Vec3d rvec, tvec;
		std::map<std::string, cv::Vec3d> rvecs, tvecs;

		auto colorFrames = connectedDevices_.getRGBFrames();
		std::vector<cv::Mat> images;
		for (auto &frame : colorFrames)
		{
			cv::Mat image = Utils::frameToMat(frame.second).clone();
			cv::Mat imageCopy;

			std::vector< int > markerIds;
			std::vector< std::vector<cv::Point2f>> corners;
			// detect markers	
			cv::aruco::detectMarkers(image, dictionary_, corners, markerIds);

			// interpolate charuco corners
			std::vector<cv::Point2f> currentCharucoCorners;
			std::vector<int> currentCharucoIds;
			int interpolatedCorners = 0;
			if (markerIds.size() > 0)
			{
				interpolatedCorners =
					cv::aruco::interpolateCornersCharuco
					(
						corners,
						markerIds,
						image,
						charucoBoard_,
						currentCharucoCorners,
						currentCharucoIds,
						cameraMatrices[frame.first],
						distortionCoefficients
					);
			}

			// estimate charuco board pose
			cv::aruco::estimatePoseCharucoBoard(
				currentCharucoCorners,
				currentCharucoIds,
				charucoBoard_,
				cameraMatrices[frame.first],
				distortionCoefficients,
				rvec,
				tvec
			);

			// draw results
			image.copyTo(imageCopy);

			// draw aruco markers
			if (markerIds.size() > 0)
			{
				cv::aruco::drawDetectedMarkers(imageCopy, corners);
			}
			// draw things
			if (interpolatedCorners > 0)
			{
				// draw corners
				cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
				// draw pose
				cv::aruco::drawAxis(imageCopy, cameraMatrices[frame.first], distortionCoefficients, rvec, tvec, 0.05);
			}
			rvecs[frame.first] = rvec;
			tvecs[frame.first] = tvec;
			images.push_back(imageCopy);

		}
		if (images.size() > 0)
		{
			//cv::imshow(window_name, images[0]);
			cv::Mat combinedImage;
			cv::hconcat(images, combinedImage);
			putText(combinedImage, "Press any button when your marker is visible in all camera streams.",
				cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
			cv::imshow(mainWindow_, combinedImage);
		}
		char key = (char)cv::waitKey(10);
		if (key > 0)
		{
			if (rvecs.size() > 0 && tvecs.size() > 0)
			{
				rotationVectors = rvecs;
				translationVectors = tvecs;
			}
			break;
		}
	}

	std::map<std::string, Eigen::Matrix4d> calibrationMatrices;
	// Create map of calibration matrices
	for (auto &vec : rotationVectors)
	{
		calibrationMatrices[vec.first] = Utils::getTransformMatrix(vec.second, translationVectors[vec.first]);
	}

	return calibrationMatrices;
}

void RoomActivityUserInterface::runMainRoomActivityLoop()
{
	int frameCount = 0;
	while (cv::waitKey(1) < 0)
	{
		connectedDevices_.pollFrames();
		auto colorFrames = connectedDevices_.getRGBFrames();

		// Every user-specified number of frames, check that a person is in the room
		if (frameCount == roomActivitySettings_.idleStateRefreshFrameCounter)
		{

			std::map<std::string, cv::Rect2d> personBboxes;

			auto depthFrames = connectedDevices_.getDepthFrames();
			bool isPersonInFrame = false;
			for (auto & framePair : colorFrames)
			{
				cv::Rect2d bbox;
				cv::Mat image = Utils::frameToMat(framePair.second).clone();

				if (detector_->detectPerson(image, bbox))
				{
					isPersonInFrame = true;
				}

				// We still want to update the person bbox for all cameras every time, because if 
				// even one device finds a person, we want to start an activity, therefore we will preserve all potential data
				personBboxes[framePair.first] = bbox;
			}

			// If person is detected, an "activity" is happening
			// Several steps will occur at this point:
			// 1. Get the initial snapshot of all objects in the room
			// 2. Track the person that just entered
			// 3. Get the final snapshot of all object in the room when person leaves
			// 4. Store all data to a folder specific to this "activity" then go back to idle mode, waiting for a person
			if (isPersonInFrame)
			{
				RoomActivity newActivity(
					"KCF",
					initialColorMats_,
					initialDepthMats_,
					depthColorMappers_,
					personBboxes,
					calibrationMatrices_,
					mainWindow_,
					*detector_,
					connectedDevices_,
					roomActivityDimensions_,
					roomActivitySettings_
				);
			}
			// If no person was detected, use this as an opportunity to update the current initial snapshots
			else
			{
				filterAndStoreColorAndDepthFramesToCurrentInitialMats(colorFrames, depthFrames);
			}
			frameCount = 0;
		}
		else
		{
			// Add some descriptions
			cv::Mat image;
			std::vector<cv::Mat> images;
			for (auto & frame : colorFrames)
			{
				cv::Rect2d bbox;
				cv::Mat cameraImage = Utils::frameToMat(frame.second);
				images.push_back(cameraImage.clone());

			}
			cv::hconcat(images, image);

			cv::putText(image,
				"Idle mode",
				cv::Point(10, 30),
				cv::FONT_HERSHEY_SIMPLEX,
				0.55,
				cv::Scalar(15, 125, 255),
				2);

			cv::putText(image,
				"Next detection in " + std::to_string(roomActivitySettings_.idleStateRefreshFrameCounter - frameCount),
				cv::Point(10, image.rows - 20),
				cv::FONT_HERSHEY_SIMPLEX,
				0.75,
				cv::Scalar(255, 0, 255),
				2);
			cv::imshow(mainWindow_, image);
		}
		frameCount++;

	}
}

void RoomActivityUserInterface::filterAndStoreColorAndDepthFramesToCurrentInitialMats(std::map<std::string, rs2::frame> colorFrames, std::map<std::string, rs2::frame> depthFrames)
{

	//connectedDevices_.pollFrames();
	//auto colorFrames = connectedDevices_.getRGBFrames();
	//auto depthFrames = connectedDevices_.getDepthFrames();

	// Realsense object for mapping color to depth
	rs2::colorizer colorMapper;

	// Run the filters
	// IMPORTANT: The filters must be run in the exact order as written below
	rs2::decimation_filter dec_filter;  // 1. Decimation - reduces depth frame density

	// Declare disparity transform from depth to disparity and vice versa
	const std::string disparity_filter_name = "Disparity"; // 2. Depth to disparity
	rs2::disparity_transform depth_to_disparity(true);

	rs2::spatial_filter spat_filter;    // 3. Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;   // 4. Temporal   - reduces temporal noise

	rs2::disparity_transform disparity_to_depth(false); // 5. Disparity back to depth

	for (auto & framePair : colorFrames)
	{

		//depthFrames[framePair.first] = dec_filter.process(depthFrames[framePair.first]); // 1
		depthFrames[framePair.first] = depth_to_disparity.process(depthFrames[framePair.first]); // 2
		depthFrames[framePair.first] = spat_filter.process(depthFrames[framePair.first]); // 3
		depthFrames[framePair.first] = temp_filter.process(depthFrames[framePair.first]); // 4
		depthFrames[framePair.first] = disparity_to_depth.process(depthFrames[framePair.first]); // 5

		initialColorMats_[framePair.first] = Utils::frameToMat(framePair.second).clone();
		initialDepthMats_[framePair.first] = Utils::frameToMat(depthFrames[framePair.first]).clone();

		rs2::frame coloredDepth = depthFrames[framePair.first].apply_filter(colorMapper);

		depthColorMappers_[framePair.first] = Utils::frameToMat(framePair.second).clone();
		//cv::Mat(frameSize_, CV_8UC3, (void*)coloredDepth.get_data(), cv::Mat::AUTO_STEP);
		// TODO: Comment below line and uncomment above line
		//depthColorMappers[framePair.first] = Utils::frameToMat(framePair.second).clone();

	}
}