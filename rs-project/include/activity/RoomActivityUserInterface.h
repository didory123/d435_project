#pragma once

class RoomActivityUserInterface
{
public:

	RoomActivityUserInterface
	(
		room_activity_dimensions roomActivityDimensions,
		charuco_dimensions charucoSettings,
		yolo_files_paths yoloFilesPaths,
		room_activity_settings roomActivitySettings
	);

	~RoomActivityUserInterface()
	{
		delete detector_;
	};



	void beginRoomActivitySession();

private:

	cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_6X6_250));

	DeviceWrapper connectedDevices_;
	std::map<std::string, cv::Vec3d> rotationVectors_, translationVectors_;
	cv::Ptr<cv::aruco::CharucoBoard> charucoBoard_;

	cv::Size frameSize_;

	std::map<std::string, Eigen::Matrix4d> calibrationMatrices_;

	const int FRAMES_PER_SECOND = static_cast<int>(E_FRAME_RATE::FPS_30);;

	room_activity_dimensions roomActivityDimensions_;
	charuco_dimensions charucoSettings_;
	yolo_files_paths yoloFilesPaths_;
	room_activity_settings roomActivitySettings_;

	ObjectDetector * detector_;

	std::map<std::string, cv::Mat> initialColorMats_;
	std::map<std::string, cv::Mat> initialDepthMats_;
	std::map<std::string, cv::Mat> depthColorMappers_;

	// Start up OpenCV windows
	std::string mainWindow_ = "Object Tracking", charucoWindow_ = "Calibration", finalWindow_ = "Final Objects";

	std::map<std::string, Eigen::Matrix4d> getCalibrationMatricesFromCharucoboard();
	void runMainRoomActivityLoop();

	void filterAndStoreColorAndDepthFramesToCurrentInitialMats(std::map<std::string, rs2::frame> colorFrames, std::map<std::string, rs2::frame> depthFrames);

	void initializeWindows();
	void initializeDevices();


	void initializeCharucoFromSettings();
};