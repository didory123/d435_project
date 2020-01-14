#pragma once

class AppConfigParser : public ConfigParser
{
public:
	AppConfigParser() {};
	~AppConfigParser() {};

	yolo_files_paths getYOLOFilePaths();

	charuco_dimensions getCharucoDimensions();

	room_activity_settings getRoomActivitySettings();

private:


	void assignFieldFromValue(const std::string& field, const std::string& value);

	// -------------------------------------------
	// Field names in the appconfig file
	// -------------------------------------------

	// yolo file paths field names
	const std::string CFG_FILE_PATH = "cfgFilePath";
	const std::string WEIGHTS_FILE_PATH = "weightsFilePath";
	const std::string NAMES_FILE_PATH = "namesFilePath";

	// charuco parameters field names
	const std::string CHARUCO_NUM_CELLS_X = "charucoNumCellsX";
	const std::string CHARUCO_NUM_CELLS_Y = "charucoNumCellsY";
	const std::string CHARUCO_SQUARE_LENGTH = "charucoSquareLength";
	const std::string CHARUCO_MARKER_LENGTH = "charucoMarkerLength";

	// room activity specific settings
	const std::string IDLE_STATE_REFRESH_FRAME_COUNTER = "idleStateRefreshFrameCounter";
	const std::string ACTIVITY_STATE_REDETECTION_FRAME_COUNTER = "activityStateRedetectionFrameCounter";
	const std::string EXIT_ACTIVITY_PERSON_MISSING_FRAME_COUNTER = "exitActivityPersonMissingFrameCounter";
	const std::string RECORD_PERSON_COORDINATE_FRAME_COUNTER = "recordPersonCoordinateFrameCounter";
	const std::string OBJECT_DETECTION_THRESHOLD = "objectDetectionThreshold";

	// -------------------------------------------
	// Default values
	// -------------------------------------------

	// Default values for yolo file paths
	const std::string DEFAULT_CFG_FILE_PATH = boost::filesystem::current_path().string() + "/yolov3.cfg";
	const std::string DEFAULT_WEIGHTS_FILE_PATH = boost::filesystem::current_path().string() + "/yolov3.weights";
	const std::string DEFAULT_NAMES_FILE_PATH = boost::filesystem::current_path().string() + "/coco.names";
	
	// Default values for charuco settings
	const int DEFAULT_CHARUCO_NUM_CELLS_X = 5;
	const int DEFAULT_CHARUCO_NUM_CELLS_Y = 7;
	const float DEFAULT_CHARUCO_SQUARE_LENGTH = 0.08;
	const float DEFAULT_CHARUCO_MARKER_LENGTH = 0.048;

	// Default values for room activity settings
	const int DEFAULT_IDLE_STATE_REFRESH_FRAME_COUNTER = 120;
	const int DEFAULT_ACTIVITY_STATE_REDETECTION_FRAME_COUNTER = 90;
	const int DEFAULT_EXIT_ACTIVITY_PERSON_MISSING_FRAME_COUNTER = 120;
	const int DEFAULT_RECORD_PERSON_COORDINATE_FRAME_COUNTER = 15;
	const double DEFAULT_CONFIDENCE_THRESHOLD = 0.5;

	// -------------------------------------------
	// Data structs for the room activity settings
	// -------------------------------------------

	yolo_files_paths filePaths_ = {
		DEFAULT_CFG_FILE_PATH,
		DEFAULT_WEIGHTS_FILE_PATH,
		DEFAULT_NAMES_FILE_PATH
	};

	charuco_dimensions charucoDimension_ = {
		DEFAULT_CHARUCO_NUM_CELLS_X,
		DEFAULT_CHARUCO_NUM_CELLS_Y,
		DEFAULT_CHARUCO_SQUARE_LENGTH,
		DEFAULT_CHARUCO_MARKER_LENGTH
	};

	room_activity_settings roomActivitySettings_ = {
		DEFAULT_IDLE_STATE_REFRESH_FRAME_COUNTER,
		DEFAULT_ACTIVITY_STATE_REDETECTION_FRAME_COUNTER,
		DEFAULT_EXIT_ACTIVITY_PERSON_MISSING_FRAME_COUNTER,
		DEFAULT_RECORD_PERSON_COORDINATE_FRAME_COUNTER,
		DEFAULT_CONFIDENCE_THRESHOLD
	};
};