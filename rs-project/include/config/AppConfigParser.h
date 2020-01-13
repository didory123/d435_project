#pragma once

class AppConfigParser : public ConfigParser
{
public:
	AppConfigParser() {};
	~AppConfigParser() {};

	yolo_files_paths getYOLOFilePaths();

	charuco_dimensions getCharucoDimensions();

	

private:


	void assignFieldFromValue(const std::string& field, const std::string& value);


	const std::string CFG_FILE_PATH = "cfgFilePath";
	const std::string WEIGHTS_FILE_PATH = "weightsFilePath";
	const std::string NAMES_FILE_PATH = "namesFilePath";

	const std::string DEFAULT_CFG_FILE_PATH = boost::filesystem::current_path().string() + "/yolov3.cfg";
	const std::string DEFAULT_WEIGHTS_FILE_PATH = boost::filesystem::current_path().string() + "/yolov3.weights";
	const std::string DEFAULT_NAMES_FILE_PATH = boost::filesystem::current_path().string() + "/coco.names";

	const std::string CHARUCO_NUM_CELLS_X = "charucoNumCellsX";
	const std::string CHARUCO_NUM_CELLS_Y = "charucoNumCellsY";
	const std::string CHARUCO_SQUARE_LENGTH = "charucoSquareLength";
	const std::string CHARUCO_MARKER_LENGTH = "charucoMarkerLength";

	const int DEFAULT_CHARUCO_NUM_CELLS_X = 5;
	const int DEFAULT_CHARUCO_NUM_CELLS_Y = 7;
	const float DEFAULT_CHARUCO_SQUARE_LENGTH = 0.08;
	const float DEFAULT_CHARUCO_MARKER_LENGTH = 0.048;

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
};