#pragma once

class AppConfigParser
{
public:
	AppConfigParser();
	~AppConfigParser();

	/*
	std::pair<std::string, std::string> getFieldAndValueFromLine(const std::string& line);

	void assignValueToField(const std::string& value, const std::string& field);

	void parseAppConfigFieldsByLine(const std::string& filePath);

	bool isLineCommented(const std::string& line);
	*/
private:

	const std::string CFG_FILE_PATH = "cfgFilePath";
	const std::string WEIGHTS_FILE_PATH = "weightsFilePath";
	const std::string NAMES_FILE_PATH = "namesFilePath";

	const std::string CHARUCO_NUM_CELLS_X = "charucoNumCellsX";
	const std::string CHARUCO_NUM_CELLS_Y = "charucoNumCellsY";
	const std::string CHARUCO_SQUARE_LENGTH = "charucoSquareLength";
	const std::string CHARUCO_MARKER_LENGTH = "charucoMarkerLength";

	yolo_files_paths filePaths_;
};