#pragma once

#include "stdafx.h"
#include "config/AppConfigParser.h"

// How appconfig file parses fields
void AppConfigParser::assignFieldFromValue(const std::string& field, const std::string& value)
{
	// YOLO Files Paths
	if (field == CFG_FILE_PATH)
	{
		filePaths_.cfgFilePath = value;
	}
	else if (field == WEIGHTS_FILE_PATH)
	{
		filePaths_.weightsFilePath = value;
	}
	else if (field == NAMES_FILE_PATH)
	{
		filePaths_.namesFilePath = value;
	}
	// Charuco settings
	else if (field == CHARUCO_NUM_CELLS_X)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			charucoDimension_.cellCountX = std::stoi(value);
		}	
	}
	else if (field == CHARUCO_NUM_CELLS_Y)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			charucoDimension_.cellCountY = std::stoi(value);
		}
	}
	else if (field == CHARUCO_MARKER_LENGTH)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			charucoDimension_.markerLength = std::stof(value);
		}
	}
	else if (field == CHARUCO_SQUARE_LENGTH)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			charucoDimension_.squareLength = std::stof(value);
		}
	}
	// Room activity settings
	else if (field == IDLE_STATE_REFRESH_FRAME_COUNTER)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivitySettings_.idleStateRefreshFrameCounter = std::stoi(value);
		}
	}
	else if (field == ACTIVITY_STATE_REDETECTION_FRAME_COUNTER)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivitySettings_.activityStateRedetectionFrameCounter = std::stoi(value);
		}
	}
	else if (field == EXIT_ACTIVITY_PERSON_MISSING_FRAME_COUNTER)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivitySettings_.exitActivityPersonMissingFrameCounter = std::stoi(value);
		}
	}
	else if (field == RECORD_PERSON_COORDINATE_FRAME_COUNTER)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivitySettings_.recordPersonCoordinateFrameCounter = std::stoi(value);
		}
	}
	else if (field == OBJECT_DETECTION_THRESHOLD)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivitySettings_.objectDetectionThreshold = std::stod(value);
		}
	}
	else
	{
		std::cout << "Unrecognized field name; please check that the appconfig file is configured properly" << std::endl;
	}
}

yolo_files_paths AppConfigParser::getYOLOFilePaths()
{
	return filePaths_;
}

charuco_dimensions AppConfigParser::getCharucoDimensions()
{
	return charucoDimension_;
}

room_activity_settings AppConfigParser::getRoomActivitySettings()
{
	return roomActivitySettings_;
}
