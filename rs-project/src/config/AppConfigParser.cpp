#pragma once

#include "stdafx.h"
#include "config/AppConfigParser.h"

// How appconfig file parses fields
void AppConfigParser::assignFieldFromValue(const std::string& field, const std::string& value)
{
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