#pragma once
#include "stdafx.h"
#include "config/RoomConfigParser.h"


// Returns NULL if invalid
std::unique_ptr<room_activity_dimensions> RoomConfigParser::getRoomActivityDimensions()
{
	return std::move(roomActivityDimensions_);
}

// How room config file parses fields
void RoomConfigParser::assignFieldFromValue(const std::string& field, const std::string& value)
{
	// If currently NULL, set to new instance
	if (roomActivityDimensions_ == nullptr)
	{
		roomActivityDimensions_.reset(new room_activity_dimensions);
	}

	if (field == LEFT_DISTANCE)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivityDimensions_->leftDistance = std::stof(value);
		}
	}
	else if (field == RIGHT_DISTANCE)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivityDimensions_->rightDistance = std::stof(value);
		}
	}
	else if (field == CEILING_DISTANCE)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivityDimensions_->ceilingDistance = std::stof(value);
		}
	}
	else if (field == FLOOR_DISTANCE)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivityDimensions_->floorDistance = std::stof(value);
		}
	}
	else if (field == CAMERA_DISTANCE)
	{
		if (isFieldHaveValidNumberAsValue(field, value))
		{
			roomActivityDimensions_->cameraDistance = std::stof(value);
		}
	}
	else if (field == FRAME_SIZE)
	{
		if (value == "1")
		{
			roomActivityDimensions_->frameSize = SMALL_DIMS;
		}
		else if (value == "2")
		{
			roomActivityDimensions_->frameSize = MEDIUM_DIMS;
		}
		else if (value == "3")
		{
			roomActivityDimensions_->frameSize = LARGE_DIMS;
		}
		else if (value == "4")
		{
			roomActivityDimensions_->frameSize = LARGEST_DIMS;
		}
		else
		{
			roomActivityDimensions_.reset(nullptr);
			std::cout << "Unrecognized value for frame size; please check that your room configuration file is configured properly" << std::endl;
		}
	}
	else
	{
		roomActivityDimensions_.reset(nullptr);
		std::cout << "Unrecognized field name; please check that the room configuration file is configured properly" << std::endl;
	}
}
