#pragma once

class RoomConfigParser : public ConfigParser
{
public:
	RoomConfigParser() {};
	~RoomConfigParser() {};

	std::unique_ptr<room_activity_dimensions> getRoomActivityDimensions();

private:

	const std::string LEFT_DISTANCE = "leftDistance";
	const std::string RIGHT_DISTANCE = "rightDistance";
	const std::string CEILING_DISTANCE = "ceilingDistance";
	const std::string FLOOR_DISTANCE = "floorDistance";
	const std::string CAMERA_DISTANCE = "cameraDistance";
	const std::string FRAME_SIZE = "frameSize";

	void assignFieldFromValue(const std::string& field, const std::string& value);

	std::unique_ptr<room_activity_dimensions> roomActivityDimensions_ = NULL;
};