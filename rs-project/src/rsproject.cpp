// Jim Park 
// 2019/01/08
// Simple console app for detecting markers from real-time frames from the Intel Realsense D435 camera
//
#include "stdafx.h"
#include "config/AppConfigParser.h"
#include "config/RoomConfigParser.h"
#include "activity/RoomActivityUserInterface.h"

// Simple IO function that parses the user's desired size (resolution) for video frames
cv::Size getFrameSizeFromUser()
{
	cv::Size frameSize;
	std::string userInput = "";
	std::cout << "Input a number from 1-4 to select the size of the video frames (1 is smallest, 4 is largest)" << std::endl;
	while (std::getline(std::cin, userInput))
	{
		if (userInput == "1")
		{
			frameSize = SMALL_DIMS;
		}
		else if (userInput == "2")
		{
			frameSize = MEDIUM_DIMS;
		}
		else if (userInput == "3")
		{
			frameSize = LARGE_DIMS;
		}
		else if (userInput == "4")
		{
			frameSize = LARGEST_DIMS;
		}
		else
		{
			std::cout << "Not a valid input, please try again." << std::endl;
			continue;
		}
		break;
	}
	return frameSize;
}

// Simple IO function that walks the user through inputting the specific dimensions of the room
room_activity_dimensions getRoomDimensionsFromUser()
{
	room_activity_dimensions dimensions;
	std::string userInput = "";
	std::cout << "Input the distance from the left wall of the room to the marker 0-axis in meters" << std::endl;
	// Input room width
	while (std::getline(std::cin, userInput))
	{
		try
		{
			dimensions.leftDistance = std::stof(userInput);
			break;
		}
		catch (const std::exception & e)
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}

	std::cout << "Input the distance from the right wall of the room to the marker 0-axis in meters" << std::endl;
	// Input room length
	while (std::getline(std::cin, userInput))
	{
		try
		{
			dimensions.rightDistance = std::stof(userInput);
			break;
		}
		catch (const std::exception & e)
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}

	std::cout << "Input the distance from the ceiling of the room to the marker 0-axis in meters" << std::endl;
	// Input room length
	while (std::getline(std::cin, userInput))
	{
		try
		{
			dimensions.ceilingDistance = std::stof(userInput);
			break;
		}
		catch (const std::exception & e)
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}

	std::cout << "Input the distance from the floor of the room to the marker 0-axis in meters" << std::endl;
	// Input room length
	while (std::getline(std::cin, userInput))
	{
		try
		{
			dimensions.floorDistance = std::stof(userInput);
			break;
		}
		catch (const std::exception & e)
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}

	std::cout << std::endl;
	std::cout << "Place the 2 cameras at the corners of the room, facing the marker on the wall." << std::endl;
	std::cout << "-------MARKER--------" << std::endl;
	std::cout << "| .   .   |.   .  . |" << std::endl;
	std::cout << "| .  .   .|  .  . . |" << std::endl;
	std::cout << "| . .  . Dist  . .. |" << std::endl;
	std::cout << "| .. .    |      .. |" << std::endl;
	std::cout << "| c1             c2 |" << std::endl;
	std::cout << "---------------------" << std::endl;
	std::cout << std::endl;

	std::cout << "Input the straight line distance between the cameras to the opposite wall in meters (ensure that it is roughly the same for both cameras)" << std::endl;
	// Input camera distance
	while (std::getline(std::cin, userInput))
	{
		try
		{
			dimensions.cameraDistance = std::stof(userInput);
			break;
		}
		catch (const std::exception & e)
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}

	dimensions.frameSize = getFrameSizeFromUser();

	return dimensions;
}

room_activity_dimensions getRoomDimensionsFromInputFile(const std::string& roomConfigFilePath)
{
	RoomConfigParser roomConfigParser;
	roomConfigParser.parseConfigFile(roomConfigFilePath);

	// get the room dimension properties from the specified pre-configured input file
	auto d = roomConfigParser.getRoomActivityDimensions();

	// If the returned struct is null, means that the file was invalid and an error occurred while parsing
	// Go to manual input from user
	if (d == nullptr)
	{
		std::cout << "Room pre-configuration file was invalid, require manual input. Please fill in the following information:" << std::endl;
		return getRoomDimensionsFromUser();
	}

	return *d;
	
}

// main program entry
int main(int argc, char** argv)
{
	std::string userInput = "";

	try
	{
		// Ensure that Intel depth cameras are actually connected to the computer
		std::vector<rs2::device> connectedDevices = DeviceWrapper::getAllConnectedDevices();
		if (connectedDevices.size() == 0)
		{
			std::cout << std::endl;
			std::cout << "-----------------------------------" << std::endl;
			std::cout << "No device connected, please connect a RealSense device" << std::endl;
			rs2::error e("No device connected, please connect a RealSense device");
			std::cout << "-----------------------------------" << std::endl;
			std::cout << std::endl;

			return 0; // terminate program early
		}
		else
		{
			std::cout << std::endl;
			std::cout << "-----------------------------------" << std::endl;
			std::cout << "The following devices were found: " << std::endl;
			for (const auto& device : connectedDevices)
			{
				// Print the name of each connected camera
				std::cout << device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << std::endl;
			}
			std::cout << "-----------------------------------" << std::endl;
			std::cout << std::endl;
		}
		
		std::cout << "Beginning setup for autonomous activity recording." << std::endl;
		std::cout << std::endl;

		room_activity_dimensions dimensions;
		
		//-------------------------------------------
		// Room config
		//-------------------------------------------

		if (argc > 1) // If the user has specified a file path for a preconfigured room settings file
		{
			dimensions = getRoomDimensionsFromInputFile(argv[1]);
		}
		else // otherwise go to manual input
		{
			std::cout << "Room pre-configuration file not found, require manual input. Please fill in the following information:" << std::endl;
			dimensions = getRoomDimensionsFromUser();
		}

		const std::string APP_CONFIG_FILE_NAME = "appconfig";

		//-------------------------------------------
		// Appconfig file parsing
		//-------------------------------------------

		AppConfigParser appConfigParser;
		appConfigParser.parseConfigFile(APP_CONFIG_FILE_NAME);
		auto charucoSettings = appConfigParser.getCharucoDimensions();
		auto yoloFilePaths = appConfigParser.getYOLOFilePaths();
	
		// Begin Room Activity
		RoomActivityUserInterface activityUserInterface(dimensions, charucoSettings, yoloFilePaths);
		activityUserInterface.beginRoomActivitySession();

	}
	catch (const rs2::error & e)
	{
		std::cerr << e.what() << std::endl;
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	catch (const std::exception & e)
	{
		std::cerr << e.what() << std::endl;
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 1;
}
