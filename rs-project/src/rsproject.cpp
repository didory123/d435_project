// Jim Park 
// 2019/01/08
// Simple console app for detecting markers from real-time frames from the Intel Realsense D435 camera
//
#include "stdafx.h"
#include "TestDriver.hpp"

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
void getRoomDimensionsFromUser(float& leftWidthVal, float& rightWidthVal, float& topHeightVal, float& bottomHeightVal, float& cameraDistanceVal)
{
	std::string userInput = "";
	std::cout << "Input the distance from the left wall of the room to the marker 0-axis in meters" << std::endl;
	// Input room width
	while (std::getline(std::cin, userInput))
	{
		try
		{
			leftWidthVal = std::stof(userInput);
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
			rightWidthVal = std::stof(userInput);
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
			topHeightVal = std::stof(userInput);
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
			bottomHeightVal = std::stof(userInput);
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
			cameraDistanceVal = std::stof(userInput);
			break;
		}
		catch (const std::exception & e)
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}
}

// Parse and run the desired miscellaneous module based on the user's input
bool runSpecifiedMiscModule(const std::string& testCode)
{
	if (testCode == "1")
	{
		TestDriver::arucoTest();
	}
	else if (testCode == "2")
	{
		TestDriver::pointCloudExportTest();
	}
	else if (testCode == "3")
	{
		TestDriver::pointCloudVisualize();
	}
	else if (testCode == "4")
	{
		TestDriver::multiCamVisualize();
	}
	else if (testCode == "5")
	{
		TestDriver::project3DMultiple();
	}
	else if (testCode == "6")
	{
		TestDriver::objectDetection();
	}
	else if (testCode == "7")
	{
		TestDriver::objectTracking();
	}
	else
	{
		return false;
	}
	return true;
}

// Get required setup information from the user before starting the room activity application
int roomActivitySetup()
{
	std::string userInput = "";
	
	// Get the dimensions of the room from the user's input
	float leftWidthVal = 0, rightWidthVal = 0, topHeightVal = 0, bottomHeightVal = 0, cameraDistanceVal = 0;
	getRoomDimensionsFromUser(leftWidthVal, rightWidthVal, topHeightVal, bottomHeightVal, cameraDistanceVal);

	// Get the desired size of the video frames from the user
	cv::Size frameSize = getFrameSizeFromUser();

	return TestDriver::roomActivityVisualize(leftWidthVal, rightWidthVal, topHeightVal, bottomHeightVal, cameraDistanceVal, frameSize);
}

void miscellaneousModulesSelection()
{
	std::string userInput = "";
	std::cout << "Input 1 for AruCo test program" << std::endl;
	std::cout << "Input 2 to test exporting pointcloud of a frame to .ply file" << std::endl;
	std::cout << "Input 3 to test pointcloud visualization" << std::endl;
	std::cout << "Input 4 to test multi-camera pointcloud visualization" << std::endl;
	std::cout << "Input 5 to test mapping multiple, 3D pointclouds to real world coordinates" << std::endl;
	std::cout << "Input 6 to test real-time YOLO object detection" << std::endl;
	std::cout << "Input 7 to test object tracking" << std::endl;
	std::cout << "Input 8 to test multicamera automatic person tracking" << std::endl;
	std::cout << "Input 9 to test automatic person tracking" << std::endl;

	// get user input
	while (std::getline(std::cin, userInput))
	{
		// Run the specified miscellaneous module
		if (runSpecifiedMiscModule(userInput))
		{
			break;
		}
		else
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}
}

// main program entry
int main()
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
		
		std::cout << "Input 1 for Room Activity Detection" << std::endl;
		std::cout << "Input 2 to see list of miscellaneous modules" << std::endl;

		// get user input
		while (std::getline(std::cin, userInput))
		{
			if (userInput == "1")
			{
				roomActivitySetup();
				break;
			}
			else if (userInput == "2")
			{
				miscellaneousModulesSelection();
				break;
			}
			else
			{
				std::cout << "Not a valid input, please try again." << std::endl;
			}
		}
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
