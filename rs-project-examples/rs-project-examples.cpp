// Jim Park 
// 2019/01/08
// Simple console app for detecting markers from real-time frames from the Intel Realsense D435 camera
//
#pragma once

#include "stdafx.h"
#include "DataTypes.h"

#include "./device/DeviceWrapper.h"
#include "./detector/ObjectDetector.h"
#include "ModulesFreeFunctions.hpp"


// Parse and run the desired miscellaneous module based on the user's input
bool runSpecifiedMiscModule(const std::string& testCode)
{
	if (testCode == "1")
	{
		ModulesFreeFunctions::arucoTest();
	}
	else if (testCode == "2")
	{
		ModulesFreeFunctions::pointCloudExportTest();
	}
	else if (testCode == "3")
	{
		ModulesFreeFunctions::pointCloudVisualize();
	}
	else if (testCode == "4")
	{
		ModulesFreeFunctions::multiCamVisualize();
	}
	else if (testCode == "5")
	{
		ModulesFreeFunctions::project3DMultiple();
	}
	else if (testCode == "6")
	{
		ModulesFreeFunctions::objectDetection();
	}
	else if (testCode == "7")
	{
		ModulesFreeFunctions::objectTracking();
	}
	else
	{
		return false;
	}
	return true;
}

void showMiscellaneousModulesSelectionPrompts()
{
	std::string userInput = "";
	std::cout << "Input 1 for AruCo test program" << std::endl;
	std::cout << "Input 2 to test exporting pointcloud of a frame to .ply file" << std::endl;
	std::cout << "Input 3 to test pointcloud visualization" << std::endl;
	std::cout << "Input 4 to test multi-camera pointcloud visualization" << std::endl;
	std::cout << "Input 5 to test mapping multiple, 3D pointclouds to real world coordinates" << std::endl;
	std::cout << "Input 6 to test real-time YOLO object detection" << std::endl;
	std::cout << "Input 7 to test object tracking" << std::endl;

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

		showMiscellaneousModulesSelectionPrompts();

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
