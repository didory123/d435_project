// Jim Park 
// 2019/01/08
// Simple console app for detecting markers from real-time frames from the Intel Realsense D435 camera
//
#include "stdafx.h"
#include "TestDriver.hpp"

// main program entry
int main()
{
	std::string userInput = "";
	try
	{
		rs2::device camera = DeviceWrapper::getADevice();
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
	//DeviceWrapper::setDepthTable(camera);
	std::cout << "Input 1 for AruCo test program" << std::endl;
	std::cout << "Input 2 to test exporting pointcloud of a frame to .ply file" << std::endl;
	std::cout << "Input 3 to test pointcloud visualization" << std::endl;
	std::cout << "Input 4 to test multi-camera pointcloud visualization" << std::endl;
	std::cout << "Input 5 to test mapping multiple, 3D pointclouds to real world coordinates" << std::endl;
	std::cout << "Input 6 to test YOLO object detection" << std::endl;
	std::cout << "Input 7 to test object tracking" << std::endl;
	std::cout << "Input 8 to test multicamera automatic personperson tracking" << std::endl;

	std::cout << "Input 9 to test automatic person tracking" << std::endl;
	// get user input
	while (std::getline(std::cin, userInput))
	{	
		if (userInput == "1")
		{
			return TestDriver::arucoTest();
		}
		else if (userInput == "2")
		{
			return TestDriver::pointCloudExportTest();
		}
		else if (userInput == "3")
		{
			return TestDriver::pointCloudVisualize();
		}
		else if (userInput == "4")
		{
			return TestDriver::multiCamVisualize();
		}
		else if (userInput == "5")
		{
			return TestDriver::project3DMultiple();
		}
		else if (userInput == "6")
		{
			return TestDriver::objectDetection();
		}
		else if (userInput == "7")
		{
			return TestDriver::objectTracking();
		}
		else if (userInput == "8")
		{
			cv::Size frameSize;
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
				return TestDriver::multipleCameraAutoTracking(frameSize);
			}
		}
		else if (userInput == "9")
		{
			cv::Size frameSize;
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
				return TestDriver::autoObjectTracking(frameSize);
			}
			

		}
		else
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}

}
