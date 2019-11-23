// rs-visualizer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

//#include "pch.h"
#include <vtkAutoInit.h>
//#define vtkRenderingCore_AUTOINIT 4(vtkRenderingOpenGL, vtkInteractionStyle, vtkRenderingFreeType, vtkRenderingVolumeOpenGL)
VTK_MODULE_INIT(vtkRenderingOpenGL);

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>

#include <iostream>
#include <string>

void displayVisualSummary(const std::string& cloudFileName, const std::string& infoFileName)
{
	try
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PLYReader Reader;
		Reader.read(cloudFileName, *cloud);

		// Initialize visualizer
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		int viewPortId(0);
		viewer->initCameraParameters();

		viewer->createViewPort(0.0, 0.0, 1.0, 1.0, viewPortId);
		viewer->setCameraPosition(0, 0, 0, 0, 0, -1, 0, -1, 0);
		viewer->addCoordinateSystem();
		viewer->addPointCloud(cloud);

		std::ifstream infoFile;
		infoFile.open(infoFileName);
		std::string line;
		std::vector<std::string> objects;

		std::set<std::string> objectToPosition;

		pcl::PointXYZRGB minPt, maxPt;
		pcl::getMinMax3D(*cloud, minPt, maxPt);

		// Object info format:
		// <initial/final> <object name> position : (<centroid_x centroid_y centroid_z>) (<text_x text_y text_z>)

		while (std::getline(infoFile, line))
		{
			// Get whether this object is initial or final position
			auto initialOrFinalIndex = line.find_first_of(' ');
			std::string positionName = line.substr(0, initialOrFinalIndex);

			int r = 0;
			int g = 0;
			int b = 0;

			if (positionName == "new")
			{
				r = 255;
			}
			else if (positionName == "initial")
			{
				g = 255;
			}
			else // positionName == "final"
			{
				b = 255;
			}

			// Get the delimiter separating the object name and point values
			auto colonDelimiter = line.find_first_of(':');

			// Get object name
			std::string currObjectName = line.substr(0, colonDelimiter - 1);

			// If object with same name exists, ignore
			if (objectToPosition.count(currObjectName) > 0)
			{
				continue;
			}

			objectToPosition.insert(currObjectName);

			// Get points
			std::vector<std::string> pointValues;
			std::string pointsString = line.substr(colonDelimiter + 2, line.size() - colonDelimiter + 2);

			std::vector<std::string> pointSets; // should contain centroid point and text point
			boost::split(pointSets, pointsString, boost::is_any_of("()")); // split the 2 point sets

			std::vector<std::string> centroidPointValues, dimsValues;

			boost::split(centroidPointValues, pointSets[1], boost::is_any_of(" "));
			boost::split(dimsValues, pointSets[3], boost::is_any_of(" "));

			pcl::PointXYZ centroidPoint;
			centroidPoint.x = std::stof(centroidPointValues[0]);
			centroidPoint.y = std::stof(centroidPointValues[1]);
			centroidPoint.z = std::stof(centroidPointValues[2]);

			/*pcl::PointXYZ textPoint;
			textPoint.x = std::stof(textPointValues[0]);
			textPoint.y = std::stof(textPointValues[1]);
			textPoint.z = std::stof(textPointValues[2]);*/

			// Divide by 4; as in, divide the original radius by 2
			float widthRadius = std::stof(dimsValues[0]) / 4;
			float heightRadius = std::stof(dimsValues[1]) / 4;
			float depthRadius = std::stof(dimsValues[2]) / 4;

			// Halved distance for each dimension from each centroid point
			// Each cube will have a dimension of 20x20x20 cm
			float radius = 0.1;

			if (currObjectName.find("table") != std::string::npos ||
				currObjectName.find("sofa") != std::string::npos ||
				currObjectName.find("bed") != std::string::npos)
			{
				viewer->addCube(
					centroidPoint.x - widthRadius,
					centroidPoint.x + widthRadius,
					centroidPoint.y + heightRadius / 3,
					centroidPoint.y + heightRadius,
					centroidPoint.z - depthRadius,
					centroidPoint.z + depthRadius,
					r, g, b,
					currObjectName + "base",
					viewPortId
				);

				viewer->addCube(
					centroidPoint.x - widthRadius,
					centroidPoint.x - widthRadius / 2,
					centroidPoint.y + heightRadius / 3,
					centroidPoint.y + heightRadius * 1.5,
					centroidPoint.z + depthRadius / 2,
					centroidPoint.z + depthRadius,
					r, g, b,
					currObjectName + "right1",
					viewPortId
				);

				viewer->addCube(
					centroidPoint.x - widthRadius,
					centroidPoint.x - widthRadius / 2,
					centroidPoint.y + heightRadius / 3,
					centroidPoint.y + heightRadius * 1.5,
					centroidPoint.z - depthRadius,
					centroidPoint.z - depthRadius / 2,
					r, g, b,
					currObjectName + "right2",
					viewPortId
				);

				viewer->addCube(
					centroidPoint.x + widthRadius / 2,
					centroidPoint.x + widthRadius,
					centroidPoint.y + heightRadius / 3,
					centroidPoint.y + heightRadius * 1.5,
					centroidPoint.z + depthRadius / 2,
					centroidPoint.z + depthRadius,
					r, g, b,
					currObjectName + "left1",
					viewPortId
				);

				viewer->addCube(
					centroidPoint.x + widthRadius / 2,
					centroidPoint.x + widthRadius,
					centroidPoint.y + heightRadius / 3,
					centroidPoint.y + heightRadius * 1.5,
					centroidPoint.z - depthRadius,
					centroidPoint.z - depthRadius / 2,
					r, g, b,
					currObjectName + "left2",
					viewPortId
				);

				centroidPoint.y += heightRadius / 3;
				//centroidPoint.z += 0.05 + depthRadius;
				//centroidPoint.y += heightRadius / 2;
			}
			else if (currObjectName.find("oven") != std::string::npos)
			{
				viewer->addCube(
					centroidPoint.x - widthRadius,
					centroidPoint.x + widthRadius,
					centroidPoint.y,
					centroidPoint.y + heightRadius,
					centroidPoint.z - depthRadius,
					centroidPoint.z + depthRadius,
					r, g, b,
					currObjectName + "base",
					viewPortId
				);
				centroidPoint.y += heightRadius / 3;
				//centroidPoint.z += 0.05 + depthRadius;
				//centroidPoint.y += heightRadius / 2;
			}
			else if (currObjectName.find("chair") != std::string::npos)
			{
				viewer->addCube(
					centroidPoint.x - widthRadius,
					centroidPoint.x + widthRadius,
					centroidPoint.y,
					centroidPoint.y + heightRadius / 3,
					centroidPoint.z - depthRadius,
					centroidPoint.z + depthRadius,
					r, g, b,
					currObjectName + "base",
					viewPortId
				);
				viewer->addCube(
					centroidPoint.x - widthRadius,
					centroidPoint.x + widthRadius,
					centroidPoint.y - heightRadius,
					centroidPoint.y,
					centroidPoint.z - depthRadius,
					centroidPoint.z - depthRadius/4,
					r, g, b,
					currObjectName + "backrest",
					viewPortId
				);
				viewer->addCube(
					centroidPoint.x - widthRadius,
					centroidPoint.x - widthRadius / 2,
					centroidPoint.y,
					centroidPoint.y + heightRadius,
					centroidPoint.z + depthRadius / 2,
					centroidPoint.z + depthRadius,
					r, g, b,
					currObjectName + "right1",
					viewPortId
				);

				viewer->addCube(
					centroidPoint.x - widthRadius,
					centroidPoint.x - widthRadius / 2,
					centroidPoint.y,
					centroidPoint.y + heightRadius,
					centroidPoint.z - depthRadius,
					centroidPoint.z - depthRadius / 2,
					r, g, b,
					currObjectName + "right2",
					viewPortId
				);

				viewer->addCube(
					centroidPoint.x + widthRadius/2,
					centroidPoint.x + widthRadius,
					centroidPoint.y,
					centroidPoint.y + heightRadius,
					centroidPoint.z + depthRadius / 2,
					centroidPoint.z + depthRadius,
					r, g, b,
					currObjectName + "left1",
					viewPortId
				);

				viewer->addCube(
					centroidPoint.x + widthRadius /2,
					centroidPoint.x + widthRadius,
					centroidPoint.y,
					centroidPoint.y + heightRadius,
					centroidPoint.z - depthRadius,
					centroidPoint.z - depthRadius / 2,
					r, g, b,
					currObjectName + "left2",
					viewPortId
				);
			}

			else if (currObjectName.find("tvmonitor") != std::string::npos)
			{
				viewer->addCube(
					centroidPoint.x - widthRadius * 2,
					centroidPoint.x + widthRadius * 2,
					centroidPoint.y - heightRadius * 2,
					centroidPoint.y + heightRadius * 2,
					centroidPoint.z - depthRadius / 4 ,
					centroidPoint.z + depthRadius / 4,
					r, g, b,
					currObjectName,
					viewPortId
				);
			}

			// if the object is small (either width or height smaller than 1m) use a sphere to represent
			else if (widthRadius < 0.25 || heightRadius < 0.25)
			{
				// Add sphere
				viewer->addSphere(
					centroidPoint,
					depthRadius,
					r, g, b,
					currObjectName,
					0);

				//centroidPoint.z += depthRadius + 0.05;
			}
			else
			{
				viewer->addCube(
					centroidPoint.x - widthRadius,
					centroidPoint.x + widthRadius,
					centroidPoint.y - heightRadius,
					centroidPoint.y + heightRadius,
					centroidPoint.z - depthRadius,
					centroidPoint.z + depthRadius,
					r, g, b,
					currObjectName,
					viewPortId
				);

				//centroidPoint.z += 0.05 + depthRadius;
			}

			// Add text
			//viewer->addText3D(currObjectName, centroidPoint, 0.035, 160, 160, 160, currObjectName, viewPortId);

			pcl::PointXYZ textPoint(centroidPoint);
			textPoint.y = minPt.y + (maxPt.y - centroidPoint.y - std::stof(dimsValues[1])/2); // place all the labels at the top of the room

			viewer->addLine(textPoint, centroidPoint, 160, 160, 160, currObjectName + "_line", viewPortId);

			// Add text
			viewer->addText3D(currObjectName, textPoint, 0.035, 160, 160, 160, currObjectName, viewPortId);
		}

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
		}

	}
	catch (const std::exception & e)
	{
		std::cout << "Could not find activity.ply file at specified location" << std::endl;
	}
}

void displaySegmented(const std::string& cloudFileName, const std::string& infoFileName)
{
	try
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PLYReader Reader;
		Reader.read(cloudFileName, *cloud);

		// Initialize visualizer
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		int viewPortId(0);
		viewer->initCameraParameters();

		viewer->createViewPort(0.0, 0.0, 1.0, 1.0, viewPortId);
		viewer->setCameraPosition(0, 0, 0, 0, 0, -1, 0, -1, 0);
		viewer->addCoordinateSystem();
		viewer->addPointCloud(cloud);

		std::ifstream infoFile;
		infoFile.open(infoFileName);
		std::string line;
		std::vector<std::string> objects;

		std::set<std::string> objectToPosition;

		pcl::PointXYZRGB minPt, maxPt;
		pcl::getMinMax3D(*cloud, minPt, maxPt);

		// Object info format:
		// <initial/final> <object name> position : (<centroid_x centroid_y centroid_z>) (<text_x text_y text_z>)

		while (std::getline(infoFile, line))
		{
			// Get whether this object is initial or final position
			auto initialOrFinalIndex = line.find_first_of(' ');
			std::string positionName = line.substr(0, initialOrFinalIndex);

			int r = 0;
			int g = 0;
			int b = 0;

			if (positionName == "new")
			{
				r = 255;
			}
			else if (positionName == "initial")
			{
				g = 255;
			}
			else // positionName == "final"
			{
				b = 255;
			}

			// Get the delimiter separating the object name and point values
			auto colonDelimiter = line.find_first_of(':');

			// Get object name
			std::string currObjectName = line.substr(0, colonDelimiter - 1);

			// If object with same name exists, ignore
			if (objectToPosition.count(currObjectName) > 0)
			{
				continue;
			}

			objectToPosition.insert(currObjectName);

			// Get points
			std::vector<std::string> pointValues;
			std::string pointsString = line.substr(colonDelimiter + 2, line.size() - colonDelimiter + 2);

			std::vector<std::string> pointSets; // should contain centroid point and text point
			boost::split(pointSets, pointsString, boost::is_any_of("()")); // split the 2 point sets

			std::vector<std::string> centroidPointValues, dimsValues;

			boost::split(centroidPointValues, pointSets[1], boost::is_any_of(" "));
			boost::split(dimsValues, pointSets[3], boost::is_any_of(" "));

			pcl::PointXYZ centroidPoint;
			centroidPoint.x = std::stof(centroidPointValues[0]);
			centroidPoint.y = std::stof(centroidPointValues[1]);
			centroidPoint.z = std::stof(centroidPointValues[2]);

			pcl::PointXYZ textPoint(centroidPoint);
			textPoint.y = minPt.y + (maxPt.y - centroidPoint.y - std::stof(dimsValues[1])/2); // place all the labels at the top of the room

			viewer->addLine(textPoint, centroidPoint, 160, 160, 160, currObjectName + "_line", viewPortId);

			// Add text
			viewer->addText3D(currObjectName, textPoint, 0.035, 160, 160, 160, currObjectName, viewPortId);

		}

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
		}

	}
	catch (const std::exception & e)
	{
		std::cout << "Could not find activity.ply file at specified location" << std::endl;
	}
}

int main()
{
	while (1)
	{
		std::string directoryName = "";
		std::cout << "Input the path to the activity directory from which to read" << std::endl;
		std::getline(std::cin, directoryName);

		std::string cloudFileName = directoryName;
		std::string infoFileName = directoryName + "./objectsInfo.txt";

		std::string option = "";
		std::cout << "Input 1 for simple 3D visual summary with shapes, input 2 for detailed 3D visualization with segmented pointclouds" << std::endl;
		while (std::getline(std::cin, option))
		{
			if (option == "1")
			{
				cloudFileName += "/room.ply";
				break;
			}
			else if (option == "2")
			{
				cloudFileName += "/activity.ply";
				break;
			}
			else
			{
				std::cout << "Not a valid input, please try again." << std::endl;
			}
		}

		// Check ply file exists
		if (!boost::filesystem::exists(cloudFileName))
		{
			std::cout << "Could not find " << cloudFileName << " file in the specified directory" << std::endl;
			return 0;
		}
		// Check objects info text file exists
		else if (!boost::filesystem::exists(infoFileName))
		{
			std::cout << "Could not find objectsInfo.txt file in the specified directory" << std::endl;
			return 0;
		}

		if (option == "1")
		{
			displayVisualSummary(cloudFileName, infoFileName);
		}
		else if (option == "2")
		{
			displaySegmented(cloudFileName, infoFileName);
		}

	}
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
