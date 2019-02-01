#include "stdafx.h"
#include "DeviceWrapper.h"


rs2::device DeviceWrapper::getADevice()
{
	// First, create a rs2::context.
	// The context represents the current platform with respect to connected devices
	rs2::context ctx;

	// Using the context we can get all connected devices in a device list
	rs2::device_list devices = ctx.query_devices();

	rs2::device selected_device;
	if (devices.size() == 0)
	{
		std::cout << "No device connected, please connect a RealSense device" << std::endl;
		throw;
	}
	else
	{
		std::cout << "Retrieving first device found" << std::endl;
		selected_device = devices[0];
	}

	return selected_device;
}

rs2::sensor DeviceWrapper::getDepthSensor(const rs2::device& dev)
{
	// A rs2::device is a container of rs2::sensors that have some correlation between them.
	// For example:
	//    * A device where all sensors are on a single board
	//    * A Robot with mounted sensors that share calibration information

	// Given a device, we can query its sensors using:
	std::vector<rs2::sensor> sensors = dev.query_sensors();

	// For Intel D435, 0 is the stereo module and 1 is the RGB module
	return  dev.query_sensors()[0];
}

void DeviceWrapper::setDepthTable(rs2::device& dev)
{
	rs400::advanced_mode adv(dev);
	// Parameters for the depth table options
	// There is very little documentation on what these parameters do
	// According to a link I found, Intel has refused to create documentation for the advanced options
	STDepthTableControl newParams = {
		100, //Depth Units
		0,  // Depth Clamp Min
		6000, // Depth Clamp Max
		0, //Disparity Mode
		0 //Disparity Shift
	};

	adv.set_depth_table(newParams);
}