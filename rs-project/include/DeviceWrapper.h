#pragma once
class DeviceWrapper
{
public:
	// Static function to retrieve a currently connected Intel device.
	// This is a simple function for a simple class, it'll just grab the first device on list
	static rs2::device getADevice();

	// Get depth sensor from camera
	static rs2::sensor getDepthSensor(const rs2::device& dev);

	// Set advanced depth table settings for a device
	// Changing some parameters in the depth table is required to get a clean point cloud object at short distances
	// Hardcoded the parameters in the function for now
	static void setDepthTable(rs2::device& dev);

};