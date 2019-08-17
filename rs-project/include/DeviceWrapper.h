#pragma once
struct stream_profile_detail;
class DeviceWrapper
{
	// Helper struct per pipeline
	struct ViewPort
	{
		std::map<int, rs2::frame> frames_per_stream; // key is the unique id of frame (stream id); value is the actual frame
		rs2::colorizer colorize_frame;
		rs2::pipeline pipe;
		rs2::pipeline_profile profile;
		rs2::points pointCloud;
		rs2::frame depthFrame;
		rs2::frame colorFrame;
	};
public:

	void enableDevice(const rs2::device& dev, std::vector<stream_profile_detail> streamProfiles);
	void removeDevices(const rs2::event_information& info);
	std::vector<std::pair<rs2::points, rs2::frame>> getPointClouds();
	size_t deviceCount();
	int streamCount();
	void pollFrames();
	std::vector<rs2::frame> getColorFrames();
	std::map<std::string, rs2::frame> getDepthFrames();
	std::map<std::string, rs2::frame> getRGBFrames();
	std::vector<cv::Mat> getCameraMatrices();
	std::map<std::string, cv::Mat> getCameraMatricesMap(); // returns camera matrices in map form

	// Static function to retrieve a currently connected Intel device.
	// This is a simple function for a simple class, it'll just grab the first device on list
	static rs2::device getADevice();

	// Get depth sensor from camera
	static rs2::sensor getDepthSensor(const rs2::device& dev);

	// Set advanced depth table settings for a device
	// Changing some parameters in the depth table is required to get a clean point cloud object at short distances
	// Hardcoded the parameters in the function for now
	static void setDepthTable(rs2::device& dev);
	std::map<std::string, ViewPort> _devices; // key is the unique serial number for a device; viewport contains device's relevant information

private:
	const std::string _platformCameraName = "Platform Camera";
	std::mutex _mutex; // mutex is unlocked when it goes out of scope
	
	bool _hasMaster = false;
}; 