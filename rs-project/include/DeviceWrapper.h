#pragma once
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

	void enableDevice(const rs2::device& dev);

	void removeDevices(const rs2::event_information& info);
	
	std::vector<std::pair<rs2::points, rs2::frame>> getPointClouds();

	size_t deviceCount();

	int streamCount();

	void pollFrames();

	std::vector<rs2::frame> getColorFrames();

	std::vector<cv::Mat> getCameraMatrices();
	/*void render_textures(int cols, int rows, float view_width, float view_height)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		int stream_no = 0;
		for (auto&& view : _devices)
		{
			// For each device get its frames
			for (auto&& id_to_frame : view.second.frames_per_stream)
			{
				rect frame_location{ view_width * (stream_no % cols), view_height * (stream_no / cols), view_width, view_height };
				if (rs2::video_frame vid_frame = id_to_frame.second.as<rs2::video_frame>())
				{
					view.second.tex.render(vid_frame, frame_location);
					stream_no++;
				}
			}
		}
	}*/




	// Static function to retrieve a currently connected Intel device.
	// This is a simple function for a simple class, it'll just grab the first device on list
	static rs2::device getADevice();

	// Get depth sensor from camera
	static rs2::sensor getDepthSensor(const rs2::device& dev);

	// Set advanced depth table settings for a device
	// Changing some parameters in the depth table is required to get a clean point cloud object at short distances
	// Hardcoded the parameters in the function for now
	static void setDepthTable(rs2::device& dev);

private:
	const std::string _platformCameraName = "Platform Camera";
	std::mutex _mutex; // mutex is unlocked when it goes out of scope
	std::map<std::string, ViewPort> _devices;
	bool _hasMaster = false;
}; 