#include "stdafx.h"
#include "./deviceWrapper/DeviceWrapper.h"


std::vector<rs2::device> DeviceWrapper::getAllConnectedDevices()
{
	// First, create a rs2::context.
	// The context represents the current platform with respect to connected devices
	rs2::context ctx;

	// Using the context we can get all connected devices in a device list
	rs2::device_list devices = ctx.query_devices();

	return devices;
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



void DeviceWrapper::enableDeviceToProfiles(const rs2::device& dev, std::vector<stream_profile_detail> streamProfiles)
{
	std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	if (devices_.find(serial_number) != devices_.end())
	{
		return; //already in
	}

	// Ignoring platform cameras (webcams, etc..)
	if (dev.get_info(RS2_CAMERA_INFO_NAME) == _platformCameraName)
	{
		return;
	}
	// Create a pipeline from the given device
	rs2::pipeline p;
	rs2::config c;
	rs2_error error();

	rs2::sensor depth_sensor;
	rs2::sensor color_sensor;
	auto advanced_dev = dev.as<rs400::advanced_mode>();

	auto advanced_sensors = advanced_dev.query_sensors();
	bool depth_found = false;
	bool color_found = false;
	for (auto&& sensor : advanced_sensors) {
		std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
		if (module_name == "Stereo Module") {
			depth_sensor = sensor;
			depth_found = true;
		}
		else if (module_name == "RGB Camera") {
			color_sensor = sensor;
			color_found = true;
		}
	} 

	/*// If physical wiring is used, set one to master and one to slave
	if (!_hasMaster)
	{
		 depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
		_hasMaster = true;
	}
	else
	{
		depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
	}
	std::cout << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE);*/

	// Enable stream profiles
	for (auto stream : streamProfiles)
	{
		c.enable_stream(stream.streamType, 0, stream.width, stream.height, stream.format, static_cast<unsigned int>(stream.frameRate));
	}

	c.enable_device(serial_number);

	// Start the pipeline with the configuration
	rs2::pipeline_profile profile = p.start(c);
	// Hold it internally
	devices_.emplace(serial_number, ViewPort{ {},{}, p, profile });
}

void DeviceWrapper::removeAllDevices(const rs2::event_information& info)
{
	std::lock_guard<std::mutex> lock(_mutex);
	// Go over the list of devices and check if it was disconnected
	auto itr = devices_.begin();
	while (itr != devices_.end())
	{
		if (info.was_removed(itr->second.profile.get_device()))
		{
			itr = devices_.erase(itr);
		}
		else
		{
			++itr;
		}
	}
}

size_t DeviceWrapper::getDevicesCount()
{
	std::lock_guard<std::mutex> lock(_mutex);
	return devices_.size();
}

int DeviceWrapper::streamCount()
{
	std::lock_guard<std::mutex> lock(_mutex);
	int count = 0;
	for (auto&& sn_to_dev : devices_)
	{
		for (auto&& stream : sn_to_dev.second.frames_per_stream)
		{
			if (stream.second)
			{
				count++;
			}
		}
	}
	return count;
}

void DeviceWrapper::pollFrames()
{
	std::lock_guard<std::mutex> lock(_mutex);
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	rs2::align align_to_color(RS2_STREAM_COLOR);
	// Go over all device
	for (auto&& view : devices_)
	{
		// Ask each pipeline if there are new frames available
		rs2::frameset frameset;
		if (view.second.pipe.poll_for_frames(&frameset))
		{
			frameset = align_to_color.process(frameset);
			//for (int i = 0; i < frameset.size(); i++)
			//{
				//rs2::frame new_frame = frameset[i];
				//int stream_id = new_frame.get_profile().unique_id();
				//view.second.frames_per_stream[stream_id] = view.second.colorize_frame.process(new_frame); //update view port with the new stream
				//view.second.depthFrame = view.second.colorize_frame.process(new_frame);
				rs2::pointcloud pc;
				//pc.calculate.process(view.second.depthFrame);
				//view.second.pointCloud = pc.calculate.process(view.second.depthFrame);
				//rs2::frame depth_frame = frameset.first(RS2_STREAM_DEPTH);
				rs2::frame depth_frame = frameset.get_depth_frame();
				view.second.colorFrame = frameset.get_color_frame();
				view.second.depthFrame = depth_frame;// view.second.colorize_frame.process(depth_frame);
				pc.map_to(view.second.depthFrame);
				view.second.pointCloud = pc.calculate(depth_frame);
				
			//}
		}
		/*
			rs2::pointcloud pc;
			rs2::frame depth = frameset.get_depth_frame();
			int depth_id = depth.get_profile().unique_id();
			view.second.frames_per_stream[depth_id] = view.second.colorize_frame.process(depth); //colorize
			view.second.depthFrame = depth;

			/*rs2::frame color = frameset.get_color_frame();
			int color_id = depth.get_profile().unique_id();
			view.second.frames_per_stream[color_id] = view.second.colorize_frame.process(color); // not  sure if necessary

			view.second.pointCloud = pc.calculate(depth);*/
		
	}
}

std::vector<std::pair<rs2::points, rs2::frame>> DeviceWrapper::getPointClouds()
{
	std::lock_guard<std::mutex> lock(_mutex);
	std::vector<std::pair<rs2::points, rs2::frame>> pointClouds;
	// Go over all device
	for (auto&& view : devices_) //2
	{
		/*// For each device get its frames
		for (auto&& id_to_frame : view.second.frames_per_stream) //1?
		{
			rs2::pointcloud pc;
			rs2::frame currFrame = id_to_frame.second;
			//if (currFrame.)
			pc.map_to(currFrame);
			view.second.pointCloud = pc.calculate(currFrame);
			pointClouds.push_back({ view.second.pointCloud, id_to_frame.second });
			
		}*/
		if (view.second.pointCloud && view.second.depthFrame)
		{
			pointClouds.push_back({ view.second.pointCloud, view.second.depthFrame });
		}
		
	}
	return pointClouds;
}

std::vector<rs2::frame> DeviceWrapper::getColorFrames()
{
	std::lock_guard<std::mutex> lock(_mutex);
	std::vector<rs2::frame> colorFrames;
	// Go over all device
	for (auto&& view : devices_) //2
	{
		/*// For each device get its frames
		for (auto&& id_to_frame : view.second.frames_per_stream) //1?
		{
		rs2::pointcloud pc;
		rs2::frame currFrame = id_to_frame.second;
		//if (currFrame.)
		pc.map_to(currFrame);
		view.second.pointCloud = pc.calculate(currFrame);
		pointClouds.push_back({ view.second.pointCloud, id_to_frame.second });

		}*/
		if (view.second.colorFrame)
		{
			colorFrames.push_back(view.second.colorFrame);
		}

	}
	return colorFrames;
}


std::map<std::string, rs2::frame> DeviceWrapper::getDepthFrames()
{
	std::lock_guard<std::mutex> lock(_mutex);
	std::map<std::string, rs2::frame> depthFrames;
	// Go over all device
	for (auto&& view : devices_) //2
	{
		if (view.second.depthFrame)
		{
			depthFrames[view.first] = view.second.depthFrame;
		}

	}
	return depthFrames;
}

std::map<std::string, rs2::frame> DeviceWrapper::getRGBFrames()
{
	std::lock_guard<std::mutex> lock(_mutex);
	std::map<std::string, rs2::frame> colorFrames;
	// Go over all device
	for (auto&& view : devices_) //2
	{
		if (view.second.colorFrame)
		{
			colorFrames[view.first] = view.second.colorFrame;
		}

	}
	return colorFrames;
}


std::vector<cv::Mat> DeviceWrapper::getCameraMatrices()
{
	try
	{
		std::vector<cv::Mat> matrices;
		for (auto d : devices_)
		{
			auto stream_profiles = d.second.profile.get_streams();
			for (rs2::stream_profile stream : stream_profiles)
			{
				if (stream.format() == RS2_FORMAT_BGR8)
				{
					auto video_stream = stream.as<rs2::video_stream_profile>();
					//If the stream is indeed a video stream, we can now simply call get_intrinsics()
					rs2_intrinsics intrinsics = video_stream.get_intrinsics();

					auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
					auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);

					cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << focal_length.first, 0, principal_point.first, 0, focal_length.second, principal_point.second, 0, 0, 1);
					matrices.push_back(cameraMatrix);
				}
			}
		}
		return matrices;
	}
	catch (const std::exception& e)
	{
		std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
	}
}

// Retrieves the camera matrices (fx, fy, cx, cy) for all connected cameras
std::map<std::string, cv::Mat> DeviceWrapper::getCameraMatricesMap()
{
	try
	{
		std::map<std::string, cv::Mat> matrices;
		for (auto d : devices_)
		{
			auto stream_profiles = d.second.profile.get_streams();
			for (rs2::stream_profile stream : stream_profiles)
			{
				if (stream.format() == RS2_FORMAT_BGR8)
				{
					auto video_stream = stream.as<rs2::video_stream_profile>();
					//If the stream is indeed a video stream, we can now simply call get_intrinsics()
					rs2_intrinsics intrinsics = video_stream.get_intrinsics();

					auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
					auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);

					cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << focal_length.first, 0, principal_point.first, 0, focal_length.second, principal_point.second, 0, 0, 1);
					matrices[d.first] = cameraMatrix;
				}
			}
		}
		return matrices;
	}
	catch (const std::exception& e)
	{
		std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
	}
}

rs2::pipeline & DeviceWrapper::getRealsensePipeline()
{
	return devices_.begin()->second.pipe;
}

/// STATIC FUNCTIONS

// Set depth table of a camera to some arbitrary values
// Shouldn't ever need to touch these, thus I have included the arbitrary units as a reference if 
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