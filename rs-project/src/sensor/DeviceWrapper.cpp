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


void DeviceWrapper::enableDevice(const rs2::device& dev)
{
	std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));


	std::lock_guard<std::mutex> lock(_mutex);

	if (_devices.find(serial_number) != _devices.end())
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

	if (!_hasMaster)
	{
		 depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
		_hasMaster = true;
	}
	else
	{
		depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
	}
	std::cout << depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE);
	c.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, rs2_format::RS2_FORMAT_BGR8, 30);
	c.enable_stream(RS2_STREAM_DEPTH, 0, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);
	c.enable_device(serial_number);
	//c.enable_stream(RS2_STREAM_COLOR, 0, 848, 480, rs2_format::RS2_FORMAT_RGB8, 30); // D435 does not support HW sync for the RGB sensors!
	// Start the pipeline with the configuration
	rs2::pipeline_profile profile = p.start(c);
	// Hold it internally
	_devices.emplace(serial_number, ViewPort{ {},{}, p, profile });
}

void DeviceWrapper::removeDevices(const rs2::event_information& info)
{
	std::lock_guard<std::mutex> lock(_mutex);
	// Go over the list of devices and check if it was disconnected
	auto itr = _devices.begin();
	while (itr != _devices.end())
	{
		if (info.was_removed(itr->second.profile.get_device()))
		{
			itr = _devices.erase(itr);
		}
		else
		{
			++itr;
		}
	}
}

size_t DeviceWrapper::deviceCount()
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _devices.size();
}

int DeviceWrapper::streamCount()
{
	std::lock_guard<std::mutex> lock(_mutex);
	int count = 0;
	for (auto&& sn_to_dev : _devices)
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
	for (auto&& view : _devices)
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
				view.second.depthFrame = view.second.colorize_frame.process(view.second.colorFrame);
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
	for (auto&& view : _devices) //2
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
	for (auto&& view : _devices) //2
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

std::vector<cv::Mat> DeviceWrapper::getCameraMatrices()
{
	try
	{
		std::vector<cv::Mat> matrices;
		for (auto d : _devices)
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