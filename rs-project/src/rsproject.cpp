// Jim Park 
// 2019/01/08
// Simple console app for detecting markers from real-time frames from the Intel Realsense D435 camera
//

#include "stdafx.h"

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(const rs2::video_frame& texture, const rs2::texture_coordinate& Texture_XY)
{
	// Get Width and Height coordinates of texture
	int width = texture.get_width();  // Frame width in pixels
	int height = texture.get_height(); // Frame height in pixels

									   // Normals to Texture Coordinates conversion
	int x_value = std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
	int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

	int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
	int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
	int Text_Index = (bytes + strides);

	const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

	// RGB components to save in tuple
	int NT1 = New_Texture[Text_Index];
	int NT2 = New_Texture[Text_Index + 1];
	int NT3 = New_Texture[Text_Index + 2];

	return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
pcl_color_ptr pointsToColorPCL(const rs2::points& points, const rs2::video_frame& color) 
{

	// Object Declaration (Point Cloud)
	pcl_color_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
	std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

	//================================
	// PCL Cloud Object Configuration
	//================================
	// Convert data captured from Realsense camera to Point Cloud
	auto sp = points.get_profile().as<rs2::video_stream_profile>();

	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	// and RGB values
	for (int i = 0; i < points.size(); ++i)
	{
		//===================================
		// Mapping Depth Coordinates
		// - Depth data stored as XYZ values
		//===================================
		cloud->points[i].x = Vertex[i].x;
		cloud->points[i].y = Vertex[i].y;
		cloud->points[i].z = Vertex[i].z;

		// Obtain color texture for specific point
		RGB_Color = RGB_Texture(color, Texture_Coord[i]);

		// Mapping Color (BGR due to Camera Model)
		cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
		cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
		cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>

	}

	return cloud; // PCL RGB Point Cloud generated
}

pcl_ptr pointsToPCL(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}

// The program logic for the aruco module testing happens here
int arucoTest()
{
	try
	{
		rs2::pipeline pipe; // Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::config cfg; // Going to create a custom configuration for our stream (mostly to have bigger frame dimensions)

						 // Create custom configuration; more detailed documentation about this will be in a separate file
						 // A custom configuration is used for this example app only, because Intel can give us camera intrinsic parameters for each
						 // stream setting rather than me having to do custom calibration. For this example app I will use 1280x720 RGB8 30Hz settings
		cfg.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, rs2_format::RS2_FORMAT_RGB8, 30);

		// Our camera calibration paramters for the above setting:
		cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 919.675, 0, 646.346, 0, 920.087, 355.558, 0, 0, 1);
		cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << 0, 0, 0, 0, 0);

		pipe.start(cfg); // Start streaming with default recommended configuration

						 // Create a simple window for displaying image, handy tool from openCV
		const auto window_name = "ArUco Marker Detection With Intel Realsense";
		cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

		// Create the Marker class
		// Must specify which pre-defined index of markers we are going to be detecting; for this example we will use 6x6 markers that range up to ID 250
		Marker markerController(cv::aruco::DICT_6X6_250);

		// Begin main loop for frame detection
		while (cv::waitKey(1) < 0)
		{
			rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
			rs2::frame rgb = data.get_color_frame(); // Get RGB frames

													 // Query frame size (width and height)
			const int w = rgb.as<rs2::video_frame>().get_width();
			const int h = rgb.as<rs2::video_frame>().get_height();

			// Create OpenCV matrix of size (w,h) from the colorized depth data
			cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);

			// This step needs to be done because for some reason OpenCV's convention of RGB is backwards (BGR instead of RGB) so colors will be inverted if this is skipped
			cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_RGB2BGR);

			markerController.detectMarker(image, cameraMatrix, distortionCoefficients);

			// Update the window with new data
			cv::imshow(window_name, image);
		}
		return EXIT_SUCCESS;
	}
	catch (const rs2::error & e)
	{
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}

// The program logic for the pointcloud module testing happens here
int pointCloudExportTest()
{
	try
	{
		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud pc;
		// We want the points object to be persistent so we can display the last cloud when a frame drops
		// The currPoints variable will be used to store the current frame on display that the user will save
		rs2::points points, currPoints;

		// The color mapper will have the pointcloud in real, 3D points copied to it
		// the depth mapper will have the pointcloud in regular stereo/depth points copied to it
		rs2::frame colorMapper, depthMapper;

		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe;

		// Map color to depth frames
		rs2::colorizer color_map;

		rs2::config cfg; // Going to create a custom configuration for our stream (mostly to have bigger frame dimensions)
		cfg.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, rs2_format::RS2_FORMAT_RGB8, 30);
		cfg.enable_stream(RS2_STREAM_DEPTH, 0, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);

		// Start streaming with default recommended configuration
		pipe.start(cfg);

		const auto window_name = "Testing pointcloud export to file";
		cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

		while (cv::waitKey(1) < 0)
		{
			// Wait for the next set of frames from the camera
			rs2::frameset frames = pipe.wait_for_frames();
			rs2::frame depth = frames.get_depth_frame();
			rs2::frame color = frames.get_color_frame();

			
			// Generate the pointcloud and texture mappings
			points = pc.calculate(depth);

			// Map the RGB texture to the points
			pc.map_to(depth);

			// Copy the points and color frame to the pointcloud that we will be exporting
			currPoints = points;

			// Copy the color frames
			colorMapper = color;
			
			// Copy the depth frames, but after colorizing it so it looks pretty
			depth = depth.apply_filter(color_map);
			depthMapper = depth;

			// Query frame size (width and height)
			const int w = depth.as<rs2::video_frame>().get_width();
			const int h = depth.as<rs2::video_frame>().get_height();

			// Create OpenCV matrix of size (w,h) from the colorized depth data
			cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

			// This step needs to be done because for some reason OpenCV's convention of RGB is backwards (BGR instead of RGB) so colors will be inverted if this is skipped
			//cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_RGB2BGR);
			cv::putText(image, 
						"Press any button to save current frame as .ply and exit", 
						cv::Point(30, 30),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 
						0.8, 
						cv::Scalar(150, 150, 250), 
						1);

			// Update the window with new data
			cv::imshow(window_name, image);
		}
		currPoints.export_to_ply("exportColor.ply", colorMapper);
		currPoints.export_to_ply("exportDepth.ply", depthMapper);
	}
	catch (const rs2::error & e)
	{
		std::cout << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	catch (const std::exception & e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

}

pcl_color_ptr affineTransformRotate(const pcl_color_ptr& source_cloud, Eigen::Vector3f::BasisReturnType axis, float theta = M_PI)
{

	/*  METHOD #2: Using a Affine3f
	This method is easier and less error prone
	*/
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// Define a translation of 2.5 meters on the x axis.
	transform_2.translation() << 0.0, 0.0, 0.0;

	// The same rotation matrix as before; theta radians around Z axis
	transform_2.rotate(Eigen::AngleAxisf(theta, axis));

	// Executing the transformation
	pcl_color_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

	return transformed_cloud;
}

pcl_color_ptr affineTransformTranslate(const pcl_color_ptr& source_cloud, float x = 0, float y = 0, float z = 0)
{

	/*  METHOD #2: Using a Affine3f
	This method is easier and less error prone
	*/
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// Define a translation of 2.5 meters on the x axis.
	transform_2.translation() << x, y, z;

	// Executing the transformation
	pcl_color_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

	return transformed_cloud;
}

int pointCloudVisualize()
{
	try
	{
		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud pc;
		// We want the points object to be persistent so we can display the last cloud when a frame drops
		// The currPoints variable will be used to store the current frame on display that the user will save
		rs2::points points, currPoints;

		// The color mapper will have the pointcloud in real, 3D points copied to it
		// the depth mapper will have the pointcloud in regular stereo/depth points copied to it
		rs2::frame colorMapper, depthMapper;

		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe;

		// Map color to depth frames
		rs2::colorizer color_map;

		rs2::config cfg; // Going to create a custom configuration for our stream (mostly to have bigger frame dimensions)
		cfg.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, rs2_format::RS2_FORMAT_RGB8, 30);
		cfg.enable_stream(RS2_STREAM_DEPTH, 0, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);

		// Start streaming with default recommended configuration
		pipe.start(cfg);

		//... populate cloud
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");


		while (!viewer.wasStopped())
		{
			// Wait for the next set of frames from the camera
			rs2::frameset frames = pipe.wait_for_frames();
			rs2::frame depth = frames.get_depth_frame();
			rs2::frame color = frames.get_color_frame();

			// Generate the pointcloud and texture mappings
			points = pc.calculate(depth);

			// Map the RGB texture to the points
			pc.map_to(depth);

			// Copy the points and color frame to the pointcloud that we will be exporting
			currPoints = points;

			// Copy the depth frames, but after colorizing it so it looks pretty
			depth = depth.apply_filter(color_map);
			pcl_color_ptr colorCloud = pointsToColorPCL(points, depth);
			colorCloud = affineTransformRotate(colorCloud, Eigen::Vector3f::UnitZ());
			colorCloud = affineTransformRotate(colorCloud, Eigen::Vector3f::UnitY());
			viewer.showCloud(colorCloud);
		}

	}
	catch (const rs2::error & e)
	{
		std::cout << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	catch (const std::exception & e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

}

int multiPointCloudVisualize()
{
	try
	{
		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud pc;
		// We want the points object to be persistent so we can display the last cloud when a frame drops
		// The currPoints variable will be used to store the current frame on display that the user will save
		rs2::points points, currPoints;

		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe;

		// Map color to depth frames
		rs2::colorizer color_map;

		rs2::config cfg; // Going to create a custom configuration for our stream (mostly to have bigger frame dimensions)
		cfg.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, rs2_format::RS2_FORMAT_BGR8, 30);
		cfg.enable_stream(RS2_STREAM_DEPTH, 0, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);

		// Start streaming with default recommended configuration
		pipe.start(cfg);

		//... populate cloud
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->initCameraParameters();

		pcl_color_ptr left(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl_color_ptr right(new pcl::PointCloud<pcl::PointXYZRGB>);
		//pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer");

		int v1(0);
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor(0, 0, 0, v1);
		viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);

		int v2(0);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor(0, 0, 0, v2);
		viewer->addText("Radius: 0.01", 10, 10, "v2 text", v2);

		viewer->addPointCloud<pcl::PointXYZRGB>(left, "left", v1);
		viewer->addPointCloud<pcl::PointXYZRGB>(right, "right", v2);

		// 0.1 is the scale of the coordinate axes markers
		// global coordinate system
		//viewer->addCoordinateSystem(0.1);
		viewer->addCoordinateSystem(0.1, "global", v1);

		while (!viewer->wasStopped())
		{

			// Wait for the next set of frames from the camera
			rs2::frameset frames = pipe.wait_for_frames();
			rs2::frame depth = frames.get_depth_frame();
			rs2::frame color = frames.get_color_frame();

			// Map the depth texture to the points
			pc.map_to(depth);

			// Generate the pointcloud and texture mappings
			points = pc.calculate(depth);

			//const int w = color.as<rs2::video_frame>().get_width();
			//const int h = color.as<rs2::video_frame>().get_height();

			// Copy the depth frames, but after colorizing it so it looks pretty
			depth = depth.apply_filter(color_map);
			pcl_color_ptr colorCloud = pointsToColorPCL(points, depth);
			
			//
			viewer->updatePointCloud(colorCloud, "left");
			pcl_color_ptr transformCloud = affineTransformRotate(colorCloud, Eigen::Vector3f::UnitZ());
			viewer->updatePointCloud(transformCloud, "right");
			viewer->spinOnce();
		}
			
	}
catch (const rs2::error & e)
{
	std::cout << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

}

int multiCamVisualize()
{
	try
	{

		DeviceWrapper connected_devices;

		rs2::context ctx;    // Create librealsense context for managing devices
		rs2::config cfg; // Going to create a custom configuration for our stream (mostly to have bigger frame dimensions)
		cfg.enable_stream(RS2_STREAM_DEPTH, 0, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);
		ctx.set_devices_changed_callback([&](rs2::event_information& info)
		{
			connected_devices.removeDevices(info);
			for (auto&& dev : info.get_new_devices())
			{
				connected_devices.enableDevice(dev);
			}
		});

		//... populate cloud
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

		viewer->initCameraParameters();

		pcl_color_ptr left(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl_color_ptr right(new pcl::PointCloud<pcl::PointXYZRGB>);
		//pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer");

		int v1(0);
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor(0, 0, 0, v1);

		int v2(0);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor(0, 0, 0, v2);

		viewer->addPointCloud<pcl::PointXYZRGB>(left, "cloud0", v1);
		viewer->addPointCloud<pcl::PointXYZRGB>(right, "cloud1", v2);

		viewer->setBackgroundColor(0, 0, 0);
		// 0.1 is the scale of the coordinate axes markers
		// global coordinate system
		viewer->addCoordinateSystem(0.1, "global");
		std::vector<pcl_color_ptr> allPointClouds;

		// Initial population of the device list
		for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
		{
			connected_devices.enableDevice(dev);
		}

		/*for (int i = 0; i < connected_devices.deviceCount(); i++)
		{
			pcl_color_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			allPointClouds.push_back(cloud);
			viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud" + i);
		}*/

		viewer->addText(" ", 10, 10, "drift");

		while (!viewer->wasStopped())
		{
			connected_devices.pollFrames();
			//const int w = color.as<rs2::video_frame>().get_width();
			//const int h = color.as<rs2::video_frame>().get_height();

			// Copy the depth frames, but after colorizing it so it looks pretty
			std::vector<std::pair<rs2::points, rs2::frame>> pointClouds = connected_devices.getPointClouds();
			std::vector<double> timestamps;
			for (int i = 0; i < pointClouds.size(); i++)
			{
				rs2::points test = pointClouds[i].first;
				rs2::frame depth = pointClouds[i].second;
				timestamps.push_back(depth.get_timestamp());
				pcl_color_ptr colorCloud = pointsToColorPCL(test, depth);
				colorCloud = affineTransformRotate(colorCloud, Eigen::Vector3f::UnitZ());
				//colorCloud = affineTransformTranslate(colorCloud, (i * 1.2) - 0.5);
				// Copy the depth frames, but after colorizing it so it looks pretty
				//rs2_timestamp_domain dom = depth.get_frame_timestamp_domain();
				//std::cout << rs2_timestamp_domain_to_string(dom) << std::endl
				std::string cloud_id = "cloud" + std::to_string(i);
				viewer->updatePointCloud(colorCloud, cloud_id);
				//test.export_to_ply("exportDepth.ply", depth);
			}

			if (timestamps.size() >= 2)
			{
				double drift = timestamps[1] - timestamps[0];

				viewer->updateText(std::to_string(drift), 10, 10, "drift");
			}
			if (timestamps.size() < 2)
			{
				std::cout << "?" << std::endl;
			}
			viewer->spinOnce();
		}
		return EXIT_SUCCESS;
	}
	catch (const rs2::error & e)
	{
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	catch (const std::exception & e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

}

int HWSyncTest()
{

	// Create a single context for managing all the realsense devices.
	rs2::context ctx;
	std::vector<std::thread> threads;

	std::size_t dev_id = 0;
	for (auto&& dev : ctx.query_devices())
	{
		threads.emplace_back([dev, dev_id]() {

			std::cout << "Device (Thread) ID: " << dev_id << std::endl;
			std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			std::cout << "Camera with serial number: " << serial << std::endl;

			auto advanced_dev = dev.as<rs400::advanced_mode>();
			auto advanced_sensors = advanced_dev.query_sensors();

			bool depth_found = false;
			bool color_found = false;
			rs2::sensor depth_sensor;
			rs2::sensor color_sensor;
			for (auto&& sensor : advanced_sensors) {
				std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
				std::cout << module_name << std::endl;

				if (module_name == "Stereo Module") {
					depth_sensor = sensor;
					depth_found = true;
				}
				else if (module_name == "RGB Camera") {
					color_sensor = sensor;
					color_found = true;
				}
			}

			if (!(depth_found && color_found)) {
				std::cout << "Unable to find both stereo and color modules" <<
					std::endl;
			}

			depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
			depth_sensor.set_option(RS2_OPTION_EXPOSURE, 8500); // microseconds
			depth_sensor.set_option(RS2_OPTION_GAIN, 16);
			depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);

			// RGB sync doesn't work, need to use depth as master.
			if (dev_id == 0) {
				std::cout << "Setting " << dev_id << " to master!" << std::endl;
				depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
			}
			else {
				std::cout << "Setting " << dev_id << " to slave!" << std::endl;
				depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
			}

			color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
			color_sensor.set_option(RS2_OPTION_EXPOSURE, 100); // 1/10 ms (10)
			color_sensor.set_option(RS2_OPTION_GAIN, 64);
			color_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);

			rs2::pipeline pipe;
			rs2::config cfg;
			cfg.enable_device(serial);
			cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
			cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);

			rs2::pipeline_profile profile = pipe.start(cfg);

			double last_time = 0;

			for (int frame_count = 0; frame_count < 1000; ++frame_count) {

				rs2::frameset frames = pipe.wait_for_frames();

				rs2::frame color_frame = frames.get_color_frame();
				rs2::frame depth_frame = frames.get_depth_frame();

				std::cout << frames.size() << " frames from " << serial << ": ";
				std::cout.precision(std::numeric_limits<double>::max_digits10);

				std::cout << "Drift: " << depth_frame.get_timestamp() - last_time << ", ";
				last_time = depth_frame.get_timestamp();
				/*
				for (const rs2::frame& f : { (rs2::frame)frames, depth_frame}) {
					switch (f.get_frame_timestamp_domain()) {
					case (RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK):
						std::cout << "Hardware Clock ";
						break;
					case (RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME):
						std::cout << "System Time ";
						break;
					default:
						std::cout << "Unknown ";
						break;
					}
					std::cout << "TS: " << std::scientific << f.get_timestamp()
						<< "(" << f.get_frame_number() << "), ";
				}*/

				std::cout << std::endl;
			}
		});

		dev_id++;
		std::this_thread::sleep_for(std::chrono::seconds(3));
	}

	for (auto& t : threads) t.join(); // Must join / detach all threads
}

int checkMode()
{

	rs2::context cxt;
	rs2::device_list devices = cxt.query_devices();
	for (auto dev : devices)
	{
		auto sensors = dev.as<rs400::advanced_mode>().query_sensors();
		std::cout << "Device:" << std::endl;
		for (auto sensor : sensors) {
			std::cout << sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION) << std::endl;
			std::cout << sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
			std::cout << sensor.get_info(RS2_CAMERA_INFO_NAME);
			std::cout << sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE) << std::endl;
		}
	}
	


	return std::getchar();
	
}
// main program entry
int main()
{
	std::string userInput = "";

	rs2::device camera = DeviceWrapper::getADevice();
	//DeviceWrapper::setDepthTable(camera);
	std::cout << "Input 1 for AruCo test program" << std::endl;
	std::cout << "Input 2 to test exporting pointcloud of a frame to .ply file" << std::endl;
	std::cout << "Input 3 to test pointcloud visualization" << std::endl;
	std::cout << "Input 4 to test multiple pointclouds visualization" << std::endl;
	std::cout << "Input 5 to test multi-camera pointcloud visualization" << std::endl;
	std::cout << "Input 6 to test hardware sync" << std::endl;
	std::cout << "Input 7 to test intercam hw sync availability" << std::endl;
	// get user input
	while (std::getline(std::cin, userInput))
	{	
		if (userInput == "1")
		{
			return arucoTest();
		}
		else if (userInput == "2")
		{
			return pointCloudExportTest();
		}
		else if (userInput == "3")
		{
			return pointCloudVisualize();
		}
		else if (userInput == "4")
		{
			return multiPointCloudVisualize();
		}
		else if (userInput == "5")
		{
			return multiCamVisualize();
		}
		else if (userInput == "6")
		{
			return HWSyncTest();
		}
		else if (userInput == "7")
		{
			return checkMode();
		}
		else
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}

}
