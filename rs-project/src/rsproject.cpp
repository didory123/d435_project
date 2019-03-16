// Jim Park 
// 2019/01/08
// Simple console app for detecting markers from real-time frames from the Intel Realsense D435 camera
//

#include "stdafx.h"

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;

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
		cfg.enable_stream(RS2_STREAM_COLOR, 0, 640, 480, rs2_format::RS2_FORMAT_RGB8, 15);
		cfg.enable_stream(RS2_STREAM_DEPTH, 0, 640, 480, rs2_format::RS2_FORMAT_Z16, 15);

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

			// Copy the color frames
			colorMapper = color;

			// Copy the depth frames, but after colorizing it so it looks pretty
			depth = depth.apply_filter(color_map);
			depthMapper = depth;

			viewer.showCloud(pointsToPCL(points));
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

// main program entry
int main()
{
	std::string userInput = "";

	rs2::device camera = DeviceWrapper::getADevice();
	//DeviceWrapper::setDepthTable(camera);
	std::cout << "Input 1 for AruCo test program" << std::endl;
	std::cout << "Input 2 to test exporting pointcloud of a frame to .ply file" << std::endl;
	std::cout << "Input 3 to test pointcloud visualization" << std::endl;

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
		else
		{
			std::cout << "Not a valid input, please try again." << std::endl;
		}
	}

}
