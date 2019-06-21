#pragma once
#include "stdafx.h"

// Fully static class that tests various functionalities with OpenCV/Intel Realsense 
class TestDriver
{
public:

	// The program logic for the aruco module testing happens here
	static int arucoTest()
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
	static int pointCloudExportTest()
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

	static int pointCloudVisualize()
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
				pcl_color_ptr colorCloud = TestDriver::pointsToColorPCL(points, depth);
				colorCloud = TestDriver::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitZ());
				colorCloud = TestDriver::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitY());
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

	static int multiCamVisualize()
	{
		try
		{
			DeviceWrapper connected_devices;

			rs2::context ctx;    // Create librealsense context for managing devices

								 // Initialize the streams that you want to start
			stream_profile_detail stream_depth =
			{
				1280,
				720,
				RS2_STREAM_DEPTH,
				rs2_format::RS2_FORMAT_Z16,
				E_FRAME_RATE::FPS_30,
			};
			stream_profile_detail stream_color =
			{
				1280,
				720,
				RS2_STREAM_COLOR,
				rs2_format::RS2_FORMAT_BGR8,
				E_FRAME_RATE::FPS_30,
			};
			std::vector<stream_profile_detail> streams = { stream_depth, stream_color };
			ctx.set_devices_changed_callback([&](rs2::event_information& info)
			{
				connected_devices.removeDevices(info);
				for (auto&& dev : info.get_new_devices())
				{
					connected_devices.enableDevice(dev, streams);
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
				connected_devices.enableDevice(dev, streams);
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
					pcl_color_ptr colorCloud = TestDriver::pointsToColorPCL(test, depth);
					colorCloud = TestDriver::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitZ());
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

	static int checkMode()
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

	static int charucoCalibration()
	{
		using namespace cv;
		try
		{
			//------------------------------------------------------------------------------
			//------------------------- OpenCV Setup ---------------------------------------
			//------------------------------------------------------------------------------
			// Create a simple window for displaying image, handy tool from openCV
			const auto window_name = "Charuco Calibration With Intel Realsense";
			cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

			// Create dictionary object from specific aruco library set
			Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_6X6_250));

			// The data from each frame will be stored here
			std::vector< std::vector< std::vector< Point2f > > > allCorners;
			std::vector< std::vector< int > > allIds;
			std::vector< Mat > allImgs;
			Size imgSize;

			// create charuco board object
			// TODO: The parameters defined here are hardcoded according to the printed out charucoboard that I am using
			Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(5, 7, 0.032, 0.019, dictionary);
			Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

			//------------------------------------------------------------------------------
			//------------------------- Intel Camera Setup ---------------------------------
			//------------------------------------------------------------------------------
			rs2::pipeline pipe; // Declare RealSense pipeline, encapsulating the actual device and sensors
			rs2::config cfg; // Going to create a custom configuration for our stream (mostly to have bigger frame dimensions)

							 // Create custom configuration; more detailed documentation about this will be in a separate file
							 // A custom configuration is used for this example app only, because Intel can give us camera intrinsic parameters for each
							 // stream setting rather than me having to do custom calibration. For this example app I will use 1280x720 RGB8 30Hz settings
			cfg.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, rs2_format::RS2_FORMAT_BGR8, 30);
			cfg.enable_stream(RS2_STREAM_DEPTH, 0, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);

			// Our camera calibration paramters for the above setting:
			//cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 919.675, 0, 646.346, 0, 920.087, 355.558, 0, 0, 1);

			pipe.start(cfg); // Start streaming with default recommended configuration


							 //------------------------------------------------------------------------------
							 //------------------------- Main Camera loop -----------------------------------
							 //------------------------------------------------------------------------------
			while (1) {
				rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
				rs2::frame rgb = data.get_color_frame(); // Get RGB frames

														 // Query frame size (width and height)
				const int w = rgb.as<rs2::video_frame>().get_width();
				const int h = rgb.as<rs2::video_frame>().get_height();

				// Create OpenCV matrix of size (w,h) from the colorized depth data
				cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
				cv::Mat imageCopy;

				// This step needs to be done because for some reason OpenCV's convention of RGB is backwards (BGR instead of RGB) so colors will be inverted if this is skipped
				//cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_RGB2BGR);

				// Update the window with new data
				cv::imshow(window_name, image);

				std::vector< int > ids;
				std::vector< std::vector< Point2f > > corners, rejected;

				// detect markers	
				aruco::detectMarkers(image, dictionary, corners, ids);

				// interpolate charuco corners
				Mat currentCharucoCorners, currentCharucoIds;

				if (ids.size() > 0)
					aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners, currentCharucoIds);

				// draw results
				image.copyTo(imageCopy);
				if (ids.size() > 0) aruco::drawDetectedMarkers(imageCopy, corners);

				if (currentCharucoCorners.total() > 0)
					aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

				putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
					Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
				putText(imageCopy, "At least 4 frames needed for calibration. Current count: " + std::to_string(allImgs.size()),
					Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
				imshow(window_name, imageCopy);
				char key = (char)waitKey(10);
				if (key == 27) break;
				if (key == 'c' && ids.size() > 0) {
					cout << "Frame captured" << endl;
					allCorners.push_back(corners);
					allIds.push_back(ids);
					// For some reason, the openCV dll throws an error in the interpolateCharucoCorners function below when trying to
					// convert an image to grayscale if it's stored in an array. We preemptively convert it here when we have full access
					// to our image, so that the dll doesn't even bother trying to do the grayscale conversion function and throw an error
					cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_BGR2GRAY);
					allImgs.push_back(image);
					imgSize = image.size();
				}
			}

			// stop pipe
			pipe.stop();

			// Check there was at least 1 capture
			if (allIds.size() < 1) {
				cerr << "Not enough captures for calibration" << endl;
				return 0;
			}

			Mat cameraMatrix, distCoeffs;
			std::vector< Mat > rvecs, tvecs;
			double repError;

			// prepare data for calibration
			std::vector< std::vector< Point2f > > allCornersConcatenated;
			std::vector< int > allIdsConcatenated;
			std::vector< int > markerCounterPerFrame;
			markerCounterPerFrame.reserve(allCorners.size());
			for (unsigned int i = 0; i < allCorners.size(); i++) {
				markerCounterPerFrame.push_back((int)allCorners[i].size());
				for (unsigned int j = 0; j < allCorners[i].size(); j++) {
					allCornersConcatenated.push_back(allCorners[i][j]);
					allIdsConcatenated.push_back(allIds[i][j]);
				}
			}

			// calibrate camera using aruco markers
			double arucoRepErr;
			arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
				markerCounterPerFrame, board, imgSize, cameraMatrix,
				distCoeffs, noArray(), noArray(), 0);

			// prepare data for charuco calibration
			int nFrames = (int)allCorners.size();
			std::vector< Mat > allCharucoCorners;
			std::vector< Mat > allCharucoIds;
			std::vector< Mat > filteredImages;
			allCharucoCorners.reserve(nFrames);
			allCharucoIds.reserve(nFrames);

			for (int i = 0; i < nFrames; i++) {
				// interpolate using camera parameters
				Mat currentCharucoCorners, currentCharucoIds;
				aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard, currentCharucoCorners, currentCharucoIds);
				allCharucoCorners.push_back(currentCharucoCorners);
				allCharucoIds.push_back(currentCharucoIds);
				filteredImages.push_back(allImgs[i]);
			}

			if (allCharucoCorners.size() < 4) {
				cerr << "Not enough corners for calibration" << endl;
				return 0;
			}

			// calibrate camera using charuco
			repError =
				aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
					cameraMatrix, distCoeffs, rvecs, tvecs, 0);

			// Print results
			cout << "Rep Error: " << repError << endl;
			cout << "Rep Error Aruco: " << arucoRepErr << endl;
			cout << "Camera Matrix" << cameraMatrix << endl;
			cout << "Distortion Coefficients" << distCoeffs << endl;
			cout << "Rotation" << endl;
			for (auto r : rvecs)
			{
				cout << r << endl;
			}
			cout << "Translation" << endl;
			for (auto t : tvecs)
			{
				cout << t << endl;
			}

			// close openCV window
			cv::destroyAllWindows();

			//------------------------------------------------------------------------------
			//------------------------- PCL Visualization ----------------------------------
			//------------------------------------------------------------------------------
			// Declare pointcloud object, for calculating pointclouds and texture mappings
			rs2::pointcloud pc;
			// We want the points object to be persistent so we can display the last cloud when a frame drops
			// The currPoints variable will be used to store the current frame on display that the user will save
			rs2::points points, currPoints;

			// The color mapper will have the pointcloud in real, 3D points copied to it
			// the depth mapper will have the pointcloud in regular stereo/depth points copied to it
			rs2::frame colorMapper, depthMapper;

			// Map color to depth frames
			rs2::colorizer color_map;

			pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

			pipe.start(cfg);


			Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();
			Mat M(100, 100, CV_64F);
			cout << M.at<double>(0, 0);
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
				pcl_color_ptr colorCloud = TestDriver::pointsToColorPCL(points, depth);
				colorCloud = TestDriver::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitZ());
				colorCloud = TestDriver::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitY());
				viewer.showCloud(colorCloud);
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

	static int project3DMultiple()
	{
		using namespace cv;
		try
		{

			//------------------------------------------------------------------------------
			//------------------------- OpenCV Setup ---------------------------------------
			//------------------------------------------------------------------------------
			// Create a simple window for displaying image, handy tool from openCV
			const auto window_name = "Charuco Calibration With Intel Realsense";
			cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

			// Create dictionary object from specific aruco library set
			Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_6X6_250));

			// create charuco board object
			// TODO: The parameters defined here are hardcoded according to the printed out charucoboard that I am using
			Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(5, 7, 0.032, 0.019, dictionary);
			Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

			DeviceWrapper connected_devices;

			// Initialize the streams that you want to start
			stream_profile_detail stream_depth =
			{
				640,
				480,
				RS2_STREAM_DEPTH,
				rs2_format::RS2_FORMAT_Z16,
				E_FRAME_RATE::FPS_30,
			};
			stream_profile_detail stream_color =
			{
				640,
				480,
				RS2_STREAM_COLOR,
				rs2_format::RS2_FORMAT_BGR8,
				E_FRAME_RATE::FPS_30,
			};
			std::vector<stream_profile_detail> streams = { stream_depth, stream_color };


			rs2::context ctx;    // Create librealsense context for managing devices
			ctx.set_devices_changed_callback([&](rs2::event_information& info)
			{
				connected_devices.removeDevices(info);
				for (auto&& dev : info.get_new_devices())
				{
					connected_devices.enableDevice(dev, streams);
				}
			});

			// Initial population of the device list
			for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
			{
				connected_devices.enableDevice(dev, streams);

			}
			cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << 0, 0, 0, 0, 0);
			std::vector<cv::Mat> cameraMatrices = connected_devices.getCameraMatrices();
			std::vector<Vec3d> rotationVectors, translationVectors;

			//------------------------------------------------------------------------------
			//------------------------- Main Camera loop -----------------------------------
			//------------------------------------------------------------------------------
			while (1) {

				connected_devices.pollFrames();

				Vec3d rvec, tvec;
				std::vector<Vec3d> rvecs, tvecs;

				std::vector<rs2::frame> colorFrames = connected_devices.getColorFrames();
				std::vector<cv::Mat> images;
				for (int i = 0; i < colorFrames.size(); i++)
				{
					const int w = colorFrames[i].as<rs2::video_frame>().get_width();
					const int h = colorFrames[i].as<rs2::video_frame>().get_height();
					cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)colorFrames[i].get_data(), cv::Mat::AUTO_STEP);
					cv::Mat imageCopy;

					std::vector< int > markerIds;
					std::vector< std::vector< Point2f > > corners;
					// detect markers	
					aruco::detectMarkers(image, dictionary, corners, markerIds);

					// interpolate charuco corners
					std::vector< Point2f > currentCharucoCorners;
					std::vector<int> currentCharucoIds;
					int interpolatedCorners = 0;
					if (markerIds.size() > 0)
						interpolatedCorners = aruco::interpolateCornersCharuco(corners, markerIds, image,
							charucoboard, currentCharucoCorners, currentCharucoIds, cameraMatrices[i], distortionCoefficients);

					// estimate charuco board pose
					aruco::estimatePoseCharucoBoard(currentCharucoCorners, currentCharucoIds, charucoboard,
						cameraMatrices[i], distortionCoefficients, rvec, tvec);

					// draw results
					image.copyTo(imageCopy);

					// draw aruco markers
					if (markerIds.size() > 0)
					{
						aruco::drawDetectedMarkers(imageCopy, corners);
					}
					// draw things
					if (interpolatedCorners > 0)
					{
						// draw corners
						aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
						// draw pose
						aruco::drawAxis(imageCopy, cameraMatrices[i], distortionCoefficients, rvec, tvec, 0.05);
					}
					rvecs.push_back(rvec);
					tvecs.push_back(tvec);
					images.push_back(imageCopy);

				}
				if (images.size() > 0)
				{
					//cv::imshow(window_name, images[0]);
					cv::Mat combinedImage;
					cv::hconcat(images, combinedImage);
					putText(combinedImage, "Press 'ESC' when your marker is visible in all camera streams.",
						Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
					cv::imshow(window_name, combinedImage);
				}
				char key = (char)waitKey(10);
				if (key == 27)
				{
					if (rvecs.size() > 0 && tvecs.size() > 0)
					{
						rotationVectors = rvecs;
						translationVectors = tvecs;
					}
					break;
				}
			}
			cv::destroyAllWindows();
			//... populate cloud
			pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
			viewer->initCameraParameters();


			pcl_color_ptr right(new pcl::PointCloud<pcl::PointXYZRGB>);
			//pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer");

			for (int i = 0; i < rotationVectors.size(); i++)
			{
				pcl_color_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				viewer->addPointCloud<pcl::PointXYZRGB>(cloud, std::to_string(i));
			}

			// 0.1 is the scale of the coordinate axes markers
			// global coordinate system
			//viewer->addCoordinateSystem(0.1);
			viewer->addCoordinateSystem(0.1, "global");

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
					rs2::points points = pointClouds[i].first;
					rs2::frame depth = pointClouds[i].second;
					timestamps.push_back(depth.get_timestamp());
					pcl_color_ptr colorCloud = TestDriver::pointsToColorPCL(points, depth);
					colorCloud = TestDriver::affineTransformMatrix(colorCloud, (TestDriver::getTransformMatrix(rotationVectors[i], translationVectors[i])).inverse());
					colorCloud = TestDriver::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitY());
					// Copy the depth frames, but after colorizing it so it looks pretty
					viewer->updatePointCloud(colorCloud, std::to_string(i));
				}

				viewer->spinOnce();
			}
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

	static int objectDetection()
	{
		try
		{

			std::vector<std::string> classes;

			const size_t inWidth = 1280;
			const size_t inHeight = 720;
			const float WHRatio = inWidth / (float)inHeight;
			const float inScaleFactor = 0.007843f;
			const float meanVal = 127.5;

			using namespace cv;
			using namespace cv::dnn;
			using namespace rs2;

			// Start streaming from Intel RealSense Camera
			pipeline pipe;
			rs2::config cfg; // Going to create a custom configuration for our stream (mostly to have bigger frame dimensions)
			cfg.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, rs2_format::RS2_FORMAT_BGR8, 30);

			pipe.start(cfg);
			rs2::align align_to(RS2_STREAM_COLOR);

			const auto window_name = "Display Image";
			namedWindow(window_name, WINDOW_AUTOSIZE);

			ObjectDetector detector;

			while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
			{
				// Wait for the next set of frames
				auto data = pipe.wait_for_frames();
				// Make sure the frames are spatially aligned
				data = align_to.process(data);

				auto color_frame = data.get_color_frame();
				auto depth_frame = data.get_depth_frame();

				// If we only received new depth frame, 
				// but the color did not update, continue
				static int last_frame_number = 0;
				if (color_frame.get_frame_number() == last_frame_number) continue;
				last_frame_number = color_frame.get_frame_number();

				const int w = color_frame.as<rs2::video_frame>().get_width();
				const int h = color_frame.as<rs2::video_frame>().get_height();

				// Convert RealSense frame to OpenCV matrix:
				//auto depth_mat = depth_frame_to_meters(pipe, depth_frame);
				cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

				cv::Mat detectedImage = detector.detect(image); // input the captured frame and perform object detection

				imshow(window_name, detectedImage);
				if (waitKey(1) >= 0) break;
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

	static int objectTracking()
	{
		try
		{
			const int FRAME_WIDTH = 1920;
			const int FRAME_HEIGHT = 1080;

			// List of tracker types in OpenCV 4.0.1
			std::string trackerTypes[8] = { "BOOSTING", "MIL", "KCF", "TLD","MEDIAN_FLOW", "GOTURN", "MOSSE", "CSRT" };
			// vector <string> trackerTypes(types, std::end(types));



			// As of OpenCV 3+ the tracker class is an abstract class
			// For older versions however, use the below function
			// #if (CV_MINOR_VERSION < 3)
			// 	{
			//		tracker = Tracker::create(trackerType);
			//	}
			// #endif

			std::string trackerType = "KCF"; // Use the KCF algorithm
			std::string mainWindow = "Object Tracking", initialWindow = "Initial Objects", finalWindow = "Final Objects";

			cv::namedWindow(initialWindow, cv::WINDOW_AUTOSIZE);
			cv::namedWindow(mainWindow, cv::WINDOW_AUTOSIZE);

			rs2::pipeline pipe; // Declare RealSense pipeline, encapsulating the actual device and sensors
			rs2::config cfg; // Going to create a custom configuration for our stream (mostly to have bigger frame dimensions)

							 // Create custom configuration; more detailed documentation about this will be in a separate file
							 // A custom configuration is used for this example app only, because Intel can give us camera intrinsic parameters for each
							 // stream setting rather than me having to do custom calibration. For this example app I will use 1280x720 RGB8 30Hz settings
			cfg.enable_stream(RS2_STREAM_COLOR, 0, FRAME_WIDTH, FRAME_HEIGHT, rs2_format::RS2_FORMAT_BGR8, 30);


			pipe.start(cfg); // Start streaming with default recommended configuration


			// Create a tracker
			cv::Ptr<cv::Tracker> tracker = TestDriver::createTrackerByName(trackerType);

			cv::Mat initialFrame;

			// Run through first 100 frames or so to "flush out" some of the bad frames at the beginning
			// Lighting is a little off for the first second or so of the stream
			for (int i = 0; i < 50; i++)
			{
				rs2::frameset data = pipe.wait_for_frames();
				rs2::frame rgb = data.get_color_frame();
				const int w = rgb.as<rs2::video_frame>().get_width();
				const int h = rgb.as<rs2::video_frame>().get_height();
				cv::Mat currFrame(cv::Size(w, h), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
				initialFrame = currFrame;
			}

			// Get initial snapshot of all objects in the room
			ObjectDetector detector;
			std::map<std::string, cv::Rect2d> detectedObjects;
			detector.detectAllObjects(initialFrame, detectedObjects);

			//cv::rotate(initialFrame, initialFrame, cv::ROTATE_90_COUNTERCLOCKWISE); // only doing this for my current camera setup

			cv::Mat initialPositions = initialFrame;

			// display all detected objects in the frame
			cv::imshow(initialWindow, initialPositions);

			cv::putText(initialFrame, "Press any key to begin manual selection of a person's bounding box", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
			cv::imshow(mainWindow, initialFrame);

			while (cv::waitKey(1) < 0)
			{
			}

			// stand by until person manually restarts application
			cv::Rect2d bbox = cv::selectROI(mainWindow, initialFrame, false);

			bool isPersonInFrame = false;

			tracker->init(initialFrame, bbox);
			isPersonInFrame = true;

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

				if (!isPersonInFrame)
				{
					bool isPersonDetected = detector.detectPerson(image, bbox);
					if (isPersonDetected)
					{
						tracker->init(image, bbox);
						isPersonInFrame = true;
					}
				}
				else
				{
					bool ok = tracker->update(image, bbox);
					if (ok)
					{
						// Tracking success : Draw the tracked object
						rectangle(image, bbox, cv::Scalar(255, 0, 0), 2, 1);
					}
					else
					{
						// Tracking failure detected.
						cv::putText(image, "Tracking failure detected", cv::Point(100, 150), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
					}
				}

				cv::putText(image, "Press any key to stop tracking and get detect final object positions", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
				// Display frame.
				//cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE); // only doing this for my current camera setup
				cv::imshow(mainWindow, image);

			}

			cv::destroyWindow(mainWindow);

			// Get final frame
			rs2::frameset data = pipe.wait_for_frames();
			rs2::frame rgb = data.get_color_frame();
			const int w = rgb.as<rs2::video_frame>().get_width();
			const int h = rgb.as<rs2::video_frame>().get_height();
			cv::Mat finalFrame(cv::Size(w, h), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);

			// Detect objects in final position
			std::map<std::string, cv::Rect2d> finalDetectedObjects;
			cv::Mat finalPositions = detector.detectAllObjects(finalFrame, finalDetectedObjects);

			while (cv::waitKey(1) < 0)
			{
				cv::imshow(finalWindow, finalPositions);
			}
			

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


private:

	// Listed below are various helper functions

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
	static std::tuple<int, int, int> RGB_Texture(const rs2::video_frame& texture, const rs2::texture_coordinate& Texture_XY)
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
	static pcl_color_ptr pointsToColorPCL(const rs2::points& points, const rs2::video_frame& color)
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
			RGB_Color = TestDriver::RGB_Texture(color, Texture_Coord[i]);

			// Mapping Color (BGR due to Camera Model)
			cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
			cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
			cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>

		}

		return cloud; // PCL RGB Point Cloud generated
	}

	static pcl_ptr pointsToPCL(const rs2::points& points)
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

	static Eigen::Matrix4d getTransformMatrix(cv::Vec3d rotationVector, cv::Vec3d translationVector)
	{
		/* Reminder: how transformation matrices work :

		|-------> This column is the translation
		| 1 0 0 x |  \
		| 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
		| 0 0 1 z |  /
		| 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

		METHOD #1: Using a Matrix4f
		This is the "manual" method, perfect to understand but error prone !
		*/
		cv::Mat rmat;
		// OpenCV estimate pose functions give us the rotation vector, not matrix
		// https://docs.opencv.org/3.4.3/d9/d0c/group__calib3d.html#ga61585db663d9da06b68e70cfbf6a1eac
		// Convert the 3x1 vector to 3x3 matrix in order to perform our transformations on the pointcloud
		cv::Rodrigues(rotationVector, rmat, cv::noArray());

		Eigen::Matrix4d transformMatrix = Eigen::Matrix4d::Identity();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				transformMatrix(i, j) = rmat.at<double>(i, j);
			}
		}
		transformMatrix(0, 3) = translationVector[0];
		transformMatrix(1, 3) = translationVector[1];
		transformMatrix(2, 3) = translationVector[2];

		return transformMatrix;
	}

	static pcl_color_ptr affineTransformMatrix(const pcl_color_ptr& source_cloud, Eigen::Matrix4d transformMat)
	{

		// The same rotation matrix as before; theta radians around Z axis
		//transform_2.rotate(Eigen::AngleAxisf(theta, axis));

		// Executing the transformation
		pcl_color_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		// You can either apply transform_1 or transform_2; they are the same
		pcl::transformPointCloud(*source_cloud, *transformed_cloud, transformMat);

		return transformed_cloud;
	}

	static pcl_color_ptr affineTransformRotate(const pcl_color_ptr& source_cloud, Eigen::Vector3f::BasisReturnType axis, float theta = M_PI)
	{

		/*  METHOD #2: Using a Affine3f
		This method is easier and less error prone
		*/
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

		// The same rotation matrix as before; theta radians around Z axis
		transform_2.rotate(Eigen::AngleAxisf(theta, axis));

		// Executing the transformation
		pcl_color_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		// You can either apply transform_1 or transform_2; they are the same
		pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

		return transformed_cloud;
	}

	static pcl_color_ptr affineTransformTranslate(const pcl_color_ptr& source_cloud, float x = 0, float y = 0, float z = 0)
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

	static inline cv::Ptr<cv::Tracker> createTrackerByName(cv::String name)
	{
		cv::Ptr<cv::Tracker> tracker;

		if (name == "KCF")
			tracker = cv::TrackerKCF::create();
		else if (name == "TLD")
			tracker = cv::TrackerTLD::create();
		else if (name == "BOOSTING")
			tracker = cv::TrackerBoosting::create();
		else if (name == "MEDIAN_FLOW")
			tracker = cv::TrackerMedianFlow::create();
		else if (name == "MIL")
			tracker = cv::TrackerMIL::create();
		else if (name == "GOTURN")
			tracker = cv::TrackerGOTURN::create();
		else if (name == "MOSSE")
			tracker = cv::TrackerMOSSE::create();
		else if (name == "CSRT")
			tracker = cv::TrackerCSRT::create();
		else
			CV_Error(cv::Error::StsBadArg, "Invalid tracking algorithm name\n");

		return tracker;
	}
};

