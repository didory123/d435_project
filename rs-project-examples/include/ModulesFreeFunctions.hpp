#pragma once
#include "stdafx.h"
#include "aruco/ArucoMarker.h"
#include "detector/ObjectDetector.h"
#include "device/DeviceWrapper.h"
#include "Utils.h"

// Functions that test various functionalities with OpenCV/Intel Realsense 
namespace ModulesFreeFunctions
{

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
			ArucoMarker markerController(cv::aruco::DICT_6X6_250);

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
				pcl_color_ptr colorCloud = Utils::pointsToColorPCL(points, depth);
				colorCloud = Utils::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitZ());
				colorCloud = Utils::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitY());
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

	int multiCamVisualize()
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
				connected_devices.removeAllDevices(info);
				for (auto&& dev : info.get_new_devices())
				{
					connected_devices.enableDeviceToProfiles(dev, streams);
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
				connected_devices.enableDeviceToProfiles(dev, streams);
			}

			/*for (int i = 0; i < connected_devices.getDevicesCount(); i++)
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
					pcl_color_ptr colorCloud = Utils::pointsToColorPCL(test, depth);
					colorCloud = Utils::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitZ());
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

	int charucoCalibration()
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
				pcl_color_ptr colorCloud = Utils::pointsToColorPCL(points, depth);
				colorCloud = Utils::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitZ());
				colorCloud = Utils::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitY());
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

	int project3DMultiple()
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
			//Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(5, 7, 0.032, 0.019, dictionary);
			Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(5, 7, 0.08, 0.048, dictionary);
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
				connected_devices.removeAllDevices(info);
				for (auto&& dev : info.get_new_devices())
				{
					connected_devices.enableDeviceToProfiles(dev, streams);
				}
			});

			// Initial population of the device list
			for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
			{
				connected_devices.enableDeviceToProfiles(dev, streams);

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
				if (key > 0)
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
				rs2::colorizer colorize;
				std::vector<double> timestamps;
				for (int i = 0; i < pointClouds.size(); i++)
				{
					rs2::points points = pointClouds[i].first;
					rs2::frame depth = pointClouds[i].second;
					timestamps.push_back(depth.get_timestamp());

					pcl_color_ptr colorCloud = Utils::pointsToColorPCL(points, colorize.process(depth));
					//pcl_color_ptr colorCloud = Utils::pointsToColorPCL(points, depth);

					colorCloud = Utils::affineTransformMatrix(colorCloud, (Utils::getTransformMatrix(rotationVectors[i], translationVectors[i])).inverse());
					colorCloud = Utils::affineTransformRotate(colorCloud, Eigen::Vector3f::UnitY());
					// Copy the depth frames, but after colorizing it so it looks pretty

					float min = colorCloud->points[0].z;
					float max = colorCloud->points[0].z;
					for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = colorCloud->begin(); cloud_it != colorCloud->end(); ++cloud_it)
					{
						min = std::min(min, cloud_it->z);
						max = std::max(max, cloud_it->z);
					}
					// Compute LUT scaling to fit the full histogram spectrum
					double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

					if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
						lut_scale = 1.0;  // Avoid rounding error in boost

					for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = colorCloud->begin(); cloud_it != colorCloud->end(); ++cloud_it)
					{
						int value = std::lround((cloud_it->z - min) * lut_scale);
						// Blue -> Green -> Red (~ rainbow)
						cloud_it->r = value;
						cloud_it->g = 0;
						cloud_it->b = 255 - value;
					}

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

	int objectDetection()
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

	int objectTracking()
	{
		try
		{

			cv::Size frameSize = MEDIUM_DIMS;

			const int FRAME_WIDTH = 1280;
			const int FRAME_HEIGHT = 720;

			std::string mainWindow = "Object Tracking", initialWindow = "Initial Objects", finalWindow = "Final Objects";

			cv::namedWindow(initialWindow, cv::WINDOW_AUTOSIZE);
			cv::namedWindow(mainWindow, cv::WINDOW_AUTOSIZE);


			rs2::pipeline pipe; // Declare RealSense pipeline, encapsulating the actual device and sensors
			rs2::config cfg; // Going to create a custom configuration for our stream (mostly to have bigger frame dimensions)

							 // Create custom configuration; more detailed documentation about this will be in a separate file
							 // A custom configuration is used for this example app only, because Intel can give us camera intrinsic parameters for each
							 // stream setting rather than me having to do custom calibration. For this example app I will use 1280x720 RGB8 30Hz settings
			cfg.enable_stream(RS2_STREAM_COLOR, 0, LARGE_DIMS.width, LARGE_DIMS.height, rs2_format::RS2_FORMAT_BGR8, 30);
			cfg.enable_stream(RS2_STREAM_DEPTH, 0, LARGE_DIMS.width, LARGE_DIMS.height, rs2_format::RS2_FORMAT_Z16, 30);

			pipe.start(cfg); // Start streaming with default recommended configuration

			rs2::frame depth;// rgb, depth;
			rs2::align align_to_color(RS2_STREAM_COLOR);


			// =========================================================================================
			// Get initial snapshot of objects in room
			// =========================================================================================

			cv::Mat initialColorMat;

			// Run through first 50 frames or so to "flush out" some of the bad frames at the beginning
			// Lighting is a little off for the first second or so of the stream
			for (int i = 0; i < 50; i++)
			{
				rs2::frameset data = pipe.wait_for_frames();
				data = align_to_color.process(data);
				rs2::frame rgb = data.get_color_frame();
				depth = data.get_depth_frame();
				/*const int w = rgb.as<rs2::video_frame>().get_width();
				const int h = rgb.as<rs2::video_frame>().get_height();
				cv::Mat currFrame = frameToMat(rgb);
				//cv::Mat currFrame(cv::Size(w, h), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
				initialColorFrame = currFrame;*/
				initialColorMat = Utils::frameToMat(rgb);
			}

			cv::Mat initialDepthMat = Utils::depthFrameToScale(pipe, depth);

			// Get initial snapshot of all objects in the room
			ObjectDetector detector(0.4);
			std::map<std::string, cv::Rect2d> detectedObjects;
			detector.detectAllObjects(initialColorMat, detectedObjects);

			for (auto &i : detectedObjects)
			{
				// If the ROI extends outside the frame (because the detector will try to extrapolate)
				// there will be an error. Therefore we trim the bbox to fit inside the frame
				cv::Rect2d bboxToFitInMat(std::max(0.0, i.second.x),
					std::max(0.0, i.second.y),
					i.second.x + i.second.width <= initialDepthMat.cols ? i.second.width : initialDepthMat.cols - i.second.x,
					i.second.y + i.second.height <= initialDepthMat.rows ? i.second.height : initialDepthMat.rows - i.second.y);
				cv::Scalar m = cv::mean(initialDepthMat(bboxToFitInMat));

				// print depth information onto the image
				cv::putText(initialColorMat,
					std::to_string(m[0]) + " meters away",
					cv::Point(i.second.x, i.second.y + bboxToFitInMat.height / 2),
					cv::FONT_HERSHEY_SIMPLEX,
					0.45,
					cv::Scalar(255, 255, 255),
					1);
			}
			cv::Mat initialPositions = initialColorMat;


			// display all detected objects in the frame
			cv::imshow(initialWindow, initialPositions);

			cv::imwrite("initialPositions.png", initialPositions);

			cv::putText(initialColorMat, "Press any key to begin manual selection of a person's bounding box", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
			cv::imshow(mainWindow, initialColorMat);


			while (cv::waitKey(1) < 0)
			{
			}

			cv::Rect2d bbox = cv::selectROI(mainWindow, initialColorMat, false);

			// =========================================================================================
			// Initialize tracker parameters
			// =========================================================================================

			// List of tracker types in OpenCV 4.0.1
			std::string trackerTypes[8] = { "BOOSTING", "MIL", "KCF", "TLD","MEDIAN_FLOW", "GOTURN", "MOSSE", "CSRT" };

			// As of OpenCV 3+ the tracker class is an abstract class
			// For older versions however, use the below function
			// #if (CV_MINOR_VERSION < 3)
			// 	{
			//		tracker = Tracker::create(trackerType);
			//	}
			// #endif

			std::string trackerType = "KCF"; // Use the KCF algorithm



			//rs2::frame backgroundColorFrame = rgb;
			//bool finished = false;
			//std::thread t1(readColorFrame, pipe, backgroundColorFrame, finished);

			//cv::Rect2d bbox;
			//cv::destroyWindow(initialWindow);
			//initialPositions = frameToMat(rgb);
				//rs2::frameset data = pipe.wait_for_frames();
				//rgb = data.get_color_frame();
				//cv::Mat initialUneditedMat = frameToMat(rgb);




				//cv::putText(initialUneditedMat, "Select ROI", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);

			//}

			// stand by until person manually restarts application


			//finished = true;

			//cv::Rect2d bbox;

			// Create a tracker
			cv::Ptr<cv::Tracker> tracker = Utils::createTrackerByName(trackerType);
			tracker->init(initialColorMat, bbox);

			bool isPersonInFrame = true;

			// =========================================================================================
			// Initialize video recording parameters
			// =========================================================================================

			// The recorded frames of the person being tracked
			// The first value of the pair is the actual video frame itself
			// The second value of the pair is the coordinates of the person being tracked in that frame at that moment
			std::vector<std::pair<cv::Mat, cv::Rect2d>> recording;

			// A counter for counting the number of frames that have elapsed
			// The stream will be 30fps, as in there will be 30 frames per second
			// We will store the current coordinate of the person being tracked every 30 frames, or 1 second
			int frameCounter = 0;

			cv::VideoWriter output;
			output.open("output.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 30, frameSize);

			int frameCount = 40, personCount = 0;


			// =========================================================================================
			// Begin main loop for frame detection
			// =========================================================================================
			while (cv::waitKey(1) < 0)
			{
				rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
				rs2::frame rgb = data.get_color_frame(); // Get RGB frames

														 // Query frame size (width and height)
				const int w = rgb.as<rs2::video_frame>().get_width();
				const int h = rgb.as<rs2::video_frame>().get_height();

				// Create OpenCV matrix of size (w,h) from the colorized depth data
				cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);


				// if 40 frames has passed, run detection
					//bool ok = tracker->init(image, bbox);
					//rectangle(image, bbox, cv::Scalar(255, 0, 0), 2, 1);
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
				cv::putText(image, "Press any key to stop tracking and get detected final object positions", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
				// Display frame.
				//cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE); // only doing this for my current camera setup

				if (frameCounter == 30)
				{
					recording.push_back(std::make_pair(image, bbox));
					frameCounter = 0;

				}
				frameCount++;
				output.write(image);
				cv::imshow(mainWindow, image);

				// if 50 frames have passed since a person was not found in the frame, auto exit
				if (personCount > 50)
				{
					break;
				}

			}

			output.release();

			//personTrackOutput.release(); // release the video writer

			cv::destroyWindow(mainWindow);

			// Get final frame
			rs2::frameset finalData = pipe.wait_for_frames();
			rs2::frame finalRgb = finalData.get_color_frame();

			cv::Mat finalFrame = Utils::frameToMat(finalRgb);

			// Detect objects in final position
			std::map<std::string, cv::Rect2d> finalDetectedObjects;

			cv::Mat finalPositions = detector.detectAllObjects(finalFrame, finalDetectedObjects);

			// Compare the two maps and if there is a difference in coordinates, update the color in the final image
			for (auto obj : finalDetectedObjects)
			{
				// An object in the new map cannot be found in the original map, means this is a new object
				// Draw a red colored rectangle
				if (detectedObjects.count(obj.first) == 0)
				{
					cv::rectangle(finalFrame, obj.second, cv::Scalar(0, 0, 255), 2);
				}
				// If the object does exist in the initial snapshot, compare the new bounding box with the previous one
				// If different, means the object has changed
				// Draw a pink colored rectangle
				else if (!ObjectDetector::bboxesEqual(obj.second, detectedObjects[obj.first]))
				{
					cv::rectangle(finalFrame, obj.second, cv::Scalar(221, 0, 255), 2);
				}
			}

			cv::namedWindow(finalWindow, cv::WINDOW_AUTOSIZE);

			cv::putText(finalPositions, "Pink: Changed positions     Red: New object", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 0), 2);

			while (cv::waitKey(1) < 0)
			{
				cv::imshow(finalWindow, finalPositions);
				cv::imwrite("finalPositions.png", finalPositions);
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

	//// Simple IO function that parses the user's desired size (resolution) for video frames
	//cv::Size getFrameSizeFromUser()
	//{
	//	cv::Size frameSize;
	//	std::string userInput = "";
	//	std::cout << "Input a number from 1-4 to select the size of the video frames (1 is smallest, 4 is largest)" << std::endl;
	//	while (std::getline(std::cin, userInput))
	//	{
	//		if (userInput == "1")
	//		{
	//			frameSize = SMALL_DIMS;
	//		}
	//		else if (userInput == "2")
	//		{
	//			frameSize = MEDIUM_DIMS;
	//		}
	//		else if (userInput == "3")
	//		{
	//			frameSize = LARGE_DIMS;
	//		}
	//		else if (userInput == "4")
	//		{
	//			frameSize = LARGEST_DIMS;
	//		}
	//		else
	//		{
	//			std::cout << "Not a valid input, please try again." << std::endl;
	//			continue;
	//		}
	//		break;
	//	}
	//	return frameSize;
	//}


};
