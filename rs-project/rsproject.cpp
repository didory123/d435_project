// Jim Park 
// 2019/01/08
// Simple console app for detecting markers from real-time frames from the Intel Realsense D435 camera
//

#include "stdafx.h"


int main()
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
