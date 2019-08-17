#include "stdafx.h"
#include "MultiCameraActivity.h"
#include <cmath> 
#include <pcl/filters/crop_box.h>


MultiCameraActivity::MultiCameraActivity(
	rs2::pipeline& pipe,
	cv::Size frameSize,
	const std::string& trackerType,
	std::map<std::string, cv::Mat> initialColorMats,
	std::map<std::string, cv::Mat> initialDepthMats,
	std::map<std::string, cv::Mat> depthColorMappers,
	std::map<std::string, cv::Rect2d> personBboxes,
	std::map<std::string, Eigen::Matrix4d> calibrationMatrices,
	const std::string& windowName,
	ObjectDetector& detector,
	DeviceWrapper& deviceWrapper
)
{
	pipe_ = &pipe; // Any pipe from any device will do, since they should all have the same stream profile
	deviceWrapper_ = &deviceWrapper;
	calibrationMatrices_ = calibrationMatrices;
	frameSize_ = frameSize;
	windowName_ = windowName;

	// Get current time (the start time of the activity will be its name)
	boost::posix_time::ptime timeLocal =
		boost::posix_time::second_clock::local_time();

	std::string activityTime = std::to_string(timeLocal.date().year()) + "_" +
		std::to_string(timeLocal.date().month()) + "_" +
		std::to_string(timeLocal.date().day()) + "_" +
		std::to_string(timeLocal.time_of_day().hours()) + "h" +
		std::to_string(timeLocal.time_of_day().minutes()) + "m" +
		std::to_string(timeLocal.time_of_day().seconds()) + "s";

	activityName_ = activityTime;

	// Create a folder to store all the test results data
	boost::filesystem::create_directory(activityName_);

	// Create directory containing pointcloud coordinates of person
	boost::filesystem::create_directory("./" + activityName_ + "/coordinates/");

	// Initialize tracker object
	trackerType_ = trackerType;

	// Create a detector object for this activity
	detector_ = &detector;

	// Begin activity
	beginActivity(initialColorMats, initialDepthMats, depthColorMappers, personBboxes);
}

void MultiCameraActivity::beginActivity(
	std::map<std::string, cv::Mat> initialColorMats,
	std::map<std::string, cv::Mat> initialDepthMats,
	std::map<std::string, cv::Mat> depthColorMappers,
	std::map<std::string, cv::Rect2d> personBboxes
)
{
	// ===============================================================
	// 1. Initialize some parameters
	// ===============================================================
	// Create video recording
	output_.open("./" + activityName_ + "./output.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 15, cv::Size(frameSize_.width * initialColorMats.size(), frameSize_.height));

	// Initialize tracker object
	for (auto& bbox : personBboxes)
	{
		trackers_[bbox.first] = helper::createTrackerByName(trackerType_);
		trackers_[bbox.first]->init(initialColorMats[bbox.first], bbox.second);
		
	}

	// ===============================================================
	// 2. Get initial snapshot of rooms
	// ===============================================================

	for (auto& mat : initialColorMats)
	{
		/*initialCapture(mat.second, initialDepthMats[mat.first], depthColorMappers[mat.first], true, "./" + activityName_ + "/camera_" + mat.first + "_" + "initialPositions.ply",
			"./" + activityName_ + "/camera_" + mat.first + "_" + "initialPositions.png"
		);*/
		workerThreads_.push_back(
			new std::thread(
				&MultiCameraActivity::initialCapture,
				this,
				mat.second.clone(),
				initialDepthMats[mat.first].clone(),
				depthColorMappers[mat.first].clone(),
				true,
				mat.first,
				"./" + activityName_ + "/camera_" + mat.first + "_" + "initialPositions.ply",
				"./" + activityName_ + "/camera_" + mat.first + "_" + "initialPositions.png"
			)
		);
	}

	// ===============================================================
	// 3. Main loop
	// ===============================================================
	start();

	// ===============================================================
	// 4. Get final snapshot of rooms
	// ===============================================================

	// Get final snapshot data
	deviceWrapper_->pollFrames();
	auto colorFrames = deviceWrapper_->getRGBFrames();
	auto depthFrames = deviceWrapper_->getDepthFrames();

	rs2::colorizer colorMapper;
	for (auto& mat : colorFrames)
	{

		rs2::frame colored = colorMapper.colorize(depthFrames[mat.first]);
		cv::Mat finalDepthColor = cv::Mat(frameSize_, CV_8UC3, (void*)colored.get_data(), cv::Mat::AUTO_STEP);

		finalCapture(
			helper::frameToMat(mat.second).clone(),
			helper::frameToMat(depthFrames[mat.first]).clone(),
			finalDepthColor.clone(),
			true,
			mat.first,
			"./" + activityName_ + "/camera_" + mat.first + "_" + "finalPositions.ply",
			"./" + activityName_ + "/camera_" + mat.first + "_" + "finalPositions.png"
		);

		/*workerThreads_.push_back(
			new std::thread(
				&MultiCameraActivity::finalCapture,
				this,
				helper::frameToMat(mat.second).clone(),
				helper::frameToMat(depthFrames[mat.first]).clone(),
				finalDepthColor.clone(),
				true,
				mat.first,
				"./" + activityName_ + "/camera_" + mat.first + "_" + "finalPositions.ply",
				"./" + activityName_ + "/camera_" + mat.first + "_" + "finalPositions.png"
			)
		);*/
	}

	// Wait for the worker threads to finish before terminating activity
	for (auto &i : workerThreads_)
	{
		i->join();
	}
}
/*
void MultiCameraActivity::processFrame(cv::Mat rgbMat, cv::Mat depthMat, cv::Mat depthColor, std::string& deviceName, int& frameCount, cv::Mat& output, bool& isPersonInFrame)
{


	if (frameCount == 60)
	{
		// Every 60 frames, if the person is found in the room store the person's movement location as pointcloud

		// Get current time at which this pointcloud was captured
		boost::posix_time::ptime timeLocal =
			boost::posix_time::second_clock::local_time();

		// Turns out the cv::Mat copy constructor doesn't do a deep copy. You need to specify clone() to do a deep copy otherwise
		// the thread will throw an error once this function's matrices go out of scope

		// Spawn a worker thread that exports the ply data
		// This thread's join will be handled at the very end of the activity's life cycle
		// Just spawn it here and don't worry about it
		workerThreads_.push_back(
			new std::thread(
				&MultiCameraActivity::exportCloud,
				this,
				depthMat.clone(),
				depthColor.clone(),
				bbox,
				deviceName,
				std::string("./" + activityName_ + "/coordinates/camera_" + deviceName + "_timestamp_" + std::to_string(timeLocal.time_of_day().total_milliseconds()) + ".ply")
			)
		);
	}
}*/

void MultiCameraActivity::start() 
{

	
	bool isPersonInFrame = false;

	// keeps track of how many frames have passed where a person wasn't found
	int personMissingCounter = 0;

	// Initialize this to 40 so that the object detection occurs at the very beginning
	int frameCount = 90;

	rs2::colorizer colorMapper;

	while (cv::waitKey(1) < 0)
	{
		deviceWrapper_->pollFrames();

		auto colorFrames = deviceWrapper_->getRGBFrames();
		auto depthFrames = deviceWrapper_->getDepthFrames();

		// Create map of output images that we'll be storing the result frames in (after processing)
		std::map<std::string, cv::Mat> outputImages, depthMats, depthColorMats;
		std::map<std::string, cv::Rect2d> bboxes;

		// Start processing threads for each captured RGB/depth frame from each device
		for (auto& frame : colorFrames)
		{
			std::string deviceName = frame.first;

			// Turn frames into mats
			cv::Mat image = helper::frameToMat(frame.second);
			cv::Mat depthImage = helper::frameToMat(depthFrames[frame.first]);

			// Colorize depth frame
			rs2::frame colorFrame = colorMapper.colorize(depthFrames[frame.first]);
			cv::Mat depthColor = cv::Mat(frameSize_, CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);

			// Store the depth frames for future use
			depthMats[deviceName] = depthImage.clone();
			
			// TODO : uncomment
			//depthColorMats[deviceName] = depthColor.clone();

			depthColorMats[deviceName] = image.clone();

			cv::Rect2d bbox;

			// if 90 frames has passed, run detection
			if (frameCount == 90)
			{
				lock_.lock();
				isPersonInFrame = detector_->detectPerson(image, bbox);
				lock_.unlock();
				if (isPersonInFrame)
				{
					// Recreate and reinitialize the tracker
					trackers_[deviceName] = helper::createTrackerByName(trackerType_);
					trackers_[deviceName]->init(image, bbox);
					//tracker_->update(image, bbox);
					//tracker->init(image, newbbox);

					//bbox = newbbox;
				}
			}
			// otherwise, simply track the person
			else
			{

				isPersonInFrame = trackers_[deviceName]->update(image, bbox);
			}

			if (isPersonInFrame)
			{
				// Tracking success : Draw the tracked object
				rectangle(image, bbox, cv::Scalar(255, 0, 0), 2, 1);
			}

			bboxes[deviceName] = bbox;
			outputImages[deviceName] = image.clone();
			//std::thread * newWorkerThread = 

		}

		if (frameCount == 20 && isPersonInFrame)
		{
			// Every 60 frames, if the person is found in the room store the person's movement location as pointcloud

			// Get current time at which this pointcloud was captured
			boost::posix_time::ptime timeLocal =
				boost::posix_time::second_clock::local_time();

			// Turns out the cv::Mat copy constructor doesn't do a deep copy. You need to specify clone() to do a deep copy otherwise
			// the thread will throw an error once this function's matrices go out of scope

			// Spawn a worker thread that exports the ply data
			// This thread's join will be handled at the very end of the activity's life cycle
			// Just spawn it here and don't worry about it

			exportMergedCloud(depthMats, depthColorMats, bboxes, std::string("./" + activityName_ + "/coordinates/timestamp_" + std::to_string(timeLocal.time_of_day().total_milliseconds()) + ".ply"));

			//exportMergedCloudFromFrames(depthFrames, bboxes, std::string("./" + activityName_ + "/coordinates/timestamp_" + std::to_string(timeLocal.time_of_day().total_milliseconds()) + ".ply"));

			/*workerThreads_.push_back(
				new std::thread(
					&MultiCameraActivity::exportMergedCloud,
					this,
					depthMats,
					depthColorMats,
					bboxes,
					std::string("./" + activityName_ + "/coordinates/timestamp_" + std::to_string(timeLocal.time_of_day().total_milliseconds()) + ".ply")
				)
			);*/
		}

		if (frameCount == 90)
		{
			frameCount = 0;
		}
		else
		{
			frameCount++;
		}

		/*// Wait for all worker threads to finish
		for (auto& t : processThreads)
		{
			t->join();
		}
		
		// Handy c++ trick for deallocating all objects within a vector
		std::vector<std::thread *>().swap(processThreads);*/

		cv::Mat image;
		

		// Images that we'll combine to write to video file
		std::vector<cv::Mat> images;

		// Combine the output images
		for (auto& frame : outputImages)
		{
			images.push_back(frame.second);
			
		}

		// Concatenate the images
		cv::hconcat(images, image);

		// Count the number of frames a person is missing if a person can't be detected in any of the cameras
		if (isPersonInFrame)
		{
			personMissingCounter = 0;
		}
		else
		{
			personMissingCounter++;
			cv::putText(image, "Tracking failure detected", cv::Point(100, 150), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
			cv::putText(image,
				"No person detected for " + std::to_string(personMissingCounter) + " frames",
				cv::Point(10, image.rows - 60),
				cv::FONT_HERSHEY_SIMPLEX,
				0.75,
				cv::Scalar(255, 0, 255),
				2);
		}

		output_.write(image);

		cv::putText(image,
			"Current activity started at " + activityName_,
			cv::Point(10, 30),
			cv::FONT_HERSHEY_SIMPLEX,
			0.55,
			cv::Scalar(120, 75, 255),
			2);

		cv::putText(image,
			"Frame count " + std::to_string(frameCount),
			cv::Point(10, image.rows - 20),
			cv::FONT_HERSHEY_SIMPLEX,
			0.75,
			cv::Scalar(255, 0, 255),
			2);

		cv::imshow(windowName_, image);

		// if 120 frames (or 4 seconds) have passed since a person was not found in the frame, auto exit
		if (personMissingCounter > 120)
		{
			break;
		}

	}
	// Release video file
	output_.release();
}

void MultiCameraActivity::initialCapture(cv::Mat initialColorMat, cv::Mat initialDepthMat, cv::Mat depthColorMapper, bool exportCloud, std::string deviceName, std::string pathToPLY, std::string pathToPNG)
{
	// Get initial snapshot of all objects in the room
	lock_.lock();
	cv::Mat initialPositions = detector_->detectAllObjects(initialColorMat, initialDetectedObjectsPerDevice_[deviceName]);
	lock_.unlock();

	// Initialize pointcloud
	//pcl_ptr initialObjectsCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl_color_ptr initialObjectsCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	initialObjectsCloud->width = initialColorMat.cols; //Dimensions must be initialized to use 2-D indexing 
	initialObjectsCloud->height = initialColorMat.rows;
	initialObjectsCloud->resize(initialObjectsCloud->width*initialObjectsCloud->height);

	for (auto &i : initialDetectedObjectsPerDevice_[deviceName])
	{
		// ===============================================================
		// 1. Add depth information to each object
		// ===============================================================
		// If the ROI extends outside the frame (because the detector will try to extrapolate)
		// there will be an error. Therefore we trim the bbox to fit inside the frame
		cv::Rect2d bboxToFitInMat(std::max(0.0, i.second.x),
			std::max(0.0, i.second.y),
			std::abs(i.second.x) + i.second.width <= initialDepthMat.cols ? i.second.width : initialDepthMat.cols - std::abs(i.second.x),
			std::abs(i.second.y) + i.second.height <= initialDepthMat.rows ? i.second.height : initialDepthMat.rows - std::abs(i.second.y));

		// Calculate real distance of the object by taking the mean value of the depth points in the ROI around the target object
		cv::Scalar m = cv::mean(initialDepthMat(bboxToFitInMat));



		// print depth information onto the image
		// print depth information onto the image
		cv::putText(initialPositions,
			std::to_string(m[0]) + "mm",
			cv::Point(i.second.x, i.second.y + bboxToFitInMat.height / 2),
			cv::FONT_HERSHEY_SIMPLEX,
			0.45,
			cv::Scalar(255, 255, 255),
			1);



		// ===============================================================
		// 2. Create pointcloud for each detected object
		// ===============================================================
		if (exportCloud)
		{
			*initialObjectsCloud += *helper::depthMatToColorPCL(initialDepthMat, depthColorMapper, (pipe_->get_active_profile().get_stream(RS2_STREAM_DEPTH)).as<rs2::video_stream_profile>(), bboxToFitInMat);
		}
	}

	if (exportCloud)
	{
		pcl::PCLPointCloud2 outputCloud;
		pcl::toPCLPointCloud2(*initialObjectsCloud, outputCloud);

		pcl::PLYWriter plyWriter;
		plyWriter.writeASCII(pathToPLY, outputCloud);
	}
	//pcl::io::savePCDFileASCII("./" + activityName_ + "/initialPositions.pcd", *initialObjectsCloud);

	// display all detected objects in the frame
	//cv::imshow(initialWindow, initialPositions);


	// Store the initial snapshot as an image file
	cv::imwrite(pathToPNG, initialPositions);

	//cv::putText(initialColorMat, "Press any key to begin manual selection of a person's bounding box", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
	//cv::imshow(mainWindow, initialColorMat);
}

void MultiCameraActivity::finalCapture(cv::Mat finalSnapshotColor, cv::Mat finalSnapshotDepth, cv::Mat depthColorMapper, bool exportCloud, std::string deviceName, std::string pathToPLY, std::string pathToPNG)
{
	// in theory this should never be reached
	lock_.lock();
	cv::Mat finalPositions = detector_->detectAllObjects(finalSnapshotColor, finalDetectedObjectsPerDevice_[deviceName]);
	lock_.unlock();

	// Initialize pointcloud
	pcl_color_ptr finalObjectsCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	finalObjectsCloud->width = finalSnapshotDepth.cols; //Dimensions must be initialized to use 2-D indexing 
	finalObjectsCloud->height = finalSnapshotDepth.rows;
	finalObjectsCloud->resize(finalObjectsCloud->width*finalObjectsCloud->height);

	// Compare the two maps and if there is a difference in coordinates, update the color in the final image
	for (auto &i : finalDetectedObjectsPerDevice_[deviceName])
	{
		// ===============================================================
		// 1. Draw specific colored bounding boxes for each detected object
		// ===============================================================

		// An object in the new map cannot be found in the original map, means this is a new object
		// Draw a red colored rectangle
		if (initialDetectedObjects_.count(i.first) == 0)
		{
			cv::rectangle(finalPositions, i.second, cv::Scalar(0, 0, 255), 2);
		}
		// If the object does exist in the initial snapshot, compare the new bounding box with the previous one
		// If different, means the object has changed
		// Draw a pink colored rectangle
		else if (!ObjectDetector::bboxesEqual(i.second, initialDetectedObjects_[i.first]))
		{
			cv::rectangle(finalPositions, i.second, cv::Scalar(221, 0, 255), 2);
		}

		// ===============================================================
		// 2. Add depth information to each object
		// ===============================================================

		// If the ROI extends outside the frame (because the detector will try to extrapolate)
		// there will be an error. Therefore we trim the bbox to fit inside the frame
		cv::Rect2d bboxToFitInMat(std::max(0.0, i.second.x),
			std::max(0.0, i.second.y),
			std::abs(i.second.x) + i.second.width <= finalSnapshotDepth.cols ? i.second.width : finalSnapshotDepth.cols - std::abs(i.second.x),
			std::abs(i.second.y) + i.second.height <= finalSnapshotDepth.rows ? i.second.height : finalSnapshotDepth.rows - std::abs(i.second.y));

		// Calculate real distance of the object by taking the mean value of the depth points in the ROI around the target object
		cv::Scalar m = cv::mean(finalSnapshotDepth(bboxToFitInMat));


		// print depth information onto the image
		cv::putText(finalPositions,
			std::to_string(m[0]) + "mm",
			cv::Point(i.second.x, i.second.y + bboxToFitInMat.height / 2),
			cv::FONT_HERSHEY_SIMPLEX,
			0.45,
			cv::Scalar(255, 255, 255),
			1);

		// ===============================================================
		// 3. Create pointcloud for each detected object
		// ===============================================================
		if (exportCloud)
		{
			*finalObjectsCloud += *helper::depthMatToColorPCL(finalSnapshotDepth, depthColorMapper, (pipe_->get_active_profile().get_stream(RS2_STREAM_DEPTH)).as<rs2::video_stream_profile>(), bboxToFitInMat);
		}
	}
	if (exportCloud)
	{
		pcl::PCLPointCloud2 outputCloud;
		pcl::toPCLPointCloud2(*finalObjectsCloud, outputCloud);

		pcl::PLYWriter plyWriter;
		plyWriter.writeASCII(pathToPLY, outputCloud);

	}
	// display all detected objects in the frame
	//cv::imshow(initialWindow, initialPositions);

	// Store the initial snapshot as an image file
	cv::imwrite(pathToPNG, finalPositions);

	//cv::putText(initialColorMat, "Press any key to begin manual selection of a person's bounding box", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
	//cv::imshow(mainWindow, initialColorMat);
}

void MultiCameraActivity::exportCloud(cv::Mat depthData, cv::Mat depthColorMapper, cv::Rect2d bbox, const std::string& deviceName, const std::string& path)
{
	pcl_color_ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pointCloud->width = depthData.cols; //Dimensions must be initialized to use 2-D indexing 
	pointCloud->height = depthData.rows;
	pointCloud->resize(pointCloud->width*pointCloud->height);

	// If the ROI extends outside the frame there will be an error. Therefore we trim the bbox to fit inside the frame
	cv::Rect2d bboxToFitInMat(
		std::max(0.0, bbox.x),
		std::max(0.0, bbox.y),
		std::abs(bbox.x) + bbox.width <= depthData.cols ? bbox.width : depthData.cols - std::abs(bbox.x),
		std::abs(bbox.y) + bbox.height <= depthData.rows ? bbox.height : depthData.rows - std::abs(bbox.y));

	*pointCloud += *helper::depthMatToColorPCL(depthData.clone(), depthColorMapper.clone(), (pipe_->get_active_profile().get_stream(RS2_STREAM_DEPTH)).as<rs2::video_stream_profile>(), bboxToFitInMat);

	// Apply transformation to the pointcloud based on device's position in the real world
	pointCloud = helper::affineTransformMatrix(pointCloud, calibrationMatrices_[deviceName].inverse());

	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*pointCloud, outputCloud);

	pcl::PLYWriter plyWriter;

	plyWriter.writeASCII(path, outputCloud);

	//Activity::exportPointCloud(depthData, depthColorMapper, bbox, path);
}

void MultiCameraActivity::exportMergedCloud
(
	std::map<std::string, cv::Mat> depthMats, 
	std::map<std::string, cv::Mat> depthColorMappers, 
	std::map<std::string, cv::Rect2d> bboxes, 
	const std::string& path
)
{
	int frameWidth = depthMats.begin()->second.cols;
	int frameHeight = depthMats.begin()->second.rows;

	std::cout << frameWidth << " " << frameHeight << std::endl;

	pcl_color_ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pointCloud->width = frameWidth; //Dimensions must be initialized to use 2-D indexing 
	pointCloud->height = frameHeight;
	pointCloud->resize(pointCloud->width*pointCloud->height);

	for (auto& frame : depthMats)
	{
		cv::Mat depthData = frame.second;
		cv::Mat depthColorMapper = depthColorMappers[frame.first];
		cv::Rect2d bbox = bboxes[frame.first];

		// If the ROI extends outside the frame there will be an error. Therefore we trim the bbox to fit inside the frame
		cv::Rect2d bboxToFitInMat(
			std::max(0.0, bbox.x),
			std::max(0.0, bbox.y),
			std::abs(bbox.x) + bbox.width <= frameWidth ? bbox.width : frameWidth - std::abs(bbox.x),
			std::abs(bbox.y) + bbox.height <= frameHeight ? bbox.height : frameHeight - std::abs(bbox.y));

		pcl_color_ptr currPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		std::cout << "Point cloud for : " << frame.first << std::endl;

		*currPointCloud = *helper::depthMatToColorPCL(depthData.clone(), depthColorMapper.clone(), (deviceWrapper_->_devices[frame.first].pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH)).as<rs2::video_stream_profile>(), bboxToFitInMat);

		// Apply transformation to the pointcloud based on device's position in the real world
		currPointCloud = helper::affineTransformMatrix(currPointCloud, calibrationMatrices_[frame.first].inverse());

		// Merge pointcloud
		*pointCloud += *currPointCloud;
	}

	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*pointCloud, outputCloud);

	pcl::PLYWriter plyWriter;

	plyWriter.writeASCII(path, outputCloud);

	//Activity::exportPointCloud(depthData, depthColorMapper, bbox, path);
}

void MultiCameraActivity::exportMergedCloudFromFrames
(
	std::map<std::string, rs2::frame> depthFrames,
	std::map<std::string, cv::Rect2d> bboxes,
	const std::string& path
)
{
	pcl_color_ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pointCloud->width = LARGE_DIMS.width; //Dimensions must be initialized to use 2-D indexing 
	pointCloud->height = LARGE_DIMS.height;
	pointCloud->resize(pointCloud->width*pointCloud->height);

	rs2::colorizer colorMapper;

	for (auto& frame : depthFrames)
	{
		rs2::points points;

		rs2::pointcloud pc;

		rs2::frame coloredDepth = colorMapper.process(frame.second);

		pc.map_to(coloredDepth);
		points = pc.calculate(coloredDepth);

		pcl_color_ptr colorCloud = helper::pointsToColorPCL(points, frame.second);

		//cv::Mat depthData = frame.second;
		//cv::Mat depthColorMapper = depthColorMappers[frame.first];
		//cv::Rect2d bbox = bboxes[frame.first];

		//// If the ROI extends outside the frame there will be an error. Therefore we trim the bbox to fit inside the frame
		//cv::Rect2d bboxToFitInMat(
		//	std::max(0.0, bbox.x),
		//	std::max(0.0, bbox.y),
		//	std::abs(bbox.x) + bbox.width <= frameWidth ? bbox.width : frameWidth - std::abs(bbox.x),
		//	std::abs(bbox.y) + bbox.height <= frameHeight ? bbox.height : frameHeight - std::abs(bbox.y));

		//pcl_color_ptr currPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		//*currPointCloud = *helper::depthMatToColorPCL(depthData.clone(), depthColorMapper.clone(), (pipe_->get_active_profile().get_stream(RS2_STREAM_DEPTH)).as<rs2::video_stream_profile>(), bboxToFitInMat);

		//// Apply transformation to the pointcloud based on device's position in the real world
		colorCloud = helper::affineTransformMatrix(colorCloud, calibrationMatrices_[frame.first].inverse());

		//// Merge pointcloud
		*pointCloud += *colorCloud;

		//pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
		//boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
		//boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
		//boxFilter.setInputCloud(colorCloud);
		//boxFilter.filter(*bodyFiltered);
		//colorCloud
	}

	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*pointCloud, outputCloud);

	pcl::PLYWriter plyWriter;

	plyWriter.writeASCII(path, outputCloud);

	//Activity::exportPointCloud(depthData, depthColorMapper, bbox, path);
}