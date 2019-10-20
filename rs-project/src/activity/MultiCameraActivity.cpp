#include "stdafx.h"
#include "MultiCameraActivity.h"



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

	workerThreads_.push_back(
		new std::thread(
			&MultiCameraActivity::initialPositionsCapture,
			this,
			initialColorMats,
			initialDepthMats,
			initialColorMats // TODO: UNCOMMENT //depthColorMappers
		)
	);
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

	std::map<std::string, cv::Mat> finalDepthMats, finalColorMats, finalColorMappers;




	for (const auto& framePair : colorFrames)
	{

		finalColorMats[framePair.first] = helper::frameToMat(framePair.second);
		// TODO : Need to replace this with color scheme rather than rgb
		finalColorMappers[framePair.first] = helper::frameToMat(framePair.second);
	}

	// Run the filters
	// IMPORTANT: The filters must be run in the exact order as written below
	rs2::decimation_filter dec_filter;  // 1. Decimation - reduces depth frame density

	// Declare disparity transform from depth to disparity and vice versa
	const std::string disparity_filter_name = "Disparity"; // 2. Depth to disparity
	rs2::disparity_transform depth_to_disparity(true);

	rs2::spatial_filter spat_filter;    // 3. Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;   // 4. Temporal   - reduces temporal noise

	rs2::disparity_transform disparity_to_depth(false); // 5. Disparity back to depth

	for (auto& framePair : depthFrames)
	{
		//framePair.second = dec_filter.process(framePair.second); // 1
		framePair.second = depth_to_disparity.process(framePair.second); // 2
		framePair.second = spat_filter.process(framePair.second); // 3
		framePair.second = temp_filter.process(framePair.second); // 4
		framePair.second = disparity_to_depth.process(framePair.second); // 5

		finalDepthMats[framePair.first] = helper::frameToMat(framePair.second);
	}

	finalPositionsCapture(finalColorMats, finalDepthMats, finalColorMats);//TODO: Uncomment: depthColorMappers);


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


	bool wasPersonInFrameLast = false;

	// keeps track of how many frames have passed where a person wasn't found
	int personMissingCounter = 0;

	// Initialize this to 40 so that the object detection occurs at the very beginning
	int frameCount = 90;

	rs2::colorizer colorMapper;

	std::map<std::string, cv::Rect2d> bbox;

	while (cv::waitKey(1) < 0)
	{
		deviceWrapper_->pollFrames();

		auto colorFrames = deviceWrapper_->getRGBFrames();
		auto depthFrames = deviceWrapper_->getDepthFrames();

		bool isPersonInFrame = false;

		// Create map of output images that we'll be storing the result frames in (after processing)
		std::map<std::string, cv::Mat> outputImages, depthMats, depthColorMats;
		std::map<std::string, std::map<std::string, cv::Rect2d>> bboxes; // <deviceName, <objectName, bbox>> 

		// Start processing threads for each captured RGB/depth frame from each device
		for (auto& framePair : colorFrames)
		{
			std::string deviceName = framePair.first;

			// Turn frames into mats
			cv::Mat image = helper::frameToMat(framePair.second);
			cv::Mat depthImage = helper::frameToMat(depthFrames[deviceName]);

			// Colorize depth frame
			rs2::frame colorFrame = colorMapper.colorize(depthFrames[deviceName]);
			cv::Mat depthColor = cv::Mat(frameSize_, CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);

			// Store the depth frames for future use
			depthMats[deviceName] = depthImage.clone();
			
			// TODO : uncomment
			//depthColorMats[deviceName] = depthColor.clone();
			depthColorMats[deviceName] = image.clone();
			

			// if 90 frames has passed, run detection
			if (frameCount == 90)
			{
				lock_.lock();
				// Bitwise OR here, because we want to continue activity if person is found in at least one camera
				isPersonInFrame = isPersonInFrame | detector_->detectPerson(image, bbox[deviceName]);
				lock_.unlock();

				if (isPersonInFrame)
				{
					// Recreate and reinitialize the tracker
					trackers_[deviceName] = helper::createTrackerByName(trackerType_);
					trackers_[deviceName]->init(image, bbox[deviceName]);
					//tracker_->update(image, bbox);
					//tracker->init(image, newbbox);
					
					//bbox = newbbox;
				}
			}
			// otherwise, simply track the person
			else
			{
				// Bitwise OR here, because we want to continue activity if person is found in at least one camera
				isPersonInFrame = isPersonInFrame | trackers_[deviceName]->update(image, bbox[deviceName]);
			}

			if (isPersonInFrame)
			{
				// Tracking success : Draw the tracked object
				rectangle(image, bbox[deviceName], cv::Scalar(255, 0, 0), 2, 1);
			}

			bboxes[deviceName]["person"] = bbox[deviceName];
			outputImages[deviceName] = image.clone();
			//std::thread * newWorkerThread = 

		}

		if (frameCount % 45 == 0 && isPersonInFrame)
		{
			// Run the filters
			// IMPORTANT: The filters must be run in the exact order as written below
			rs2::decimation_filter dec_filter;  // 1. Decimation - reduces depth frame density

			// Declare disparity transform from depth to disparity and vice versa
			const std::string disparity_filter_name = "Disparity"; // 2. Depth to disparity
			rs2::disparity_transform depth_to_disparity(true);

			rs2::spatial_filter spat_filter;    // 3. Spatial    - edge-preserving spatial smoothing
			rs2::temporal_filter temp_filter;   // 4. Temporal   - reduces temporal noise

			rs2::disparity_transform disparity_to_depth(false); // 5. Disparity back to depth

			for (auto& framePair : depthFrames)
			{

				//framePair.second = dec_filter.process(framePair.second); // 1
				framePair.second = depth_to_disparity.process(framePair.second); // 2
				framePair.second = spat_filter.process(framePair.second); // 3
				framePair.second = temp_filter.process(framePair.second); // 4
				framePair.second = disparity_to_depth.process(framePair.second); // 5

				std::string deviceName = framePair.first;
				cv::Mat depthImage = helper::frameToMat(framePair.second);

				// Store the depth frames for pointcloud conversion
				depthMats[deviceName] = depthImage.clone();

			}
			// Every 60 frames, if the person is found in the room store the person's movement location as pointcloud



			// Get current time at which this pointcloud was captured
			boost::posix_time::ptime timeLocal =
				boost::posix_time::second_clock::local_time();

			// Turns out the cv::Mat copy constructor doesn't do a deep copy. You need to specify clone() to do a deep copy otherwise
			// the thread will throw an error once this function's matrices go out of scope

			// Spawn a worker thread that exports the ply data
			// This thread's join will be handled at the very end of the activity's life cycle
			// Just spawn it here and don't worry about it

			//exportMergedCloud(depthMats, depthColorMats, bboxes, std::string("./" + activityName_ + "/coordinates/timestamp_" + std::to_string(timeLocal.time_of_day().total_milliseconds()) + ".ply"));

			//exportMergedCloudFromFrames(depthFrames, bboxes, std::string("./" + activityName_ + "/coordinates/timestamp_" + std::to_string(timeLocal.time_of_day().total_milliseconds()) + ".ply"));

			workerThreads_.push_back(
				new std::thread(
					&MultiCameraActivity::exportMergedCloud,
					this,
					depthMats,
					depthColorMats,
					bboxes,
					std::string("./" + activityName_ + "/coordinates/timestamp_" + std::to_string(timeLocal.time_of_day().total_milliseconds()) + ".ply")
				)
			);
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


void MultiCameraActivity::initialPositionsCapture
(
	const std::map<std::string, cv::Mat>& initialColorMats, 
	const std::map<std::string, cv::Mat>& initialDepthMats,
	const std::map<std::string, cv::Mat>& depthColorMappers
)
{

	// Get initial snapshot of all objects in the room
	for (auto & colorMat : initialColorMats)
	{
		std::string deviceName = colorMat.first;
		cv::Mat image = colorMat.second;

		lock_.lock();
		cv::Mat initialPositions = detector_->detectAllObjects(image, initialDetectedObjectsPerDevice_[deviceName]);
		lock_.unlock();

		pcl_color_ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pointCloud->width = image.cols; //Dimensions must be initialized to use 2-D indexing 
		pointCloud->height = image.rows;
		pointCloud->resize(pointCloud->width*pointCloud->height);

		for (auto &objects : initialDetectedObjectsPerDevice_[deviceName])
		{
			std::string objectName = objects.first;
			cv::Rect2d objectBbox = objects.second;

			cv::Mat initialDepthMat = initialDepthMats.at(deviceName);

			// ===============================================================
			// 1. Add depth information to each object
			// ===============================================================
			// If the ROI extends outside the frame (because the detector will try to extrapolate)
			// there will be an error. Therefore we trim the bbox to fit inside the frame
			cv::Rect2d bboxToFitInMat(std::max(0.0, objectBbox.x),
				std::max(0.0, objectBbox.y),
				std::abs(objectBbox.x) + objectBbox.width <= initialDepthMat.cols ? objectBbox.width : initialDepthMat.cols - std::abs(objectBbox.x),
				std::abs(objectBbox.y) + objectBbox.height <= initialDepthMat.rows ? objectBbox.height : initialDepthMat.rows - std::abs(objectBbox.y));

			// Calculate real distance of the object by taking the mean value of the depth points in the ROI around the target object
			cv::Scalar m = cv::mean(initialDepthMat(bboxToFitInMat));


			// print depth information onto the image
			// print depth information onto the image
			cv::putText(initialPositions,
				std::to_string(m[0]) + "mm",
				cv::Point(objectBbox.x, objectBbox.y + bboxToFitInMat.height / 2),
				cv::FONT_HERSHEY_SIMPLEX,
				0.45,
				cv::Scalar(255, 255, 255),
				1);


		}

		// Store the initial snapshot as an image file
		cv::imwrite("./" + activityName_ + "/camera_" + deviceName + "_" + "initialPositions.png", initialPositions);
	}


	// ===============================================================
	// 2. Create pointcloud for each detected object
	// ===============================================================
	exportMergedCloud(initialDepthMats, depthColorMappers, initialDetectedObjectsPerDevice_, "./" + activityName_ + "/initialPositions.ply");
	

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

void MultiCameraActivity::finalPositionsCapture
(
	const std::map<std::string, cv::Mat>& finalColorMats,
	const std::map<std::string, cv::Mat>& finalDepthMats,
	const std::map<std::string, cv::Mat>& depthColorMappers
)
{







	// Get initial snapshot of all objects in the room
	for (auto & colorMat : finalColorMats)
	{
		std::string deviceName = colorMat.first;
		cv::Mat image = colorMat.second;

		lock_.lock();
		cv::Mat finalPositions = detector_->detectAllObjects(image, finalDetectedObjectsPerDevice_[deviceName]);
		lock_.unlock();

		pcl_color_ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pointCloud->width = image.cols; //Dimensions must be initialized to use 2-D indexing 
		pointCloud->height = image.rows;
		pointCloud->resize(pointCloud->width*pointCloud->height);

		for (auto &objects : finalDetectedObjectsPerDevice_[deviceName])
		{
			std::string objectName = objects.first;
			cv::Rect2d objectBbox = objects.second;

			cv::Mat finalDepthMat = finalDepthMats.at(deviceName);

			// ===============================================================
			// 1. Draw specific colored bounding boxes for each detected object
			// ===============================================================

			// An object in the new map cannot be found in the original map, means this is a new object
			// Draw a red colored rectangle
			if (initialDetectedObjectsPerDevice_[deviceName].count(objectName) == 0)
			{
				cv::rectangle(finalPositions, objectBbox, cv::Scalar(0, 0, 255), 2);
			}
			// If the object does exist in the initial snapshot, compare the new bounding box with the previous one
			// If different, means the object has changed
			// Draw a pink colored rectangle
			else if (!ObjectDetector::bboxesEqual(objectBbox, initialDetectedObjectsPerDevice_[deviceName][objectName]))
			{
				cv::rectangle(finalPositions, objectBbox, cv::Scalar(221, 0, 255), 2);
			}

			// ===============================================================
			// 2. Add depth information to each object
			// ===============================================================
			// If the ROI extends outside the frame (because the detector will try to extrapolate)
			// there will be an error. Therefore we trim the bbox to fit inside the frame
			cv::Rect2d bboxToFitInMat(std::max(0.0, objectBbox.x),
				std::max(0.0, objectBbox.y),
				std::abs(objectBbox.x) + objectBbox.width <= finalDepthMat.cols ? objectBbox.width : finalDepthMat.cols - std::abs(objectBbox.x),
				std::abs(objectBbox.y) + objectBbox.height <= finalDepthMat.rows ? objectBbox.height : finalDepthMat.rows - std::abs(objectBbox.y));

			// Calculate real distance of the object by taking the mean value of the depth points in the ROI around the target object
			cv::Scalar m = cv::mean(finalDepthMat(bboxToFitInMat));


			// print depth information onto the image
			// print depth information onto the image
			cv::putText(finalPositions,
				std::to_string(m[0]) + "mm",
				cv::Point(objectBbox.x, objectBbox.y + bboxToFitInMat.height / 2),
				cv::FONT_HERSHEY_SIMPLEX,
				0.45,
				cv::Scalar(255, 255, 255),
				1);


		}

		// Store the initial snapshot as an image file
		cv::imwrite("./" + activityName_ + "/camera_" + deviceName + "_" + "finalPositions.png", finalPositions);
	}


	// ===============================================================
	// 2. Create pointcloud for each detected object
	// ===============================================================
	exportMergedCloud(finalDepthMats, depthColorMappers, finalDetectedObjectsPerDevice_, "./" + activityName_ + "/finalPositions.ply");

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
	const std::map<std::string, cv::Mat>& depthMats, 
	const std::map<std::string, cv::Mat>& depthColorMappers, 
	const std::map<std::string, std::map<std::string, cv::Rect2d>>& bboxes,
	const std::string& path
)
{
	int frameWidth = depthMats.begin()->second.cols;
	int frameHeight = depthMats.begin()->second.rows;

	std::cout << frameWidth << " " << frameHeight << std::endl;

	pcl_color_ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*pointCloud->width = frameWidth; //Dimensions must be initialized to use 2-D indexing 
	pointCloud->height = frameHeight;
	pointCloud->resize(pointCloud->width*pointCloud->height);*/

	for (auto& framePair : depthMats)
	{
		std::string deviceName = framePair.first;
		cv::Mat depthData = framePair.second;

		cv::Mat depthColorMapper = depthColorMappers.at(deviceName);

		// For every detected object in each frame captured by a device
		for (auto& bboxPair : bboxes.at(deviceName))
		{

			cv::Rect2d bbox = bboxPair.second;

			// If the ROI extends outside the frame there will be an error. Therefore we trim the bbox to fit inside the frame
			cv::Rect2d bboxToFitInMat(
				std::max(0.0, bbox.x),
				std::max(0.0, bbox.y),
				std::abs(bbox.x) + bbox.width <= frameWidth ? bbox.width : frameWidth - std::abs(bbox.x),
				std::abs(bbox.y) + bbox.height <= frameHeight ? bbox.height : frameHeight - std::abs(bbox.y));

			pcl_color_ptr currPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

			std::cout << "Point cloud for : " << deviceName << std::endl;

			*currPointCloud = *helper::depthMatToColorPCL(depthData.clone(), depthColorMapper.clone(), (deviceWrapper_->_devices[deviceName].profile.get_stream(RS2_STREAM_COLOR)).as<rs2::video_stream_profile>(), bboxToFitInMat);

			// Apply transformation to the pointcloud based on device's position in the real world
			currPointCloud = helper::affineTransformMatrix(currPointCloud, calibrationMatrices_[deviceName].inverse());

			// Merge pointcloud
			*pointCloud += *currPointCloud;
		}
	}

	pcl::PCLPointCloud2 outputCloudRGB;
	pcl::toPCLPointCloud2(*pointCloud, outputCloudRGB);

	pcl::PLYWriter plyWriter;

	plyWriter.writeASCII(path + "RGB.ply", outputCloudRGB);

	pcl::PointXYZRGB minPt, maxPt;
	pcl::getMinMax3D(*pointCloud, minPt, maxPt);

	double min = minPt.z;
	double max = maxPt.z;

	// Compute LUT scaling to fit the full histogram spectrum
	double lut_scale = (255.0 * 3) / (max - min);  // max is 255, min is 0

	if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
		lut_scale = 1.0;  // Avoid rounding error in boost

	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = pointCloud->begin(); cloud_it != pointCloud->end(); ++cloud_it)
	{
		// Points that are closer will give higher value
		// i.e. for closest points, value will give values closer to 255 * 3
		int value = std::lround((cloud_it->z - min) * lut_scale);



		// Blue -> Green -> Red (~ rainbow)
        
		if (value <= 255)
		{
			cloud_it->r = 0;
			cloud_it->g = value;
			cloud_it->b = 255 - value;
		}
		else if (value > 255 && value <= 510)
		{
			// Convert to 255 scale
			int scaledValue = value - 255;
			cloud_it->r = scaledValue;
			cloud_it->g = 255;
			cloud_it->b = 0;
		}
		// Closest points (value > 510)
		else
		{
			// Convert to 255 scale
			int scaledValue = value - (255 * 2);
			cloud_it->r = 255;
			cloud_it->g = 255 - scaledValue;
			cloud_it->b = 0;
		}

	}


	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*pointCloud, outputCloud);

	//pcl::PLYWriter plyWriter;

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