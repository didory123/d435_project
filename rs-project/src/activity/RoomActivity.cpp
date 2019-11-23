#include "stdafx.h"
#include "RoomActivity.h"


RoomActivity::RoomActivity(
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
	DeviceWrapper& deviceWrapper,
	float leftWidth,
	float rightWidth,
	float topHeight,
	float bottomHeight,
	float cameraDistance
) 
	:
	leftWidth_(leftWidth),
	rightWidth_(rightWidth),
	topHeight_(topHeight),
	bottomHeight_(bottomHeight),
	cameraDistance_(cameraDistance),
	finalOutputCloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
	roomCloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
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

	// Initialize list of colors
	std::vector<color_map> uniqueColors = 
	{
		{ "green", std::make_tuple(60, 180, 75) },
		//{ "purple", std::make_tuple(145, 30, 180) },
		{ "orange", std::make_tuple(245, 130, 49) },
		{ "teal", std::make_tuple(0, 128, 128) },
		{ "yellow", std::make_tuple(255, 225, 25) },
		{ "olive", std::make_tuple(128, 128, 0) },
		{ "lime", std::make_tuple(188, 246, 12) },
		{ "brown", std::make_tuple(154, 99, 36) },
		{ "cyan", std::make_tuple(70, 240, 240) },
		{ "beige", std::make_tuple(255, 250, 200) },
		{ "grey", std::make_tuple(128, 128, 128) },
		//{ "magenta", std::make_tuple(240, 50, 230) },
		//{ "pink", std::make_tuple(250, 190, 190) },
		//{ "coral", std::make_tuple(255, 216, 177) },
		{ "mint", std::make_tuple(170, 255, 195) },
		//{ "maroon", std::make_tuple(128, 0, 0) },
		//{ "navy", std::make_tuple(0, 0, 117) },
		
	};
	for (const auto& i : uniqueColors)
	{
		colorQueue_.push(i);
	}

	// Draw walls for the room scene pointcloud

	for (float y = topHeight_ * -1; y < bottomHeight_; y += 0.01)
	{
		for (float z = 0.0; z < cameraDistance_; z += 0.01)
		{
			pcl::PointXYZRGB pLeft;
			pLeft.y = y;
			pLeft.z = z;
			pLeft.x = leftWidth_;
			pLeft.r = 255;
			pLeft.g = 255;
			pLeft.b = 255;

			pcl::PointXYZRGB pRight;
			pRight.y = y;
			pRight.z = z;
			pRight.x = rightWidth_ * -1;
			pRight.r = 255;
			pRight.g = 255;
			pRight.b = 255;

			finalOutputCloud_->points.push_back(pLeft);
			finalOutputCloud_->points.push_back(pRight);
			roomCloud_->points.push_back(pLeft);
			roomCloud_->points.push_back(pRight);
		}
		for (float x = rightWidth_ * -1; x < leftWidth_; x += 0.01)
		{
			pcl::PointXYZRGB pFarWall;
			pFarWall.y = y;
			pFarWall.x = x;
			pFarWall.z = 0;
			pFarWall.r = 255;
			pFarWall.g = 255;
			pFarWall.b = 255;
			finalOutputCloud_->points.push_back(pFarWall);
			roomCloud_->points.push_back(pFarWall);
		}

	}

	// ===============================================================
	// 1. Initialize some parameters
	// ===============================================================
	// Create video recording
	output_.open("./" + activityName_ + "./output.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 15, cv::Size(frameSize_.width * initialColorMats.size(), frameSize_.height));

	// Open the text file to which we will log the information about the detected objects
	objectInfoFile_.open("./" + activityName_ + "./objectsInfo.txt");

	// Open the text file to which we will log the timestamps of the coordinate points
	coordinateTimestampsFile_.open("./" + activityName_ + "./coordinates/timestamps.txt");

	// Initialize tracker object
	for (auto& bbox : personBboxes)
	{
		trackers_[bbox.first] = helper::createTrackerByName(trackerType_);
		trackers_[bbox.first]->init(initialColorMats[bbox.first], bbox.second);

	}

	// Begin activity
	beginActivity(initialColorMats, initialDepthMats, depthColorMappers, personBboxes);

}

void RoomActivity::beginActivity(
	std::map<std::string, cv::Mat> initialColorMats,
	std::map<std::string, cv::Mat> initialDepthMats,
	std::map<std::string, cv::Mat> depthColorMappers,
	std::map<std::string, cv::Rect2d> personBboxes
)
{

	// ===============================================================
	// 2. Get initial snapshot of rooms
	// ===============================================================
	initialPositionsCapture(initialColorMats, initialDepthMats, depthColorMappers);

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


	std::map<std::string, cv::Mat> finalDepthMats, finalColorMats, finalColorMappers;

	for (const auto& framePair : colorFrames)
	{

		finalColorMats[framePair.first] = helper::frameToMat(framePair.second);
		
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

	rs2::colorizer colorMapper;

	for (auto& framePair : depthFrames)
	{
		//framePair.second = dec_filter.process(framePair.second); // 1
		framePair.second = depth_to_disparity.process(framePair.second); // 2
		framePair.second = spat_filter.process(framePair.second); // 3
		framePair.second = temp_filter.process(framePair.second); // 4
		framePair.second = disparity_to_depth.process(framePair.second); // 5

		finalDepthMats[framePair.first] = helper::frameToMat(framePair.second);

		rs2::frame coloredDepth = framePair.second.apply_filter(colorMapper);
		finalColorMappers[framePair.first] = cv::Mat(frameSize_, CV_8UC3, (void*)coloredDepth.get_data(), cv::Mat::AUTO_STEP);
	}

	// Get final positions
	finalPositionsCapture(finalColorMats, finalDepthMats, finalColorMappers);

	// Log object info to final output text file
	// Format:
	// objectName : (centroid_x_value centroid_y_value centroid_z_value)
	for (auto it = detectedObjectsInFinalPointCloud_.begin(); it != detectedObjectsInFinalPointCloud_.end(); it++)
	{
		float totalX = 0.0, totalY = 0.0, totalZ = 0.0;
		float finalWidth = 0.0;
		float finalHeight = 0.0;
		for (object_info obj : it->second)
		{
			finalWidth += obj.width;
			finalHeight += obj.height;
			totalX += obj.centroidPoint.x;
			totalY += obj.centroidPoint.y;
			totalZ += obj.centroidPoint.z;
		}

		// Get average dimensions of the object
		finalWidth = finalWidth / it->second.size();
		finalHeight = finalHeight / it->second.size();

		// Get average centroid point of the object
		pcl::PointXYZ centroidPoint(totalX / it->second.size(), totalY / it->second.size(), totalZ / it->second.size());

		// Print object name
		objectInfoFile_ << it->first << " : ";

		// Print centroid point info
		objectInfoFile_ << "(" << centroidPoint.x << " " << centroidPoint.y << " " << centroidPoint.z << ") ";

		// Print approximate object size info
		objectInfoFile_ << "(" << finalWidth << " " << finalHeight << " " << std::min(finalWidth, finalHeight) << ")\n";
	}

	pcl::PLYWriter plyWriter;

	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*finalOutputCloud_, outputCloud);

	pcl::PCLPointCloud2 roomCloud;
	pcl::toPCLPointCloud2(*roomCloud_, roomCloud);

	plyWriter.writeASCII("./" + activityName_ + "./activity.ply", outputCloud);
	plyWriter.writeASCII("./" + activityName_ + "./room.ply", roomCloud);

	// Close output text files
	objectInfoFile_.close();
	coordinateTimestampsFile_.close();

	// Wait for the worker threads to finish before terminating activity
	for (auto &i : workerThreads_)
	{
		i->join();
	}
}

void RoomActivity::start()
{


	bool wasPersonInFrameLast = false;

	// keeps track of how many frames have passed where a person wasn't found
	int personMissingCounter = 0;

	// Initialize this to 90 so that the object detection occurs at the very beginning
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
		std::map<std::string, cv::Rect2d> bboxes; // <deviceName, <objectName, bbox>> 

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
				else
				{
					trackers_[deviceName]->clear(); // remove previous box that we were tracking
				}
			}
			// otherwise, simply track the person
			else if (!trackers_[deviceName]->empty())
			{
				// Bitwise OR here, because we want to continue activity if person is found in at least one camera
				isPersonInFrame = isPersonInFrame | trackers_[deviceName]->update(image, bbox[deviceName]);
			}
				
			

			if (isPersonInFrame)
			{
				// Tracking success : Draw the tracked object
				rectangle(image, bbox[deviceName], cv::Scalar(255, 0, 0), 2, 1);
			}

			bboxes[deviceName] = bbox[deviceName];
			outputImages[deviceName] = image.clone();
			//std::thread * newWorkerThread = 

		}

		// Record coordinates
		if (frameCount % 15 == 0 && isPersonInFrame)
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

			
			/*
			for (auto& framePair : depthFrames)
			{

				std::string deviceName = framePair.first;
				cv::Mat depthImage = helper::frameToMat(framePair.second);

				// Store the depth frames for pointcloud conversion
				depthMats[deviceName] = depthImage.clone();

			}*/

			// Get current time at which this pointcloud was captured
			boost::posix_time::ptime timeLocal =
				boost::posix_time::second_clock::local_time();

			// Record the person's centroid coordinate and draw trajectory line
			recordCoordinate(depthMats, bboxes);

			/*workerThreads_.push_back(
				new std::thread(
					&RoomActivity::exportMergedCloud,
					this,
					depthMats,
					depthColorMats,
					bboxes,
					std::string("./" + activityName_ + "/coordinates/timestamp_" + std::to_string(timeLocal.time_of_day().total_milliseconds()) + ".ply"),
					true
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
		if (personMissingCounter > 90)
		{
			break;
		}

	}
	// Release video file
	output_.release();
}

void RoomActivity::initialPositionsCapture
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

		for (auto &objects : initialDetectedObjectsPerDevice_[deviceName])
		{
			std::string objectName = objects.first;
			std::string labelName = "initial " + objectName + " position";
			cv::Rect2d objectBbox = objects.second;

			allInitialObjects_.insert(objectName);

			cv::Mat initialDepthMat = initialDepthMats.at(deviceName);

			// Get base color for this object
			auto baseColor = getColorForObject(objectName);

			// ===============================================================
			// 1. Add depth information to each object
			// ===============================================================
			// If the ROI extends outside the frame (because the detector will try to extrapolate)
			// there will be an error. Therefore we trim the bbox to fit inside the frame
			cv::Rect2d bboxToFitInMat(std::max(0.0, objectBbox.x),
				std::max(0.0, objectBbox.y),
				std::abs(objectBbox.x) + objectBbox.width <= initialDepthMat.cols ? objectBbox.width : initialDepthMat.cols - std::abs(objectBbox.x),
				std::abs(objectBbox.y) + objectBbox.height <= initialDepthMat.rows ? objectBbox.height : initialDepthMat.rows - std::abs(objectBbox.y));

			// Draw shape with random unique color, but with white streaks
			addToFinalOutputCloud(initialDepthMat, bboxToFitInMat, deviceName, labelName, COLOR_WHITE_STREAK.rgbValue, baseColor);

			// Calculate real distance of the object by taking the mean value of the depth points in the ROI around the target object
			cv::Scalar m = cv::mean(initialDepthMat(bboxToFitInMat));

			// print depth information onto the image
			cv::putText(initialPositions,
				std::to_string(m[0]) + "mm",
				cv::Point(objectBbox.x, objectBbox.y + bboxToFitInMat.height / 2),
				cv::FONT_HERSHEY_SIMPLEX,
				0.45,
				cv::Scalar(255, 255, 255),
				1);


		}

		// Store the initial RGB snapshot as an image file
		cv::imwrite("./" + activityName_ + "/camera_" + deviceName + "_" + "initialPositionsRGB.png", initialPositions);

		// Store the initial depth snapshot as an image file
		//cv::Mat depthImage = depthColorMappers.at(deviceName).clone();
		//cv::imwrite("./" + activityName_ + "/camera_" + deviceName + "_" + "initialPositionsDepth.png", depthImage);
	}


	// ===============================================================
	// 2. Create pointcloud for each detected object
	// ===============================================================
	//exportMergedCloud(initialDepthMats, depthColorMappers, initialDetectedObjectsPerDevice_, "./" + activityName_ + "/initialPositions.ply", false);


}

void RoomActivity::finalPositionsCapture
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

		for (auto &objects : finalDetectedObjectsPerDevice_[deviceName])
		{
			std::string objectName = objects.first;
			std::string labelName = objectName + " position";
			cv::Rect2d objectBbox = objects.second;

			cv::Mat finalDepthMat = finalDepthMats.at(deviceName);

			// Get base color for this object
			auto baseColor = getColorForObject(objectName);

			// ===============================================================
			// 1. Draw specific colored bounding boxes for each detected object
			// ===============================================================

			// If the ROI extends outside the frame (because the detector will try to extrapolate)
			// there will be an error. Therefore we trim the bbox to fit inside the frame
			cv::Rect2d bboxToFitInMat(std::max(0.0, objectBbox.x),
				std::max(0.0, objectBbox.y),
				std::abs(objectBbox.x) + objectBbox.width <= finalDepthMat.cols ? objectBbox.width : finalDepthMat.cols - std::abs(objectBbox.x),
				std::abs(objectBbox.y) + objectBbox.height <= finalDepthMat.rows ? objectBbox.height : finalDepthMat.rows - std::abs(objectBbox.y));

			// An object in the new map cannot be found in the original map, means this is a new object
			// Draw a red colored rectangle/draw red colored pointcloud
			if (allInitialObjects_.count(objectName) == 0)
			{
				cv::rectangle(finalPositions, bboxToFitInMat, cv::Scalar(0, 0, 255), 2);
				// For new objects, make the segmented pointcloud be completely red
				addToFinalOutputCloud(finalDepthMat, bboxToFitInMat, deviceName, "new " + labelName, COLOR_RED.rgbValue, COLOR_RED.rgbValue);
			}
			// If the object does exist in the initial snapshot, compare the new bounding box with the previous one
			// If different, means the object has changed
			// Draw a blue colored rectangle/draw blue streaked pointcloud
			else if (initialDetectedObjectsPerDevice_[deviceName].count(objectName) != 0 && !ObjectDetector::bboxesEqual(objectBbox, initialDetectedObjectsPerDevice_[deviceName][objectName]))
			{
				cv::rectangle(finalPositions, bboxToFitInMat, cv::Scalar(255, 0, 0), 2);
				addToFinalOutputCloud(finalDepthMat, bboxToFitInMat, deviceName, "final " + labelName, COLOR_PURPLE_STREAK.rgbValue, baseColor);
			}

			// ===============================================================
			// 2. Add depth information to each object
			// ===============================================================

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
		cv::imwrite("./" + activityName_ + "/camera_" + deviceName + "_" + "finalPositionsRGB.png", finalPositions);

		// Store the initial depth snapshot as an image file
		//cv::Mat depthImage = depthColorMappers.at(deviceName).clone();
		//cv::imwrite("./" + activityName_ + "/camera_" + deviceName + "_" + "finalPositionsDepth.png", depthImage);
	}


	// ===============================================================
	// 2. Create pointcloud for each detected object
	// ===============================================================
	

	//exportMergedCloud(finalDepthMats, depthColorMappers, finalDetectedObjectsPerDevice_, "./" + activityName_ + "/finalPositions.ply", );

}

void RoomActivity::addToFinalOutputCloud
(
	const cv::Mat& depthMat,
	const cv::Rect2d & bbox,
	const std::string& deviceName,
	const std::string& objectName,
	std::tuple<int, int, int> color,
	std::tuple<int, int, int> baseColor
)
{
	int frameWidth = depthMat.cols;
	int frameHeight = depthMat.rows;

	// If the ROI extends outside the frame there will be an error. Therefore we trim the bbox to fit inside the frame
	cv::Rect2d bboxToFitInMat(
		std::max(0.0, bbox.x),
		std::max(0.0, bbox.y),
		std::abs(bbox.x) + bbox.width <= frameWidth ? bbox.width : frameWidth - std::abs(bbox.x),
		std::abs(bbox.y) + bbox.height <= frameHeight ? bbox.height : frameHeight - std::abs(bbox.y));

	pcl_color_ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	*pointCloud = *helper::depthMatToColorPCL(depthMat.clone(), depthMat.clone(), (deviceWrapper_->_devices[deviceName].profile.get_stream(RS2_STREAM_COLOR)).as<rs2::video_stream_profile>(), bboxToFitInMat);

	// IMPORTANT: We must get the minimum and maximum points before we apply the calibration matrix operations
	// Otherwise, the min/max points will be scaled to the ROOM dimensions, but what we want is the actual min/max points of the object dimensions
	// as seen by the camera at a straight angle.
	pcl::PointXYZRGB minPt, maxPt;
	pcl::getMinMax3D(*pointCloud, minPt, maxPt);

	// Apply transformation to the pointcloud based on device's position in the real world
	pointCloud = helper::affineTransformMatrix(pointCloud, calibrationMatrices_[deviceName].inverse());

	pcl::PointXYZRGB centroidPoint = getCentroidPoint(pointCloud);

	// Store every same object recorded by each camera; the average of these points will be used as the final centroid point
	object_info obj =
	{
		pcl::PointXYZ(centroidPoint.x, centroidPoint.y, centroidPoint.z),
		abs(maxPt.x - minPt.x),
		abs(maxPt.y - minPt.y)
	};
	detectedObjectsInFinalPointCloud_[objectName].push_back(obj);

	/*
	double minz = minPt.z;
	double maxz = maxPt.z;

	double lut_scale = 1.0; // In case the cloud is flat on the chosen direction (x,y or z) i.e. max == min

	// Make sure to avoid divide by zero
	if (minz != maxz)
	{
		// Compute LUT scaling to fit the full histogram spectrum
		lut_scale = 255.0 / (maxz - minz);  // max is 255, min is 0
	}*/

	int count = 0;
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = pointCloud->begin(); cloud_it != pointCloud->end(); ++cloud_it)
	{

		// alternate between the color denoting its position, and the object base color, every 3rd point
		if (count == 3)
		{
			cloud_it->r = std::get<0>(color);
			cloud_it->g = std::get<1>(color);
			cloud_it->b = std::get<2>(color);
			count = 0;
		}
		else
		{
			cloud_it->r = std::get<0>(baseColor);
			cloud_it->g = std::get<1>(baseColor);
			cloud_it->b = std::get<2>(baseColor);
			count++;
		}


		/*if (color == "red")
		{
			cloud_it->r = 255;
			cloud_it->g = 255 - value;
			cloud_it->b = 255 - value;
		}
		else if (color == "green")
		{
			cloud_it->r = 255 - value;
			cloud_it->g = 255;
			cloud_it->b = 255 - value;
		}
		else
		{
			cloud_it->r = 255 - value;
			cloud_it->g = 255 - value;
			cloud_it->b = 255;
		}*/

	}

	finalOutputCloudLock_.lock();
	*finalOutputCloud_ += *pointCloud;
	finalOutputCloudLock_.unlock();

}

void RoomActivity::exportCloud(cv::Mat depthData, cv::Mat depthColorMapper, cv::Rect2d bbox, const std::string& deviceName, const std::string& path)
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

void RoomActivity::recordCoordinate
(
	const std::map<std::string, cv::Mat>& depthMats,
	const std::map<std::string, cv::Rect2d>& bboxes
)
{
	int frameWidth = depthMats.begin()->second.cols;
	int frameHeight = depthMats.begin()->second.rows;

	/*pointCloud->width = frameWidth; //Dimensions must be initialized to use 2-D indexing
	pointCloud->height = frameHeight;
	pointCloud->resize(pointCloud->width*pointCloud->height);*/
	pcl_color_ptr centroidPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	float sum_x = 0, sum_y = 0, sum_z = 0;

	std::vector<pcl::PointXYZRGB> centroidPoints; // centroid points of all person bboxes from all cameras

	for (auto& framePair : depthMats)
	{
		std::string deviceName = framePair.first;
		cv::Mat depthData = framePair.second;

		// Check that a person has been detected in this frame i.e. this camera has detected a person
		if (bboxes.count(deviceName) == 0)
		{
			continue;
		}

		// Get the detected person's bbox
		cv::Rect2d bbox = bboxes.at(deviceName);

		// If the ROI extends outside the frame there will be an error. Therefore we trim the bbox to fit inside the frame
		cv::Rect2d bboxToFitInMat(
			std::max(0.0, bbox.x),
			std::max(0.0, bbox.y),
			std::abs(bbox.x) + bbox.width <= frameWidth ? bbox.width : frameWidth - std::abs(bbox.x),
			std::abs(bbox.y) + bbox.height <= frameHeight ? bbox.height : frameHeight - std::abs(bbox.y));

		pcl_color_ptr currPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		std::cout << "Point cloud for : " << deviceName << std::endl;

		*currPointCloud = *helper::depthMatToColorPCL(depthData.clone(), depthData.clone(), (deviceWrapper_->_devices[deviceName].profile.get_stream(RS2_STREAM_COLOR)).as<rs2::video_stream_profile>(), bboxToFitInMat);

		// Apply transformation to the pointcloud based on device's position in the real world
		currPointCloud = helper::affineTransformMatrix(currPointCloud, calibrationMatrices_[deviceName].inverse());

		centroidPoints.push_back(getCentroidPoint(currPointCloud));
	}


	// The centroid point will be the average of all the centroid points calculated from all cameras
	pcl::PointXYZRGB centroidPoint;
	float sumx = 0.0, sumy = 0.0, sumz = 0.0;
	for (const auto& point : centroidPoints)
	{
		sumx += point.x;
		sumy += point.y;
		sumz += point.z;
	}

	centroidPoint.x = sumx / centroidPoints.size();
	centroidPoint.y = sumy / centroidPoints.size();
	centroidPoint.z = sumz / centroidPoints.size();

	// Mark the centroid point as pink
	centroidPoint.r = 255;
	centroidPoint.g = 20;
	centroidPoint.b = 147;

	// Ensure that this isn't the first centroid point calculated (i.e. first recorded coordinate)
	if (prevCentroidPoint_ != nullptr)
	{
		// Parametric equation
		// Line in 3D = p1 + t*v = <x0, y0, z0> + t*<v1, v2, v3>
		// v = p2 - p1  where v = lineVector
		// t is the parametric variable

		float v1 = centroidPoint.x - prevCentroidPoint_->x;
		float v2 = centroidPoint.y - prevCentroidPoint_->y;
		float v3 = centroidPoint.z - prevCentroidPoint_->z;


		pcl::PointXYZRGB newTrajectoryPoint = *prevCentroidPoint_;

		float distance =
			(centroidPoint.x - prevCentroidPoint_->x) * (centroidPoint.x - prevCentroidPoint_->x) +
			(centroidPoint.y - prevCentroidPoint_->y) * (centroidPoint.y - prevCentroidPoint_->y) +
			(centroidPoint.z - prevCentroidPoint_->z) * (centroidPoint.z - prevCentroidPoint_->z);

		// Check that the person has moved a meaningful enough distance to warrant recording the trajectory
		// If the distance is less than 10cm, return
		if (distance < 0.1)
		{
			return;
		}
		else
		{
			int count = 1;

			// Use distance to measure if we have reached the end of the vector as we draw the trajectory
			// As we go along the vector line from previous centroid point to current centroid point,
			// the stop condition is if the current distance is equal to the desired vector length
			while
				(
				(newTrajectoryPoint.x - prevCentroidPoint_->x) * (newTrajectoryPoint.x - prevCentroidPoint_->x) +
					(newTrajectoryPoint.y - prevCentroidPoint_->y) * (newTrajectoryPoint.y - prevCentroidPoint_->y) +
					(newTrajectoryPoint.z - prevCentroidPoint_->z) * (newTrajectoryPoint.z - prevCentroidPoint_->z) < distance
					)
			{
				newTrajectoryPoint.x = prevCentroidPoint_->x + count * 0.001 * v1;
				newTrajectoryPoint.y = prevCentroidPoint_->y + count * 0.001 * v2;
				newTrajectoryPoint.z = prevCentroidPoint_->z + count * 0.001 * v3;

				centroidPointCloud->points.push_back(newTrajectoryPoint);

				count++;
			}
		}
	}
	else
	{
		// Record the new centroid point
		prevCentroidPoint_ = new pcl::PointXYZRGB(centroidPoint);
	}

	// Record current coordinate and its timestamp
	coordinateTimestampsFile_ << centroidPoint.x << " " << centroidPoint.y << " " << centroidPoint.z << " " << std::chrono::system_clock::now().time_since_epoch().count() << "\n";

	// Add centroid point to output pointcloud
	centroidPointCloud->points.push_back(centroidPoint);

	// Scatter some points next to the centroid point for better visibility
	for (int i = 0; i < 3; i++)
	{
		pcl::PointXYZRGB pointAdj1 = centroidPoint;
		pcl::PointXYZRGB pointAdj2 = centroidPoint;
		if (i == 0)
		{
			pointAdj1.x += 0.005;
			pointAdj2.x -= 0.005;
		}
		else if (i == 1)
		{
			pointAdj1.y += 0.005;
			pointAdj2.y -= 0.005;
		}
		else if (i == 3)
		{
			pointAdj1.z += 0.005;
			pointAdj2.z -= 0.005;
		}
		centroidPointCloud->points.push_back(pointAdj1);
		centroidPointCloud->points.push_back(pointAdj2);
	}

	// Record the new centroid point
	*prevCentroidPoint_ = centroidPoint;

	*finalOutputCloud_ += *centroidPointCloud;
	*roomCloud_ += *centroidPointCloud;

}

void RoomActivity::exportMergedCloud
(
	const std::map<std::string, cv::Mat>& depthMats,
	const std::map<std::string, cv::Mat>& depthColorMappers,
	const std::map<std::string, std::map<std::string, cv::Rect2d>>& bboxes,
	const std::string& path,
	std::tuple<int, int, int> rgbColor
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

	double lut_scale = 1.0; // In case the cloud is flat on the chosen direction (x,y or z) i.e. max == min

	// Make sure to avoid divide by zero
	if (min != max)  
	{
		// Compute LUT scaling to fit the full histogram spectrum
		lut_scale = 255.0 / (max - min);  // max is 255, min is 0
	}

	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = pointCloud->begin(); cloud_it != pointCloud->end(); ++cloud_it)
	{
		// Points that are closer will give higher value
		// i.e. for closest points, value will give values closer to 255 * 3
		auto value = std::lround((cloud_it->z - min) * lut_scale);

		cloud_it->r = std::max(cloud_it->r, static_cast<uint8_t>(255 - value));
		cloud_it->g = std::max(cloud_it->g, static_cast<uint8_t>(255 - value));
		cloud_it->b = std::max(cloud_it->b, static_cast<uint8_t>(255 - value));

	}


	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*pointCloud, outputCloud);

	//pcl::PLYWriter plyWriter;

	plyWriter.writeASCII(path, outputCloud);

	//Activity::exportPointCloud(depthData, depthColorMapper, bbox, path);
}

std::tuple<int, int, int> RoomActivity::getColorForObject(const std::string& objectName)
{
	if (objectToColorMap_.count(objectName) > 0)
	{
		return objectToColorMap_[objectName].rgbValue;
	}

	// Get next available color
	color_map nextColor = colorQueue_.front();

	// Assign this color for this object
	objectToColorMap_[objectName].colorName = nextColor.colorName;
	objectToColorMap_[objectName].rgbValue = nextColor.rgbValue;

	// Put this color at the back fo the queue, so we may reuse it later if we run out of colors
	colorQueue_.pop();
	colorQueue_.push(nextColor);
	return nextColor.rgbValue;
}


pcl::PointXYZRGB RoomActivity::getCentroidPoint(pcl_color_ptr pointCloud)
{
	// Calculate centroid point
	float sumx = 0.0, sumy = 0.0, sumz = 0.0;
	int pointsSize = 0;

	// Find sum of all valid points
	// pointsSize is the number of all valid points
	// sum / pointsSize give us the average (centroid)
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = pointCloud->begin(); cloud_it != pointCloud->end(); ++cloud_it)
	{
		sumx += cloud_it->x;
		sumy += cloud_it->y;
		sumz += cloud_it->z;
		pointsSize++;
	}

	pcl::PointXYZRGB centroidPoint;

	// Check pointsSize > 0 to avoid division by zero
	if (pointsSize > 0)
	{

		centroidPoint.x = sumx / pointsSize;
		centroidPoint.y = sumy / pointsSize;
		centroidPoint.z = sumz / pointsSize;

	}
	return centroidPoint;
}