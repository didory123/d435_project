#include "stdafx.h"
#include "Activity.h"

Activity::Activity(
	cv::Size frameSize,
	rs2::pipeline& pipe, 
	const std::string& trackerType, 
	const cv::Mat& initialColorMat, 
	const cv::Mat& initialDepthMat, 
	const cv::Mat& depthColorMapper,
	const std::string& windowName, 
	const cv::Rect2d& personBbox, 
	ObjectDetector& detector,
	rs2::align& aligner)
{

	frameSize_ = frameSize;

	align_to_color_ = &aligner;

	pipe_ = &pipe;

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
	windowName_ = windowName;

	// Create a folder to store all the test results data
	boost::filesystem::create_directory(activityName_);

	// Initialize tracker object
	trackerType_ = trackerType;

	// Create a detector object for this activity
	detector_ = &detector;

	beginActivity(initialColorMat, initialDepthMat, depthColorMapper, personBbox);

}

Activity::~Activity()
{
	for (auto& i : workerThreads_)
	{
		delete i;
	}
}

void Activity::beginActivity(const cv::Mat& initialColorMat, const cv::Mat& initialDepthMat, const cv::Mat& depthColorMapper, const cv::Rect2d& personBbox)
{
	// ===============================================================
	// 1. Initialize some parameters
	// ===============================================================
	// Create video recording
	output_.open("./" + activityName_ + "./output.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 15, cv::Size(initialColorMat.cols, initialColorMat.rows));

	// Initialize tracker object
	tracker_ = helper::createTrackerByName(trackerType_);
	tracker_->init(initialColorMat, personBbox);

	// ===============================================================
	// 2. Get initial snapshot of rooms
	// ===============================================================

	std::thread initialSnapshotThread(
		&Activity::initialSnapshot, 
		this, 
		initialColorMat, 
		initialDepthMat, 
		depthColorMapper, 
		true, 
		"./" + activityName_ + "/initialPositions.ply", 
		"./" + activityName_ + "/initialPositions.png");

	//initialSnapshot(initialColorMat, initialDepthMat);

	// ===============================================================
	// 3. Main loop
	// ===============================================================
	start();

	// ===============================================================
	// 4. Get final snapshot of rooms
	// ===============================================================
	
	// Get final snapshot data
	rs2::frameset data = pipe_->wait_for_frames(); // Wait for next set of frames from the camera
	data = align_to_color_->process(data);
	rs2::frame depth = data.get_depth_frame(); // Get depth frames
	rs2::frame rgb = data.get_color_frame(); // Get RGB frames

	cv::Mat finalRgb = helper::frameToMat(data.get_color_frame());
	//cv::Mat finalDepth = helper::depthFrameToScale(*pipe_, data.get_depth_frame());
	cv::Mat finalDepth = helper::frameToMat(data.get_depth_frame());
	rs2::colorizer colorMapper;
	depth = colorMapper.colorize(depth);

	cv::Mat finalDepthColor = cv::Mat(frameSize_, CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

	// Get final snapshot of room
	std::thread finalSnapshotThread(
		&Activity::finalSnapshot, 
		this, 
		finalRgb, 
		finalDepth, 
		finalDepthColor, 
		true,
		"./" + activityName_ + "/finalPositions.ply",
		"./" + activityName_ + "/finalPositions.png");

	//finalSnapshot(finalRgb, finalDepth);

	// Wait for the worker threads to finish before terminating activity
	initialSnapshotThread.join();
	finalSnapshotThread.join();
	for (auto &i : workerThreads_)
	{
		i->join();
	}
}

void Activity::initialSnapshot(cv::Mat initialColorMat, cv::Mat initialDepthMat, cv::Mat depthColorMapper, bool exportCloud, std::string pathToPLY, std::string pathToPNG)
{
	// Get initial snapshot of all objects in the room
	lock_.lock();
	cv::Mat initialPositions = detector_->detectAllObjects(initialColorMat, initialDetectedObjects_);
	lock_.unlock();

	// Initialize pointcloud
	//pcl_ptr initialObjectsCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl_color_ptr initialObjectsCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	initialObjectsCloud->width = initialColorMat.cols; //Dimensions must be initialized to use 2-D indexing 
	initialObjectsCloud->height = initialColorMat.rows;
	initialObjectsCloud->resize(initialObjectsCloud->width*initialObjectsCloud->height);

	for (auto &i : initialDetectedObjects_)
	{
		// ===============================================================
		// 1. Add depth information to each object
		// ===============================================================
		// If the ROI extends outside the frame (because the detector will try to extrapolate)
		// there will be an error. Therefore we trim the bbox to fit inside the frame
		cv::Rect2d bboxToFitInMat(std::max(0.0, i.second.x),
			std::max(0.0, i.second.y),
			i.second.x + i.second.width <= initialDepthMat.cols ? i.second.width : initialDepthMat.cols - i.second.x,
			i.second.y + i.second.height <= initialDepthMat.rows ? i.second.height : initialDepthMat.rows - i.second.y);

		// Calculate real distance of the object by taking the mean value of the depth points in the ROI around the target object
		cv::Scalar m = cv::mean(initialDepthMat(bboxToFitInMat));

		// print depth information onto the image
		// print depth information onto the image
		cv::putText(initialPositions,
			std::to_string(m[0]) + " meters away",
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

void Activity::start()
{
	cv::Rect2d bbox;
	bool isPersonInFrame = false;

	// keeps track of how many frames have passed where a person wasn't found
	int personMissingCounter = 0;

	// Initialize this to 40 so that the object detection occurs at the very beginning
	int frameCount = 90;

	while (cv::waitKey(1) < 0)
	{
		rs2::frameset data = pipe_->wait_for_frames(); // Wait for next set of frames from the camera
		data = align_to_color_->process(data);
		rs2::frame rgb = data.get_color_frame(); // Get RGB frames

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		cv::Mat image = helper::frameToMat(rgb);
		
		// if 90 frames has passed, run detection
		if (frameCount == 90)
		{
			lock_.lock();
			isPersonInFrame = detector_->detectPerson(image, bbox);
			lock_.unlock();
			if (isPersonInFrame)
			{
				// Recreate and reinitialize the tracker
				tracker_ = helper::createTrackerByName(trackerType_);
				tracker_->init(image, bbox);
				//tracker_->update(image, bbox);
				personMissingCounter = 0;
				//tracker->init(image, newbbox);

				//bbox = newbbox;
			}
			frameCount = 0;
		}
		// otherwise, simply track the person
		else
		{

			isPersonInFrame = tracker_->update(image, bbox);

			if (isPersonInFrame)
			{
				personMissingCounter = 0;
				// Tracking success : Draw the tracked object
				rectangle(image, bbox, cv::Scalar(255, 0, 0), 2, 1);
			}
			frameCount++;
		}

		if (!isPersonInFrame)
		{
			cv::putText(image, "Tracking failure detected", cv::Point(100, 150), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
			personMissingCounter++;
			cv::putText(image,
				"No person detected for " + std::to_string(personMissingCounter),
				cv::Point(10, image.rows - 60),
				cv::FONT_HERSHEY_SIMPLEX,
				0.75,
				cv::Scalar(255, 0, 255),
				2);
		}

		else if (frameCount == 60)
		{
			// Every 60 frames, if the person is found in the room store the person's movement location as pointcloud

			// Get depth frame
			rs2::frame depthFrame = data.get_depth_frame();
			cv::Mat depthMat = helper::depthFrameToScale(*pipe_, depthFrame);// helper::frameToMat(depthFrame);

			// Get depth color mat
			rs2::colorizer colorMapper;
			depthFrame = colorMapper.colorize(depthFrame);

			cv::Mat depthColor = cv::Mat(frameSize_, CV_8UC3, (void*)depthFrame.get_data(), cv::Mat::AUTO_STEP);

			// Spawn a new worker thread to export the data

			// Get current time at which this pointcloud was captured
			boost::posix_time::ptime timeLocal =
				boost::posix_time::second_clock::local_time();

			// Turns out the cv::Mat copy constructor doesn't do a deep copy. You need to specify clone() to do a deep copy otherwise
			// the thread will throw an error once this function's matrices go out of scope

			workerThreads_.push_back(
				new std::thread(
					&Activity::exportPointCloud,
					this, 
					depthMat.clone(), 
					depthColor.clone(), 
					bbox, 
					"./" + activityName_ + "/timestamp_" + std::to_string(timeLocal.time_of_day().hours()) + "h" +
					std::to_string(timeLocal.time_of_day().minutes()) + "m" +
					std::to_string(timeLocal.time_of_day().seconds()) + "s" + ".ply"
				)
			);

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
					"Next detection in " + std::to_string(90 - frameCount), 
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


void Activity::exportPointCloud(cv::Mat depthData, cv::Mat depthColorMapper, cv::Rect2d bbox, std::string path)
{
	pcl_color_ptr finalObjectsCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	finalObjectsCloud->width = depthData.cols; //Dimensions must be initialized to use 2-D indexing 
	finalObjectsCloud->height = depthData.rows;
	finalObjectsCloud->resize(finalObjectsCloud->width*finalObjectsCloud->height);

	// If the ROI extends outside the frame there will be an error. Therefore we trim the bbox to fit inside the frame
	cv::Rect2d bboxToFitInMat(
		std::max(0.0, bbox.x),
		std::max(0.0, bbox.y),
		bbox.x + bbox.width <= depthData.cols ? bbox.width : depthData.cols - bbox.x,
		bbox.y + bbox.height <= depthData.rows ? bbox.height : depthData.rows - bbox.y);

	*finalObjectsCloud += *helper::depthMatToColorPCL(depthData, depthColorMapper, (pipe_->get_active_profile().get_stream(RS2_STREAM_DEPTH)).as<rs2::video_stream_profile>(), bboxToFitInMat);
	
	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*finalObjectsCloud, outputCloud);

	pcl::PLYWriter plyWriter;

	plyWriter.writeASCII(path, outputCloud);
}

void Activity::finalSnapshot(cv::Mat finalSnapshotColor, cv::Mat finalSnapshotDepth, cv::Mat depthColorMapper, bool exportCloud, std::string pathToPLY, std::string pathToPNG)
{

	// in theory this should never be reached
	lock_.lock();
	cv::Mat finalPositions = detector_->detectAllObjects(finalSnapshotColor, finalDetectedObjects_);
	lock_.unlock();

	// Initialize pointcloud
	pcl_color_ptr finalObjectsCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	finalObjectsCloud->width = finalSnapshotDepth.cols; //Dimensions must be initialized to use 2-D indexing 
	finalObjectsCloud->height = finalSnapshotDepth.rows;
	finalObjectsCloud->resize(finalObjectsCloud->width*finalObjectsCloud->height);

	// Compare the two maps and if there is a difference in coordinates, update the color in the final image
	for (auto &i : finalDetectedObjects_)
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
			i.second.x + i.second.width <= finalSnapshotDepth.cols ? i.second.width : finalSnapshotDepth.cols - i.second.x,
			i.second.y + i.second.height <= finalSnapshotDepth.rows ? i.second.height : finalSnapshotDepth.rows - i.second.y);

		// Calculate real distance of the object by taking the mean value of the depth points in the ROI around the target object
		cv::Scalar m = cv::mean(finalSnapshotDepth(bboxToFitInMat));


		// print depth information onto the image
		cv::putText(finalPositions,
			std::to_string(m[0]) + " meters",
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