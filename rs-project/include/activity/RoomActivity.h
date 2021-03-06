#pragma once

class RoomActivity
{
public:

	RoomActivity();

	RoomActivity(
		const std::string& trackerType,
		std::map<std::string, cv::Mat> initialColorMats,
		std::map<std::string, cv::Mat> initialDepthMats,
		std::map<std::string, cv::Mat> depthColorMappers,
		std::map<std::string, cv::Rect2d> personBboxes,
		std::map<std::string, Eigen::Matrix4d> calibrationMatrices,
		const std::string& windowName,
		ObjectDetector& detector,
		DeviceWrapper& deviceWrapper,
		room_activity_dimensions dimensions,
		room_activity_settings settings
	);

	~RoomActivity() 
	{ 
		delete prevCentroidPoint_; 
	};

private:

	std::string activityName_ = "";
	std::string windowName_ = "";
	rs2::pipeline * pipe_;
	std::map<std::string, cv::Rect2d> initialDetectedObjects_;
	std::map<std::string, cv::Rect2d> finalDetectedObjects_;
	ObjectDetector * detector_;
	cv::VideoWriter output_;

	int activityStateRedetectionFrameCounter_ = 90;
	int exitActivityPersonMissingFrameCounter_ = 120;
	int recordPersonCoordinateFrameCounter_ = 15;

	std::string trackerType_ = "";
	std::mutex lock_;

	cv::Size frameSize_;

	std::vector<std::thread*> workerThreads_; // List of worker threads; they're all for exporting pointcloud tracking the person's movements

	void start();

	// Private members
	const color_map COLOR_RED = { "red", std::make_tuple(230, 25, 75) };
	const color_map COLOR_GREEN = { "green", std::make_tuple(60, 180, 75) };
	const color_map COLOR_BLUE = { "blue", std::make_tuple(67, 99, 216) };
	const color_map COLOR_WHITE_STREAK = { "white", std::make_tuple(235, 235, 235) };
	const color_map COLOR_PURPLE_STREAK = { "purple", std::make_tuple(145, 30, 180) };


	// Object tracker for each camera
	std::map<std::string, cv::Ptr<cv::Tracker>> trackers_;

	// Controller for interfacing with the sensors
	DeviceWrapper * deviceWrapper_;

	// Calibration matrices for each camera
	std::map<std::string, Eigen::Matrix4d> calibrationMatrices_;

	// Map of detected objects
	std::map<std::string, std::map<std::string, cv::Rect2d>> initialDetectedObjectsPerDevice_;
	std::map<std::string, std::map<std::string, cv::Rect2d>> finalDetectedObjectsPerDevice_;

	std::set<std::string> allInitialObjects_;
	std::map<std::string, std::vector<object_info>> detectedObjectsInFinalPointCloud_;
	
	std::map<std::string, color_map> objectToColorMap_; // maps each unique object to a different color
	std::queue<color_map> colorQueue_; // queue of different colors, when next object is being drawn use this queue to determine what color to use

	std::ofstream objectInfoFile_, coordinateTimestampsFile_;
	pcl::PointXYZRGB * prevCentroidPoint_ = nullptr;

	// Dimensions of the room relative to the marker in meters
	float leftWidth_ = 0, rightWidth_ = 0, topHeight_ = 0, bottomHeight_ = 0, cameraDistance_ = 0;
	std::mutex finalOutputCloudLock_;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalOutputCloud_; // final output cloud containing the segmented point clouds as well

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr roomCloud_; // output cloud that contains only the room dimensions and activity

	//void initialCapture(cv::Mat initialColorMat, cv::Mat initialDepthMat, cv::Mat depthColorMapper, bool exportCloud, std::string deviceName, std::string pathToPLY, std::string pathToPNG);
	//void finalCapture(cv::Mat finalSnapshotColor, cv::Mat finalSnapshotDepth, cv::Mat depthColorMapper, bool exportCloud, std::string deviceName, std::string pathToPLY, std::string pathToPNG);
	void exportCloud(cv::Mat depthData, cv::Mat depthColorMapper, cv::Rect2d bbox, const std::string& deviceName, const std::string& path);

	// Get centroid point of input pointcloud
	pcl::PointXYZRGB getCentroidPoint(pcl_color_ptr pointCloud);

	void addToFinalOutputCloud
	(
		const cv::Mat& depthMat,
		const cv::Rect2d & bbox,
		const std::string& deviceName,
		const std::string& objectName,
		std::tuple<int, int, int> color,
		std::tuple<int, int, int> baseColor
	);

	void exportMergedCloud
	(
		const std::map<std::string, cv::Mat>& depthMats,
		const std::map<std::string, cv::Mat>& depthColorMappers,
		const std::map<std::string, std::map<std::string, cv::Rect2d>>& bboxes,
		const std::string& path,
		std::tuple<int, int, int> rgbColor
	);

	void recordCoordinate
	(
		const std::map<std::string, cv::Mat>& depthMats,
		const std::map<std::string, cv::Rect2d>& bboxes
	);

	void beginActivity(
		std::map<std::string, cv::Mat> initialColorMats,
		std::map<std::string, cv::Mat> initialDepthMats,
		std::map<std::string, cv::Mat> depthColorMappers,
		std::map<std::string, cv::Rect2d> personBboxes
	);

	void initialPositionsCapture
	(
		const std::map<std::string, cv::Mat>& initialColorMats,
		const std::map<std::string, cv::Mat>& initialDepthMats,
		const std::map<std::string, cv::Mat>& depthColorMappers
	);

	void finalPositionsCapture
	(
		const std::map<std::string, cv::Mat>& finalColorMats,
		const std::map<std::string, cv::Mat>& finalDepthMats,
		const std::map<std::string, cv::Mat>& depthColorMappers
	);

	// Small helper function to get unique color for each object
	std::tuple<int, int, int> RoomActivity::getColorForObject
	(
		const std::string& objectName
	);

	
};