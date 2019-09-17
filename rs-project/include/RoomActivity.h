#pragma once

class RoomActivity : Activity
{
public:

	RoomActivity();

	RoomActivity(
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
	);

	~RoomActivity() { delete prevCentroidPoint_; };


private:

	// Private members
	std::map<std::string, cv::Ptr<cv::Tracker>> trackers_;
	DeviceWrapper * deviceWrapper_;
	std::map<std::string, Eigen::Matrix4d> calibrationMatrices_;
	std::map<std::string, std::map<std::string, cv::Rect2d>> initialDetectedObjectsPerDevice_;
	std::map<std::string, std::map<std::string, cv::Rect2d>> finalDetectedObjectsPerDevice_;
	std::map<std::string, pcl::PointXYZ> detectedObjectsInFinalPointCloud_;
	pcl::PointXYZRGB * prevCentroidPoint_ = nullptr;
	//pcl::PointXYZRGB prevCentroidPoint;

	// Dimensions of the room relative to the marker in meters
	float leftWidth_ = 0, rightWidth_ = 0, topHeight_ = 0, bottomHeight_ = 0, cameraDistance_ = 0;
	std::mutex finalOutputCloudLock_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalOutputCloud_;

	void start() override;
	//void initialCapture(cv::Mat initialColorMat, cv::Mat initialDepthMat, cv::Mat depthColorMapper, bool exportCloud, std::string deviceName, std::string pathToPLY, std::string pathToPNG);
	//void finalCapture(cv::Mat finalSnapshotColor, cv::Mat finalSnapshotDepth, cv::Mat depthColorMapper, bool exportCloud, std::string deviceName, std::string pathToPLY, std::string pathToPNG);
	void exportCloud(cv::Mat depthData, cv::Mat depthColorMapper, cv::Rect2d bbox, const std::string& deviceName, const std::string& path);

	void addToFinalOutputCloud
	(
		const cv::Mat& depthMat,
		const cv::Rect2d & bbox,
		const std::string& deviceName,
		const std::string& objectName,
		std::tuple<int, int, int> color
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

	void RoomActivity::finalPositionsCapture
	(
		const std::map<std::string, cv::Mat>& finalColorMats,
		const std::map<std::string, cv::Mat>& finalDepthMats,
		const std::map<std::string, cv::Mat>& depthColorMappers
	);

	// Process a captured rgbFrame and depth frame (detect and track)
	// Output: returns the output RGB image with everything drawn on it
	// return: whether a person was found in the image or not
	/*void processFrame(cv::Mat rgbMat, cv::Mat depthMat, cv::Mat depthColor, std::string& deviceName, int& frameCount, cv::Mat& output, bool& isPersonInFrame);*/
};