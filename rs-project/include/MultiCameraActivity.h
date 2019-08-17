#pragma once

class MultiCameraActivity : Activity
{
public:

	MultiCameraActivity();

	MultiCameraActivity(
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
	);

	~MultiCameraActivity() {};


private:

	// Private members
	std::map<std::string, cv::Ptr<cv::Tracker>> trackers_;
	DeviceWrapper * deviceWrapper_;
	std::map<std::string, Eigen::Matrix4d> calibrationMatrices_;
	std::map<std::string, std::map<std::string, cv::Rect2d>> initialDetectedObjectsPerDevice_;
	std::map<std::string, std::map<std::string, cv::Rect2d>> finalDetectedObjectsPerDevice_;


	void start() override;
	void initialCapture(cv::Mat initialColorMat, cv::Mat initialDepthMat, cv::Mat depthColorMapper, bool exportCloud, std::string deviceName, std::string pathToPLY, std::string pathToPNG);
	void finalCapture(cv::Mat finalSnapshotColor, cv::Mat finalSnapshotDepth, cv::Mat depthColorMapper, bool exportCloud, std::string deviceName, std::string pathToPLY, std::string pathToPNG);
	void exportCloud(cv::Mat depthData, cv::Mat depthColorMapper, cv::Rect2d bbox, const std::string& deviceName, const std::string& path);

	void exportMergedCloud
	(
		std::map<std::string, cv::Mat> depthMats,
		std::map<std::string, cv::Mat> depthColorMappers,
		std::map<std::string, cv::Rect2d> bboxes,
		const std::string& path
	);

	void exportMergedCloudFromFrames
	(
		std::map<std::string, rs2::frame> depthFrames,
		std::map<std::string, cv::Rect2d> bboxes,
		const std::string& path
	);

	void beginActivity(
		std::map<std::string, cv::Mat> initialColorMats,
		std::map<std::string, cv::Mat> initialDepthMats,
		std::map<std::string, cv::Mat> depthColorMappers,
		std::map<std::string, cv::Rect2d> personBboxes
	);

	// Process a captured rgbFrame and depth frame (detect and track)
	// Output: returns the output RGB image with everything drawn on it
	// return: whether a person was found in the image or not
	/*void processFrame(cv::Mat rgbMat, cv::Mat depthMat, cv::Mat depthColor, std::string& deviceName, int& frameCount, cv::Mat& output, bool& isPersonInFrame);*/
};