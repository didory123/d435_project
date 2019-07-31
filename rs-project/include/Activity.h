#pragma once
class Activity
{
public:

	Activity(rs2::pipeline& pipe, 
		const std::string& trackerType, 
		const cv::Mat& initialColorMat, 
		const cv::Mat& initialDepthMat, 
		const cv::Mat& depthColorMapper,
		const std::string& windowName,
		const cv::Rect2d& personBbox,
		ObjectDetector& detector,
		rs2::align& aligner);

	~Activity();

	void beginActivity(const cv::Mat& initialColorMat, const cv::Mat& initialDepthMat, const cv::Mat& depthColorMapper, const cv::Rect2d& personBbox);



private:

	std::string activityName_ = "";
	std::string windowName_ = "";
	rs2::pipeline * pipe_;
	std::map<std::string, cv::Rect2d> initialDetectedObjects_;
	std::map<std::string, cv::Rect2d> finalDetectedObjects_;
	ObjectDetector * detector_;
	cv::VideoWriter output_;
	cv::Ptr<cv::Tracker> tracker_;// = TestDriver::createTrackerByName(trackerType);
	std::string trackerType_ = "";
	std::mutex lock_;
	rs2::align * align_to_color_;
	
	std::vector<std::thread*> workerThreads_; // List of worker threads; they're all for exporting pointcloud tracking the person's movements

	void initialSnapshot(cv::Mat initialColorMat, cv::Mat initialDepthMat, cv::Mat depthColorMapper, bool exportCloud = true);
	void start();
	void finalSnapshot(cv::Mat finalSnapshotColor, cv::Mat finalSnapshotDepth, cv::Mat depthColorMapper, bool exportCloud = true);
	void exportPointCloud(cv::Mat depthData, cv::Mat depthColorMapper, cv::Rect2d bbox);
};