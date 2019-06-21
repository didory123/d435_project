#pragma once

const std::string DEFAULT_CFG_PATH = "D:/School/Research/rs-project/x64/Debug/yolov3.cfg";
const std::string DEFAULT_WEIGHTS_PATH = "D:/School/Research/rs-project/x64/Debug/yolov3.weights";
const std::string DEFAULT_NAMES_PATH = "C:/Users/Jim/Documents/darknet/build/darknet/x64/data/coco.names";
const float inScaleFactor = 0.007843f;
const float meanVal = 127.5;

class ObjectDetector
{
public:
	ObjectDetector(const std::string& cfgPath = DEFAULT_CFG_PATH, const std::string& weightsPath = DEFAULT_WEIGHTS_PATH, const std::string& namesPath = DEFAULT_NAMES_PATH, double confidence = 0.7);
	~ObjectDetector();
	cv::Mat detect(cv::Mat image);
	bool detectPerson(cv::Mat image, cv::Rect2d& bbox);
	cv::Mat detectAllObjects(cv::Mat image, std::map<std::string, cv::Rect2d>& objects); // map keeps track of all detected objects; using map over unordered_map because we want the objects to be in order

private:

	void ObjectDetector::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

	std::vector<std::string> classes_;
	cv::dnn::Net net_;
	std::vector<std::string> outNames_;
	double confidence_ = 0.0;
};

