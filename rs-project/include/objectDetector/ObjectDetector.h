#pragma once

const std::string DEFAULT_CFG_PATH = boost::filesystem::current_path().string() + "/yolov3.cfg";
const std::string DEFAULT_WEIGHTS_PATH = boost::filesystem::current_path().string() + "/yolov3.weights";
//const std::string DEFAULT_NAMES_PATH = "C:/Users/Jim/Documents/darknet/build/darknet/x64/data/coco.names";
const std::string DEFAULT_NAMES_PATH = boost::filesystem::current_path().string() + "/coco.names";
const float inScaleFactor = 1.0/255.0;
const cv::Scalar meanVal = cv::Scalar(0, 0, 0);

// Sizes must be multiples of 32
const cv::Size SMALL_SIZE = cv::Size(320, 320);
const cv::Size REGULAR_SIZE = cv::Size(416, 416);
const cv::Size LARGE_SIZE = cv::Size(608, 608);
const cv::Size LARGER_SIZE = cv::Size(768, 768);


// Singleton class for object detection
// Initializing the model datasets and stuff take some time, therefore there will only be one instance of ObjectDetector ever
class ObjectDetector
{
public:
	ObjectDetector(double confidence = 0.5, const std::string& cfgPath = DEFAULT_CFG_PATH, const std::string& weightsPath = DEFAULT_WEIGHTS_PATH, const std::string& namesPath = DEFAULT_NAMES_PATH);
	~ObjectDetector();
	cv::Mat detect(cv::Mat image);
	bool detectPerson(cv::Mat image, cv::Rect2d& bbox);
	cv::Mat detectAllObjects(cv::Mat image, std::map<std::string, cv::Rect2d>& objects); // map keeps track of all detected objects; using map over unordered_map because we want the objects to be in order

	// Check if two given rectangle boxes are "roughly" the same
	// The function checks for the intersecting area between the two, and if it is greater than the given threshold (good enough) they are considered equal
	static bool bboxesEqual(const cv::Rect& bbox1, const cv::Rect& bbox2)
	{
		// Reference: from Pattern Recognition and Image Analysis: 6th Iberian Conference
		// "Two BB's are said to match if the area of intersection of the two rectangles is larger than half of the area of the union (Pascal VOC criterion)"
		// Using this method that I found on the internet
		cv::Rect2d unionRect = bbox1 | bbox2;
		cv::Rect2d intersectRect = bbox1 & bbox2;
		return intersectRect.area() > (unionRect.area() * 0.5);
	}


private:
	void ObjectDetector::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

	std::vector<std::string> classes_;
	cv::dnn::Net net_;
	std::vector<std::string> outNames_;
	double confidence_ = 0.0;
};

