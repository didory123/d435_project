#pragma once

class ObjectDetector
{
public:
	ObjectDetector(yolo_files_paths yoloFilesPaths, double confidence = 0.5);
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

	void setConfidence(double confidence);
private:
	void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

	std::vector<std::string> classes_;
	cv::dnn::Net net_;
	std::vector<std::string> outNames_;
	double confidence_ = 0.0;

	const float inScaleFactor = 1.0 / 255.0;
	const cv::Scalar meanVal = cv::Scalar(0, 0, 0);

	// Sizes must be multiples of 32
	// These size are used during the detection process, converting blobs
	const cv::Size SMALL_SIZE = cv::Size(320, 320);
	const cv::Size REGULAR_SIZE = cv::Size(416, 416);
	const cv::Size LARGE_SIZE = cv::Size(608, 608);
	const cv::Size LARGER_SIZE = cv::Size(768, 768);
};

