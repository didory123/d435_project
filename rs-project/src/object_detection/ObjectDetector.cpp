#include "stdafx.h"
#include "ObjectDetector.h"

using namespace cv;
using namespace cv::dnn;

ObjectDetector::ObjectDetector(const std::string& cfgPath, const std::string& weightsPath, const std::string& namesPath, double confidence)
{

	confidence_ = confidence;
	this->net_ = readNetFromDarknet(cfgPath, weightsPath);
	std::string file(DEFAULT_NAMES_PATH);
	std::ifstream ifs(file.c_str());
	if (!ifs.is_open())
		CV_Error(Error::StsError, "File " + file + " not found");
	std::string line;
	while (std::getline(ifs, line))
	{
		this->classes_.push_back(line);
	}

	outNames_ = this->net_.getUnconnectedOutLayersNames();
	this->net_.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);

}


ObjectDetector::~ObjectDetector()
{
}

cv::Mat ObjectDetector::detect(cv::Mat image)
{

	Mat inputBlob;
	blobFromImage(image, inputBlob, inScaleFactor, Size(320, 320), meanVal, false); //Convert Mat to batch of images
	net_.setInput(inputBlob); //set the network input

	std::vector<Mat> outs;
	net_.forward(outs, outNames_);

	static std::vector<int> outLayers = net_.getUnconnectedOutLayers();
	static std::string outLayerType = net_.getLayer(outLayers[0])->type;

	std::vector<int> classIds;
	std::vector<float> confidences;
	std::vector<Rect> boxes;
	if (outLayerType == "DetectionOutput")
	{
		// Network produces output blob with a shape 1x1xNx7 where N is a number of
		// detections and an every detection is a vector of values
		// [batchId, classId, confidence, left, top, right, bottom]
		CV_Assert(outs.size() > 0);
		for (size_t k = 0; k < outs.size(); k++)
		{
			float* data = (float*)outs[k].data;
			for (size_t i = 0; i < outs[k].total(); i += 7)
			{
				float confidence = data[i + 2];
				if (confidence > confidence_)
				{
					int left = (int)data[i + 3];
					int top = (int)data[i + 4];
					int right = (int)data[i + 5];
					int bottom = (int)data[i + 6];
					int width = right - left + 1;
					int height = bottom - top + 1;
					if (width * height <= 1)
					{
						left = (int)(data[i + 3] * image.cols);
						top = (int)(data[i + 4] * image.rows);
						right = (int)(data[i + 5] * image.cols);
						bottom = (int)(data[i + 6] * image.rows);
						width = right - left + 1;
						height = bottom - top + 1;
					}
					classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
					boxes.push_back(Rect(left, top, width, height));
					confidences.push_back(confidence);
				}
			}
		}
	}
	else if (outLayerType == "Region")
	{
		for (size_t i = 0; i < outs.size(); ++i)
		{
			// Network produces output blob with a shape NxC where N is a number of
			// detected objects and C is a number of classes + 4 where the first 4
			// numbers are [center_x, center_y, width, height]
			float* data = (float*)outs[i].data;
			for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
			{
				Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
				Point classIdPoint;
				double confidence;
				minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
				if (confidence > confidence_)
				{
					int centerX = (int)(data[0] * image.cols);
					int centerY = (int)(data[1] * image.rows);
					int width = (int)(data[2] * image.cols);
					int height = (int)(data[3] * image.rows);
					int left = centerX - width / 2;
					int top = centerY - height / 2;

					classIds.push_back(classIdPoint.x);
					confidences.push_back((float)confidence);
					boxes.push_back(Rect(left, top, width, height));
				}
			}
		}
	}
	else
	{
		CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);
	}

	std::vector<int> indices;
	NMSBoxes(boxes, confidences, 0.5, 4, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		Rect box = boxes[idx];
		drawPred(classIds[idx], confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, image);
	}
	return image;
}

bool ObjectDetector::detectPerson(cv::Mat image, cv::Rect2d& bbox)
{

	Mat inputBlob;
	blobFromImage(image, inputBlob, inScaleFactor, Size(320, 320), meanVal, false); //Convert Mat to batch of images
	net_.setInput(inputBlob); //set the network input

	std::vector<Mat> outs;
	net_.forward(outs, outNames_);

	static std::vector<int> outLayers = net_.getUnconnectedOutLayers();
	static std::string outLayerType = net_.getLayer(outLayers[0])->type;

	std::vector<int> classIds;
	std::vector<float> confidences;
	std::vector<Rect> boxes;
	if (outLayerType == "DetectionOutput")
	{
		// Network produces output blob with a shape 1x1xNx7 where N is a number of
		// detections and an every detection is a vector of values
		// [batchId, classId, confidence, left, top, right, bottom]
		CV_Assert(outs.size() > 0);
		for (size_t k = 0; k < outs.size(); k++)
		{
			float* data = (float*)outs[k].data;
			for (size_t i = 0; i < outs[k].total(); i += 7)
			{
				float confidence = data[i + 2];
				if (confidence > confidence_)
				{
					int left = (int)data[i + 3];
					int top = (int)data[i + 4];
					int right = (int)data[i + 5];
					int bottom = (int)data[i + 6];
					int width = right - left + 1;
					int height = bottom - top + 1;
					if (width * height <= 1)
					{
						left = (int)(data[i + 3] * image.cols);
						top = (int)(data[i + 4] * image.rows);
						right = (int)(data[i + 5] * image.cols);
						bottom = (int)(data[i + 6] * image.rows);
						width = right - left + 1;
						height = bottom - top + 1;
					}
					classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
					boxes.push_back(Rect(left, top, width, height));
					confidences.push_back(confidence);
				}
			}
		}
	}
	else if (outLayerType == "Region")
	{
		for (size_t i = 0; i < outs.size(); ++i)
		{
			// Network produces output blob with a shape NxC where N is a number of
			// detected objects and C is a number of classes + 4 where the first 4
			// numbers are [center_x, center_y, width, height]
			float* data = (float*)outs[i].data;
			for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
			{
				Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
				Point classIdPoint;
				double confidence;
				minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
				if (confidence > confidence_)
				{
					int centerX = (int)(data[0] * image.cols);
					int centerY = (int)(data[1] * image.rows);
					int width = (int)(data[2] * image.cols);
					int height = (int)(data[3] * image.rows);
					int left = centerX - width / 2;
					int top = centerY - height / 2;

					classIds.push_back(classIdPoint.x);
					confidences.push_back((float)confidence);
					boxes.push_back(Rect(left, top, width, height));
				}
			}
		}
	}
	else
	{
		CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);
	}
		

	std::vector<int> indices;
	NMSBoxes(boxes, confidences, 0.5, 4, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		Rect box = boxes[idx];

		int classId = classIds[idx];
		CV_Assert(classId < (int)classes_.size());
		if (!classes_.empty() && classes_[classId] == "person")
		{
			
			//drawPred(classIds[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, image);
			bbox = box;
			return true;
		}
	}
	return false;
}

cv::Mat ObjectDetector::detectAllObjects(cv::Mat image, std::map<std::string, cv::Rect2d>& objects)
{

	Mat inputBlob;
	blobFromImage(image, inputBlob, inScaleFactor, Size(320, 320), meanVal, false); //Convert Mat to batch of images
	net_.setInput(inputBlob); //set the network input

	std::vector<Mat> outs;
	net_.forward(outs, outNames_);

	static std::vector<int> outLayers = net_.getUnconnectedOutLayers();
	static std::string outLayerType = net_.getLayer(outLayers[0])->type;

	std::vector<int> classIds;
	std::vector<float> confidences;
	std::vector<Rect> boxes;
	if (outLayerType == "DetectionOutput")
	{
		// Network produces output blob with a shape 1x1xNx7 where N is a number of
		// detections and an every detection is a vector of values
		// [batchId, classId, confidence, left, top, right, bottom]
		CV_Assert(outs.size() > 0);
		for (size_t k = 0; k < outs.size(); k++)
		{
			float* data = (float*)outs[k].data;
			for (size_t i = 0; i < outs[k].total(); i += 7)
			{
				float confidence = data[i + 2];
				if (confidence > confidence_)
				{
					int left = (int)data[i + 3];
					int top = (int)data[i + 4];
					int right = (int)data[i + 5];
					int bottom = (int)data[i + 6];
					int width = right - left + 1;
					int height = bottom - top + 1;
					if (width * height <= 1)
					{
						left = (int)(data[i + 3] * image.cols);
						top = (int)(data[i + 4] * image.rows);
						right = (int)(data[i + 5] * image.cols);
						bottom = (int)(data[i + 6] * image.rows);
						width = right - left + 1;
						height = bottom - top + 1;
					}
					classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
					boxes.push_back(Rect(left, top, width, height));
					confidences.push_back(confidence);
				}
			}
		}
	}
	else if (outLayerType == "Region")
	{
		for (size_t i = 0; i < outs.size(); ++i)
		{
			// Network produces output blob with a shape NxC where N is a number of
			// detected objects and C is a number of classes + 4 where the first 4
			// numbers are [center_x, center_y, width, height]
			float* data = (float*)outs[i].data;
			for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
			{
				Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
				Point classIdPoint;
				double confidence;
				minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
				if (confidence > confidence_)
				{
					int centerX = (int)(data[0] * image.cols);
					int centerY = (int)(data[1] * image.rows);
					int width = (int)(data[2] * image.cols);
					int height = (int)(data[3] * image.rows);
					int left = centerX - width / 2;
					int top = centerY - height / 2;

					classIds.push_back(classIdPoint.x);
					confidences.push_back((float)confidence);
					boxes.push_back(Rect(left, top, width, height));
				}
			}
		}
	}
	else
	{
		CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);
	}

	std::vector<int> indices;
	NMSBoxes(boxes, confidences, 0.5, 4, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		Rect box = boxes[idx];
		drawPred(classIds[idx], confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, image);
		std::string objectName = classes_[classIds[idx]];

		objects[objectName] = box; // add object to map
	}
	return image;
}

void ObjectDetector::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
	using namespace cv;
	using namespace cv::dnn;
	using namespace rs2;

	rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

	std::string label = format("%.2f", conf);
	if (!classes_.empty())
	{
		CV_Assert(classId < (int)classes_.size());
		label = classes_[classId] + ": " + label;
	}

	int baseLine;
	Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

	top = max(top, labelSize.height);
	rectangle(frame, Point(left, top - labelSize.height),
		Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
	putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}