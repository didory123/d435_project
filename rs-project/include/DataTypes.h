#include <librealsense2/rs.hpp>
#include <tuple>

#pragma once


// Sizes
const cv::Size LARGEST_DIMS = cv::Size(1280, 720);
const cv::Size LARGE_DIMS = cv::Size(848, 480);
const cv::Size MEDIUM_DIMS = cv::Size(640, 480);
const cv::Size SMALL_DIMS = cv::Size(640, 360);
const cv::Size SMALLEST_DIMS = cv::Size(424, 240);

enum class E_FRAME_RATE : unsigned int
{
	FPS_15 = 15,
	FPS_30 = 30,
	FPS_60 = 60
};

struct stream_profile_detail
{
	int width;
	int height;
	rs2_stream streamType;
	rs2_format format;
	E_FRAME_RATE frameRate;
};

struct object_info
{
	pcl::PointXYZ centroidPoint;
	float width = 0;
	float height = 0;
};

struct color_map
{
	std::string colorName;
	std::tuple<int, int, int> rgbValue;
};

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_color_ptr;

// User configuration data structs

struct yolo_files_paths
{
	std::string cfgFilePath = "";
	std::string weightsFilePath = "";
	std::string namesFilePath = "";
};

struct room_activity_settings
{
	int idleStateRefreshFrameCounter;
	int activityStateRedetectionFrameCounter;
	int exitActivityPersonMissingFrameCounter;
	int recordPersonCoordinateFrameCounter;
	double objectDetectionThreshold;
};

struct room_activity_dimensions
{
	float leftDistance;
	float rightDistance;
	float ceilingDistance;
	float floorDistance;
	float cameraDistance;
	cv::Size frameSize;
};

struct charuco_dimensions
{
	int cellCountX;
	int cellCountY;
	float squareLength; // measured in meters
	float markerLength; // measured in meters
};

