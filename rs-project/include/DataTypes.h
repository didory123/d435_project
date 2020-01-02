#include <librealsense2/rs.hpp>
#include <tuple>

#pragma once
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