// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once


#include <opencv2/opencv.hpp> 
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/tracking.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rs_advanced_mode.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <stdio.h>
#include <tchar.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cmath>
#include <map>
#include <vector>
#include <unordered_map>

#include "Marker.h"
#include "DeviceWrapper.h"
#include "targetver.h"
#include "DataTypes.h"
#include "ObjectDetector.h"

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_color_ptr;

// TODO: reference additional headers your program requires here
