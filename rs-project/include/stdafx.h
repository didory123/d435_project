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
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>

#include <stdio.h>
#include <tchar.h>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cmath>
#include <map>
#include <vector>
#include <unordered_map>
#include <ctime>
#include <thread>

#include "Marker.h"
#include "DeviceWrapper.h"
#include "targetver.h"
#include "DataTypes.h"
#include "ObjectDetector.h"


// Sizes
const cv::Size LARGEST_DIMS = cv::Size(1280, 720);
const cv::Size LARGE_DIMS = cv::Size(848, 480);
const cv::Size MEDIUM_DIMS = cv::Size(640, 480);
const cv::Size SMALL_DIMS = cv::Size(640, 360);
const cv::Size SMALLEST_DIMS = cv::Size(424, 240);
