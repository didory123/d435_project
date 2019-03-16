// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once


#include <opencv2/opencv.hpp> 
#include <opencv2/aruco.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rs_advanced_mode.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

#include <stdio.h>
#include <tchar.h>
#include <string>
#include <iostream>

#include "Marker.h"
#include "DeviceWrapper.h"
#include "targetver.h"


// TODO: reference additional headers your program requires here