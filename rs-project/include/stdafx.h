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
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>

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
#include <ctime>
#include <thread>

#include "Marker.h"
#include "DeviceWrapper.h"
#include "targetver.h"
#include "DataTypes.h"
#include "ObjectDetector.h"
#include "Activity.h"

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_color_ptr;

// Sizes
const cv::Size LARGEST_DIMS = cv::Size(1280, 720);
const cv::Size LARGE_DIMS = cv::Size(848, 480);
const cv::Size MEDIUM_DIMS = cv::Size(640, 480);
const cv::Size SMALL_DIMS = cv::Size(640, 360);
const cv::Size SMALLEST_DIMS = cv::Size(424, 240);

// Helper functions
namespace helper {
	//======================================================
	// RGB Texture
	// - Function is utilized to extract the RGB data from
	// a single point return R, G, and B values. 
	// Normals are stored as RGB components and
	// correspond to the specific depth (XYZ) coordinate.
	// By taking these normals and converting them to
	// texture coordinates, the RGB components can be
	// "mapped" to each individual point (XYZ).
	//======================================================
	inline static std::tuple<int, int, int> RGB_Texture(const rs2::video_frame& texture, const rs2::texture_coordinate& Texture_XY)
	{
		// Get Width and Height coordinates of texture
		int width = texture.get_width();  // Frame width in pixels
		int height = texture.get_height(); // Frame height in pixels

										   // Normals to Texture Coordinates conversion
		int x_value = std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
		int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

		int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
		int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
		int Text_Index = (bytes + strides);

		const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

		// RGB components to save in tuple
		int NT1 = New_Texture[Text_Index];
		int NT2 = New_Texture[Text_Index + 1];
		int NT3 = New_Texture[Text_Index + 2];

		return std::tuple<int, int, int>(NT1, NT2, NT3);
	}

	//===================================================
	//  PCL_Conversion
	// - Function is utilized to fill a point cloud
	//  object with depth and RGB data from a single
	//  frame captured using the Realsense.
	//=================================================== 
	inline static pcl_color_ptr pointsToColorPCL(const rs2::points& points, const rs2::video_frame& color)
	{

		// Object Declaration (Point Cloud)
		pcl_color_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		// Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
		std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

		//================================
		// PCL Cloud Object Configuration
		//================================
		// Convert data captured from Realsense camera to Point Cloud
		auto sp = points.get_profile().as<rs2::video_stream_profile>();

		cloud->width = static_cast<uint32_t>(sp.width());
		cloud->height = static_cast<uint32_t>(sp.height());
		cloud->is_dense = false;
		cloud->points.resize(points.size());

		auto Texture_Coord = points.get_texture_coordinates();
		auto Vertex = points.get_vertices();

		// Iterating through all points and setting XYZ coordinates
		// and RGB values
		for (int i = 0; i < points.size(); ++i)
		{
			//===================================
			// Mapping Depth Coordinates
			// - Depth data stored as XYZ values
			//===================================
			cloud->points[i].x = Vertex[i].x;
			cloud->points[i].y = Vertex[i].y;
			cloud->points[i].z = Vertex[i].z;

			// Obtain color texture for specific point
			RGB_Color = helper::RGB_Texture(color, Texture_Coord[i]);

			// Mapping Color (BGR due to Camera Model)
			cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
			cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
			cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>

		}

		return cloud; // PCL RGB Point Cloud generated
	}

	inline static pcl_ptr pointsToPCL(const rs2::points& points)
	{
		pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		auto sp = points.get_profile().as<rs2::video_stream_profile>();
		cloud->width = sp.width();
		cloud->height = sp.height();
		cloud->is_dense = false;
		cloud->points.resize(points.size());
		auto ptr = points.get_vertices();
		for (auto& p : cloud->points)
		{
			p.x = ptr->x;
			p.y = ptr->y;
			p.z = ptr->z;
			ptr++;
		}

		return cloud;
	}

	inline static Eigen::Matrix4d getTransformMatrix(cv::Vec3d rotationVector, cv::Vec3d translationVector)
	{
		/* Reminder: how transformation matrices work :

		|-------> This column is the translation
		| 1 0 0 x |  \
		| 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
		| 0 0 1 z |  /
		| 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

		METHOD #1: Using a Matrix4f
		This is the "manual" method, perfect to understand but error prone !
		*/
		cv::Mat rmat;
		// OpenCV estimate pose functions give us the rotation vector, not matrix
		// https://docs.opencv.org/3.4.3/d9/d0c/group__calib3d.html#ga61585db663d9da06b68e70cfbf6a1eac
		// Convert the 3x1 vector to 3x3 matrix in order to perform our transformations on the pointcloud
		cv::Rodrigues(rotationVector, rmat, cv::noArray());

		Eigen::Matrix4d transformMatrix = Eigen::Matrix4d::Identity();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				transformMatrix(i, j) = rmat.at<double>(i, j);
			}
		}
		transformMatrix(0, 3) = translationVector[0];
		transformMatrix(1, 3) = translationVector[1];
		transformMatrix(2, 3) = translationVector[2];

		return transformMatrix;
	}

	inline static pcl_color_ptr affineTransformMatrix(const pcl_color_ptr& source_cloud, Eigen::Matrix4d transformMat)
	{

		// The same rotation matrix as before; theta radians around Z axis
		//transform_2.rotate(Eigen::AngleAxisf(theta, axis));

		// Executing the transformation
		pcl_color_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		// You can either apply transform_1 or transform_2; they are the same
		pcl::transformPointCloud(*source_cloud, *transformed_cloud, transformMat);

		return transformed_cloud;
	}

	inline static pcl_color_ptr affineTransformRotate(const pcl_color_ptr& source_cloud, Eigen::Vector3f::BasisReturnType axis, float theta = M_PI)
	{

		/*  METHOD #2: Using a Affine3f
		This method is easier and less error prone
		*/
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

		// The same rotation matrix as before; theta radians around Z axis
		transform_2.rotate(Eigen::AngleAxisf(theta, axis));

		// Executing the transformation
		pcl_color_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		// You can either apply transform_1 or transform_2; they are the same
		pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

		return transformed_cloud;
	}

	inline static pcl_color_ptr affineTransformTranslate(const pcl_color_ptr& source_cloud, float x = 0, float y = 0, float z = 0)
	{

		/*  METHOD #2: Using a Affine3f
		This method is easier and less error prone
		*/
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

		// Define a translation of 2.5 meters on the x axis.
		transform_2.translation() << x, y, z;

		// Executing the transformation
		pcl_color_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		// You can either apply transform_1 or transform_2; they are the same
		pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

		return transformed_cloud;
	}

	inline static cv::Ptr<cv::Tracker> createTrackerByName(cv::String name)
	{
		cv::Ptr<cv::Tracker> tracker;

		if (name == "KCF")
			tracker = cv::TrackerKCF::create();
		else if (name == "TLD")
			tracker = cv::TrackerTLD::create();
		else if (name == "BOOSTING")
			tracker = cv::TrackerBoosting::create();
		else if (name == "MEDIAN_FLOW")
			tracker = cv::TrackerMedianFlow::create();
		else if (name == "MIL")
			tracker = cv::TrackerMIL::create();
		else if (name == "GOTURN")
			tracker = cv::TrackerGOTURN::create();
		else if (name == "MOSSE")
			tracker = cv::TrackerMOSSE::create();
		else if (name == "CSRT")
			tracker = cv::TrackerCSRT::create();
		else
			CV_Error(cv::Error::StsBadArg, "Invalid tracking algorithm name\n");

		return tracker;
	}

	inline static cv::Mat frameToMat(const rs2::frame& f)
	{
		using namespace cv;
		using namespace rs2;

		auto vf = f.as<video_frame>();
		const int w = vf.get_width();
		const int h = vf.get_height();

		if (f.get_profile().format() == RS2_FORMAT_BGR8)
		{
			return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
		}
		else if (f.get_profile().format() == RS2_FORMAT_RGB8)
		{
			auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
			cvtColor(r, r, COLOR_RGB2BGR);
			return r;
		}
		else if (f.get_profile().format() == RS2_FORMAT_Z16)
		{
			return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
		}
		else if (f.get_profile().format() == RS2_FORMAT_Y8)
		{
			return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
		}

		throw std::runtime_error("Frame format is not supported yet!");
	}

	inline static cv::Mat depthFrameToScale(const rs2::pipeline& pipe, const rs2::depth_frame& f)
	{
		using namespace cv;
		using namespace rs2;

		Mat dm = frameToMat(f);
		dm.convertTo(dm, CV_64F);
		auto depth_scale = pipe.get_active_profile()
			.get_device()
			.first<depth_sensor>()
			.get_depth_scale();
		dm = dm * depth_scale;
		return dm;
	}

	/*inline static void readColorFrame(const rs2::pipeline& pipe, rs2::frame& f, bool finished)
	{
		while (!finished)
		{

			rs2::frameset data = pipe.wait_for_frames();
			f = data.get_color_frame();
			cv::imshow("Object Tracking", frameToMat(f));
			/*const int w = rgb.as<rs2::video_frame>().get_width();
			const int h = rgb.as<rs2::video_frame>().get_height();
			cv::Mat currFrame = frameToMat(rgb);
			//cv::Mat currFrame(cv::Size(w, h), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
			initialColorFrame = currFrame;
		}
	}*/

	inline static pcl_ptr depthMatToPCL(const cv::Mat& depthMat, const rs2::video_stream_profile& profile, const cv::Rect2d& roi)
	{
		//If the stream is indeed a video stream, we can now simply call get_intrinsics()
		rs2_intrinsics intrinsics = profile.get_intrinsics();

		auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
		auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);

		pcl_ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

		float fx = focal_length.first;
		float fy = focal_length.second;
		float cx = principal_point.first;
		float cy = principal_point.second;

		float factor = 1;

		cv::Mat depth_image;
		depthMat.convertTo(depth_image, CV_32F); // convert the image data to float type 

		if (!depth_image.data) {
			std::cerr << "No depth data!!!" << std::endl;
			exit(EXIT_FAILURE);
		}

		pointcloud->width = depth_image.cols; //Dimensions must be initialized to use 2-D indexing 
		pointcloud->height = depth_image.rows;
		pointcloud->resize(pointcloud->width*pointcloud->height);

		for (int y = 0; y < depth_image.rows; y += 2)
		{
			if (y >= roi.y && y <= roi.y + roi.height)
			{
				for (int x = 0; x < depth_image.cols; x += 2)
				{
					if (x >= roi.x && x <= roi.x + roi.width)
					{
						float Z = depth_image.at<float>(y, x) / factor;

						pcl::PointXYZ p;

						p.z = Z;
						p.x = (x - cx) * Z / fx;
						p.y = (y - cy) * Z / fy;

						p.z = p.z / 1000;
						p.x = p.x / 1000;
						p.y = p.y / 1000;

						pointcloud->points.push_back(p);
					}
				}
			}

		}

		return pointcloud;

	}

	inline pcl_color_ptr depthMatToColorPCL(const cv::Mat& depthMat, const cv::Mat& colorMat, const rs2::video_stream_profile& profile, const cv::Rect2d& roi)
	{
		//If the stream is indeed a video stream, we can now simply call get_intrinsics()
		rs2_intrinsics intrinsics = profile.get_intrinsics();

		auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
		auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);

		pcl_color_ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		float fx = focal_length.first;
		float fy = focal_length.second;
		float cx = principal_point.first;
		float cy = principal_point.second;

		float factor = 1;

		cv::Mat depth_image;
		depthMat.convertTo(depth_image, CV_32F); // convert the image data to float type 

		if (!depth_image.data) {
			std::cerr << "No depth data!!!" << std::endl;
			exit(EXIT_FAILURE);
		}

		pointcloud->width = depth_image.cols; //Dimensions must be initialized to use 2-D indexing 
		pointcloud->height = depth_image.rows;
		pointcloud->resize(pointcloud->width*pointcloud->height);

		for (int y = 0; y < depth_image.rows; y += 2)
		{
			if (y >= roi.y && y <= roi.y + roi.height)
			{
				for (int x = 0; x < depth_image.cols; x += 2)
				{
					if (x >= roi.x && x <= roi.x + roi.width)
					{
						float Z = depth_image.at<float>(y, x) / factor;

						pcl::PointXYZRGB p;

						/*point.x = depth_image.at<double>(0, y*depthMat.cols + x);
						point.y = depth_image.at<double>(1, y*depthMat.cols + x);
						point.z = depth_image.at<double>(2, y*depthMat.cols + x);*/

						cv::Vec3b color = colorMat.at<cv::Vec3b>(cv::Point(x, y));
						uint8_t r = (color[2]);
						uint8_t g = (color[1]);
						uint8_t b = (color[0]);

						int32_t rgb = (r << 16) | (g << 8) | b;
						p.rgb = *reinterpret_cast<float*>(&rgb);

						p.z = Z;
						p.x = (x - cx) * Z / fx;
						p.y = (y - cy) * Z / fy;

						p.z = p.z / 1000;
						p.x = p.x / 1000;
						p.y = p.y / 1000;

						pointcloud->points.push_back(p);
					}
				}
			}

		}

		return pointcloud;

	}
}