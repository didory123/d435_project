#pragma once

// Helper functions
namespace Utils 
{
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
	std::tuple<int, int, int> RGB_Texture(const rs2::video_frame& texture, const rs2::texture_coordinate& Texture_XY);

	//===================================================
	//  PCL_Conversion
	// - Function is utilized to fill a point cloud
	//  object with depth and RGB data from a single
	//  frame captured using the Realsense.
	//=================================================== 
	pcl_color_ptr pointsToColorPCL(const rs2::points& points, const rs2::video_frame& color);

	pcl_ptr pointsToPCL(const rs2::points& points);

	Eigen::Matrix4d getTransformMatrix(cv::Vec3d rotationVector, cv::Vec3d translationVector);

	pcl_color_ptr affineTransformMatrix(const pcl_color_ptr& source_cloud, Eigen::Matrix4d transformMat);

	pcl_color_ptr affineTransformRotate(const pcl_color_ptr& source_cloud, Eigen::Vector3f::BasisReturnType axis, float theta = M_PI);

	pcl_color_ptr affineTransformTranslate(const pcl_color_ptr& source_cloud, float x = 0, float y = 0, float z = 0);

	cv::Ptr<cv::Tracker> createTrackerByName(cv::String name);

	cv::Mat frameToMat(const rs2::frame& f);

	cv::Mat depthFrameToScale(const rs2::pipeline& pipe, const rs2::depth_frame& f);

	pcl_ptr depthMatToPCL(const cv::Mat& depthMat, const rs2::video_stream_profile& profile, const cv::Rect2d& roi);

	pcl_color_ptr depthMatToColorPCL(const cv::Mat& depthMat, const cv::Mat& colorMat, const rs2::video_stream_profile& profile, const cv::Rect2d& roi);

	// Write text that fits within a bounding box
	// Copied from answer at: https://stackoverflow.com/questions/32755439/how-to-put-text-into-a-bounding-box-in-opencv
	void putTextWithinBoundingBox(cv::Mat& img, const std::string& text, const cv::Rect& roi, const cv::Scalar& color, int fontFace, double fontScale, int thickness = 1, int lineType = 8);

};

