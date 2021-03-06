#pragma once
#include "stdafx.h"
#include "Utils.h"


// Helper functions

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
std::tuple<int, int, int> Utils::RGB_Texture(const rs2::video_frame& texture, const rs2::texture_coordinate& Texture_XY)
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
pcl_color_ptr Utils::pointsToColorPCL(const rs2::points& points, const rs2::video_frame& color)
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
		RGB_Color = Utils::RGB_Texture(color, Texture_Coord[i]);

		// Mapping Color (BGR due to Camera Model)
		cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
		cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
		cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>

	}

	return cloud; // PCL RGB Point Cloud generated
}

pcl_ptr Utils::pointsToPCL(const rs2::points& points)
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

Eigen::Matrix4d Utils::getTransformMatrix(cv::Vec3d rotationVector, cv::Vec3d translationVector)
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

pcl_color_ptr Utils::affineTransformMatrix(const pcl_color_ptr& source_cloud, Eigen::Matrix4d transformMat)
{

	// The same rotation matrix as before; theta radians around Z axis
	//transform_2.rotate(Eigen::AngleAxisf(theta, axis));

	// Executing the transformation
	pcl_color_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transformMat);

	return transformed_cloud;
}

pcl_color_ptr Utils::affineTransformRotate(const pcl_color_ptr& source_cloud, Eigen::Vector3f::BasisReturnType axis, float theta)
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

pcl_color_ptr Utils::affineTransformTranslate(const pcl_color_ptr& source_cloud, float x, float y, float z)
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

cv::Ptr<cv::Tracker> Utils::createTrackerByName(cv::String name)
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

cv::Mat Utils::frameToMat(const rs2::frame& f)
{
	using namespace cv;
	using namespace rs2;

	auto vf = f.as<video_frame>();
	const int w = vf.get_width();
	const int h = vf.get_height();

	auto format = f.get_profile().format();

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

cv::Mat Utils::depthFrameToScale(const rs2::pipeline& pipe, const rs2::depth_frame& f)
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

pcl_ptr Utils::depthMatToPCL(const cv::Mat& depthMat, const rs2::video_stream_profile& profile, const cv::Rect2d& roi)
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

	// The 1000 is the scaling factor; typically measured in meters, but since our depth units are in mm we divide by 1000
	float factor = 1000;

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


					//p.z = p.z / 1000;
					//p.x = p.x / 1000;
					//p.y = p.y / 1000;

					pointcloud->points.push_back(p);
				}
			}
		}

	}

	return pointcloud;

}

pcl_color_ptr Utils::depthMatToColorPCL(const cv::Mat& depthMat, const cv::Mat& colorMat, const rs2::video_stream_profile& profile, const cv::Rect2d& roi)
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

	//pointcloud->width = depth_image.cols; //Dimensions must be initialized to use 2-D indexing 
	//pointcloud->height = depth_image.rows;
	//pointcloud->resize(pointcloud->width*pointcloud->height);

	// y and x are incremented by 1 (i.e. go through every point)
	// for better performance (at the sake of cloud detail) you could increment by a bigger number like 2 or 3
	for (int y = 0; y < depth_image.rows; y += 1)
	{
		if (y >= roi.y && y <= roi.y + roi.height)
		{
			for (int x = 0; x < depth_image.cols; x += 1)
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

// Write text that fits within a bounding box
// Copied from answer at: https://stackoverflow.com/questions/32755439/how-to-put-text-into-a-bounding-box-in-opencv
void Utils::putTextWithinBoundingBox(cv::Mat& img, const std::string& text, const cv::Rect& roi, const cv::Scalar& color, int fontFace, double fontScale, int thickness, int lineType)
{
	CV_Assert(!img.empty() && (img.type() == CV_8UC3 || img.type() == CV_8UC1));
	CV_Assert(roi.area() > 0);
	CV_Assert(!text.empty());

	int baseline = 0;

	// Calculates the width and height of a text string
	cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);

	// Y-coordinate of the baseline relative to the bottom-most text point
	baseline += thickness;

	// Render the text over here (fits to the text size)
	cv::Mat textImg(textSize.height + baseline, textSize.width, img.type());

	if (color == cv::Scalar::all(0)) textImg = cv::Scalar::all(255);
	else textImg = cv::Scalar::all(0);

	// Estimating the resolution of bounding image
	cv::Point textOrg((textImg.cols - textSize.width) / 2, (textImg.rows + textSize.height - baseline) / 2);

	// TR and BL points of the bounding box
	cv::Point tr(textOrg.x, textOrg.y + baseline);
	cv::Point bl(textOrg.x + textSize.width, textOrg.y - textSize.height);

	cv::putText(textImg, text, textOrg, fontFace, fontScale, color, thickness);

	// Resizing according to the ROI
	cv::resize(textImg, textImg, roi.size());

	cv::Mat textImgMask = textImg;
	if (textImgMask.type() == CV_8UC3)
		cv::cvtColor(textImgMask, textImgMask, cv::COLOR_BGR2GRAY);

	// Creating the mask
	cv::equalizeHist(textImgMask, textImgMask);

	if (color == cv::Scalar::all(0)) cv::threshold(textImgMask, textImgMask, 1, 255, cv::THRESH_BINARY_INV);
	else cv::threshold(textImgMask, textImgMask, 254, 255, cv::THRESH_BINARY);

	// Put into the original image
	cv::Mat destRoi = img(roi);
	textImg.copyTo(destRoi, textImgMask);
}


