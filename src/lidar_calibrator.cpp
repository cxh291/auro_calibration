#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class LidarCalibrator{

public:

/**
 * @brief      Constructor
 *
 * @param[in]  nh    the public ros nodehandle
 */
LidarCalibrator(ros::NodeHandle nh){
	cv::namedWindow("view");
	cv::namedWindow("view2");
	cv::startWindowThread();

	ROS_INFO("Waiting for Camera Info to be publised");
	camera_info_ = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", nh);
	// camera_m_ = cv::Mat(camera_info_.K);
	// camera_d_ = cv::Mat(camera_info_.D);
	BOOST_FOREACH(const double & v, camera_info_.K) camera_m_.push_back(v);
	BOOST_FOREACH(const double & v, camera_info_.D) camera_d_.push_back(v);
	camera_m_ = camera_m_.reshape(1,3);
	// std::cout << camera_m_.size() << std::endl;
	// std::cout << camera_m_.size() << " " << camera_d_.size() << std::endl;
	ROS_INFO("Got Camera Info");
	pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filterd_point_cloud", 1);
	// camera_info_sub_ = nh.Subscriber<sensor_msgs::CameraInfo>("camera_info", 1);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "point_cloud", 1);
	// typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, pc_sub);
	sync.registerCallback(boost::bind(&LidarCalibrator::sensorCallback, this, _1, _2));
	ros::spin();
}

// void cameraInfoCallback(const sensor_msgs::CameraInfo& camera_info_msg){
// 	camera_info_ = *msg;
// }

/**
 * @brief      sync image and point cloud call back of message_filter::Synchronizer
 *
 * @param[in]  image_msg  The image message
 * @param[in]  pc_msg     The point_cloud message
 */
void sensorCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
	ROS_INFO_ONCE("Recieved Point Cloud");
	ROS_INFO_ONCE("Recieved Images");

	cv::Mat image;
	try
	{
		image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
	}

	imageCheckerBoardDetection(image);

	// detecting the checkerboard in point cloud
	PointCloud pc;
	PointCloud pc2;
	pcl::fromROSMsg(*pc_msg, pc);
	pc2 = pc;
	PCFilterByDistance(2, 5, pc, pc2);
	PointCloud::Ptr opc_ptr = pc2.makeShared();
	PointCloudRGB pc_rgb;
	pcl::ModelCoefficients::Ptr coefficients = PCPlaneDetection(opc_ptr, pc_rgb);
	if (coefficients){
		std::cout << "PC Plane detected" << std::endl;
	}else{
		std::cout << "PC Plane not detected" << std::endl;
	}
	sensor_msgs::PointCloud2 opc_msg;
	pcl::toROSMsg(pc_rgb, opc_msg);
	opc_msg.header.frame_id = pc_msg->header.frame_id;
	pc_pub_.publish(opc_msg);
}

bool imageCheckerBoardDetection(cv::Mat& image){
	cv::Size patternsize(5, 7); //number of corners
	double square = 0.05;
	std::vector<cv::Point2f> corners; //this will be filled by the detected centers

	bool patternfound = cv::findChessboardCorners(image, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH 
																				+ cv::CALIB_CB_NORMALIZE_IMAGE
																				+ cv::CALIB_CB_FAST_CHECK);
	if (patternfound){
		cv::Mat axis = (cv::Mat_<double>(3,3) << 3,0,0,0,3,0,0,0,-3);
		axis = axis.reshape(1, 3);
		bool sub_pix = false;
		if (sub_pix){
			// Calculate the refined corner locations
			cv::Mat src_gray;
			cv::Size winSize = cv::Size( 11, 11 );
			cv::Size zeroZone = cv::Size( -1, -1 );
			cv::TermCriteria criteria(  cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 );
			cv::cvtColor( image, src_gray, CV_BGR2GRAY );
			cv::cornerSubPix( src_gray, corners, winSize, zeroZone, criteria );
		}
		
		std::vector<cv::Point3f> objp;
		for (int i = 0; i < patternsize.width; i++){
			for (int j = 0; j < patternsize.height; j++){
				double x = j;
				double y = i;
				double z = 0;
				cv::Point3f point(x,y,z);
				// std::cout << corners[i*5 + j] << " " << point << std::endl;
				objp.push_back(point);
			}
		}
		try{
			// cv::Vec3f rvec;
			// cv::Vec3f tvec;
			cv::Mat rvec(3, 1, CV_64F);
			cv::Mat tvec(3, 1, CV_64F);
			// cv::solvePnP(objp, corners, camera_m_, camera_d_, rvec, tvec);
			cv::solvePnPRansac(objp, corners, camera_m_, camera_d_, rvec, tvec);
			std::cout << rvec << " " << tvec << std::endl;
			std::cout << axis.size() << " " << axis.channels()<< std::endl;
			std::vector<cv::Point2d> ppoints;
			cv::projectPoints(axis, rvec, tvec, camera_m_, camera_d_, ppoints);
			cv::imshow("view2", ppoints);
		}catch(cv::Exception e){
			return false;
		}
		
	}
	cv::drawChessboardCorners(image, patternsize, cv::Mat(corners), patternfound);

	cv::imshow("view", image);
	cv::waitKey(30);
}

/**
 * @brief      Use PCL plane detection function to detect the checker board
 *
 * @param[in]  ipc     The input point cloud
 * @param      pc_rgb  The point_cloud rgb
 *
 * @return     the plane coefficient
 */
pcl::ModelCoefficients::Ptr PCPlaneDetection(const PointCloud::Ptr &ipc, PointCloudRGB &pc_rgb){
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	pcl::copyPointCloud(*ipc, pc_rgb);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	// seg.setProbability(0.99);

	seg.setInputCloud (ipc);
	seg.segment (*inliers, *coefficients);
	int num_threshold = 60;
	double prob_threshold = 0.41;
	int inter_size = inliers->indices.size();
	if (inter_size < num_threshold)
	{
		coefficients.reset();
		return coefficients;
	}
	double inlier_prob = double(inliers->indices.size())/ipc->size();
	std::cout << "prob: " << inlier_prob << " size: " << inter_size << " ";
	if (inlier_prob < prob_threshold){
		coefficients.reset();
		return coefficients;
	}
	// plane detected, change the color
	for (size_t i = 0; i < inliers->indices.size (); ++i){
		pc_rgb.points[inliers->indices[i]].g = 0;
		pc_rgb.points[inliers->indices[i]].b = 0;
		pc_rgb.points[inliers->indices[i]].r = 255;
	}
	return coefficients;
}

/**
 * @brief      Point Cloud filtered by distance
 *
 * @param[in]  mind  The mind
 * @param[in]  maxd  The maxd
 * @param[in]  ipc   The ipc
 * @param      opc   The opc
 */
void PCFilterByDistance(double mind, double maxd, const PointCloud &ipc, PointCloud& opc){
	opc.clear();
	BOOST_FOREACH(const pcl::PointXYZ& pt, ipc.points)
	{
		Eigen::Vector3f v(pt.x, pt.y, pt.z);
		double radius = v.squaredNorm();
		if (radius > mind && radius < maxd){
			opc.push_back(pt);
		}
	}
}

/**
 * @brief      Point Cloud filtered by yaw angle
 *
 * @param[in]  sangle  The sangle
 * @param[in]  eangle  The eangle
 * @param[in]  ipc     The ipc
 * @param      opc     The opc
 */
void PCFilterByYawAngle(double sangle, double eangle, const PointCloud &ipc, PointCloud& opc){
	opc.clear();
	BOOST_FOREACH(const pcl::PointXYZ& pt, ipc.points)
	{
		double phi = atan2(pt.y, pt.x);
		if (phi > sangle && phi < eangle){
			opc.push_back(pt);
		}
	}
}

private:
ros::Publisher pc_pub_;
cv::Mat camera_m_;
cv::Mat camera_d_;
sensor_msgs::CameraInfo camera_info_;

}; // end of class

/**
 * @brief      Regester the ROS node
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     0
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_calibrator");
	ros::NodeHandle nh;
	LidarCalibrator lc(nh);
	// ros::spin();
	return 0;
}