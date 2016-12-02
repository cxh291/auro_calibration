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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class LidarCalibrator{

public:

LidarCalibrator(ros::NodeHandle nh){
	cv::namedWindow("view");
	cv::startWindowThread();

	pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filterd_point_cloud", 1);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "point_cloud", 1);
	// typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, pc_sub);
	sync.registerCallback(boost::bind(&LidarCalibrator::sensorCallback, this, _1, _2));

	ros::spin();
}

void sensorCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
	ROS_INFO_ONCE("Recieved Point Cloud");
	ROS_INFO_ONCE("Recieved Images");

	PointCloud pc;
	PointCloud pc2;
	pcl::fromROSMsg(*pc_msg, pc);
	try
	{
		cv::imshow("view", cv_bridge::toCvShare(image_msg, "bgr8")->image);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
	}
	pc2 = pc;
	PCFilterByDistance(2, 5, pc, pc2);
	PointCloud::Ptr opc_ptr = pc2.makeShared();
	PointCloudRGB pc_rgb;
	pcl::ModelCoefficients::Ptr coefficients = PCPlaneDetection(opc_ptr, pc_rgb);
	if (coefficients){
		std::cout << "Plane detected" << std::endl;
	}else{
		std::cout << "Plane not detected" << std::endl;
	}
	sensor_msgs::PointCloud2 opc_msg;
	pcl::toROSMsg(pc_rgb, opc_msg);
	opc_msg.header.frame_id = pc_msg->header.frame_id;
	pc_pub_.publish(opc_msg);
}

void copyPointCloud(const PointCloud& cloud_xyz, PointCloudRGB& cloud_xyzrgb){
	cloud_xyzrgb.clear();
	cloud_xyzrgb.points.resize(cloud_xyz.size());
	for (size_t i = 0; i < cloud_xyz.points.size(); i++) {
	    cloud_xyzrgb.points[i].x = cloud_xyz.points[i].x;
	    cloud_xyzrgb.points[i].y = cloud_xyz.points[i].y;
	    cloud_xyzrgb.points[i].z = cloud_xyz.points[i].z;
	}
}

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
	std::cout << "prob: " << inlier_prob << " size" << inter_size << " ";
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

void PCFilterByDistance(double mind, double maxd, const PointCloud &ipc, PointCloud& opc){
	opc.clear();
	BOOST_FOREACH(const pcl::PointXYZ& pt, ipc.points)
	{
		Eigen::Vector3f v(pt.x, pt.y, pt.z);
		double radius = v.squaredNorm();
		// std::cout << phi << std::endl;
		if (radius > mind && radius < maxd){
			opc.push_back(pt);
		}
	}
}

void PCFilterByYawAngle(double sangle, double eangle, const PointCloud &ipc, PointCloud& opc){
	opc.clear();
	BOOST_FOREACH(const pcl::PointXYZ& pt, ipc.points)
	{
		// Eigen::Vector3f v(pt.x, pt.y, pt.z);
		// double theta = acos(pt.z/v.squaredNorm());
		double phi = atan2(pt.y, pt.x);
		// std::cout << phi << std::endl;
		if (phi > sangle && phi < eangle){
			opc.push_back(pt);
		}
	}
	// std::cout << ipc.size() << " " << opc.size() << std::endl;
}

private:
ros::Publisher pc_pub_;

}; // end of class

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_calibrator");
	ros::NodeHandle nh;
	LidarCalibrator lc(nh);
	// ros::spin();
}