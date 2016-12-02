#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
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

class LidarCalibrator{

public:

LidarCalibrator(ros::NodeHandle nh){
	cv::namedWindow("view");
	cv::startWindowThread();
	// ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("point_cloud", 1, boost::bind(&LidarCalibrator::pointCloudCallback, this, _1));
	// image_transport::ImageTransport it(nh);
	// image_transport::Subscriber sub = it.subscribe("image", 1, &LidarCalibrator::imageCallback, this);
	pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filterd_point_cloud", 1);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "point_cloud", 1);
	// typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	// message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, pc_sub, 20);
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, pc_sub);
	sync.registerCallback(boost::bind(&LidarCalibrator::sensorCallback, this, _1, _2));

	ros::spin();
}

void sensorCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
	ROS_INFO_ONCE("Recieved Point Cloud");

	// printf ("Cloud: width = %d, height = %d\n", pc_msg->width, pc_msg->height);
	PointCloud pc;
	PointCloud opc;
	pcl::fromROSMsg(*pc_msg, pc);
	// BOOST_FOREACH (const pcl::PointXYZ& pt, pc.points)
	//   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

	ROS_INFO_ONCE("Recieved Images");
	try
	{
		cv::imshow("view", cv_bridge::toCvShare(image_msg, "bgr8")->image);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
	}
	opc = pc;
	filterByYawAngle(-1,1,pc,opc);
	sensor_msgs::PointCloud2 opc_msg;
	pcl::toROSMsg(opc, opc_msg);
	pc_pub_.publish(opc_msg);
}

void filterByYawAngle(double sangle, double eangle, const PointCloud &ipc, PointCloud& opc){
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
// void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {
// 	ROS_INFO_ONCE("Recieved Point Cloud");
// 	printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
// 	PointCloud pc;
// 	pcl::fromROSMsg(*msg, pc);
// 	BOOST_FOREACH (const pcl::PointXYZ& pt, pc.points)
// 	  printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
// }

// void imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
// 	ROS_INFO_ONCE("Recieved Images");
// 	try
// 	{
// 		cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
// 		cv::waitKey(30);
// 	}
// 	catch (cv_bridge::Exception& e)
// 	{
// 		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
// 	}
// }


}; // end of class

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_calibrator");
	ros::NodeHandle nh;
	LidarCalibrator lc(nh);
	// ros::spin();
}