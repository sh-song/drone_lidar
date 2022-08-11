#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <iostream>       // std::cout
#include <typeinfo>       // operator typeid

ros::Publisher pub;


void pointsCallback (const sensor_msgs::PointCloud2 msg) {

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(msg, pcl_pc);
  std::cout << "cloud is: " << typeid(msg).name() << '\n';

  sensor_msgs::PointCloud2 output;
  output = msg;
  pub.publish(output);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ros_pcl_minimal_pub_sub");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("velodyne_points", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, pointsCallback);


  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("test_output", 1);

  // Spin
  ros::spin ();
}
