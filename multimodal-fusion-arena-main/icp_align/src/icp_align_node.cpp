#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr changeCloud;
double startTime = 0;

void processICP() {
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr destinationCloud;
    icp.setInputSource(sourceCloud);
    icp.setInputTarget(changeCloud);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-8);
    icp.align(*destinationCloud);
    std::cout << icp.getFinalTransformation() << std::endl;
    ros::shutdown();
}

void cbSourceCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {
    pcl::fromROSMsg(*input, *sourceCloud);
    if (startTime - ros::Time::now().toSec() > 10) {
        processICP();
    }
}

void cbChangeCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {
    pcl::fromROSMsg(*input, *changeCloud);
    if (startTime - ros::Time::now().toSec() > 10) {
        processICP();
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("PATH TO SOURCE TOPIC", 1000, cbSourceCloud);
    ros::Subscriber sub2 = nh.subscribe("PATH TO CHANGECLOUD TOPIC", 1000, cbChangeCloud);
    startTime = ros::Time::now().toSec();
    ros::spin();
}
