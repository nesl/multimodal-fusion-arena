#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <mutex>
#include <Eigen/Dense>
#include <NESLMessages/NeslCoord.h>
#include <NESLMessages/Person.h>
#include <NESLMessages/PersonArr.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
ros::Publisher cloud_publisher;
ros::Publisher coord_publisher;
using namespace message_filters;
int count = 0;
void callback(sensor_msgs::PointCloud2::ConstPtr cloud1, sensor_msgs::PointCloud2::ConstPtr cloud3) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud1, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc3(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pc1);
    pcl_conversions::toPCL(*cloud3, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pc3);
    *pc1 += *pc3;;
    std::cout << "published"  << count++ << std::endl;
    cloud_publisher.publish(pc1);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "displayer");
    ros::NodeHandle nh;
    cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/combined/cloud", 2);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc1(nh, "camera1/cloud", 2);
    //message_filters::Subscriber pc2(nh, "camera2/cloud", 2);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc3(nh, "camera3/cloud", 2);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc1, pc3);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();
}