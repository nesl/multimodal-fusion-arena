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

//Determine whether two people from two cameras are the same person
bool areEqual(NESLMessages::Person person1, NESLMessages::Person person3) {
    double centroidDiff = sqrt(pow(person1.personCoord.x - person3.personCoord.x, 2) + pow(person1.personCoord.y - person3.personCoord.y, 2));
    double colorDiff = sqrt(pow(person1.colorArr[0] - person3.colorArr[0], 2) + pow(person1.colorArr[1] - person3.colorArr[1], 2) + 
        pow(person1.colorArr[2] - person3.colorArr[2], 2));
    if (colorDiff < 75 && centroidDiff < 0.5) {
        return true;
    }
    return centroidDiff < 0.2;

}

//Callback function when message filters identifies matching frames
void callback(sensor_msgs::PointCloud2::ConstPtr cloud1, sensor_msgs::PointCloud2::ConstPtr cloud3, const boost::shared_ptr<const NESLMessages::PersonArr> arr1Param, 
    const boost::shared_ptr<const NESLMessages::PersonArr> arr3Param) {
    NESLMessages::PersonArr arr1 = *arr1Param;
    NESLMessages::PersonArr arr3 = *arr3Param;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud1, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc3(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pc1);
    pcl_conversions::toPCL(*cloud3, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pc3);
    *pc1 += *pc3;
    NESLMessages::PersonArr finalArr;
    for (int i = 0; i < arr1.personArr.size();) {
        NESLMessages::Person person1 = arr1.personArr.at(i);
        for (int j = 0; j < arr3.personArr.size(); j++) {
            NESLMessages::Person person3 = arr3.personArr.at(i);
            if (areEqual(person1, person3)) {
                person1.personCoord.x = (person1.personCoord.x + person3.personCoord.x) / 2;
                person1.personCoord.y = (person1.personCoord.y + person3.personCoord.y) / 2;
                person1.personCoord.z = (person1.personCoord.z + person3.personCoord.z) / 2;
                person1.bbx = (person1.bbx + person3.bbx) / 2.0;
                person1.bby = (person1.bby + person3.bby) / 2.0;
                person1.bbz = (person1.bbz + person3.bbz) / 2.0;
                finalArr.personArr.push_back(person1);
                arr1.personArr.erase(arr1.personArr.begin() + i);
                arr3.personArr.erase(arr3.personArr.begin() + j);
                break;
            }
            //No match found, the centroid in cam 1 is unique
            else if (j == arr3.personArr.size() - 1) {
                i++;
            }
        }
    }
    //Add remaining elements, these have no match found
    for (int i = 0; i < arr1.personArr.size(); i++) {
        finalArr.personArr.push_back(arr1.personArr.at(i));
    }
    for (int i = 0; i < arr3.personArr.size(); i++) {
        finalArr.personArr.push_back(arr3.personArr.at(i));
    }
    cloud_publisher.publish(pc1);
    coord_publisher.publish(finalArr);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "displayer");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc1(nh, "camera1/clouds/subtracted", 5);
    //message_filters::Subscriber pc2(nh, "camera2/cloud", 2);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc3(nh, "camera3/clouds/subtracted", 5);
    message_filters::Subscriber<NESLMessages::PersonArr> coords1(nh, "camera1/people", 5);
    //message_filters::Subscriber<NESLMessages::PersonArr> coord2(nh, "camera2/people", 2);
    message_filters::Subscriber<NESLMessages::PersonArr> coords3(nh, "camera3/people", 5);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
       NESLMessages::PersonArr, NESLMessages::PersonArr> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc1, pc3, coords1, coords3);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
    cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/world/cloud_combined", 1000);
    coord_publisher = nh.advertise<NESLMessages::PersonArr>("/world/people", 1000);
    ros::spin();

}
