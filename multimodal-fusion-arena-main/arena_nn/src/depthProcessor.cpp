#include "ros/ros.h"
#include "beginner_tutorials/Detection.h"
#include "beginner_tutorials/DetectionArr.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/Detection2DArray.h"
#include "vision_msgs/Detection2D.h"
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <queue>
#include <vector>
#include <stdlib.h>
    
using namespace sensor_msgs;
using namespace message_filters;
using namespace vision_msgs;
using namespace beginner_tutorials;

struct Point {
public:
  int x;
  int y;
  Point() {
    
  }
  Point(int x, int y) {
    this->x = x;
    this->y = y;
  }
};

struct Person {
public:
  Point corner1, corner2;
  //add color histogram here to help distinguish between people
  Person() {

  }
  Person(Point p1, Point p2) {
    corner1 = p1;
    corner2 = p2;
  }
};


//ros::Publisher pub_points2_;
ros::Publisher detection_pub;
int count = 0;
std::queue<Image::ConstPtr> imageQueue;
double startTime = 0;
void imgCallBack(const Image::ConstPtr& img) {
  imageQueue.push(img);
  if (imageQueue.size() > 5) {
    std::queue<Image::ConstPtr> empty;
    std::swap(imageQueue, empty);
  }

  /*

   for (int i = 0; i < length; i++) {
     
     //Perform stage 1 filtering on the x axis
     pass.setInputCloud(personBox);
     pass.setFilterFieldName ("x");
     pass.setFilterLimits (personArr[i].corner1.x, personArr[i].corner2.x);
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
     pass.filter (*cloud_filtered);
     //Perform stage 2 filtering on the y axis
     pass.setInputCloud(cloud_filtered);
     pass.setFilterFieldName ("y");
     pass.setFilterLimits (personArr[i].corner1.y, personArr[i].corner2.y);
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
     pass.filter (*cloud_filtered2);
    *outputCloud += *cloud_filtered2;
 
   }
   outputCloud->header.frame_id = "box";
   pub_points2_.publish(outputCloud);
   
    pass.setInputCloud(personBox);
     pass.setFilterFieldName ("x");
     pass.setFilterLimits (personArr[0].corner1.x, personArr[0].corner2.x);
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
     pass.filter (*cloud_filtered);
     cloud_filtered->header.frame_id = "box";
     pub_points2_.publish(cloud_filtered);
   */
}

void dataCallBack(const Detection2DArray::ConstPtr& data) {
  if (imageQueue.empty() || ros::Time::now().toSec() - startTime < 1) {
    return;
  }
  Image::ConstPtr& img = imageQueue.front();
  imageQueue.pop();
  int length = data->detections.size();
  if (length <= 0) {
    return;
  }
  Person personArr[length];
  std::vector<Detection> detectArr;
  Point edge1, edge2;
  //Assign the bounding box to each person 
  for (int i = 0; i < length; i++) {
    int x = data->detections[i].bbox.center.x;
    int y = data->detections[i].bbox.center.y;
    int xSize = data->detections[i].bbox.size_x;
    int ySize = data->detections[i].bbox.size_y;
    int edge1X = (x - xSize / 2 > 0) ? x - xSize / 2 : 0;
    int edge1Y = (y - ySize / 2 > 0) ? y - ySize / 2: 0;
    int edge2X = (x + xSize / 2 <= 1280) ? x + xSize / 2 : 1280;
    int edge2Y = (y + ySize / 2 <= 720) ? y + ySize / 2 : 720;
    personArr[i] = Person(Point(edge1X, edge1Y),
			  Point(edge2X, edge2Y)); //idk if this is necessary hm
  }

  for (int i = 0; i < length; i++) {
    int id = data->detections[i].results[0].id;
    if (id != 1) {
      continue;
    }
    int centerX = (personArr[i].corner1.x + personArr[i].corner2.x) / 2;
    int centerY = (personArr[i].corner1.y + personArr[i].corner2.y) / 2;
    int sum = 0;
    int numSums = 0;
    int guessAvg = 0;
    int numAttempts = 0;
    while (numSums < 30) {
      int randomVal1 = rand() % 26;
      int randomVal2 = rand() % 26;
      uint16_t depthVal = (uint16_t) (img->data[(centerX + randomVal1) * 720 * 2 + (centerY + randomVal2) * 2 + 1] << 8 |
					    img->data[(centerX + randomVal1) * 720 * 2 + (centerY + randomVal2) * 2]);
      if (depthVal > 400 && depthVal < 4000) {
	guessAvg += depthVal;
	numSums++;
      }
      //center point chosen is not good
      if (numAttempts++ > 100) {
	return;
      }
    }
    numSums = 0;
    guessAvg /= 30;
    // std::cout << guessAvg << std::endl;
    for (int i = 0; i < 25; i++) {
        for (int j = 0; j < 25; j++) {
            uint16_t depthVal = (uint16_t) (img->data[(centerX + i) * 720 * 2 + (centerY + j) * 2 + 1] << 8 |
					    img->data[(centerX + i) * 720 * 2 + (centerY + j) * 2]);
            if (depthVal > 400 && std::abs(depthVal - guessAvg) < 0.5 * guessAvg) {
	      sum += depthVal;
	      numSums++;
	    }
         }
    }
    double depth = (numSums > 0) ? sum / numSums : 0;
    depth /= 1000;
    double x = (centerX - 651.198) / 911.3503;
    double y = (centerY - 366.864) / 911.4847;
    double r2  = x*x + y*y;
    double f = 1 + 0.1653425246477127*r2 + -0.5329044461250305*r2*r2 + 0.4601594805717468*r2*r2*r2;
    double ux = x*f + 2*0.0009987360099330544*x*y + 3.125920784441405e-06*(r2 + 2*x*x);
    double uy = y*f + 2*3.125920784441405e-06*x*y + 0.0009987360099330544*(r2 + 2*y*y);
    x = ux;
    y = uy;
    Detection temp;
    depth *= 5;
    temp.x = x * depth;
    temp.y = y * depth;
    temp.z = depth;
    temp.id = id;
    detectArr.push_back(temp);
    //std::cout << "x: " << x * depth << ", y: " << y * depth << ", z: " << depth << std::endl;
  }

  beginner_tutorials::DetectionArr dataArr;
  dataArr.objects = detectArr;
  detection_pub.publish(dataArr);

  /*
			 
  std::cout << (uint16_t)( img->data[centerX * 720 * 2 + centerY * 2 + 1] << 8 |
			   img->data[centerX * 720 * 2 + centerY * 2]) << std::endl;
  */			
}

int main(int argc, char** argv) {
     ros::init(argc, argv, "listener");
     ros::NodeHandle nh;
     std::cout << "entered main" << std::endl;
     startTime = ros::Time::now().toSec();
     // pub_points2_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/cloud_out", 100);
     ros::Subscriber data_sub = nh.subscribe("/detectnet/detections", 3, dataCallBack);
     ros::Subscriber img_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 3, imgCallBack);
     detection_pub = nh.advertise<DetectionArr>("/detectionprocessor/detections/", 1000);
     std::cout << "finished making nodes" << std::endl;
     ros::spin();

     return 0;


}


