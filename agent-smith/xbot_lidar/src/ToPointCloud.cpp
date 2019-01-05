#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  ros::Subscriber laser_sub_;
  ros::Publisher cloud_pub_;
  sensor_msgs::PointCloud2 cloud;
  pcl::PCLPointCloud2 pclCloud;
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  float* pFootprintX_;
  float* pFootprintY_;
  int footprintSize_;

  int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy) {
    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>testy) != (verty[j]>testy)) &&
        (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
        c = !c;
    }
    return c;
  }

  LaserScanToPointCloud(ros::NodeHandle n, int footprintSize, float* x, float* y) {
    n_ = n;
    footprintSize_ = footprintSize;
    pFootprintX_ = x;
    pFootprintY_ = y;
    laser_sub_ = n_.subscribe<sensor_msgs::LaserScan> ("/scan_raw", 100, &LaserScanToPointCloud::scanCallback, this);
    cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/point_cloud",1);
    listener_.setExtrapolationLimit(ros::Duration(0.1));
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    
    try
    {
        projector_.transformLaserScanToPointCloud(
          "base_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    // Do something with cloud.
    // First, remove points inside of the robot, using the footprint.
    //for () Put in the iters over the cloud

    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    pcl_conversions::moveToPCL(cloud, pclCloud);
    pcl::fromPCLPointCloud2( pclCloud, point_cloud);
    pcl::PointCloud<pcl::PointXYZ>::iterator iter;
  
    for(iter = point_cloud.begin(); iter != point_cloud.end(); iter++) {

      
      if (pnpoly(footprintSize_, pFootprintX_, pFootprintY_, iter->x, iter->y) % 2 == 1) {
        //Inside! (since it is odd)
        //Remove it!
        point_cloud.erase(iter);
      }
    }

    pcl::toPCLPointCloud2 ( point_cloud, pclCloud);
    pcl_conversions::moveFromPCL(pclCloud, cloud);

    cloud_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "laserScanToCloud");
  ros::NodeHandle n;
  std::vector<float> footprint;
  std::vector<float> footprintX;
  std::vector<float> footprintY;
  float* pFootprintX;
  float* pFootprintY;
  if (!n.getParam("footprint", footprint)) ROS_ERROR("Failed to get parameter from server.");
  for (int i = 0;i < footprint.size(); i+=2) {
    footprintX.push_back(footprint[i]);
    footprintY.push_back(footprint[i+1]);
  }
  pFootprintX = footprintX.data();
  pFootprintY = footprintY.data();

  const geometry_msgs::TransformStamped transform;
  
  LaserScanToPointCloud lstopc(n, footprint.size(), pFootprintX, pFootprintY);
  
  ros::spin();
  
  return 0;
}