#ifndef FIELDEDGEDETECTOR_H
#define FIELDEDGEDETECTOR_H

#include <sophus/se3.h>
#include <ros/ros.h>
//#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>
#include <svo_msgs/MapPoints.h>
#include <geometry_msgs/Point.h>

namespace svo {

class Field_edge_detector
{
public:
   Field_edge_detector();
   void  convert_to_pcl();
   ros::Subscriber sb;

};
void subCB(const svo_msgs::MapPoints::ConstPtr & msg);

}
#endif // FIELDEDGEDETECTOR_H
