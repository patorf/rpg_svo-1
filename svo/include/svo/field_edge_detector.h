#ifndef FIELDEDGEDETECTOR_H
#define FIELDEDGEDETECTOR_H

#include <sophus/se3.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <svo_msgs/MapPoints.h>
#include <geometry_msgs/Point.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/common/intersections.h>
#include <pcl/filters/extract_indices.h>
namespace svo {

class Field_edge_detector
{
public:
   Field_edge_detector();
   void  convert_to_pcl();
   ros::Subscriber sb;
   ros::Publisher plane_pub_1;
   ros::Publisher plane_pub_2;
   ros::Publisher planeIntersection;
   Eigen::Vector3d edgeStart;
   Eigen::Vector3d edgeEnd;

   ros::NodeHandle nh;
   void subCB(const svo_msgs::MapPoints::ConstPtr & msg);
   void IntersectPlancesAndDrawLine(pcl::ModelCoefficients::Ptr coefficientsA,pcl::ModelCoefficients::Ptr coefficientsB);
   bool check_right_angle_of_Planes(pcl::ModelCoefficients::Ptr coefficientsA,pcl::ModelCoefficients::Ptr coefficientsB);

};

}
#endif // FIELDEDGEDETECTOR_H
