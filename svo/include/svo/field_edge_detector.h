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

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <algorithm>    // std::min_element, std::max_element

typedef std::pair<int,float> mypair;

namespace svo {

class Field_edge_detector

{
public:
   Field_edge_detector();
   void  convert_to_pcl();
   ros::Subscriber sb_mappoints;
   ros::Subscriber sb_pose;
   ros::Publisher plane_pub_1;
   ros::Publisher plane_pub_2;
   ros::Publisher planeIntersection;
   Eigen::Vector3d edgeStart;
   Eigen::Vector3d edgeEnd;
   Eigen::Vector4f pose_pos;

   ros::NodeHandle nh;
   void subCB_mapPoints(const svo_msgs::MapPoints::ConstPtr & msg);
   void subCB_Pose (const geometry_msgs::PoseWithCovarianceStamped & msg);
   void IntersectPlanes(pcl::ModelCoefficients::Ptr coefficientsA,pcl::ModelCoefficients::Ptr coefficientsB);
   bool check_right_angle_of_Planes(pcl::ModelCoefficients::Ptr coefficientsA,pcl::ModelCoefficients::Ptr coefficientsB);
   bool mycompare (const mypair l, const mypair r);
};

}
#endif // FIELDEDGEDETECTOR_H
