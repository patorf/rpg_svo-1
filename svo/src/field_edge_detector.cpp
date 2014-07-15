#include <svo/field_edge_detector.h>


namespace svo {
Field_edge_detector::Field_edge_detector(){}
void subCB(const svo_msgs::MapPoints::ConstPtr & msg)
{
    ROS_INFO_STREAM("create pointcloud");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->resize(msg->points.size());
   for (unsigned int i=0; i < msg->points.size(); ++i)
        {
          const geometry_msgs::Point &data = msg->points[i];
          cloud->points[i].x=data.x;

          cloud->points[i].y=data.y;
          cloud->points[i].z=data.z;
   /*       ROS_INFO_STREAM("x: " << data.x << "y: " << data.y <<
                          "z: " << data.z );*/
        }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pub_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pub_cloud->resize(inliers->indices.size());
    //neuer Versuch
    for (unsigned int i = 0; i < inliers->indices.size(); ++i) {
        pub_cloud->points[i]=cloud->points[inliers->indices[i]];
    }
    ros::NodeHandle nh;

    std::string topic = nh.resolveName("point_cloud");
    uint32_t queue_size = 1;
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > (topic, queue_size);
      ROS_INFO_STREAM("publish plane Points"<<pub_cloud->points.size());
      pub_cloud->header.frame_id="my_frame_ID";
    pub.publish(pub_cloud);
    ros::spinOnce();


}

void Field_edge_detector::convert_to_pcl(){

  //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      //cloud.push_back(pcl::PointXYZ(1,2,3));

      ros::NodeHandle np;
      sb = np.subscribe("mapPoints",1000,svo::subCB);
}


/*void mycallback(list<Point*> msg){

    ROS_INFO_STREAM(msg.size());
}*/

}//namespace fed
