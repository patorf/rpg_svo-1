#include <svo/field_edge_detector.h>


namespace svo {
Field_edge_detector::Field_edge_detector(){

    plane_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("myPointCloud", 1);

}
void Field_edge_detector::subCB(const svo_msgs::MapPoints::ConstPtr & msg)
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
    seg.setDistanceThreshold (0.05);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);



    pcl::PointCloud<pcl::PointXYZ>::Ptr pub_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pub_cloud->resize(inliers->indices.size());
    //neuer Versuch
    for (unsigned int i = 0; i < inliers->indices.size(); ++i) {
        pub_cloud->points[i]=cloud->points[inliers->indices[i]];
    }


    if (nh.ok()){
        ROS_INFO_STREAM("publish plane Points1: "<<pub_cloud->points.size());
       //Damit rviz den Koordinatenrahmen kennt
        pub_cloud->header.frame_id="/world";

        plane_pub.publish(pub_cloud);
    }


}

void Field_edge_detector::convert_to_pcl(){

  //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      //cloud.push_back(pcl::PointXYZ(1,2,3));

      //ros::NodeHandle np;
      sb = nh.subscribe("mapPoints",
                        3000,&Field_edge_detector::subCB,
                        this);
}


/*void mycallback(list<Point*> msg){

    ROS_INFO_STREAM(msg.size());
}*/

}//namespace fed
