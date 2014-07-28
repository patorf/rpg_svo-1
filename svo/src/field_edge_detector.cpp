#include <svo/field_edge_detector.h>


namespace svo {
Field_edge_detector::Field_edge_detector(){

    plane_pub_1 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("myPointCloud1", 1);
    plane_pub_2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("myPointCloud2", 1);
    planeIntersection = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("planeIntersection", 1);


}
void Field_edge_detector::subCB(const svo_msgs::MapPoints::ConstPtr & msg)
{
  ROS_INFO_STREAM("create pointcloud");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

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





    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.05);

    pcl::ModelCoefficients::Ptr allCoeffis[2];

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    for (int i = 0; i <= 1; ++i) {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        ROS_INFO_STREAM("loop: "<<i);
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
            {
              std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
              break;
            }
        allCoeffis[i]=coefficients;


        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        //Plane Points = cloud_p

        if (nh.ok()){
           //Damit rviz den Koordinatenrahmen kennt
            cloud_p->header.frame_id="/world";
            if (i==0){
                ROS_INFO_STREAM("publish plane Points1: "<<cloud_p->points.size());

                plane_pub_1.publish(cloud_p);

            }else if (i==1){
                ROS_INFO_STREAM("publish plane Points2: "<<cloud_p->points.size());

                plane_pub_2.publish(cloud_p);
                ROS_INFO_STREAM(allCoeffis[0]);

               IntersectPlancesAndDrawLine(allCoeffis[0],allCoeffis[1]);

            }
        }


        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);




    }




   // pcl::PointCloud<pcl::PointXYZ>::Ptr pub_cloud(new pcl::PointCloud<pcl::PointXYZ>);

   /* pub_cloud->resize(inliers->indices.size());
    for (unsigned int i = 0; i < inliers->indices.size(); ++i) {
        pub_cloud->points[i]=cloud->points[inliers->indices[i]];
    }
*/


    const Eigen::Vector4f plane_a;


}

void Field_edge_detector::IntersectPlancesAndDrawLine(pcl::ModelCoefficients::Ptr coefficientsA,pcl::ModelCoefficients::Ptr coefficientsB){

    Eigen::Vector4f v_coeff_A(coefficientsA->values.data());
    Eigen::Vector4f v_coeff_B(coefficientsB->values.data());


    Eigen::VectorXf line;
    bool intSuccess=  pcl::planeWithPlaneIntersection(v_coeff_A,v_coeff_B,line);
    if (intSuccess){
        pcl::PointCloud<pcl::PointXYZ>::Ptr linePoints(new pcl::PointCloud<pcl::PointXYZ>);
        linePoints->resize(100);
        linePoints->header.frame_id="/world";
        for (int i = 0; i < 100; ++i) {
            linePoints->points[i].x= line[0]+i*line[3];
            linePoints->points[i].y= line[1]+i*line[4];

            linePoints->points[i].z= line[2]+i*line[5];

        }

        planeIntersection.publish(linePoints);
        ROS_INFO_STREAM(line);

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
