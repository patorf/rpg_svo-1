#include <svo/field_edge_detector.h>


namespace svo {
Field_edge_detector::Field_edge_detector(){}
void subCB(const svo_msgs::MapPoints::ConstPtr & msg)
{
    ROS_INFO_STREAM("lksdjfl");
   /* for (unsigned int i=0; i < msg->points.size(); ++i)
        {
          const geometry_msgs::Point &data = msg->points[i];
          ROS_INFO_STREAM("x: " << data.x << "y: " << data.y <<
                          "z: " << data.z );
        }
      */
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
