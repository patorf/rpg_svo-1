#include <svo/field_edge_detector.h>


namespace svo {
Field_edge_detector::Field_edge_detector(){

    plane_pub_1 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("myPointCloud1", 1);
    plane_pub_2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("myPointCloud2", 1);
    planeIntersection = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("planeIntersection", 1);
    //  edgeStart=Eigen::Vector3d();
    // edgeEnd=Eigen::Vector3d();

}
void Field_edge_detector::subCB(const svo_msgs::MapPoints::ConstPtr & msg)
{
    ROS_INFO_STREAM("create pointcloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3f cloudCenter(0,0,0);
    cloud->resize(msg->points.size());
    for (unsigned int i=0; i < msg->points.size(); ++i)
    {
        const geometry_msgs::Point &data = msg->points[i];
        cloud->points[i].x=data.x;

        cloud->points[i].y=data.y;
        cloud->points[i].z=data.z;

        cloudCenter[0]=cloudCenter[0]+data.x;
        cloudCenter[1]=cloudCenter[1]+data.y;
        cloudCenter[2]=cloudCenter[2]+data.z;

        /*       ROS_INFO_STREAM("x: " << data.x << "y: " << data.y <<
                          "z: " << data.z );*/
    }
    cloudCenter/msg->points.size();

    /******** MODE 1 Ebenschnitt *****/
    if (true){

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.05);

        pcl::ModelCoefficients::Ptr allCoeffis[3];

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        int n_found_plane = 0;
        while (n_found_plane<3) {


            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            if (cloud->points.size()>5){
                seg.setInputCloud (cloud);
                seg.segment (*inliers, *coefficients);
            }
            // ROS_INFO_STREAM(inliers->indices.size());
            if (coefficients->values.size()==0)
            {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            allCoeffis[n_found_plane]=coefficients;
            n_found_plane++;

            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);
            //Plane Points = cloud_p

            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud.swap (cloud_f);


        }

        ROS_INFO_STREAM("N Plane found:"<<n_found_plane);
        bool orthogonal_plane_found=false;
        for ( int i = 0; i < n_found_plane-1; i++) {
            for (int j = i+1; j < n_found_plane; j++) {

                if (check_right_angle_of_Planes(allCoeffis[i],allCoeffis[j])){
                    orthogonal_plane_found=true;
                    IntersectPlancesAndDrawLine(allCoeffis[i],allCoeffis[j]);
                }
            }
        }
        if (!orthogonal_plane_found){
            ROS_INFO_STREAM("no orthogonal plane found!");

        }


        //    if (nh.ok()){
        //        //Damit rviz den Koordinatenrahmen kennt
        //        cloud_p->header.frame_id="/world";
        //        if (n_found_plane==0){
        //            ROS_INFO_STREAM("publish plane Points1: "<<cloud_p->points.size());

        //            plane_pub_1.publish(cloud_p);

        //        }else if (n_found_plane==1){
        //            ROS_INFO_STREAM("publish plane Points2: "<<cloud_p->points.size());

        //            plane_pub_2.publish(cloud_p);
        //            ROS_INFO_STREAM(allCoeffis[0]);
        //        }
        //    }
    }
    /****** END MODE 1*/

    /**** MODE 2 Kruemmung ***/

}
bool Field_edge_detector::check_right_angle_of_Planes(pcl::ModelCoefficients::Ptr coefficientsA,pcl::ModelCoefficients::Ptr coefficientsB){
    double angle_tolerance = 20 *3.14/180;
    double maxAngle = 90*3.14/180  +angle_tolerance;
    double minAngle = 90*3.14/180  -angle_tolerance;
    Eigen::Vector4f v_coeff_A(coefficientsA->values.data());
    Eigen::Vector4f v_coeff_B(coefficientsB->values.data());
    v_coeff_A[3]=0;
    v_coeff_B[3]=0;



    double  angle_rad = pcl::getAngle3D(v_coeff_A,v_coeff_B);

    ROS_INFO_STREAM(angle_tolerance);
    if (angle_rad< maxAngle  &&  angle_rad> minAngle){
        ROS_INFO_STREAM("true" << angle_rad);


        return true;
    }else{
        ROS_INFO_STREAM("false"<<angle_rad);

        return false;
    }


}

void Field_edge_detector::IntersectPlancesAndDrawLine(pcl::ModelCoefficients::Ptr coefficientsA,pcl::ModelCoefficients::Ptr coefficientsB){

    Eigen::Vector4f v_coeff_A(coefficientsA->values.data());
    Eigen::Vector4f v_coeff_B(coefficientsB->values.data());
    //    ROS_INFO_STREAM(v_coeff_A);
    //    ROS_INFO_STREAM(v_coeff_B);
    Eigen::VectorXf line;
    bool intSuccess=  pcl::planeWithPlaneIntersection(v_coeff_A,v_coeff_B,line);
    if (intSuccess){

        edgeStart = Eigen::Vector3d(line[0],line[1],line[2]);
        edgeEnd= Eigen::Vector3d(line[0]+line[3],line[1]+line[4],line[2]+line[5]);

        //        pcl::PointCloud<pcl::PointXYZ>::Ptr linePoints(new pcl::PointCloud<pcl::PointXYZ>);
        //        linePoints->resize(100);
        //        linePoints->header.frame_id="/world";
        //        for (int i = 0; i < 100; ++i) {
        //            linePoints->points[i].x= line[0]+(i-50)*line[3];
        //            linePoints->points[i].y= line[1]+(i-50)*line[4];
        //            linePoints->points[i].z= line[2]+(i-50)*line[5];

        //        }
        //        edgeStart=Eigen::Vector3d(linePoints->points[50].x,
        //                linePoints->points[50].y,
        //                linePoints->points[50].z);

        //        edgeEnd=Eigen::Vector3d(linePoints->points[55].x,
        //                linePoints->points[55].y,
        //                linePoints->points[55].z);
        //        //  edgeEnd=linePoints->points[99];
        //        ROS_INFO_STREAM("LIIIINIIIE");
        //        planeIntersection.publish(linePoints);
        //        //ROS_INFO_STREAM(line);

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
