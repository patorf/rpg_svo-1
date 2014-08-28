#include <svo/field_edge_detector.h>

typedef std::pair<int,float> mypair;
namespace svo {
Field_edge_detector::Field_edge_detector(){

    plane_pub_1 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("myPointCloud1", 1);
    plane_pub_2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("myPointCloud2", 1);
    planeIntersection = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("planeIntersection", 1);
    //  edgeStart=Eigen::Vector3d();
    // edgeEnd=Eigen::Vector3d();

}


void Field_edge_detector::subCB_Pose(const geometry_msgs::PoseWithCovarianceStamped &msg){

    pose_pos[0] =msg.pose.pose.position.x;
    pose_pos[1] =msg.pose.pose.position.y;
    pose_pos[2] =msg.pose.pose.position.z;

}

void Field_edge_detector::subCB_mapPoints(const svo_msgs::MapPoints::ConstPtr & msg)
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

    ros::Time begin = ros::Time::now();

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
        seg.setDistanceThreshold (0.02);//0.05
      //  seg.setMaxIterations(2000);


        //  pcl::ModelCoefficients::Ptr allCoeffis[3];
        std::vector<  pcl::ModelCoefficients::Ptr> allCoeffis;
        //
        //     double distToFrame[3];
        std::vector<double> distToFrame;
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        int n_found_plane = 0;
        while (n_found_plane<5) {



            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            if (cloud->points.size()>30){
                seg.setInputCloud (cloud);
                seg.segment (*inliers, *coefficients);
                ROS_INFO_STREAM("planeSIZE: "<<inliers->indices.size());
            }

            // ROS_INFO_STREAM(inliers->indices.size());
            if (coefficients->values.size()==0)
            {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            allCoeffis.push_back(coefficients);

            n_found_plane++;

            extract.setInputCloud (cloud);

            extract.setIndices (inliers);

            extract.setNegative (false);
            extract.filter (*cloud_p);
            //Plane Points = cloud_p


            Eigen::Vector4f xyz_centroid;

            // Estimate the XYZ centroid
            pcl::compute3DCentroid (*cloud_p, xyz_centroid);

            distToFrame.push_back((xyz_centroid-pose_pos).norm());

            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud.swap (cloud_f);


        }
        if (n_found_plane<=1)
                return ;

        if (false){
            ROS_INFO_STREAM("N Plane found:"<<n_found_plane);
            bool orthogonal_plane_found=false;
            for ( int i = 0; i < n_found_plane-1; i++) {
                for (int j = i+1; j < n_found_plane; j++) {

                    if (check_right_angle_of_Planes(allCoeffis[i],allCoeffis[j])){
                        orthogonal_plane_found=true;
                        IntersectPlanes(allCoeffis[i],allCoeffis[j]);
                    }
                }
            }
            if (!orthogonal_plane_found){

                ROS_ERROR_STREAM("no orthogonal plane found!");
                edgeStart = Eigen::Vector3d(0,0,0);
                edgeEnd= Eigen::Vector3d(0,0,0);
            }
            //Methode 2
        }else{
            ROS_INFO_STREAM("N Plane found:"<<n_found_plane);
            //create Pair
            std::vector<mypair> planePairVektor;

            for (int i = 0; i < distToFrame.size(); ++i) {
                        mypair newPair;
                        newPair.first=i;
                        newPair.second=distToFrame[i];
                        planePairVektor.push_back(newPair);
            }


            //sort pairvektor
            std::sort(planePairVektor.begin(),planePairVektor.end(),bind(&Field_edge_detector::mycompare,
                      this,_1,_2));







            for ( int i = 0; i < n_found_plane-1; i++) {
                //Poition nach der sortierung
                int originalIndex_i = planePairVektor[i].first;

                for (int j = i+1; j < n_found_plane; j++) {
                    int originalIndex_j = planePairVektor[j].first;



                    double angular_tolerance = 15 *3.14/180;
                    double maxAngle = 180*3.14/180  -angular_tolerance;
                    double minAngle = 0+angular_tolerance;
                    Eigen::Vector4f v_coeff_A(allCoeffis[originalIndex_i]->values.data());
                    Eigen::Vector4f v_coeff_B(allCoeffis[originalIndex_j]->values.data());
                    v_coeff_A[3]=0;
                    v_coeff_B[3]=0;



                    double  angle_rad = pcl::getAngle3D(v_coeff_A,v_coeff_B);

                    if (!(angle_rad<minAngle || angle_rad>maxAngle)){
                        ROS_INFO_STREAM("plane_nea1:"<<originalIndex_i);
                        ROS_INFO_STREAM("plane_nea2:"<<originalIndex_j);
                        ROS_INFO_STREAM(angle_rad*180/3.14);
                        ROS_INFO_STREAM("inserset nearest planes");

                        IntersectPlanes(allCoeffis[originalIndex_i],allCoeffis[originalIndex_j]);
                        goto     finishMethod;


                    }

                    // if (check_right_angle_of_Planes(allCoeffis[i],allCoeffis[j])){
                   // orthogonal_plane_found=true;
                    //  IntersectPlanes(allCoeffis[i],allCoeffis[j]);
                }
            }

/*
            // get the two nearst planes
            int i_plane1=-1; //nearest
            int i_plane2=-1; //second nearest
            double min1 = *std::max_element(distToFrame.begin(),distToFrame.end());


            for (int i = 0; i < distToFrame.size(); ++i) {
                if (distToFrame[i]<=min1) {
                    min1=distToFrame[i];
                    i_plane1=i;
                }
                ROS_INFO_STREAM("planes."<<distToFrame[i]);

            }

            double min2 = *std::max_element(distToFrame.begin(),distToFrame.end());
            for (int i = 0; i < distToFrame.size(); ++i) {

                if (i_plane1 != i ){
                    if (distToFrame[i]<=min2) {
                        min2=distToFrame[i];
                        i_plane2=i;
                    }
                }

            }

*/




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
    else {

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        //ne.setRadiusSearch (0.2);
        ne.setKSearch(10);
        // Compute the features
        ne.compute (*cloud_normals);
        ROS_INFO_STREAM("cloud: "<<cloud->points.size());

        ROS_INFO_STREAM("cloudNormal "<<cloud_normals->points.size());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_k->resize(cloud_normals->points.size());
        cloud_k->header.frame_id="/world";
        for (int i = 0; i < cloud_normals->points.size(); ++i) {
          //  ROS_INFO_STREAM(cloud_normals->points[i].curvature);
            if (cloud_normals->points[i].curvature>0.1){
                //ROS_INFO_STREAM(cloud_normals->points[i].curvature);
                cloud_k->points[i]= cloud->points[i];
            }
        }

        planeIntersection.publish(cloud_k);


        // Line Aproximation
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_LINE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.1);

        pcl::ModelCoefficients::Ptr line (new pcl::ModelCoefficients);

        seg.setInputCloud (cloud);
        seg.segment (       *inliers, *line);
        // Eigen::VectorXf lineCoeff(6);
        std::vector<float> lineCoeff =  line->values;
        ROS_INFO_STREAM("linieeeeeee");
        edgeStart = Eigen::Vector3d(lineCoeff[0],lineCoeff[1],lineCoeff[2]);
        edgeEnd= Eigen::Vector3d(lineCoeff[0]+lineCoeff[3],lineCoeff[1]+lineCoeff[4],lineCoeff[2]+lineCoeff[5]);
        ROS_INFO_STREAM(edgeStart);
        ROS_INFO_STREAM(edgeEnd);
    }

    finishMethod:
    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("time:");
    ROS_INFO_STREAM((end-begin).toSec());


}
  bool Field_edge_detector::mycompare ( const mypair l, const mypair r){ return l.first < r.first; }

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

void Field_edge_detector::IntersectPlanes(pcl::ModelCoefficients::Ptr coefficientsA,pcl::ModelCoefficients::Ptr coefficientsB){

    Eigen::Vector4f v_coeff_A(coefficientsA->values.data());
    Eigen::Vector4f v_coeff_B(coefficientsB->values.data());
    //    ROS_INFO_STREAM(v_coeff_A);
    //    ROS_INFO_STREAM(v_coeff_B);
    Eigen::VectorXf line;
    bool intSuccess=  pcl::planeWithPlaneIntersection(v_coeff_A,v_coeff_B,line,0.04);
    if (intSuccess){

        edgeStart = Eigen::Vector3d(line[0],line[1],line[2]);
        edgeEnd= Eigen::Vector3d(line[0]+line[3],line[1]+line[4],line[2]+line[5]);
    }else{
        edgeStart = Eigen::Vector3d(0,0,0);
        edgeEnd= Eigen::Vector3d(0,0,0);



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
    sb_mappoints = nh.subscribe("mapPoints",
                                3000,&Field_edge_detector::subCB_mapPoints,
                                this);

    sb_pose = nh.subscribe("svo/pose",
                           10,&Field_edge_detector::subCB_Pose,
                           this);
}


/*void mycallback(list<Point*> msg){

    ROS_INFO_STREAM(msg.size());
}*/

}//namespace fed
