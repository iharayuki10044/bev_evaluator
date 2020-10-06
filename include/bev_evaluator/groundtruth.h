#ifndef __GROUNDTRUTH_H
#define __GROUNDTRUTH_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

class GroundTruth
{
public:

    typedef pcl::PointXYZI PointXYZIN;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;
    typedef pcl::PointXYZ PointXYZ;
    typedef pcl::PointCloud<PointXYZ> CloudXYZ;
    typedef pcl::PointCloud<PointXYZ>::Ptr CloudXYZPtr;
    typedef pcl::PointCloud<PointN>::Ptr CloudNPtr;

    GroundTruth(void);
    executor(void);
    pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    odom_callback(const nav_msgs::OdometryConstPtr &msg);

    class People
    {
    public:
        People(void);

        double point_x;
        double point_y;
        int point_hit_num;
        double move_vector_x;
        double move_vector_y;

    private:
    };
    typedef std::vector<People> PeopleData;

    calculation_peple_point(const cloud_ptr);
    calculation_people_vector(PeopleData&, PeopleData&);
    copy_people_data(PeopleData&, PeopleData&);

    double time_seq;

private:
    bool pc_callback_frag = false;
    bool odom_callback_flag = false;

    double RESOLUTION;
    double WIDTH;
    double WIDTH_2;
    int GRID_WIDTH;
    int GRID_WIDTH_2;
    int GRID_NUM;
    int PEOPLE_NUM;

    ros::NodeHandle n;
    ros::NodeHandle nh;
	ros::Subscriber pc_subscriber;
	ros::Subscriber odom_subscriber;
	ros::Publisher bev_grid_publisher;

    PeopleData people_data_new;
    PeopleData people_data_old;

}

#endif // __GROUNDTRUTH


// while(1){
//     people_data_new[i].point_x = 5664;

//     people_data_old = people_data_new;
// }