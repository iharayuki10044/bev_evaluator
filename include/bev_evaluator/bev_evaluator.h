#ifndef __BEV_EVALUATOR_H
#define __BEV_EVALUATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <pcl/point_cloud.h>
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

#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


class BEVEvaluator
{
public:

    typedef pcl::PointXYZI PointXYZIN;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;
    typedef pcl::PointXYZ PointXYZ;
    typedef pcl::PointCloud<PointXYZ> CloudXYZ;
    typedef pcl::PointCloud<PointXYZ>::Ptr CloudXYZPtr;
    typedef pcl::PointCloud<PointN>::Ptr CloudNPtr;

class People
    {
    public:
        People(void);
        double point_x;
        double point_y;
        double length;
        int point_hit_num;
        double move_vector_x;
        double move_vector_y;
    private:
    };
    typedef std::vector<People> PeopleData;

class Gridcell : public
    {
    public:
        Gridcell(void);
        int index_x;
        int index_y;
        int hit_people_id;
        double move_vector_x;
        double move_vector_y;
        bool is_people_exist;
    private:
    };
    typedef std::vector<Gridcell> OccupancyGridMap;

    bool is_valid_point(double x, double y);

    int get_index_from_xy(const double x, const double y);
    int get_x_index_from_index(const int index);
    int get_y_index_from_index(const int index);
    double get_x_from_index(const int index);
    double get_y_from_index(const int index);

    void GroundTruth(void);
    void executor(void);
    void formatter(void);
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
    
    void calcurate_affinematrix(Eigen::Vector3d , double, Eigen::Vector3d, double);
    void transform_cloudpoint_coordinate(void);

    void calculation_people_vector(PeopleData&, PeopleData&);
    void initializer(void);
    void ogm_initializer(OccupancyGridMap&);
    cv::Mat generate_bev_image(PeopleData&);


private:
    bool pc_callback_frag = false;
    bool odom_callback_flag = false;
    bool person_position_callback = false;
    bool IS_SAVE_IMAGE = false;

    double current_yow;
    double pre_yow;
    double WIDTH;
    double WIDTH_2;
    int GRID_WIDTH;
    int GRID_WIDTH_2;
    int GRID_NUM;
    int PEOPLE_NUM;
    int pc_seq;

    std::string PKG_PATH; 
    std::string CMD_VEL_TOPIC;

    PeopleData current_people_data;
    PeopleData pre_people_data;
    OccupancyGridMap occupancy_grid_map;

    ros::NodeHandle nh;
	ros::Subscriber pc_subscriber;
	ros::Subscriber odom_subscriber;
	ros::Publisher bev_grid_publisher;

    Eigen::Vector3d current_position;
    Eigen::Vector3d pre_position;
    Eigen::Affine3d affine_transform;

	pcl::PointCloud<pcl::PointXYZ> src_euqlid_3pts;
	pcl::PointCloud<pcl::PointXYZ> dst_euqlid_3pts;
    //to transform pointcloud coordinate

    cv::Mat bev_flow_image;

}

#endif

