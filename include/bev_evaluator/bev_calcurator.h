#ifndef __BEV_CALCURATOR_H
#define __BEV_CALCURATOR_H

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

//for SSIM MSE
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class BEVEvaluator
{
public:

    BEVCalcurator(void);

    void executor(void);
    void formatter(void);

private:
    bool bev_flow_estimator_callback_frag = false;
    bool bev_evaluator_callback_flag = false;
    bool IS_SAVE_IMAGE = false;

    int IMAGE_SIZE;
    int pc_seq;

    ros::NodeHandle nh;
	ros::Subscriber pc_subscriber;
	ros::Subscriber odom_subscriber;
	ros::Publisher bev_grid_publisher;

    Eigen::Vector3d current_position;
    Eigen::Vector3d pre_position;
    Eigen::Affine3d affine_transform;

    cv::Mat bev_flow_image;
    cv::Mat bev_groundtruth;

}
#endif