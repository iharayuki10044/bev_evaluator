#ifndef __GROUNDTRUTH_IMAGE_GENERATOR_H
#define __GROUNDTRUTH_IMAGE_GENERATOR_H

#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>
#include <opencv2/core/base.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

class GroundtruthImageGenerator
{
public:
    GroundtruthImageGenerator(void);

    void executor(void);
    void formatter(void);

    cv::Mat image_generator();

private:



}
