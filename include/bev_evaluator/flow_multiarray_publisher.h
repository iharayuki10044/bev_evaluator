#ifndef __FLOW_MULTIARRAY_PUBLISHER_H
#define __FLOW_MULTIARRAY_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"
//#include "Eigen/Geometry"

#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <pedsim_msgs/TrackedPerson.h>
#include <pedsim_msgs/TrackedPersons.h>

#include <gazebo_msgs/ModelStates.h>

#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class FlowMultiArray
{
	public:

		class People
			{
			public:
				double point_x;
				double point_y;
				double length;
				double move_vector_x;
				double move_vector_y;
				double local_point_x;
				double local_point_y;
				bool is_people_exist_in_local;
			private:
			};
		typedef std::vector<People> PeopleData;

		class Gridcell
			{
			public:
				int index_x;
				int index_y;
				int hit_people_id;
				double move_vector_x;
				double move_vector_y;
				double pc_point_x;
				double pc_point_y;
				bool is_people_exist;
			private:
			};
		typedef std::vector<Gridcell> OccupancyGridMap;

		FlowMultiArray(void);

		bool is_valid_point(double, double);
		int find_num_from_name(const std::string& , const std::vector<std::string> &);
		int get_index_from_xy(const double, const double);
		int get_x_index_from_index(const int);
		int get_y_index_from_index(const int);
		int cropped_grid_num;
		double get_x_from_index(const int);
		double get_y_from_index(const int);
		double calculate_2Ddistance(const double, const double, const double, const double);

		void executor(void);
		void formatter(void);
		void model_states_callback(const gazebo_msgs::ModelStates::ConstPtr&);
		void tracked_person_callback(const pedsim_msgs::TrackedPersons::ConstPtr&);
		void obstacle_points_callback(const sensor_msgs::PointCloud2::ConstPtr&);
		void calculate_people_vector(PeopleData&, PeopleData&);
		void initializer(void);
		void ogm_initializer(OccupancyGridMap&);
		// cv::Mat generate_bev_image(PeopleData&, OccupancyGridMap&);
		void generate_flow(PeopleData&, OccupancyGridMap&);
		void transform_person_coordinates_to_local(PeopleData &);
		void macthing_pc_to_person(PeopleData&, OccupancyGridMap&);
		cv::Mat image_fliper(cv::Mat src_img);
		void image_cropper(cv::Mat&, cv::Mat&);
		void pointcloud_annotater(void);
		void grid_annotater(void);
		void move_grid_annotater(void);

	private:
		bool model_states_callback_flag = false;
		bool tracked_person_callback_flag = false;
		bool vp_callback_flag = false;
		bool IS_SAVE_IMAGE = false;
		bool IS_ANNOTATE_OCCUPANCY = false;

		double THREHOLD_OF_DISTANCE_BTW_PC_AND_PERSON;
		double current_yaw;
		double pre_yaw;
		double WIDTH;
		double WIDTH_2;
		double RANGE;
		double Hz;
		double RESOLUTION;
		double CYLINDER_RADIUS;
		double CYLINDER_RADIUS_ERROR;
		double dt;
		double grid_size;
		int FLOW_IMAGE_SIZE;
		int SAVE_NUMBER;
		int GRID_WIDTH;
		int GRID_WIDTH_2;
		int GRID_NUM;
		int PEOPLE_NUM;
		int pc_seq;
		int MANUAL_CROP_SIZE;

		PeopleData current_people_data;
		PeopleData pre_people_data;
		OccupancyGridMap occupancy_grid_map;

		ros::NodeHandle nh;
		ros::Subscriber model_states_subscriber;
		ros::Subscriber tracked_person_subscriber;
		ros::Subscriber obstacle_points_subscriber;

		ros::Publisher flow_image_publisher;
		ros::Publisher current_yaw_publisher;
		ros::Publisher flow_multiarray_x_publisher;
		ros::Publisher flow_multiarray_y_publisher;

		geometry_msgs::Pose2D current_pose2D;

		Eigen::Vector3d current_position;
		Eigen::Vector3d pre_position;

		pcl::PointXYZ pt0, pt1, pt2;
		pcl::PointCloud<pcl::PointXYZ> src_euqlid_3pts;
		pcl::PointCloud<pcl::PointXYZ> dst_euqlid_3pts;
		pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud {new pcl::PointCloud<pcl::PointXYZI>};

		std_msgs::Float32MultiArray flow_multiarray_x;
		std_msgs::Float32MultiArray flow_multiarray_y;
		cv::Mat bev_flow_image;

};

#endif //__FLOW_MULTIARRAY_PUBLISHER_H
