#include "bev_evaluator/groundtruth.h"

GroundTruth::GroundTruth(void)
:local_nh("~")
{
    local_nh.param("RESOLUTION", RESOLUTION, {});
    local_nh.param("WIDTH", WIDTH, {});
    local_nh.param("WIDTH_2", WIDTH_2,{});
    local_nh.param("GRID_WIDTH", GRID_WIDTH, {});
    local_nh.param("GRID_WIDTH", GRID_WIDTH_2, {});
    local_nh.param("GRID_NUM", GRID_NUM, {});
    local_nh.param("PEOPLE_NUM", PEOPLE_NUM, {30});

    pc_subscriber = nh.subscribe("/cloud/dynamic", 10, &GroundTruth::pc_callback, this);
    odom_subscriber = nh.subscribe("/odom", 10, &GroundTruth::odom_callback, this);
    //bev_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/bev/grid", 10);
}

void GroundTruth::executor(void)
{
    formatter();

    ros::Rate r(Hz);
	while(ros::ok()){

        
        if(pc_callback_flag && odm_callback_frag){
            std::cout << "people data calculate" << std::endl;
            copy_people_data(current_people_data, pre_people_data);
            calcurate_people_point(cloud_ptr);        
            transform_position(current_people_data;
    		calcurate_people_vector(current_people_data, pre_people_data);
        }

        if(IS_READY){
            std::cout << "generate image" << std::endl;
            generate_bev_iamge(pre_people_data);
        }

        pc_callback_frag = false;
        odom_callback_flag = false;

		r.sleep();
		ros::spinOnce();
    }

}

void GroundTruth::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */
    dt = 1.0 / Hz;
    grid_size = RANGE / GRID_NUM;
    cmd_vel_callback_flag = false;
    pc_callback_flag = false;
    odom_callback_flag = false;
}

void GroundTruth::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 input_pc;
    pc_seq = input_pc.header.seq;

    input_pc = *msg;
	pcl::fromROSMsg(input_pc, *pcl_input_pc);
    pc_callback_flag = true;
}

void GroundTruth::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
//calucurate robot coodinate
{
	if(USE_CMD_VEL){
		geometry_msgs::Twist cmd_vel = *msg;
		static bool first_flag = false;

		if(!first_flag){
			initializer();
			first_flag = true;
		}

		current_position.x() += cmd_vel.linear.x * dt * cos(current_position.z());
		current_position.y() += cmd_vel.linear.x * dt * sin(current_position.z());
		current_yaw += cmd_vel.angular.z * dt;

		while(current_yaw >= M_PI){
			current_yaw -= 2 * M_PI;
		}
		while(current_yaw <= -M_PI){
			current_yaw += 2 * M_PI;
		}

		if(step % STEP_BORDER == 0){
			initializer();
		}

		cmd_vel_callback_flag = true;
	}
}


void GroundTruth::copy_people_data(PeopleData &current, PeopleData &pre)
{
    pre = current;
}

void GroundTruth::calcurate_peple_point(const CloudXYZIPtr& cloud_ptr )
{
    std::cout << "--- calcurate people point ---" << std::endl;
    int cloud_size = cloud_ptr->points.size();

    Eigen::Matrix<double, 3, PEOPLE_NUM> people_point = Eigen::MatrixXD::Zero(3, PEOPLE_NUM);
    //(x, y, hit_num)*people_num Array, ZeroInit

    for(int i=0;i<cloud_size;i++){
        auto p = cloud_ptr->points[i];
        double id = cloud_ptr->intensity[i] - 1;
        people_point(1, id) += p.x;
        people_point(2, id) += p.y;
        people_point(3, id) += 1;
    }

    for(int i=0;i<PEOPLE_NUM;i++){
        
        if(people_point != 0){
            current[i].point_x = people_point(1, id) /people(3, id);
            current[i].point_y = people_point(2, id) /people(3, id);
            current[i].length = sqrt(pow(current[i].point_x, 2) +pow(current[i].point_y, 2));   
        }
    }
}

void GroundTruth::calcurate_people_vector(PeopleData &current, PeopleData &pre)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        pre[i].move_vector_x = current[i].point_x - pre[i].point.x;
        pre[i].move_vector_y = current[i].point_y - pre[i].point.y;
    }
}

void GroundTruth::transform_coordinate(PeopleData &current)
{
    double yow = atan2(current_position.x, current_position.y);
    double current_length = sqrt(pow(current_position.x, 2) +pow(current_position.y, 2));
    
    for(int i=0;i<PEOPLE_NUM;i++){
        current.point_x = current_length *cos(current_yow) + current[i].length *cos(current_yow +yow);
        current.point_y = current_length *sin(current_yow) + current[i].length *sin(current_yow +yow);
    }
}

// void GroundTruth::transform_coordinate(PeopleData &pre, double current_yow, double pre_yow)
// {
//     double d_yow = current_yow - pre_yow;
//     d_yow = atan2(sin(d_yow), cos(d_tow) );
//     Eigen::Matrix3d r;
//     r = Eigen::AngleAxisd(-d_yow, Eigen::Vector3d::UnitZ() );

//     Eigen::Matrix3d pre_yow_rotation;
//     pre_yow_rotation = Eigen::AngleAxisd(-pre_yow, Eigen::Vector3d::UnitZ());

//     Eigen::Vector3d _current_position = pre_yaw_rotation * current_position;
// 	Eigen::Vector3d _pre_position = pre_yaw_rotation * pre_position;
// 	Eigen::Translation<double, 3> t(_pre_position - _current_position);

// 	affine_transform = t * r;

// }

void GroundTruth::initializer(void)
{
//	std::cout << "initializer" << std::endl;
	current_position = Eigen::Vector3d::Zero();
	pre_position = Eigen::Vector3d::Zero();
	current_yaw = 0.0;
	pre_yaw = 0.0;
}

cv::Mat GroundTruth::generate_bev_image(PeopleData &pre)
{
    

}