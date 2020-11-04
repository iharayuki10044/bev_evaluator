#include "bev_evaluator/bev_evaluator.h"

BEVEvaluator::BEVEvaluator(void)
:nh("~")
{
	nh.param("RANGE", RANGE, {10.0});
    nh.param("GRID_NUM", GRID_NUM, {50});
    nh.param("Hz", Hz, {100.0});
    nh.param("FLOW_IMAGE_SIZE", FLOW_IMAGE_SIZE, {50});
    nh.param("PEOPLE_NUM", PEOPLE_NUM, {30});
    nh.param("SAVE_NUMBER", SAVE_NUMBER, {1});
	nh.param("CMD_VEL_TOPIC", CMD_VEL_TOPIC, {"/cmd_vel"});
	nh.param("PKG_PATH", PKG_PATH, {"/home/amsl/ros_catkin_ws/src/bev_evaluator/bev_img"});

	GRID_WIDTH = WIDTH / RESOLUTION;
    GRID_NUM = GRID_WIDTH * GRID_WIDTH;
    WIDTH_2 = WIDTH / 2.0;
    GRID_WIDTH_2 = GRID_WIDTH / 2.0;

    pc_subscriber = nh.subscribe("/cloud/dynamic", 10, &BEVEvaluator::pc_callback, this);
    odom_subscriber = nh.subscribe("/odom", 10, &BEVEvaluator::odom_callback, this);
	tracked_person_subscriber = nh.subscribe("/", 10, &BEVEvaluator::tracked_person_callback, this);
	flow_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/flow_image", 10);
}

void BEVEvaluator::executor(void)
{
    formatter();
	int i=0;
	int j=0;
    ros::Rate r(Hz);
	while(ros::ok()){

        
        if(pc_callback_flag && odm_callback_frag){
            std::cout << "people data calculate" << std::endl;
    		calcurate_people_vector(current_people_data, pre_people_data);
			generate_occupancy_grid_map(cloud_ptr, occupancy_grid_map);

            std::cout << "generate image" << std::endl;
            bev_flow_image = generate_bev_iamge(pre_people_data, occupancy_grid_map);
			
			cv::flip(bev_flow_image, CV_8U, 255);
			bev_flow.convertTo(bev_flow_image, CV_8U, 255);

			std::cout << "pub img" << std::endl;
			cv::Mat flow_img;
			bev_flow.copyTo(flow_img);
			sensor_msgs::ImagePtr flow_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", flow_img).toImageMsg();
			flow_img_msg->header.seq = pc_seq;
			flow_image_publisher.publish(flow_img_msg);

			i++;
        }

		if(IS_SAVE_IMAGE){
			std::vector<int> params(2);
					// .png
			const std::string folder_name = PKG_PATH + "/data_" + std::to_string(SAVE_NUMBER);
			params[0] = CV_IMWRITE_PNG_COMPRESSION;
			params[1] = 9;
			struct stat statBuf;
				if(stat(folder_name.c_str(), &statBuf) == 0){
					std::cout << "exist dir" << std::endl;
				}else{
					std::cout << "mkdir" << std::endl;
					if(mkdir(folder_name.c_str(), 0755) != 0){
						std::cout << "mkdir error" << std::endl;
					}
				}
			/* cv::imwrite("/home/amsl/ros_catkin_ws/src/bev_converter/bev_img/data_" + std::to_string(SAVE_NUMBER) + "/" + "flow_" + std::to_string(i) + ".png", bev_flow, params); */
			cv::imwrite(folder_name + "/" + "flow_" + std::to_string(i) + ".png", bev_flow, params);
			/* std::cout << "SAVE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl; */
			j++;

		}

		pc_callback_flag ＝false;
        odom_callback_flag = false;

		r.sleep();
		ros::spinOnce();
    }

}

void BEVEvaluator::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */
    dt = 1.0 / Hz;
    grid_size = RANGE / GRID_NUM;
    cmd_vel_callback_flag = false;
    pc_callback_flag = false;
	IS_SAVE_IMAGE = false;
}

void BEVEvaluator::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 input_pc;
    pc_seq = input_pc.header.seq;

    input_pc = *msg;
	pcl::fromROSMsg(input_pc, *pcl_input_pc);
    pc_callback_flag = true;
}

void BEVEvaluator::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr　&msg)
//calcurate robot coodinate
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

void BEVEvaluator::person_position_callback(const pedsim_msgs::TrackedPersons::Constptr& msg)
{
	pedsim_msgs::TrackedPersons tracked_person;
	for(int i=0;i<PEOPLE_NUM;i++){
	pre_people_data[i] = current_people_data[i];
	current_people_data[i].x = tracked_person.pose.x;
	current_people_data[i].y = tracked_person.pose.y;
	}
	person_position_callback = true;
}

void BEVEvaluator::transform_cloudpoint_coordinate(void)
{
    pcl::transformPointCloud(src_euqlid_3pts, dst_euqlid_3pts, affine_transform);
}

void BEVEvaluator::calcurate_people_vector(PeopleData &current, PeopleData &pre)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        pre[i].move_vector_x = current[i].point_x - pre[i].point.x;
        pre[i].move_vector_y = current[i].point_y - pre[i].point.y;
    }
}

void BEVEvaluator::initializer(void)
{
//	std::cout << "initializer" << std::endl;
	current_position = Eigen::Vector3d::Zero();
	pre_position = Eigen::Vector3d::Zero();
	current_yaw = 0.0;
	pre_yaw = 0.0;
}

void BEVEvaluator::ogm_initializer(OccupancyGridMap& map)
{
	for(int i=0;i<GRID_NUM;i++){
		map[i].is_people_exist = false;
	}
}

cv::Mat BEVEvaluator::generate_bev_image(PeopleData& pre, OccupancyGridMap& map)
{
	std::cout << "generate_bev_image" << std::endl;

    cv::Mat flow_image;
	int img_size = GRID_NUM;
	cv::Mat flow_x = cv::Mat::zeros(img_size, img_size, CV_32F);
	cv::Mat flow_y = cv::Mat::zeros(img_size, img_size, CV_32F);

	for(int  i = 0; i < GRID_NUM; i++){
		if(!map[i].is_people_exist){
			continue;
		}
		int id = map.hit_people_id;
		flow_y.at<float>(map[i].index_x, map[i].index_y) = pre[id].move_vector_x;
		flow_x.at<float>(map[i].index_x, map[i].index_y) = pre[id].move_vector_y;
	}

    //そのまま使える
	cv::Mat magnitude, angle;
	cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);

	cv::Mat hsv_planes[3];
	hsv_planes[0] = angle;
	cv::normalize(magnitude, magnitude, 0, 1, cv::NORM_MINMAX);
	/* cv::normalize(magnitude, magnitude, 1.0, 0.0, cv::NORM_L1); */
	hsv_planes[1] = magnitude;
	hsv_planes[2] = cv::Mat::ones(magnitude.size(), CV_32F);
	
	cv::Mat hsv;
	cv::merge(hsv_planes, 3, hsv);
	cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);

	IS_SAVE_IMAGE = true;

    return flow_image;

}

int BEVEvaluator::get_index_from_xy(const double x, const double y)
{
    int _x = floor(x / RESOLUTION + 0.5) + GRID_WIDTH_2;
    int _y = floor(y / RESOLUTION + 0.5) + GRID_WIDTH_2;
    return _y * GRID_WIDTH + _x;
}

int BEVEvaluator::get_x_index_from_index(const int index)
{
    return index % GRID_WIDTH;
}

int BEVEvaluator::get_y_index_from_index(const int index)
{
    return index / GRID_WIDTH;
}

double BEVEvaluator::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

double BEVEvaluator::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

bool BEVEvaluator::is_valid_point(double x, double y)
{
    int index = get_index_from_xy(x, y);
    if(x < -WIDTH_2 || x > WIDTH_2 || y < -WIDTH_2 || y > WIDTH_2){
        return false;
    }else if(index < 0 || GRID_NUM <= index){
        return false;
    }else{
        return true;
    }
}

void BEVEvaluator::generate_occupancy_grid_map(const CloudXYZINPtr& cloud_ptr, OccupancyGridMap& map)
{
	int cloud_size = cloud_ptr->points.size();
	for(int i=0;i<cloud_size;i++){
		auto p = cloud_ptr->points[i];
		if(!is_valid_point(p.x, p.y)){
			continue;
		}

		int index = get_index_from_xy(p.x, p.y);
		map[index].is_people_exist = true;
		map[index].hit_people_id = cloud_ptr->intensity;
		map[index].index_x = get_x_index_from_index(index);
		map[index].index_y = get_y_index_from_index(index);
	}

}