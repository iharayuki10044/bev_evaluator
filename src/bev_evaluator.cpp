#include "bev_evaluator/bev_evaluator.h"

BEVEvaluator::BEVEvaluator(void)
:nh("~")
{
	nh.param("THREHOLD_OF_DISTANCE_BTW_PC_AND_PERSON", THREHOLD_OF_DISTANCE_BTW_PC_AND_PERSON, {0.35});
	nh.param("RANGE", RANGE, {10.0});
    nh.param("GRID_NUM", GRID_NUM, {50});
	nh.param("GRID_WIDTH", GRID_WIDTH, {10});
    nh.param("Hz", Hz, {1.0});
	nh.param("WIDTH", WIDTH,{5});
    nh.param("FLOW_IMAGE_SIZE", FLOW_IMAGE_SIZE, {50});
    nh.param("PEOPLE_NUM", PEOPLE_NUM, {30});
    nh.param("SAVE_NUMBER", SAVE_NUMBER, {1});
	nh.param("PKG_PATH", PKG_PATH, {"/home/amsl/ros_catkin_ws/src/bev_evaluator/bev_img"});

    pc_subscriber = nh.subscribe("/cloud/dynamic", 10, &BEVEvaluator::pc_callback, this);
    gazebo_model_states_subscriber = nh.subscribe("/gazebo/model_states", 10, &BEVEvaluator::gazebo_model_states_callback, this);
	tracked_person_subscriber = nh.subscribe("/pedsim_visualizer/tracked_persons", 10, &BEVEvaluator::tracked_person_callback, this);
	flow_image_publisher = nh.advertise<sensor_msgs::Image>("/bev_true/true_flow_image", 10);
}

void BEVEvaluator::executor(void)
{
    formatter();
	int i=0;
	//int j=0;
    ros::Rate r(Hz);
	while(ros::ok()){

//        std::cout << "hello !!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        if(pc_callback_flag && gazebo_model_states_callback_flag && tracked_person_callback_flag){
            std::cout << "people data calculate" << std::endl;
    		calculate_people_vector(current_people_data, pre_people_data);

			std::cout << "ogm" << std::endl;
			ogm_initializer(occupancy_grid_map);
//			generate_occupancy_grid_map(pcl_input_pc, occupancy_grid_map);

			transform_person_coordinates_to_local(current_people_data);
			generate_occupancy_grid_map(pcl_input_pc, occupancy_grid_map);
			macthing_pc_to_person(current_people_data, occupancy_grid_map);

            std::cout << "generate image" << std::endl;
            bev_flow_image = generate_bev_image(current_people_data, occupancy_grid_map);

			bev_flow_image.convertTo(bev_flow_image, CV_8U, 255);

			std::cout << "pub img" << std::endl;
			cv::Mat flow_img;
			bev_flow_image.copyTo(flow_img);

			cv::Mat true_img;
			cv::flip(flow_img, true_img, -1);

			sensor_msgs::ImagePtr flow_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", true_img).toImageMsg();
			flow_img_msg->header.seq = pc_seq;

			flow_image_publisher.publish(flow_img_msg);

			std::cout << "complete pub img" << std::endl;
			i++;

			pc_callback_flag =false;
        	gazebo_model_states_callback_flag = false;
			tracked_person_callback_flag =false;

			std::cout<<""<<std::endl;

			// std::cout << "RESOLUTION = "<<RESOLUTION << std::endl;
			// std::cout << "GRID_WIDTH = "<<GRID_WIDTH << std::endl;
			// std::cout << "WIDTH = "<<WIDTH << std::endl;
        }

		// if(IS_SAVE_IMAGE){
		// 	std::vector<int> params(2);
		// 
		// 	const std::string folder_name = PKG_PATH + "/true_data_" + std::to_string(SAVE_NUMBER);
		// 	params[0] = CV_IMWRITE_PNG_COMPRESSION;
		// 	params[1] = 9;
		// 	struct stat statBuf;
		// 		if(stat(folder_name.c_str(), &statBuf) == 0){
		// 			std::cout << "exist dir" << std::endl;
		// 		}else{
		// 			std::cout << "mkdir" << std::endl;
		// 			if(mkdir(folder_name.c_str(), 0755) != 0){
		// 				std::cout << "mkdir error" << spcl_input_pc, td::endl;
		// 			}
		// 		}
		// 	/* cv::imwrite("/home/amsl/ros_catkin_ws/src/bev_converter/bev_img/data_" + std::to_string(SAVE_NUMBER) + "/" + "flow_" + std::to_string(i) + ".png", bev_flow, params); */
		// 	cv::imwrite(folder_name + "/" + "true_flow_" + std::to_string(i) + ".png", bev_flow_image, params);
		// 	/* std::cout << "SAVE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl; */

		// 	j++;

		// }
	r.sleep();
	ros::spinOnce();
	}
}

void BEVEvaluator::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */
    gazebo_model_states_callback_flag = false;
    pc_callback_flag = false;
	tracked_person_callback_flag = false;
	IS_SAVE_IMAGE = false;

	dt = 1.0 / Hz;
	GRID_NUM = GRID_WIDTH * GRID_WIDTH;
    WIDTH_2 = RANGE / 2.0;
    GRID_WIDTH_2 = GRID_WIDTH / 2.0;
	RESOLUTION = RANGE / GRID_WIDTH;

	current_people_data.resize(PEOPLE_NUM);
	pre_people_data.resize(PEOPLE_NUM);
	occupancy_grid_map.resize(GRID_NUM);
}

void BEVEvaluator::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud2 input_pc;
	input_pc = *msg;
    pcl::fromROSMsg(input_pc, *pcl_input_pc);
	pc_seq = input_pc.header.seq;
    pc_callback_flag = true;
}

int BEVEvaluator::find_num_from_name(const std::string &name,const std::vector<std::string> &states)
{
	int id = 1;
	int size = states.size();
    for(int i = 0; i < size; i++){
		if(states[i] == name){
		id = i;
		}
	}
	return id;
}

void BEVEvaluator::gazebo_model_states_callback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
	pre_position = current_position;
    if(msg->name.size() != 0){
	int robot_model_id = find_num_from_name("turtlebot3_burger",msg->name);
	current_position.x() = msg->pose[robot_model_id].position.x;
	current_position.y() = msg->pose[robot_model_id].position.y;
	current_yaw = tf::getYaw(msg->pose[robot_model_id].orientation);
	}
	gazebo_model_states_callback_flag = true;
}

void BEVEvaluator::tracked_person_callback(const pedsim_msgs::TrackedPersons::ConstPtr& msg)
{
	pedsim_msgs::TrackedPersons tracked_person = *msg;	
	for(int i=0;i<PEOPLE_NUM;i++){
	pre_people_data[i] = current_people_data[i];
	current_people_data[i].point_x = tracked_person.tracks[i].pose.pose.position.x;
	current_people_data[i].point_y = tracked_person.tracks[i].pose.pose.position.y;
	}
	tracked_person_callback_flag = true;
}

void BEVEvaluator::calculate_people_vector(PeopleData &cur, PeopleData &pre)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        cur[i].move_vector_x = cur[i].point_x - pre[i].point_x;
        cur[i].move_vector_y = cur[i].point_y - pre[i].point_y;
    }
}

void BEVEvaluator::initializer(void)
{
	std::cout << "initializer" << std::endl;
	current_position = Eigen::Vector3d::Zero();
	pre_position = Eigen::Vector3d::Zero();
	current_yaw = 0.0;
	pre_yaw = 0.0;
}

void BEVEvaluator::ogm_initializer(OccupancyGridMap& map)
{
//	std::cout << "initialize ogm" << std::endl;
	for(int i=0;i<GRID_NUM;i++){
		map[i].is_people_exist = false;
	}
}

cv::Mat BEVEvaluator::generate_bev_image(PeopleData& cur, OccupancyGridMap& map)
{
	std::cout << "generate_bev_image" << std::endl;

	cv::Mat flow_bgr;
	int img_size = GRID_WIDTH;
	cv::Mat flow_x = cv::Mat::zeros(img_size, img_size, CV_32F);
	cv::Mat flow_y = cv::Mat::zeros(img_size, img_size, CV_32F);
	for(int  i = 0; i < GRID_NUM; i++){
		if(map[i].is_people_exist && (map[i].hit_people_id < PEOPLE_NUM)){
			//int id = map[i].hit_people_id;
			// flow_x.at<float>(map[i].index_x, map[i].index_y) = cur[id].move_vector_x;
			// flow_y.at<float>(map[i].index_x, map[i].index_y) = cur[id].move_vector_y;
			flow_x.at<float>(map[i].index_x, map[i].index_y) = map[i].move_vector_x;
			flow_y.at<float>(map[i].index_x, map[i].index_y) = map[i].move_vector_y;
		
		}
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
	std::cout << "complete generate image!" << std::endl;
    return flow_bgr;
}

int BEVEvaluator::get_index_from_xy(const double x, const double y)
{
    int _x = floor(x / RESOLUTION + 0.5) + GRID_WIDTH_2;
    int _y = floor(y / RESOLUTION + 0.5) + GRID_WIDTH_2;
    return _y + _x * GRID_WIDTH;
}

int BEVEvaluator::get_x_index_from_index(const int index)
{
    return index / GRID_WIDTH;
}

int BEVEvaluator::get_y_index_from_index(const int index)
{
    return index % GRID_WIDTH;
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

void BEVEvaluator::generate_occupancy_grid_map(const CloudXYZIPtr& cloud_ptr, OccupancyGridMap& map)
{
	std::cout << "generate ogm" << std::endl;
	int cloud_size = cloud_ptr->points.size();
	ogm_initializer(map);

	int counter = 0;

	for(int i=0;i<cloud_size;i++){
		auto p = cloud_ptr->points[i];

		if(is_valid_point(p.x, p.y)){
			//std::cout << "receive point cloud!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			int index = get_index_from_xy(p.x, p.y);
			map[index].is_people_exist = true;
			map[index].index_x = get_x_index_from_index(index);
			map[index].index_y = get_y_index_from_index(index);
			map[index].pc_point_x = p.x;
			map[index].pc_point_y = p.y;
			map[index].hit_people_id = PEOPLE_NUM +1;
			
			counter++;

		}
	}
	std::cout << "complete ogm" << std::endl;
	std::cout << "counter = " << counter << std::endl;
}

double BEVEvaluator::calculate_2Ddistance(const double x, const double y, const double _x, const double _y)
{
	return sqrt(pow(x - _x, 2) + pow(y - _y, 2));
}

void BEVEvaluator::transform_person_coordinates_to_local(PeopleData &cur)
{
	double distance;
	double threhold = WIDTH_2 /cos(M_PI/4);
	Eigen::Matrix2d rotation_matrix;
//	rotation_matrix = Eigen::RotationBase<1, 2>::toRotationMatrix(current_yaw);
	rotation_matrix <<  cos(current_yaw), -sin(current_yaw),
						sin(current_yaw), cos(current_yaw);

	for(int i =0;i<PEOPLE_NUM; i++){
		cur[i].is_people_exist_in_local = false;
		distance = calculate_2Ddistance(cur[i].point_x, cur[i].point_y,current_position.x(), current_position.y());

		if(distance < threhold){
			Eigen::Vector2d local_position(0, 0);
			Eigen::Vector2d global_position(cur[i].point_x, cur[i].point_y);
			global_position.x() = global_position.x() -current_position.x();
			global_position.y() = global_position.y() -current_position.y();

			local_position = rotation_matrix * global_position;
			cur[i].local_point_x = local_position.x();
			cur[i].local_point_y = local_position.y();
			cur[i].is_people_exist_in_local = true;

			if(cur[i].is_people_exist_in_local){
				std::cout << "person is here id = "<< i <<"!"<< std::endl;
			}
			std::cout << "global_x = " << cur[i].point_x << std::endl;
			std::cout << "global_y = " << cur[i].point_y << std::endl;
			std::cout << "local_x = " << cur[i].local_point_x << std::endl;
			std::cout << "local_y = " << cur[i].local_point_y << std::endl;
		}
	}
}

void BEVEvaluator::macthing_pc_to_person(PeopleData &cur, OccupancyGridMap& map)
{
	for(int i = 0; i < PEOPLE_NUM; i++){
		if(cur[i].is_people_exist_in_local){
			int index;
			index = get_index_from_xy(cur[i].local_point_x, cur[i].local_point_y);
			std::cout << "id = " << i << std::endl;
			std::cout << "index = " << index << std::endl;
			map[index].hit_people_id = i;

			map[index].is_people_exist = true;
			map[index].index_x = get_x_index_from_index(index);
			map[index].index_y = get_y_index_from_index(index);
			map[index].move_vector_x = cur[i].move_vector_x;
			map[index].move_vector_y = cur[i].move_vector_y;

			double x ,y;
			x = get_x_from_index(index);
			y = get_y_from_index(index);
			std::cout << "x = " << x << std::endl;
			std::cout << "y = " << y << std::endl;
		}
	}
}

// void BEVEvaluator::macthing_pc_to_person(PeopleData &cur, OccupancyGridMap& map)
// {
// 	for(int i = 0; i < GRID_NUM; i++){
// 		if(map[i].is_people_exist){
// 			std::cout <<"start macthing"<< std::endl;
// 			double distance;
// 			for(int j = 0; j <PEOPLE_NUM; j++){
// 				if(cur[j].is_people_exist_in_local){
// 					std::cout << "suspision"<< std::endl;
// 					distance = calculate_2Ddistance(map[i].pc_point_x, map[i].pc_point_y,cur[j].local_point_x, cur[j].local_point_y);
// 					if(distance < THREHOLD_OF_DISTANCE_BTW_PC_AND_PERSON){
// 						map[i].hit_people_id = j;
// 						std::cout << "macthing success" << std::endl;
// 					}
// 				}
// 			}
// 		}
// 	}
// }

