#include "bev_evaluator/flow_multiarray_publisher.h"

FlowMultiArray::FlowMultiArray(void)
:nh("~")
{
	nh.param("RANGE", RANGE, {10.0});
	nh.param("GRID_WIDTH", GRID_WIDTH, {50});
	nh.param("MANUAL_CROP_SIZE", MANUAL_CROP_SIZE, {5});
    nh.param("Hz", Hz, {10.0});
    nh.param("PEOPLE_NUM", PEOPLE_NUM, {30});
    nh.param("IS_ANNOTATE_OCCUPANCY", IS_ANNOTATE_OCCUPANCY, {true});
    nh.param("CYLINDER_RADIUS", CYLINDER_RADIUS, {0.212});
    nh.param("CYLINDER_RADIUS_ERROR", CYLINDER_RADIUS_ERROR, {0.050});

    model_states_subscriber = nh.subscribe("/gazebo/model_states", 10, &FlowMultiArray::model_states_callback, this);
	tracked_person_subscriber = nh.subscribe("/pedsim_visualizer/tracked_persons", 10, &FlowMultiArray::tracked_person_callback, this);
    obstacle_points_subscriber = nh.subscribe("/velodyne_obstacles", 10, &FlowMultiArray::obstacle_points_callback, this);

	flow_image_publisher = nh.advertise<sensor_msgs::Image>("/bev_true/true_flow_image", 10);
	flow_multiarray_x_publisher = nh.advertise<std_msgs::Float32MultiArray>("/bev/flow_array_x", 10);
	flow_multiarray_y_publisher = nh.advertise<std_msgs::Float32MultiArray>("/bev/flow_array_y", 10);
}

void FlowMultiArray::executor(void)
{
    formatter();
    ros::Rate r(Hz);
	while(ros::ok()){

        if(model_states_callback_flag && tracked_person_callback_flag){
			// std::cout << "people data calculate" << std::endl;
    		calculate_people_vector(current_people_data, pre_people_data);

			// std::cout << "ogm" << std::endl;
			ogm_initializer(occupancy_grid_map);
			transform_person_coordinates_to_local(current_people_data);
			macthing_pc_to_person(current_people_data, occupancy_grid_map);

			///////////////////
			if(IS_ANNOTATE_OCCUPANCY){
				pointcloud_annotater();
				grid_annotater();
				move_grid_annotater();
			}
			///////////////////

			// std::cout << "generate flow" << std::endl;
            // bev_flow_image = generate_bev_image(current_people_data, occupancy_grid_map);
			generate_flow(current_people_data, occupancy_grid_map);
			flow_multiarray_x_publisher.publish(flow_multiarray_x);
			flow_multiarray_y_publisher.publish(flow_multiarray_y);

			// std::cout << "complete pub img" << std::endl;


        	model_states_callback_flag = false;
			tracked_person_callback_flag =false;
        }


	r.sleep();
	ros::spinOnce();
	}
}

void FlowMultiArray::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */
    model_states_callback_flag = false;
	tracked_person_callback_flag = false;
    vp_callback_flag = false;

	dt = 1.0 / Hz;
	GRID_NUM = GRID_WIDTH * GRID_WIDTH;
    WIDTH_2 = RANGE / 2.0;
    GRID_WIDTH_2 = GRID_WIDTH / 2.0;
	RESOLUTION = RANGE / GRID_WIDTH;
	cropped_grid_num = GRID_WIDTH - 2 * MANUAL_CROP_SIZE;

	current_people_data.resize(PEOPLE_NUM);
	pre_people_data.resize(PEOPLE_NUM);
	occupancy_grid_map.resize(GRID_NUM);

	flow_multiarray_x.data.resize(0);
	flow_multiarray_y.data.resize(0);
	int multiarray_size = cropped_grid_num * cropped_grid_num;
	for(int i = 0; i < multiarray_size; i++){
		flow_multiarray_x.data.push_back(0.0);
		flow_multiarray_y.data.push_back(0.0);
	}
	flow_multiarray_x.layout.data_offset = (uint32_t)cropped_grid_num;
	flow_multiarray_y.layout.data_offset = (uint32_t)cropped_grid_num;
	flow_multiarray_x.layout.dim.resize(1);
	flow_multiarray_y.layout.dim.resize(1);
	flow_multiarray_x.layout.dim[0].size = 2;
	flow_multiarray_y.layout.dim[0].size = 2;
}

int FlowMultiArray::find_num_from_name(const std::string &name,const std::vector<std::string> &states)
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

void FlowMultiArray::model_states_callback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
	pre_position = current_position;
    if(msg->name.size() != 0){
		int robot_model_id = find_num_from_name("turtlebot3_burger",msg->name);
		current_position.x() = msg->pose[robot_model_id].position.x;
		current_position.y() = msg->pose[robot_model_id].position.y;
		current_yaw = tf::getYaw(msg->pose[robot_model_id].orientation);
	}
	model_states_callback_flag = true;
}

void FlowMultiArray::tracked_person_callback(const pedsim_msgs::TrackedPersons::ConstPtr& msg)
{
	pedsim_msgs::TrackedPersons tracked_person = *msg;	
	for(int i=0;i<PEOPLE_NUM;i++){
		pre_people_data[i] = current_people_data[i];
		current_people_data[i].point_x = tracked_person.tracks[i].pose.pose.position.x;
		current_people_data[i].point_y = tracked_person.tracks[i].pose.pose.position.y;
	}
	tracked_person_callback_flag = true;
}

void FlowMultiArray::obstacle_points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 input_pc = *msg;
    pc_seq = input_pc.header.seq;

	pcl::fromROSMsg(input_pc, *pcl_pointcloud);
    vp_callback_flag = true;
}

void FlowMultiArray::calculate_people_vector(PeopleData &cur, PeopleData &pre)
{
    for(int i=0;i<PEOPLE_NUM;i++){
		// Eigen::Vector2d cur_local_people_position;
		// Eigen::Vector2d pre_local_people_position;
		// cur_local_people_position.x() = cur[i].point_x - current_position.x();
		// cur_local_people_position.y() = cur[i].point_y - current_position.x();
		// pre_local_people_position.x() = pre[i].point_x - pre_position.x();
		// pre_local_people_position.y() = pre[i].point_y - pre_position.x();
        //
        // cur[i].move_vector_y = (cur_local_people_position.x() - pre_local_people_position.x()) * -1;
        // cur[i].move_vector_x = cur_local_people_position.y() - pre_local_people_position.y();
		//
		cur[i].move_vector_y = (cur[i].point_x - pre[i].point_x) * -1;
		cur[i].move_vector_x = (cur[i].point_y - pre[i].point_y) * -1;
		// //rotate
		// cur[i].move_vector_x = cur[i].move_vector_x * cos(current_yaw);
		// cur[i].move_vector_y = cur[i].move_vector_y * sin(current_yaw);
    }
}

void FlowMultiArray::initializer(void)
{
	// std::cout << "initializer" << std::endl;
	current_position = Eigen::Vector3d::Zero();
	pre_position = Eigen::Vector3d::Zero();
	current_yaw = 0.0;
	pre_yaw = 0.0;
}

void FlowMultiArray::ogm_initializer(OccupancyGridMap& map)
{
//	std::cout << "initialize ogm" << std::endl;
	for(int i=0;i<GRID_NUM;i++){
		map[i].is_people_exist = false;
		map[i].hit_people_id = -1;
		map[i].move_vector_x = 0.0;
		map[i].move_vector_y = 0.0;
	}
}

void FlowMultiArray::generate_flow(PeopleData& cur, OccupancyGridMap& map)
{
	// std::cout << "generate_bev_image" << std::endl;

	cv::Mat flow_bgr;
	int img_size = GRID_WIDTH;
	cv::Mat flow_x = cv::Mat::zeros(img_size, img_size, CV_32FC1);
	cv::Mat flow_y = cv::Mat::zeros(img_size, img_size, CV_32FC1);
	for(int  i = 0; i < GRID_NUM; i++){
		// if(map[i].is_people_exist && (map[i].hit_people_id < PEOPLE_NUM)){
		if(map[i].is_people_exist){
			flow_x.at<float>(map[i].index_x, map[i].index_y) = map[i].move_vector_x;
			flow_y.at<float>(map[i].index_x, map[i].index_y) = map[i].move_vector_y;
		}
	}

	// std::cout << "flip" << std::endl;
	cv::Mat flipped_flow_x = cv::Mat::zeros(img_size, img_size, CV_32FC1);
	cv::Mat flipped_flow_y = cv::Mat::zeros(img_size, img_size, CV_32FC1);
	flipped_flow_x = image_fliper(flow_x);
	flipped_flow_y = image_fliper(flow_y);
	
	// std::cout << "crop" << std::endl;
	cv::Mat cropped_flow_x = cv::Mat::zeros(cv::Size(cropped_grid_num, cropped_grid_num), CV_32FC1);
	cv::Mat cropped_flow_y = cv::Mat::zeros(cv::Size(cropped_grid_num, cropped_grid_num), CV_32FC1);
	image_cropper(flipped_flow_x, cropped_flow_x);
	image_cropper(flipped_flow_y, cropped_flow_y);

	// std::cout << "array" << std::endl;
	int i = 0;
	std::cout << "flow_multiarray_x.data[i]" << std::endl;
	for(int row = 0; row < cropped_grid_num; row++){
		for(int col = 0; col < cropped_grid_num; col++){
			flow_multiarray_x.data[i] = cropped_flow_x.at<float>(row, col);
			flow_multiarray_y.data[i] = cropped_flow_x.at<float>(row, col);
			// printf("%.3f ", flow_multiarray_x.data[i]);
			i++;
		}
		// printf("\n");
	}

	// for debug
	// std::cout << "image" << std::endl;
	cv::Mat magnitude, angle;
	cv::cartToPolar(flipped_flow_x, flipped_flow_y, magnitude, angle, true);
	cv::Mat hsv_planes[3];
	hsv_planes[0] = angle;
	cv::normalize(magnitude, magnitude, 0, 1, cv::NORM_MINMAX);
	hsv_planes[1] = magnitude;
	hsv_planes[2] = cv::Mat::ones(magnitude.size(), CV_32F);
	cv::Mat hsv;
	cv::merge(hsv_planes, 3, hsv);
	cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);
	flow_bgr.convertTo(bev_flow_image, CV_8U, 255);
	sensor_msgs::ImagePtr flow_img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", bev_flow_image).toImageMsg();
	flow_image_publisher.publish(flow_img_msg);

	// std::cout << "complete generate flow!" << std::endl;
}

void FlowMultiArray::image_cropper(cv::Mat& src_image, cv::Mat& dst_image)
{
    /* std::cout << "BEVFlowEstimator::image_cropper" << std::endl; */
	cv::Mat original_image = src_image.clone();
    cv::Rect roi(cv::Point(MANUAL_CROP_SIZE, MANUAL_CROP_SIZE), cv::Size(cropped_grid_num, cropped_grid_num));
    cv::Mat roi_image = original_image(roi);
    dst_image = roi_image.clone();
}


int FlowMultiArray::get_index_from_xy(const double x, const double y)
{
    int _x = floor(x / RESOLUTION + 0.5) + GRID_WIDTH_2;
    int _y = floor(y / RESOLUTION + 0.5) + GRID_WIDTH_2;
	// std::cout << "pos_x = " << x << " pos_y = " << y <<std::endl;

	// std::cout << "y(" << _y <<") * GRID_WIDTH(" << GRID_WIDTH <<") + x("<< _x << ") = index"<< _y * GRID_WIDTH + _x  << std::endl;
    return _y * GRID_WIDTH + _x ;

	// int _x = floor( (x + WIDTH_2) / RESOLUTION + 0.5);
    // int _y = floor( (y + WIDTH_2) / RESOLUTION + 0.5);
    // return _x + (_y - 1) * GRID_WIDTH;
}

int FlowMultiArray::get_x_index_from_index(const int index)
{
    return index % GRID_WIDTH;
}

int FlowMultiArray::get_y_index_from_index(const int index)
{
    return index / GRID_WIDTH;
}

double FlowMultiArray::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

double FlowMultiArray::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

bool FlowMultiArray::is_valid_point(double x, double y)
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

double FlowMultiArray::calculate_2Ddistance(const double x, const double y, const double _x, const double _y)
{
	return sqrt(pow(x - _x, 2) + pow(y - _y, 2));
}

void FlowMultiArray::transform_person_coordinates_to_local(PeopleData &cur)
{
	double distance;
	double threhold = WIDTH_2 /cos(M_PI/4);

	for(int i =0;i<PEOPLE_NUM; i++){
		cur[i].is_people_exist_in_local = false;
		distance = calculate_2Ddistance(cur[i].point_x, cur[i].point_y,current_position.x(), current_position.y());

		if(distance < threhold){
			cur[i].local_point_x = cur[i].point_x - current_position.x();
			cur[i].local_point_y = cur[i].point_y - current_position.y();
			cur[i].is_people_exist_in_local = true;

			if(cur[i].is_people_exist_in_local){
				// std::cout << "person is here id = "<< i <<"!"<< std::endl;
			}
			// std::cout << "global_x = " << cur[i].point_x << std::endl;
			// std::cout << "global_y = " << cur[i].point_y << std::endl;
			// std::cout << "local_x = " << cur[i].local_point_x << std::endl;
			// std::cout << "local_y = " << cur[i].local_point_y << std::endl;
		}
	}
}

void FlowMultiArray::macthing_pc_to_person(PeopleData &cur, OccupancyGridMap& map)
{
	for(int i = 0; i < PEOPLE_NUM; i++){
		bool flag = is_valid_point(cur[i].local_point_x, cur[i].local_point_y);

		if(cur[i].is_people_exist_in_local && flag){
			int index;
			// std::cout << "=====================" << std::endl;
			// std::cout << "id = " << i << std::endl;
			index = get_index_from_xy(cur[i].local_point_x, cur[i].local_point_y);
			map[index].hit_people_id = i;
			map[index].is_people_exist = true;
			map[index].index_x = get_x_index_from_index(index);
			map[index].index_y = get_y_index_from_index(index);
			map[index].move_vector_x = cur[i].move_vector_x;
			map[index].move_vector_y = cur[i].move_vector_y;

			double x ,y;
			x = get_x_from_index(index);
			y = get_y_from_index(index);
			
			// std::cout << "index = " << index << std::endl;
			// std::cout << "x = " << x << std::endl;
			// std::cout << "y = " << y << std::endl;
			// std::cout << "index_x = " << map[index].index_x <<std::endl;
			// std::cout << "index_y = " << map[index].index_y <<std::endl;
			// std::cout << "=====================" << std::endl;
		}
	}
}

cv::Mat FlowMultiArray::image_fliper(cv::Mat input_img)
{
    cv::Mat output_img = cv::Mat::zeros(GRID_WIDTH, GRID_WIDTH, CV_32F);
	cv::rotate(input_img, output_img, cv::ROTATE_90_CLOCKWISE);
    //
	// cv::flip(output_img, input_img,  0);
	// cv::flip(input_img, output_img, 1);

    return output_img;
    // return input_img;
}



void FlowMultiArray::pointcloud_annotater(void)
{
	for(auto& pt : pcl_pointcloud->points){
		pt.intensity = -1.0;
		for(int ped_id = 0; ped_id < PEOPLE_NUM; ped_id++){
			double ped_local_x = current_people_data[ped_id].local_point_x;
			double ped_local_y = current_people_data[ped_id].local_point_y;
			
			double distance_x_pow = (pt.x - ped_local_x) * (pt.x - ped_local_x);
			double distance_y_pow = (pt.y - ped_local_y) * (pt.y - ped_local_y);
			double distance_from_ped = sqrt(distance_x_pow + distance_y_pow);
			if((CYLINDER_RADIUS - CYLINDER_RADIUS_ERROR < distance_from_ped) && (distance_from_ped < CYLINDER_RADIUS + CYLINDER_RADIUS_ERROR)){
				// std::cout << "pointcloud annotated!!!" << std::endl;
				pt.intensity = (float)ped_id;
				break;
			}
		}
	}
}



void FlowMultiArray::grid_annotater(void)
{
	for(auto& pt : pcl_pointcloud->points){
		int hit_index = get_index_from_xy(pt.x, pt.y);
		occupancy_grid_map[hit_index].hit_people_id = pt.intensity;
	}
}


void FlowMultiArray::move_grid_annotater(void)
{
	typedef struct PreCheckID{
		int map_id;
		int ped_id;
	}PRE_CHECK_ID;
	std::vector<PreCheckID> occupancy_mapid_list(0);

	for(int i = 0; i < GRID_NUM; i++){
		if(occupancy_grid_map[i].hit_people_id >= 0){
			PreCheckID check_occupancy_id;
			check_occupancy_id.map_id = i;
			check_occupancy_id.ped_id = occupancy_grid_map[i].hit_people_id;
			occupancy_mapid_list.push_back(check_occupancy_id);
		}
	}

	std::vector<int> exist_id_list(0);
	for(int i = 0; i < GRID_NUM; i++){
		if(occupancy_grid_map[i].is_people_exist){
			exist_id_list.push_back(i);
		}
	}

	for(int i = 0; i < exist_id_list.size(); i++){
		int exist_map_id = exist_id_list[i];
		int target_ped_id = occupancy_grid_map[exist_map_id].hit_people_id;
		for(int i2  = 0; i2 < occupancy_mapid_list.size(); i2++){
			if(occupancy_mapid_list[i2].ped_id == target_ped_id){
				int target_index = occupancy_mapid_list[i2].map_id;
				occupancy_grid_map[target_index].move_vector_x = occupancy_grid_map[exist_map_id].move_vector_x;
				occupancy_grid_map[target_index].move_vector_y = occupancy_grid_map[exist_map_id].move_vector_y;
				occupancy_grid_map[target_index].is_people_exist = true;
			}
		}
	}
}




