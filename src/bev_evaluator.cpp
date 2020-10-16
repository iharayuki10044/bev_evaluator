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

    pc_subscriber = nh.subscribe("/cloud/dynamic", 10, &BEVEvaluator::pc_callback, this);
    odom_subscriber = nh.subscribe("/odom", 10, &BEVEvaluator::odom_callback, this);
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
            copy_people_data(current_people_data, pre_people_data);
			calucurate_affinematrix(current_position, current_yaw, pre_position, pre_yaw);
			transform_pointcloud_coordinate();
            calcurate_people_point(cloud_ptr);
            transform_position(current_people_data;
    		calcurate_people_vector(current_people_data, pre_people_data);

            std::cout << "generate image" << std::endl;
            bev_flow_image = generate_bev_iamge(pre_people_data);
			
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


        pc_callback_frag = false;
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
    odom_callback_flag = false;

	src_euqlid_3pts.points.resize(0);
	pt0.x = 0.0;
	pt0.y = 0.0;
	pt0.z = 0.0;
	pt1.x = 0.5 * RANGE;
	pt1.y = 0.0;
	pt1.z = 0.0;
	pt2.x = 0.0;
	pt2.y = 0.5 * RANGE;
	pt2.z = 0.0;
	src_euqlid_3pts.points.push_back(pt0);
	src_euqlid_3pts.points.push_back(pt1);
	src_euqlid_3pts.points.push_back(pt2);

}

void BEVEvaluator::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 input_pc;
    pc_seq = input_pc.header.seq;

    input_pc = *msg;
	pcl::fromROSMsg(input_pc, *pcl_input_pc);
    pc_callback_flag = true;！
}

void BEVEvaluator::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
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

void BEVEvaluator::copy_people_data(PeopleData &current, PeopleData &pre)
{
    pre = current;
}

void BEVEvaluator::calucurate_affinematrix(Eigen::Vector3d current_position, double current_yaw, Eigen::Vector3d pre_position, double pre_yaw)
{
	/* std::cout << "BEVImageGenerator::cropped_transformed_grid_img_generator" << std::endl; */
	double d_yaw = current_yaw - pre_yaw;
	d_yaw = atan2(sin(d_yaw), cos(d_yaw));
	Eigen::Matrix3d r;
	r = Eigen::AngleAxisd(-d_yaw, Eigen::Vector3d::UnitZ());

	Eigen::Matrix3d pre_yaw_rotation;
	pre_yaw_rotation = Eigen::AngleAxisd(-pre_yaw, Eigen::Vector3d::UnitZ());
	Eigen::Vector3d _current_position = pre_yaw_rotation * current_position;
	Eigen::Vector3d _pre_position = pre_yaw_rotation * pre_position;
	Eigen::Translation<double, 3> t(_pre_position - _current_position);

	affine_transform = t * r;
	/* std::cout << "affine transformation: \n" << affine_transform.translation() << "\n" << affine_transform.rotation().eulerAngles(0,1,2) << std::endl; */
}

void BEVEvaluator::transform_cloudpoint_coordinate(void)
{
    pcl::transformPointCloud(src_euqlid_3pts, dst_euqlid_3pts, affine_transform);
}

void BEVEvaluator::calcurate_peple_point(const CloudXYZIPtr& cloud_ptr )
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

cv::Mat BEVEvaluator::generate_bev_image(PeopleData &pre)
{
	std::cout << "generate_bev_image" << std::endl;

    cv::Mat flow_image;
	int img_size = GRID_NUM;
	cv::Mat flow_x = cv::Mat::zeros(img_size, img_size, CV_32F);
	cv::Mat flow_y = cv::Mat::zeros(img_size, img_size, CV_32F);

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
