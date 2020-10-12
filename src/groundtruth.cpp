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
    pc_callback_flag = true;！

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

        if(yow < 0){
            yow += 2 *M_PI;
        }

    double current_length = sqrt(pow(current_position.x, 2) +pow(current_position.y, 2));
    
    for(int i=0;i<PEOPLE_NUM;i++){
        current[i].point_x = current_length *cos(current_yow) + current[i].length *cos(current_yow +yow);
        current[i].point_y = current_length *sin(current_yow) + current[i].length *sin(current_yow +yow);
    }
}

void GroundTruth::initializer(void)
{
//	std::cout << "initializer" << std::endl;
	current_position = Eigen::Vector3d::Zero();
	pre_position = Eigen::Vector3d::Zero();
	current_yaw = 0.0;
	pre_yaw = 0.0;
}

 cv::Mat BEVImageGenerator::image_transformer(cv::Mat src_img)
{
	std::cout << "BEVImageGenerator::image_transformer" << std::endl;

	float alpha = 1.0;
    pcl::transformPointCloud(src_euqlid_3pts, dst_euqlid_3pts, affine_transform);
    // pointcloud transform coodirate

	const cv::Point2f src_pt[] = {cv::Point2f(src_euqlid_3pts.points[0].x / grid_size, src_euqlid_3pts.points[0].y / grid_size),
								  cv::Point2f(src_euqlid_3pts.points[1].x / grid_size, src_euqlid_3pts.points[1].y / grid_size),
								  cv::Point2f(src_euqlid_3pts.points[2].x / grid_size, src_euqlid_3pts.points[2].y / grid_size)};
	const cv::Point2f dst_pt[] = {cv::Point2f(alpha * dst_euqlid_3pts.points[0].x / grid_size, alpha * dst_euqlid_3pts.points[0].y / grid_size),
								  cv::Point2f(alpha * dst_euqlid_3pts.points[1].x / grid_size, alpha * dst_euqlid_3pts.points[1].y / grid_size),
								  cv::Point2f(alpha * dst_euqlid_3pts.points[2].x / grid_size, alpha * dst_euqlid_3pts.points[2].y / grid_size)};

    const cv::Mat affine_matrix = cv::getAffineTransform(src_pt, dst_pt);
    cv::Mat dst_img;
    cv::warpAffine(src_img, dst_img, affine_matrix, src_img.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

	std::cout << "src_pt" << std::endl;
	for(int i = 0; i < 3; i++){
		std::cout << src_pt[i] << std::endl;
	}
	std::cout << "dst_pt" << std::endl;
	for(int i = 0; i < 3; i++){
		std::cout << dst_pt[i] << std::endl;
	}
    return dst_img;
}

cv::Mat GroundTruth::generate_bev_image(PeopleData &pre)
{
	std::cout << "flow_estimator" << std::endl;

    cv::Mat flow_bgr;
	int img_size = GRID_NUM - 2 * MANUAL_CROP_SIZE;
	cv::Mat flow_x = cv::Mat::zeros(img_size, img_size, CV_32F);
	cv::Mat flow_y = cv::Mat::zeros(img_size, img_size, CV_32F);

		std::vector<cv::Point2f> pre_corners;
		std::vector<cv::Point2f> cur_corners;
		cv::goodFeaturesToTrack(pre_img, pre_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE);
		cv::goodFeaturesToTrack(cur_img, cur_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE);
		std::cout << "pre_img.size() = " << pre_img.size() << std::endl;
		std::cout << "cur_img.size() = " << cur_img.size() << std::endl;
		std::cout << "pre_corners:" << pre_corners.size() << std::endl;
		std::cout << "cur_corners:" << cur_corners.size() << std::endl;
		if(pre_corners.size() > 0 && cur_corners.size() > 0){
			cv::cornerSubPix(pre_img, pre_corners, cv::Size(WIN_SIZE, WIN_SIZE), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, QUALITY_LEVEL));
			cv::cornerSubPix(cur_img, cur_corners, cv::Size(WIN_SIZE, WIN_SIZE), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, QUALITY_LEVEL));
			std::vector<uchar> features_found;
			std::vector<float> features_errors;
			cv::calcOpticalFlowPyrLK(pre_img, cur_img, pre_corners, cur_corners, features_found, features_errors);

			for(size_t i = 0; i < features_found.size(); i++){
				cv::Point flow_vector = cv::Point((cur_corners[i].x - pre_corners[i].x), (cur_corners[i].y - pre_corners[i].y));
				flow_x.at<float>(pre_corners[i].x, pre_corners[i].y) = flow_vector.x;
				flow_y.at<float>(pre_corners[i].x, pre_corners[i].y) = flow_vector.y;
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

    return flow_bgr;

}