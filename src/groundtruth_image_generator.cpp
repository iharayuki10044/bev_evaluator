



cv::Mat BEVImageGenerator::cropped_transformed_grid_img_generator(cv::Mat src_img, Eigen::Vector3d current_position, double current_yaw, Eigen::Vector3d last_position, double last_yaw)
{
	/* std::cout << "BEVImageGenerator::cropped_transformed_grid_img_generator" << std::endl; */
	double d_yaw = current_yaw - last_yaw;
	d_yaw = atan2(sin(d_yaw), cos(d_yaw));
	Eigen::Matrix3d r;
	r = Eigen::AngleAxisd(-d_yaw, Eigen::Vector3d::UnitZ());

	Eigen::Matrix3d last_yaw_rotation;
	last_yaw_rotation = Eigen::AngleAxisd(-last_yaw, Eigen::Vector3d::UnitZ());
	Eigen::Vector3d _current_position = last_yaw_rotation * current_position;
	Eigen::Vector3d _last_position = last_yaw_rotation * last_position;
	Eigen::Translation<double, 3> t(_last_position - _current_position);

	affine_transform = t * r;
	/* std::cout << "affine transformation: \n" << affine_transform.translation() << "\n" << affine_transform.rotation().eulerAngles(0,1,2) << std::endl; */
	
    cv::Mat transformed_grid_img, dst_img;

	transformed_grid_img = image_transformer(src_img);
	dst_img = image_cropper(transformed_grid_img);
	odom_callback_flag = false;

    return dst_img;
}

int Groundtruth_image_generator::get_index_from_xy(const double x,const double y)
{
	int _x = floor(x / RESOLUTION + 0.5) + GRID_WIDTH_2;
	int _y = floor(y / RESOLUTION + 0.5) + GRID_WIDTH_2;
	return _y * GRID_WIDTH_2 + _x;
}

int Groundtruth_image_generator::get_x_index_from_index(const int index)
{
    return index % GRID_WIDTH;
}

int Groundtruth_image_generator::get_y_index_from_index(const int index)
{
    return index / GRID_WIDTH;
}

double Groundtruth_image_generator::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

double Groundtruth_image_generator::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}
