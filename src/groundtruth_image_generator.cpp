







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
