#include "bev_converter/groundtruth.h"

GroundTruth::GroundTruth(void)
:local_nh("~")
{
    local_nh.param("RESOLUTION", RESOLUTION, {});
    local_nh.param("WIDTH", WIDTH, {});
    local_nh.param("WIDTH_2", WIDTH_2,{});
    local_nh.param("GRID_WIDTH", GRID_WIDTH, {});
    local_nh.param("GRID_WIDTH", GRID_WIDTH_2, {});
    local_nh.param("GRID_NUM", GRID_NUM, {});
    local_nh.param("PEOPLE_NUM", PEOPLE_NUM, {});

    pc_subscriber = nh.subscribe("/cloud/dynamic", 10, &GroundTruth::pc_callback, this);
    odom_subscriber = nh.subscribe("/odom", 10, &GroundTruth::odom_callback, this);
    bev_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/bev/grid", 10);
}

void GroundTruth::executor(void)
{
    formatter();

    ros::Rate r(Hz);
	while(ros::ok()){
        
        
        if(pc_callback_flag && odm_callback_frag){
            std::cout << "people calculate" << std::endl;
            copy_people_data(PeopleData&, PeopleData&);
            calculation_people_point(cloud_ptr);
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
}

void GroundTruth::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 input_pc;

    input_pc = *msg;
	pcl::fromROSMsg(input_pc, *pcl_input_pc);
    pc_callback_flag = true;
}

void GroundTruth::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
    odom = *msg;
    odom_callback_flag = true;
}

void GroundTruth::copy_people_data(PeopleData &new, PeopleData &old)
{
    old = new;
}
void GroundTruth::calculation_peple_point(const CloudXYZIPtr& cloud_ptr ,People)
{
    std::cout << "--- calculation people point ---" << std::endl;
    int cloud_size = cloud_ptr->points.size();

    Eigen::Matrix<double, 3, PEOPLE_NUM> people_point = Eigen::MatrixXD::Zero(3, PEOPLE_NUM);
    //(x, y, hit_num)*people_num Zero init

    for(int i=0;i<cloud_size;i++){
        auto p = cloud_ptr->points[i];
        double id = cloud_ptr->intensity[i] - 1;
        people_point(1, id) += p.x;
        people_point(2, id) += p.y;
        people_point(3, id) += 1;
    }

    for(int i=0;i<PEOPLE_NUM;i++){
        
        if(people_point != 0){
            people_data_new[i].x = people_point(1, id) /people(3, id);
            people_data_new[i].y = people_point(2, id) /people(3, id);
        }
    }
}

void GroundTruth::calculation_people_vector(PeopleData &now, PeopleData &past)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        past[i].move_vector_x = now[i].point_x - past[i].point.x;
        past[i].move_vector_y = now[i].point_y - past[i].point.y;
    }
}

