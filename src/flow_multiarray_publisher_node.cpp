#include "bev_evaluator/flow_multiarray_publisher.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "/bev_evaluator/flow_multiarray_publisher");

	FlowMultiArray flow_multiarray;
	flow_multiarray.executor();

	return 0;
}
