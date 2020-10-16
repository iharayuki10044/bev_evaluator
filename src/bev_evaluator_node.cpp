#include "bev_evaluator/bev_evaluator.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "/bev_evaluator/bev_evaluator");

	BEVEvaluator bev_evaluator;
	bev_evaluator.execution();

	return 0;
}
