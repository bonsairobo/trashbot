#ifndef GRASPING_MODEL_HPP
#define GRASPING_MODEL_HPP

#include <opencv2/opencv.hpp>
#include "../../common/socket_types.hpp"

class GraspingModel {

public:
	GraspingPoints search_grasping_points(
		const cv::Mat& depth, const cv::Mat& color);
};

#endif // GRASPING_MODEL_HPP
