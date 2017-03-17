#ifndef LOCALIZATION_MODEL_HPP
#define LOCALIZATION_MODEL_HPP

#include <opencv2/opencv.hpp>

class LocalizationModel {

public:
	void update(const cv::Mat& depth, const cv::Mat& color);
};

#endif // LOCALIZATION_MODEL_HPP
