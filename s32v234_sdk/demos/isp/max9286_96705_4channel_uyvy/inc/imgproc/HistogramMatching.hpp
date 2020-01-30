#pragma once

#include <vector>
#include <opencv2/core.hpp>

/**
 * matching the illumination between images.
 *
 * @param bgr BGR images
 */
void illuminationMatching(std::vector<cv::Mat>& bgr);

/**
 * matching the illumination
 */
void illuminationMatching(const cv::Mat& ref, cv::Mat& target)
