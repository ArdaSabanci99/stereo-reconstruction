#pragma once
#include <opencv2/opencv.hpp>
#include "utils.h"

struct PointCloud {
    std::vector<cv::Point3f> points;
    std::vector<cv::Vec3b>   colors;   // BGR
};

// Convert disparity map to depth map using Z = f*B / d
cv::Mat disparityToDepth(const cv::Mat& disparity, const CalibData& calib);

// Back-project depth map to 3D point cloud
// color_img: optional, same size as disparity (for colored point cloud)
PointCloud depthToPointCloud(const cv::Mat& depth, const CalibData& calib,
                              const cv::Mat& color_img = cv::Mat());

// Save point cloud as .ply (ASCII)
void savePointCloud(const PointCloud& cloud, const std::string& path);
