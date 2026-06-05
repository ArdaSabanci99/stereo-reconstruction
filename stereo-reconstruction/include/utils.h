#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

struct CalibData {
    cv::Mat K0;        // camera matrix left
    cv::Mat K1;        // camera matrix right
    double baseline;   // in mm
    double doffs;      // x-difference of principal points
    int width, height;
    int ndisp = 256;   // recommended num_disparities from dataset
    int vmin  = 0;     // min valid disparity (for evaluation)
    int vmax  = 255;   // max valid disparity
};

// Parse calib.txt from Middlebury 2014 format
CalibData loadCalib(const fs::path& scene_dir);

// Save/load disparity map as .pfm or .png
void saveDisparity(const cv::Mat& disp, const fs::path& path);
cv::Mat loadDisparity(const fs::path& path);

// Print a cv::Mat summary (for debugging)
void printMatInfo(const std::string& name, const cv::Mat& m);
