#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>


// TODO: change with DTU

namespace fs = std::filesystem;

struct CalibData {
    cv::Mat K0;        // camera matrix left
    cv::Mat K1;        // camera matrix right
    cv::Mat R0, R1;    // rotation matrices from pos files (world → cam)
    cv::Mat t0, t1;    // translation vectors from pos files
    cv::Mat R_rel;     // rotation:    X_right = R_rel * X_left + T_rel
    cv::Mat T_rel;     // translation: (in left camera frame, mm)
    double baseline;   // |T_rel| in mm
    int vmin  = 0;     // min valid disparity (for evaluation)
    int vmax  = 255;   // max valid disparity
    int doffs = 0;     // not used in DTU
};

// Save/load disparity map as .pfm or .png
void saveDisparity(const cv::Mat& disp, const fs::path& path);
cv::Mat loadDisparity(const fs::path& path);

// Format DTU view IDs as three-digit zero-padded strings.
std::string padViewId(int viewId);

// Print a cv::Mat summary (for debugging)
void printMatInfo(const std::string& name, const cv::Mat& m);
