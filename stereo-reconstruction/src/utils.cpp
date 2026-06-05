#include "utils.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

CalibData loadCalib(const fs::path& scene_dir) {
    fs::path calib_path = scene_dir / "calib.txt";
    std::ifstream f(calib_path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open " + calib_path.string());

    CalibData c;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string key;
        std::getline(ss, key, '=');

        auto parseK = [&](cv::Mat& K) {
            std::string val;
            std::getline(ss, val);
            for (char& ch : val) if (ch == '[' || ch == ']' || ch == ';') ch = ' ';
            std::istringstream v(val);
            double m[9]; for (auto& x : m) v >> x;
            K = (cv::Mat_<double>(3,3) <<
                 m[0], m[1], m[2],
                 m[3], m[4], m[5],
                 m[6], m[7], m[8]);
        };

        if      (key == "cam0")     parseK(c.K0);
        else if (key == "cam1")     parseK(c.K1);
        else if (key == "baseline") ss >> c.baseline;
        else if (key == "doffs")    ss >> c.doffs;
        else if (key == "width")    ss >> c.width;
        else if (key == "height")   ss >> c.height;
        else if (key == "ndisp")    ss >> c.ndisp;
        else if (key == "vmin")     ss >> c.vmin;
        else if (key == "vmax")     ss >> c.vmax;
    }

    // Round ndisp up to nearest multiple of 16 (OpenCV requirement)
    c.ndisp = ((c.ndisp + 15) / 16) * 16;

    std::cout << "[calib] f=" << c.K0.at<double>(0,0)
              << " baseline=" << c.baseline << "mm"
              << " ndisp=" << c.ndisp
              << " doffs=" << c.doffs << "\n";
    return c;
}

void saveDisparity(const cv::Mat& disp, const fs::path& path) {
    cv::Mat disp_16u;
    disp.convertTo(disp_16u, CV_16U, 256.0);
    cv::imwrite(path.string(), disp_16u);
}

cv::Mat loadDisparity(const fs::path& path) {
    cv::Mat disp_16u = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    cv::Mat disp;
    disp_16u.convertTo(disp, CV_32F, 1.0 / 256.0);
    return disp;
}

void printMatInfo(const std::string& name, const cv::Mat& m) {
    std::cout << name << ": " << m.rows << "x" << m.cols
              << " type=" << m.type()
              << " channels=" << m.channels() << "\n";
}
