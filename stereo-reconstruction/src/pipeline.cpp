#include "DataLoader.h"
#include "depth.h"
#include "rectification.h"
#include "matching.h"
#include <iostream>
#include <string>
#include <stdexcept>

static void printUsage(const char* name) {
    std::cerr << "Usage: " << name << " <data_path> <scene_id> <view_left_id> <view_right_id> [options]\n"
              << "  --method      bm|sgbm|sad|ssd|ncc|census|sgm   (default: sgbm)\n"
              << "  --manual-rect                        (use manual rectification)\n"
              << "  --window      <size>                 (default: 5)\n"
              << "  --ndisp       <num_disparities>      (default: 64)\n"
              << "  --light       <light_id>             (default: 0)\n";
}

int main(int argc, char** argv) {
    if (argc < 5) { printUsage(argv[0]); return 1; }

    fs::path dataPath(argv[1]);
    
    const std::string sceneId(argv[2]);
    const std::string viewLeftId = padViewId(std::stoi(argv[3]));
    const std::string viewRightId = padViewId(std::stoi(argv[4]));

    MatchParams mp;
    mp.method      = MatchMethod::OPENCV_SGBM;
    mp.window_size = 5;
    bool manual_rect = false;
    double img_scale = 0.25;  // downsample to bring disparity into SGBM range

    std::string lightId = "0";

    for (int i = 5; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "--manual-rect")      manual_rect = true;
        else if (a == "--method" && i+1 < argc) {
            std::string m(argv[++i]);
            if      (m == "bm")   mp.method = MatchMethod::OPENCV_BM;
            else if (m == "sgbm") mp.method = MatchMethod::OPENCV_SGBM;
            else if (m == "sad")  mp.method = MatchMethod::MANUAL_SAD;
            else if (m == "ssd")  mp.method = MatchMethod::MANUAL_SSD;
            else if (m == "ncc")  mp.method = MatchMethod::MANUAL_NCC;
            else if (m == "census") mp.method = MatchMethod::MANUAL_CENSUS;
            else if (m == "sgm")    mp.method = MatchMethod::MANUAL_SGM;
        }
        else if (a == "--window" && i+1 < argc) mp.window_size = std::stoi(argv[++i]);
        else if (a == "--ndisp"  && i+1 < argc) mp.num_disparities = std::stoi(argv[++i]);
        else if (a == "--mindisp"&& i+1 < argc) mp.min_disparity   = std::stoi(argv[++i]);
        else if (a == "--scale"  && i+1 < argc) img_scale = std::stod(argv[++i]);
        else if (a == "--light"  && i+1 < argc) lightId = argv[++i];
    }

    // ── 1. Load data ───────────────────────────────────────────
    std::cout << "=== Loading scene: " << sceneId << " ===\n";
    DTUDataLoader loader(dataPath.string());
    CalibData calib = loader.loadCalib(viewLeftId, viewRightId);

    cv::Mat imgLeft  = loader.loadImage(sceneId, viewLeftId, lightId);
    cv::Mat imgRight = loader.loadImage(sceneId, viewRightId, lightId);

    if (imgLeft.empty() || imgRight.empty()) {
        std::cerr << "Could not load images.\n"; return 1;
    }

    std::cout << "Loaded views " << viewLeftId << " (left) and " << viewRightId << " (right)\n";
    printMatInfo("Left Image",   imgLeft);
    printMatInfo("Right Image:", imgRight);

    // ── Ensure left camera is geometrically to the left (T_rel[0] < 0) ─────
    // stereoRectify/SGBM require the left camera center to be at negative
    // x in the right camera's frame; if T_rel[0] > 0 the pair is reversed.
    if (calib.T_rel.at<double>(0) > 0) {
        std::cout << "[pipeline] Swapping left/right cameras (T_rel[0] > 0)\n";
        std::swap(calib.K0, calib.K1);
        std::swap(calib.R0, calib.R1);
        std::swap(calib.t0, calib.t1);
        std::swap(imgLeft,  imgRight);
        calib.R_rel = calib.R1 * calib.R0.t();
        calib.T_rel = calib.t1 - calib.R_rel * calib.t0;
    }

    // ── Downscale images and intrinsics ────────────────────────
    if (img_scale != 1.0) {
        cv::resize(imgLeft,  imgLeft,  cv::Size(), img_scale, img_scale, cv::INTER_AREA);
        cv::resize(imgRight, imgRight, cv::Size(), img_scale, img_scale, cv::INTER_AREA);
        for (cv::Mat* K : {&calib.K0, &calib.K1}) {
            K->at<double>(0,0) *= img_scale;
            K->at<double>(1,1) *= img_scale;
            K->at<double>(0,2) = (K->at<double>(0,2) + 0.5) * img_scale - 0.5;
            K->at<double>(1,2) = (K->at<double>(1,2) + 0.5) * img_scale - 0.5;
        }
        std::cout << "Downscaled to " << imgLeft.cols << "x" << imgLeft.rows
                  << " (scale=" << img_scale << ")\n";
    }

    // ── 2. Rectification ───────────────────────────────────────
    std::cout << "\n=== Rectification ===\n";
    RectifyResult rect;
    try {
        rect = manual_rect ? rectifyManual(imgLeft, imgRight, calib)
                           : rectifyOpenCV(imgLeft, imgRight, calib);
    } catch (const std::exception& e) {
        std::cerr << "[pipeline] Rectification failed: " << e.what() << "\n";
        return 1;
    }

    std::string save_path_rect = "results/scene" + sceneId + "/rectification";
    fs::create_directories(save_path_rect);
    cv::imwrite(save_path_rect + "/view_" + viewLeftId + ".png",  rect.left_rect);
    cv::imwrite(save_path_rect + "/view_" + viewRightId + ".png", rect.right_rect);

    // ── 3. Stereo matching ─────────────────────────────────────
    std::cout << "\n=== Stereo Matching (ndisp=" << mp.num_disparities
              << " window=" << mp.window_size << ") ===\n";
    cv::Mat disp;
    try {
        disp = computeDisparity(rect.left_rect, rect.right_rect, mp);
    } catch (const std::exception& e) {
        std::cerr << "[pipeline] Stereo matching failed: " << e.what() << "\n";
        return 1;
    }

    cv::Mat disp_vis;
    cv::normalize(disp, disp_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(disp_vis, disp_vis, cv::COLORMAP_JET);

    std::string save_path_match = "results/scene" + sceneId + "/matching";
    fs::create_directories(save_path_match);
    cv::imwrite(save_path_match + "/view_" + viewLeftId + "_" + viewRightId + "_disparity.png", disp_vis);
    saveDisparity(disp, save_path_match + "/view_" + viewLeftId + "_" + viewRightId + "_disparity_raw.png");

    // ── 4. Depth + point cloud ─────────────────────────────────
    std::cout << "\n=== Depth & Point Cloud ===\n";
    PointCloud cloud;
    try {
        cloud = disparityToCloud(disp, rect.Q, rect.left_rect, (float)mp.min_disparity);
    } catch (const std::exception& e) {
        std::cerr << "[pipeline] Depth/point cloud failed: " << e.what() << "\n";
        return 1;
    }

    // ── 5. Transform from rectified cam0 space → world space ────
    // X_world = R0^T * (R0_rect^T * X_rect - t0)
    {
        cv::Mat R0t       = calib.R0.t();
        cv::Mat R0_rect_t = rect.R0_rect.t();
        for (auto& p : cloud.points) {
            cv::Mat x = (cv::Mat_<double>(3,1) << p.x(), p.y(), p.z());
            cv::Mat w = R0t * (R0_rect_t * x - calib.t0);
            p = Eigen::Vector3f((float)w.at<double>(0),
                                (float)w.at<double>(1),
                                (float)w.at<double>(2));
        }
        std::cout << "[pipeline] Transformed " << cloud.points.size()
                  << " points to world space.\n";
    }

    std::string save_path_pointcloud = "results/scene" + sceneId + "/pointcloud";
    fs::create_directories(save_path_pointcloud);
    savePointCloud(cloud, save_path_pointcloud + "/views_" + viewLeftId + "_" + viewRightId + ".ply");
    savePointCloudOFF(cloud, save_path_pointcloud + "/views_" + viewLeftId + "_" + viewRightId + ".off");

    std::cout << "\n=== Pipeline complete. Results in results/ ===\n";
    return 0;
}
