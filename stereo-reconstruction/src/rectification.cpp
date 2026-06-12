#include "DataLoader.h"
#include "rectification.h"
#include "sparse_matching.h"
#include "utils.h"
#include <iostream>
#include <stdexcept>

// ── OpenCV baseline (DONE) ─────────────────────────────────────────────────
RectifyResult rectifyOpenCV(const cv::Mat& left, const cv::Mat& right,
                             const CalibData& calib) {
    cv::Size img_size(left.cols, left.rows);

    // DTU Dataset is undistorted
    cv::Mat D0 = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat D1 = cv::Mat::zeros(5, 1, CV_64F);

    // TODO: temporary identity and no translation? Later will use R, t computed in sparse matching?
    cv::Mat T  = (cv::Mat_<double>(3,1) << -calib.baseline, 0, 0);
    cv::Mat R  = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat R0, R1, P1, P2, Q;
    cv::stereoRectify(calib.K0, D0, calib.K1, D1,
                      img_size, R, T,
                      R0, R1, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0);

    cv::Mat map0x, map0y, map1x, map1y;
    cv::initUndistortRectifyMap(calib.K0, D0, R0, P1, img_size, CV_32FC1, map0x, map0y);
    cv::initUndistortRectifyMap(calib.K1, D1, R1, P2, img_size, CV_32FC1, map1x, map1y);

    RectifyResult result;
    cv::remap(left,  result.left_rect,  map0x, map0y, cv::INTER_LINEAR);
    cv::remap(right, result.right_rect, map1x, map1y, cv::INTER_LINEAR);

    if (result.left_rect.empty() || result.right_rect.empty())
        throw std::runtime_error("[rectifyOpenCV] Rectified images are empty after remap.");

    result.Q  = Q;
    result.P1 = P1;
    result.P2 = P2;

    std::cout << "[rectification] OpenCV done.\n";
    return result;
}

// ── Manual rectification — Loop & Zhang 1999 (Member 2) ───────────────────
RectifyResult rectifyManual(const cv::Mat& left, const cv::Mat& right,
                             const CalibData& calib) {
    // Step 1: Get relative pose from sparse matching (Member 1's output)
    SparseMatchResult sparse;
    try {
        sparse = computeSparseMatches(left, right, calib);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("[rectifyManual] Sparse matching failed: ") + e.what());
    }

    if (sparse.R.empty()) {
        std::cout << "[rectification] Pose recovery failed, falling back to OpenCV.\n";
        return rectifyOpenCV(left, right, calib);
    }

    // TODO-6: Compute rectifying homographies from recovered R and t
    //
    //   Option A — use OpenCV stereoRectify with our R, t:
    //   cv::Size img_size(left.cols, left.rows);
    //   cv::Mat D0 = cv::Mat::zeros(5, 1, CV_64F);
    //   cv::Mat D1 = cv::Mat::zeros(5, 1, CV_64F);
    //   cv::Mat R0, R1, P1, P2, Q;
    //   cv::stereoRectify(calib.K0, D0, calib.K1, D1,
    //                     img_size, sparse.R, sparse.t,
    //                     R0, R1, P1, P2, Q,
    //                     cv::CALIB_ZERO_DISPARITY, 0);
    //
    //   Option B — Loop & Zhang direct homography (see paper):
    //   Compute epipole e0 in the left image from F:
    //     SVD(F^T) → e0 is left null-vector (last column of V)
    //   Build H0 that maps e0 to infinity (projective transform)
    //   Find H1 minimising vertical disparity via affine correction:
    //     Ha = argmin Σ ||Ha * H0 * m_l - H1 * m_r||^2

    // TODO-7: Warp both images with computed rectification maps
    //   cv::Mat map0x, map0y, map1x, map1y;
    //   cv::initUndistortRectifyMap(calib.K0, D0, R0, P1, img_size, CV_32FC1, map0x, map0y);
    //   cv::initUndistortRectifyMap(calib.K1, D1, R1, P2, img_size, CV_32FC1, map1x, map1y);
    //   cv::remap(left,  result.left_rect,  map0x, map0y, cv::INTER_LINEAR);
    //   cv::remap(right, result.right_rect, map1x, map1y, cv::INTER_LINEAR);
    //   result.Q = Q; result.P1 = P1; result.P2 = P2;

    std::cout << "[rectification] Manual (TODO-6/7) not yet implemented, falling back to OpenCV.\n";
    return rectifyOpenCV(left, right, calib);
}

#ifndef PIPELINE_BUILD
int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: rectification <data_path> <scene_id> <left_view_id> <right_view_id> [--manual] [--light <id>]\n";
        return 1;
    }
    fs::path dataPath(argv[1]);
    const std::string sceneId(argv[2]);
    const std::string viewLeftId = padViewId(std::stoi(argv[3]));
    const std::string viewRightId = padViewId(std::stoi(argv[4]));

    bool manual = false;
    std::string lightId = "0";

    for (int i = 5; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "--manual")
            manual = true;

        else if (a == "--light" && i+1 < argc)
            lightId = argv[++i];
    }

    DTUDataLoader loader(dataPath.string());
    CalibData calib = loader.loadCalib(viewLeftId, viewRightId);

    cv::Mat imgLeft  = loader.loadImage(sceneId, viewLeftId, lightId);
    cv::Mat imgRight = loader.loadImage(sceneId, viewRightId, lightId);
    
    
    if (imgLeft.empty() || imgRight.empty()) {
        std::cerr << "Could not load images.\n"; return 1;
    }

    std::cout << "Loaded views " << viewLeftId << " (left) and " << viewRightId << " (right)" << std::endl;

    auto result = manual ? rectifyManual(imgLeft, imgRight, calib)
                         : rectifyOpenCV(imgLeft, imgRight, calib);

    std::string save_path = "results/scene" + sceneId + "/rectification";

    fs::create_directories(save_path);
    cv::imwrite(save_path + "/view_" + viewLeftId + ".png",  result.left_rect);
    cv::imwrite(save_path + "/view_" + viewRightId + ".png", result.right_rect);
    std::cout << "Saved rectified images to results/\n";
    return 0;
}
#endif
