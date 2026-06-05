#include "rectification.h"
#include <iostream>

// ── OpenCV baseline (DONE) ─────────────────────────────────────
RectifyResult rectifyOpenCV(const cv::Mat& left, const cv::Mat& right,
                             const CalibData& calib) {
    cv::Size img_size(left.cols, left.rows);

    cv::Mat D0 = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat D1 = cv::Mat::zeros(5, 1, CV_64F);
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
    result.Q  = Q;
    result.P1 = P1;
    result.P2 = P2;

    std::cout << "[rectification] OpenCV done.\n";
    return result;
}

// ── Manual rectification (TODO) ────────────────────────────────
RectifyResult rectifyManual(const cv::Mat& left, const cv::Mat& right,
                             const CalibData& calib) {
    // TODO-1: SIFT/ORB keypoint detection on left and right images
    //   cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    //   sift->detectAndCompute(left_gray, ..., keypoints_l, desc_l);

    // TODO-2: Feature matching (BFMatcher or FLANN)
    //   Keep only good matches (ratio test: distance < 0.75 * second_best)

    // TODO-3: Build correspondence matrix for 8-point algorithm
    //   For each match: row = [x1*x2, x1*y2, x1, y1*x2, y1*y2, y1, x2, y2, 1]
    //   Stack into Nx9 matrix A, solve with SVD → F (reshape last row of V^T)
    //   Enforce rank-2: F = U * diag(s1,s2,0) * V^T

    // TODO-4: Compute Essential matrix from Fundamental matrix
    //   E = K1^T * F * K0

    // TODO-5: Recover R, T from Essential matrix
    //   cv::recoverPose(E, points_l, points_r, K0, R, T);

    // TODO-6: Compute rectifying homographies (Loop & Zhang 1999)
    //   cv::stereoRectify(...) with computed R, T
    //   OR implement homography computation manually

    // TODO-7: Warp both images with computed homographies
    //   cv::warpPerspective(left, left_rect, H_left, img_size);

    std::cout << "[rectification] Manual not yet implemented, falling back to OpenCV.\n";
    return rectifyOpenCV(left, right, calib);
}

#ifndef PIPELINE_BUILD
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: rectification <scene_dir> [--manual]\n";
        return 1;
    }
    fs::path scene(argv[1]);
    bool manual = (argc >= 3 && std::string(argv[2]) == "--manual");

    CalibData calib = loadCalib(scene);
    cv::Mat left  = cv::imread((scene / "im0.png").string());
    cv::Mat right = cv::imread((scene / "im1.png").string());

    if (left.empty() || right.empty()) {
        std::cerr << "Could not load images from " << scene << "\n";
        return 1;
    }

    auto result = manual ? rectifyManual(left, right, calib)
                         : rectifyOpenCV(left, right, calib);

    fs::create_directories("results");
    cv::imwrite("results/left_rect.png",  result.left_rect);
    cv::imwrite("results/right_rect.png", result.right_rect);
    std::cout << "Saved rectified images to results/\n";
    return 0;
}
#endif
