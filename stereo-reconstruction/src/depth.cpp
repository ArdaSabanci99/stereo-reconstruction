#include "depth.h"
#include <fstream>
#include <iostream>

// ── Disparity → Depth (DONE) ───────────────────────────────────
cv::Mat disparityToDepth(const cv::Mat& disparity, const CalibData& calib) {
    // Z = f * B / (d + doffs)
    double f = calib.K0.at<double>(0, 0);

    cv::Mat depth(disparity.size(), CV_32F, 0.0f);
    for (int y = 0; y < disparity.rows; ++y)
        for (int x = 0; x < disparity.cols; ++x) {
            float d = disparity.at<float>(y, x);
            if (d <= 0) continue;
            depth.at<float>(y, x) = (float)(f * calib.baseline / (d + calib.doffs));
        }
    return depth;
}

// ── Depth → Point Cloud (DONE) ─────────────────────────────────
PointCloud depthToPointCloud(const cv::Mat& depth, const CalibData& calib,
                              const cv::Mat& color_img) {
    double fx = calib.K0.at<double>(0, 0);
    double fy = calib.K0.at<double>(1, 1);
    double cx = calib.K0.at<double>(0, 2);
    double cy = calib.K0.at<double>(1, 2);

    bool has_color = !color_img.empty();
    PointCloud cloud;

    for (int y = 0; y < depth.rows; ++y) {
        for (int x = 0; x < depth.cols; ++x) {
            float Z = depth.at<float>(y, x);
            if (Z <= 0 || Z > 1e4f) continue;

            float X = (float)((x - cx) * Z / fx);
            float Y = (float)((y - cy) * Z / fy);
            cloud.points.emplace_back(X, Y, Z);

            if (has_color)
                cloud.colors.push_back(color_img.at<cv::Vec3b>(y, x));
            else
                cloud.colors.emplace_back(200, 200, 200);
        }
    }
    std::cout << "[depth] Point cloud: " << cloud.points.size() << " points\n";
    return cloud;
}

// ── Save Point Cloud as .ply (DONE) ───────────────────────────
void savePointCloud(const PointCloud& cloud, const std::string& path) {
    std::ofstream f(path);
    f << "ply\nformat ascii 1.0\n"
      << "element vertex " << cloud.points.size() << "\n"
      << "property float x\nproperty float y\nproperty float z\n"
      << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
      << "end_header\n";
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];
        const auto& c = cloud.colors[i];
        f << p.x << " " << p.y << " " << p.z << " "
          << (int)c[2] << " " << (int)c[1] << " " << (int)c[0] << "\n";
    }
    std::cout << "[depth] Saved point cloud to " << path << "\n";
}

// ── TODO: Evaluation against ground truth ─────────────────────
// TODO-E: Load ground truth disparity from disp0GT.pfm
//   Middlebury stores ground truth as PFM (portable float map)
//   Implement a PFM reader: check header "Pf", read scale, read float data
//   cv::Mat loadPFM(const fs::path& path);

// TODO-F: Compute error metrics
//   bad1  = % of pixels where |disp_est - disp_gt| > 1px  (exclude invalid GT pixels)
//   bad2  = % of pixels where |disp_est - disp_gt| > 2px
//   RMSE  = sqrt(mean((disp_est - disp_gt)^2))
//   void evaluateDisparity(const cv::Mat& estimated, const cv::Mat& ground_truth,
//                          int vmin, int vmax);

// TODO-G: Mesh reconstruction (without Open3D)
//   Option 1: Simple triangulation from depth map
//     For each 2x2 block of valid depth pixels, create 2 triangles
//     Save as .obj with vertex positions and face indices
//   Option 2: Marching Cubes on voxel grid
//     Convert point cloud to voxel grid, run marching cubes

#ifndef PIPELINE_BUILD
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: depth <scene_dir>\n"; return 1;
    }
    fs::path scene(argv[1]);
    CalibData calib = loadCalib(scene);

    cv::Mat disp  = loadDisparity("results/disparity_raw.png");
    cv::Mat left  = cv::imread("results/left_rect.png");

    auto depth = disparityToDepth(disp, calib);
    auto cloud = depthToPointCloud(depth, calib, left);

    fs::create_directories("results");
    savePointCloud(cloud, "results/pointcloud.ply");
    return 0;
}
#endif
