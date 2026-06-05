#include "depth.h"
#include <fstream>
#include <iostream>

// ── TODO: Mesh reconstruction from depth map ──────────────────

// TODO-G1: Simple depth map triangulation (no external library needed)
//
//   For each 2x2 block of valid pixels:
//
//     (x,y) --- (x+1,y)
//       |    \      |
//       |      \    |
//     (x,y+1)-(x+1,y+1)
//
//   Create triangle 1: (x,y), (x+1,y), (x,y+1)
//   Create triangle 2: (x+1,y), (x+1,y+1), (x,y+1)
//   Skip triangle if any vertex has invalid depth or depth jump > threshold
//
//   void triangulateDepthMap(const cv::Mat& depth,
//                             const CalibData& calib,
//                             const cv::Mat& color,
//                             const std::string& out_path);

// TODO-G2: Save as .obj
//   Write vertices: "v X Y Z"
//   Write texture coords: "vt u v"  (optional, for colored mesh)
//   Write faces: "f v1 v2 v3"
//
//   void saveMeshOBJ(const std::vector<cv::Point3f>& verts,
//                    const std::vector<cv::Vec3i>& faces,
//                    const std::string& path);

// TODO-G3: Smoothing
//   After triangulation, apply Laplacian smoothing:
//   For each vertex v_i: v_i_new = (1-lambda)*v_i + lambda * mean(neighbors)
//   Iterate N times (e.g. 5-20 iterations)

#ifndef PIPELINE_BUILD
int main(int argc, char** argv) {
    // TODO: implement mesh reconstruction standalone executable
    std::cout << "Mesh reconstruction not yet implemented.\n";
    std::cout << "See TODO-G1, TODO-G2, TODO-G3 in this file.\n";
    return 0;
}
#endif
