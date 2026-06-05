# Stereo Reconstruction

Classical stereo reconstruction pipeline in C++ using OpenCV.

## Pipeline

```
im0.png + im1.png + calib.txt
        │
        ▼
1. Stereo Rectification   →  left_rect.png, right_rect.png
        │
        ▼
2. Stereo Matching        →  disparity.png, disparity_raw.png
        │
        ▼
3. Disparity → Depth      →  pointcloud.ply
        │
        ▼
4. Mesh Reconstruction    →  mesh.ply  (requires Open3D)
```

Each step has both an **OpenCV baseline** and a **manual implementation**.

## Dependencies

| Library | Purpose | Install |
|---------|---------|---------|
| OpenCV ≥ 4.5 | Core CV operations | `brew install opencv` / `apt install libopencv-dev` |
| Open3D ≥ 0.17 | Mesh reconstruction | [open3d.org](http://www.open3d.org) |
| CMake ≥ 3.16 | Build system | `brew install cmake` / `apt install cmake` |

## Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Dataset

Download a scene from [Middlebury 2014](https://vision.middlebury.edu/stereo/data/scenes2014/) and place it under `data/`:

```
data/
└── Adirondack/
    ├── im0.png
    ├── im1.png
    └── calib.txt
```

## Usage

### Full pipeline
```bash
./build/pipeline data/Adirondack
./build/pipeline data/Adirondack --method sgbm
./build/pipeline data/Adirondack --method sad --window 11
./build/pipeline data/Adirondack --manual-rect
```

### Individual steps
```bash
./build/rectification data/Adirondack
./build/rectification data/Adirondack --manual

./build/matching data/Adirondack bm
./build/matching data/Adirondack sad

./build/depth data/Adirondack
```

### Available matching methods
| Flag | Description |
|------|-------------|
| `bm` | OpenCV BlockMatching (fast baseline) |
| `sgbm` | OpenCV Semi-Global BM (better quality) |
| `sad` | Manual Sum of Absolute Differences |
| `ssd` | Manual Sum of Squared Differences |
| `ncc` | Manual Normalized Cross-Correlation |

## Project Structure

```
stereo-reconstruction/
├── include/
│   ├── utils.h
│   ├── rectification.h
│   ├── matching.h
│   └── depth.h
├── src/
│   ├── utils.cpp
│   ├── rectification.cpp   # OpenCV + manual (TODO)
│   ├── matching.cpp        # OpenCV BM/SGBM + SAD/SSD/NCC
│   ├── depth.cpp           # Disparity → depth → .ply
│   ├── mesh.cpp            # Point cloud → mesh (Open3D)
│   └── pipeline.cpp        # Full pipeline runner
├── data/                   # Dataset scenes (git-ignored)
├── results/                # Output files (git-ignored)
└── CMakeLists.txt
```

## References

- Loop & Zhang, *Computing Rectifying Homographies for Stereo Vision*, 1999
- [Middlebury Stereo Evaluation](https://vision.middlebury.edu/stereo/)
- [OpenCV Stereo Tutorial](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html)
