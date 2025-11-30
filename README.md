# Micro Stereo Reconstruction

**[中文文档](README_CN.md) | [English Documentation](README.md)**

> A complete stereo vision processing system with stereo calibration, stereo matching, and 3D reconstruction capabilities. Supports both Linux and Windows platforms.

## 📋 Project Overview

This project is a comprehensive stereo vision processing system that includes:

- **Stereo Calibration**: Camera calibration for stereo vision systems
- **Stereo Matching**: Multiple algorithms (SGBM, BM, ELAS) with post-processing filters (WLS, FBS)
- **3D Reconstruction**: Point cloud generation from stereo image pairs
- **Image Rectification**: Stereo image pair rectification
- **Dual Camera Acquisition**: Synchronized dual microscope camera capture

The project provides both **graphical user interfaces (GUI)** and **command-line tools** for different use cases.

## 🌿 Branch Structure

This repository contains multiple branches for different platforms:

- **`main`**: Main branch with cross-platform documentation
- **`dev_linux`**: Linux development branch (Ubuntu 24.04 LTS)
  - Full Linux support with Qt5 GUI
  - See [dev_linux branch README](https://github.com/your-repo/micro_stereo_recon/tree/dev_linux) for detailed Linux setup
- **`dev_win`**: Windows development branch
  - Windows-specific configurations
  - See [dev_win branch README](https://github.com/your-repo/micro_stereo_recon/tree/dev_win) for Windows setup

### Which Branch Should I Use?

- **For Linux users**: Checkout `dev_linux` branch
  ```bash
  git checkout dev_linux
  ```
- **For Windows users**: Checkout `dev_win` branch
  ```bash
  git checkout dev_win
  ```
- **For documentation only**: Stay on `main` branch

## 🖥️ Platform Support

### Linux (dev_linux branch)

**System Requirements:**
- Ubuntu 24.04 LTS (Linux 6.14.0-29-generic)
- GCC 14.2.0
- CMake 3.16+
- C++17

**Dependencies:**
- OpenCV 4.6.0
- PCL 1.14.0
- Qt5 5.15.13
- Boost, FLANN, Eigen3, VTK, OpenMP

**Quick Start:**
```bash
git checkout dev_linux
mkdir build && cd build
cmake ..
make build_all
```

For detailed Linux setup instructions, see the [dev_linux README](https://github.com/your-repo/micro_stereo_recon/tree/dev_linux).

### Windows (dev_win branch)

**System Requirements:**
- Windows 10/11
- Visual Studio 2019 or later
- CMake 3.16+

**Dependencies:**
- OpenCV (Windows build)
- PCL (Windows build)
- Qt5 (Windows build)

**Quick Start:**
```bash
git checkout dev_win
# Follow Windows-specific build instructions
```

For detailed Windows setup instructions, see the [dev_win README](https://github.com/your-repo/micro_stereo_recon/tree/dev_win).

## 🚀 Quick Start

### 1. Clone the Repository

```bash
git clone <your-repository-url>
cd micro_stereo_recon
```

### 2. Choose Your Platform Branch

```bash
# For Linux
git checkout dev_linux

# For Windows
git checkout dev_win
```

### 3. Build the Project

**Linux:**
```bash
mkdir build && cd build
cmake ..
make build_all
```

**Windows:**
```bash
# Use Visual Studio or CMake GUI
# Follow platform-specific instructions
```

## 📁 Project Structure

```
micro_stereo_recon/
├── CMakeLists.txt              # Main build file
├── README.md                   # This file (English)
├── README_CN.md                # Chinese documentation
├── DualMicroscopeCamera/      # Dual camera acquisition module
│   ├── main_linux.cpp         # Linux version
│   └── main.cpp               # Windows version
├── StereoCalibration/         # Stereo calibration module
│   ├── panel.h/.cpp          # GUI interface
│   └── test.cpp              # Calibration test
├── StereoMatch/              # Stereo matching module
│   ├── src/
│   │   ├── console/          # Console version
│   │   ├── GUI/              # GUI version
│   │   └── include/          # Algorithm headers
│   └── 3rdparty/elas/        # ELAS algorithm library
├── StereoReconstruction/     # 3D reconstruction module
├── StereoRectifier/          # Stereo rectification module
└── CustomLib-CV/            # Custom CV library
    ├── DirTools.h/.cpp
    ├── StereoMatchingTools.h/.cpp
    ├── VisualizationTools.h/.cpp
    └── OCV_PCL.h/.cpp
```

## 🎯 Features

### Algorithms

- **SGBM (Semi-Global Block Matching)**: High-quality stereo matching with post-processing
  - WLS (Weighted Least Squares) filter
  - FBS (Fast Bilateral Solver) filter
  - Configurable parameters
- **BM (Block Matching)**: Fast block-based matching
- **ELAS (Efficient Large-scale Stereo)**: Efficient large-scale stereo matching

### Applications

- **StereoCalib**: GUI for stereo camera calibration
- **StereoMatchGUI**: GUI for stereo matching with real-time preview
- **StereoMatchConsole**: Command-line tool for batch processing
- **StereoReconstruct**: 3D point cloud generation
- **stereoRectifier**: Stereo image rectification
- **DualMicroscopeCamera**: Dual camera synchronization

## 📖 Usage Workflow

1. **Camera Calibration** → Generate calibration parameters
2. **Image Acquisition** → Capture stereo image pairs
3. **Stereo Matching** → Generate disparity maps
4. **3D Reconstruction** → Create 3D point clouds
5. **Image Rectification** → Rectify stereo pairs

For detailed usage instructions, refer to the platform-specific README in `dev_linux` or `dev_win` branches.

## 🔧 Build Options

### Build All Projects
```bash
# Linux
make build_all

# Windows
# Use Visual Studio solution or CMake
```

### Build Specific Modules
```bash
# GUI programs
make StereoCalib
make StereoMatchGUI
make stereoRectifier

# Console programs
make DualMicroscopeCamera
make StereoReconstruct
make StereoMatchConsole
```

## 📚 Documentation

- **Main Documentation**: This README (English)
- **中文文档**: See `README_CN.md` (if available)
- **Linux Details**: See `dev_linux` branch README
- **Windows Details**: See `dev_win` branch README

## 🤝 Contributing

This is a bachelor dissertation project. For contributions or questions:

1. Check the appropriate development branch (`dev_linux` or `dev_win`)
2. Review the platform-specific documentation
3. Follow the coding standards and build procedures

## 📝 License

This project is part of a bachelor dissertation. Please refer to the license file for details.

## 🔗 Links

- **Linux Development**: [dev_linux branch](https://github.com/your-repo/micro_stereo_recon/tree/dev_linux)
- **Windows Development**: [dev_win branch](https://github.com/your-repo/micro_stereo_recon/tree/dev_win)
- **Chinese Documentation**: `README_CN.md` (in dev_linux branch)

## 📧 Contact

For questions or issues, please refer to the platform-specific branches for detailed support information.

---

**Note**: This is the main branch with cross-platform overview. For platform-specific setup and detailed instructions, please checkout the appropriate development branch (`dev_linux` or `dev_win`).
