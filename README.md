# Micro Stereo Reconstruction

**[中文文档](README_CN.md) | [English Documentation](README.md)**

## 📋 Project Overview

This project is a complete stereo vision processing system that includes stereo calibration, stereo matching, and 3D reconstruction capabilities. The project has been successfully migrated from Windows to Linux environment, supporting multiple stereo matching algorithms (SGBM, BM, ELAS), with both graphical interface and command-line tools.

## 🖥️ System Requirements

### Operating System
- **Ubuntu 24.04 LTS** (Linux 6.14.0-29-generic)
- **Shell**: /usr/bin/bash

### Development Environment
- **Compiler**: GCC 14.2.0
- **CMake**: 3.16+
- **C++ Standard**: C++17

### Dependency Versions
- **OpenCV**: 4.6.0
  - Includes all modules: calib3d, core, imgproc, highgui, imgcodecs, etc.
  - Installation path: `/usr/include/opencv4`
- **PCL**: 1.14.0
  - Includes all modules: common, filters, io, visualization, etc.
  - Installation path: `/usr/lib/x86_64-linux-gnu/cmake/pcl`
- **Qt5**: 5.15.13
  - Components: Widgets, Core
  - Supports GUI development
- **Other Dependencies**:
  - Boost 1.83.0
  - FLANN 1.9.2
  - Eigen3 3.4.0
  - VTK (via PCL)
  - OpenMP 4.5

## 🚀 Quick Start

### 1. Environment Setup
Ensure your Ubuntu system has the following dependencies installed:
```bash
# Install basic development tools
sudo apt update
sudo apt install build-essential cmake git

# Install OpenCV
sudo apt install libopencv-dev

# Install PCL
sudo apt install libpcl-dev

# Install Qt5
sudo apt install qt5-default qtbase5-dev

# Install other dependencies
sudo apt install libboost-all-dev libflann-dev libeigen3-dev
```

### 2. Get the Project
```bash
git clone <your-repository-url>
cd micro_stereo_recon
```

### 3. One-Click Build
```bash
# Create build directory
mkdir build && cd build

# Configure project
cmake ..

# Build all projects
make build_all
```

### 4. Verify Build Results
```bash
# View generated executables
find . -name "*" -type f -executable | grep -E "(DualMicroscopeCamera|test|StereoReconstruct|StereoMatchConsole|StereoCalib|StereoMatchGUI|stereoRectifier)"
```

## 📁 Project Structure

```
micro_stereo_recon/
├── CMakeLists.txt              # Main build file
├── README_CN.md               # Chinese documentation
├── README.md                   # English documentation
├── DualMicroscopeCamera/      # Dual camera acquisition module
│   ├── CMakeLists.txt
│   ├── main_linux.cpp         # Linux version main program
│   └── ...
├── StereoCalibration/         # Stereo calibration module
│   ├── CMakeLists.txt
│   ├── panel.h/.cpp          # GUI interface
│   └── ...
├── StereoMatch/              # Stereo matching module
│   ├── src/
│   │   ├── console/          # Console version
│   │   └── GUI/              # GUI version
│   └── 3rdparty/elas/        # ELAS algorithm library
├── StereoReconstruction/     # 3D reconstruction module
├── StereoRectifier/          # Stereo rectification module
└── CustomLib-CV/            # Custom CV library
    ├── DirTools.h/.cpp
    ├── StereoMatchingTools.h/.cpp
    ├── VisualizationTools.h/.cpp
    └── OCV_PCL.h/.cpp
```

## 🎯 Functional Modules

### Console Programs

#### 1. DualMicroscopeCamera - Dual Camera Acquisition
```bash
./DualMicroscopeCamera/DualMicroscopeCamera
```
- **Function**: Synchronized dual microscope camera image acquisition
- **Features**: Real-time image display, automatic parameter adjustment, image saving
- **Use Case**: Stereo image pair acquisition

#### 2. StereoReconstruct - 3D Reconstruction
```bash
./StereoReconstruction/StereoReconstruct
```
- **Function**: Generate 3D point clouds from stereo image pairs
- **Input**: Stereo image pairs + calibration parameters
- **Output**: 3D point cloud files

#### 3. StereoMatchConsole - Stereo Matching Console
```bash
./StereoMatch/src/console/StereoMatchConsole
```
- **Function**: Command-line stereo matching
- **Algorithms**: SGBM, BM, ELAS
- **Use Case**: Batch processing, automation scripts

#### 4. test - Calibration Test
```bash
./StereoCalibration/test
```
- **Function**: Stereo calibration testing and validation
- **Purpose**: Calibration accuracy assessment

### GUI Programs

#### 1. StereoCalib - Stereo Calibration GUI
```bash
./StereoCalibration/StereoCalib
```
- **Function**: Graphical stereo calibration
- **Features**: 
  - Real-time calibration preview
  - Checkerboard detection
  - Calibration parameter save/load
  - Calibration accuracy assessment

#### 2. StereoMatchGUI - Stereo Matching GUI
```bash
./StereoMatch/src/GUI/StereoMatchGUI
```
- **Function**: Graphical stereo matching
- **Algorithm Support**:
  - **SGBM**: Semi-Global Block Matching
  - **BM**: Block Matching
  - **ELAS**: Efficient Large-scale Stereo Matching
- **Features**:
  - Real-time parameter adjustment
  - Disparity map visualization
  - Batch processing
  - Result saving

#### 3. stereoRectifier - Stereo Rectification GUI
```bash
./StereoRectifier/stereoRectifier
```
- **Function**: Stereo image rectification
- **Features**:
  - Rectification parameter settings
  - Real-time rectification preview
  - Batch rectification processing

## 🔧 Build Options

### Building Specific Projects
```bash
# Build only GUI programs
make StereoCalib
make StereoMatchGUI
make stereoRectifier

# Build only console programs
make DualMicroscopeCamera
make StereoReconstruct
make StereoMatchConsole
make test

# Build only algorithm library
make elaslib
```

### Build Configuration
```bash
# Debug mode build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make build_all

# Release mode build
cmake -DCMAKE_BUILD_TYPE=Release ..
make build_all

# Parallel build (using 4 threads)
make -j4 build_all
```

## 📖 Usage Workflow

### Complete Stereo Vision Processing Pipeline

1. **Camera Calibration**
   ```bash
   # Launch calibration program
   ./StereoCalibration/StereoCalib
   ```
   - Prepare calibration board (checkerboard recommended)
   - Capture multiple calibration images
   - Execute calibration calculation
   - Save calibration parameters

2. **Image Acquisition**
   ```bash
   # Launch dual camera acquisition
   ./DualMicroscopeCamera/DualMicroscopeCamera
   ```
   - Connect dual cameras
   - Adjust camera parameters
   - Capture stereo image pairs

3. **Stereo Matching**
   ```bash
   # GUI version
   ./StereoMatch/src/GUI/StereoMatchGUI
   
   # Or console version
   ./StereoMatch/src/console/StereoMatchConsole
   ```
   - Select matching algorithm
   - Adjust parameters
   - Generate disparity map

4. **3D Reconstruction**
   ```bash
   ./StereoReconstruction/StereoReconstruct
   ```
   - Input stereo image pairs
   - Input calibration parameters
   - Generate 3D point cloud

5. **Image Rectification**
   ```bash
   ./StereoRectifier/stereoRectifier
   ```
   - Load calibration parameters
   - Rectify stereo image pairs

## ⚙️ Parameter Configuration

### Stereo Matching Parameters

#### SGBM Parameters
- **BlockSize**: Block size (usually odd number)
- **numDisp**: Disparity range
- **minDisp**: Minimum disparity
- **p1, p2**: Smoothness parameters
- **preFilterCap**: Pre-filter truncation value
- **uniquenessRatio**: Uniqueness ratio

#### BM Parameters
- **BlockSize**: Block size
- **numDisp**: Disparity range
- **minDisp**: Minimum disparity
- **preFilterCap**: Pre-filter truncation value
- **preFilterSize**: Pre-filter kernel size

#### ELAS Parameters
- **support_threshold**: Support threshold
- **texture_threshold**: Texture threshold
- **disp_min**: Minimum disparity
- **disp_max**: Maximum disparity

## 🐛 Troubleshooting

### Build Issues

#### 1. OpenCV Not Found
```bash
# Check OpenCV installation
pkg-config --modversion opencv4

# If not installed, execute
sudo apt install libopencv-dev
```

#### 2. PCL Not Found
```bash
# Check PCL installation
sudo apt install libpcl-dev
```

#### 3. Qt5 Not Found
```bash
# Check Qt5 installation
sudo apt install qt5-default qtbase5-dev
```

#### 4. CMake Version Too Low
```bash
# Upgrade CMake
sudo apt install cmake
```

### Runtime Issues

#### 1. GUI Programs Cannot Start
- Check X11 display environment
- Confirm Qt5 libraries are correctly installed
- Check display settings

#### 2. Camera Cannot Be Accessed
```bash
# Add user to video group
sudo usermod -a -G video $USER
# Re-login to take effect
```

#### 3. Permission Issues
```bash
# Add execute permission to executables
chmod +x ./StereoCalibration/StereoCalib
chmod +x ./StereoMatch/src/GUI/StereoMatchGUI
chmod +x ./StereoRectifier/stereoRectifier
```

### Performance Optimization

#### 1. Build Optimization
```bash
# Release mode build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc) build_all
```

#### 2. Runtime Optimization
- Use SSD storage for image data
- Ensure sufficient memory (8GB+ recommended)
- Use multi-core CPU for parallel processing
