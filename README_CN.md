# 微立体重建项目 (Micro Stereo Reconstruction)

## 📋 项目简介

本项目是一个完整的立体视觉处理系统，包含立体标定、立体匹配、3D重建等功能。项目已成功从Windows环境迁移到Linux环境，支持多种立体匹配算法（SGBM、BM、ELAS），提供图形化界面和命令行工具。

## 🖥️ 运行环境

### 操作系统
- **Ubuntu 24.04 LTS** (Linux 6.14.0-29-generic)
- **Shell**: /usr/bin/bash

### 开发环境
- **编译器**: GCC 14.2.0
- **CMake**: 3.16+
- **C++标准**: C++17

### 依赖库版本
- **OpenCV**: 4.6.0
  - 包含所有模块：calib3d, core, imgproc, highgui, imgcodecs等
  - 安装路径：`/usr/include/opencv4`
- **PCL**: 1.14.0
  - 包含所有模块：common, filters, io, visualization等
  - 安装路径：`/usr/lib/x86_64-linux-gnu/cmake/pcl`
- **Qt5**: 5.15.13
  - 组件：Widgets, Core
  - 支持图形界面开发
- **其他依赖**:
  - Boost 1.83.0
  - FLANN 1.9.2
  - Eigen3 3.4.0
  - VTK (通过PCL)
  - OpenMP 4.5

## 🚀 快速开始

### 1. 环境准备
确保您的Ubuntu系统已安装以下依赖：
```bash
# 安装基础开发工具
sudo apt update
sudo apt install build-essential cmake git

# 安装OpenCV
sudo apt install libopencv-dev

# 安装PCL
sudo apt install libpcl-dev

# 安装Qt5
sudo apt install qt5-default qtbase5-dev

# 安装其他依赖
sudo apt install libboost-all-dev libflann-dev libeigen3-dev
```

### 2. 获取项目
```bash
git clone <your-repository-url>
cd micro_stereo_recon
```

### 3. 一键编译
```bash
# 创建构建目录
mkdir build && cd build

# 配置项目
cmake ..

# 编译所有项目
make build_all
```

### 4. 验证编译结果
```bash
# 查看生成的可执行文件
find . -name "*" -type f -executable | grep -E "(DualMicroscopeCamera|test|StereoReconstruct|StereoMatchConsole|StereoCalib|StereoMatchGUI|stereoRectifier)"
```

## 📁 项目结构

```
micro_stereo_recon/
├── CMakeLists.txt              # 主构建文件
├── README_CN.md               # 中文说明文档
├── DualMicroscopeCamera/      # 双相机采集模块
│   ├── CMakeLists.txt
│   ├── main_linux.cpp         # Linux版本主程序
│   └── ...
├── StereoCalibration/         # 立体标定模块
│   ├── CMakeLists.txt
│   ├── panel.h/.cpp          # GUI界面
│   └── ...
├── StereoMatch/              # 立体匹配模块
│   ├── src/
│   │   ├── console/          # 控制台版本
│   │   └── GUI/              # 图形界面版本
│   └── 3rdparty/elas/        # ELAS算法库
├── StereoReconstruction/     # 3D重建模块
├── StereoRectifier/          # 立体校正模块
└── CustomLib-CV/            # 自定义CV库
    ├── DirTools.h/.cpp
    ├── StereoMatchingTools.h/.cpp
    ├── VisualizationTools.h/.cpp
    └── OCV_PCL.h/.cpp
```

## 🎯 功能模块

### 控制台程序

#### 1. DualMicroscopeCamera - 双相机采集
```bash
./DualMicroscopeCamera/DualMicroscopeCamera
```
- **功能**: 同步双显微镜相机图像采集
- **特性**: 实时图像显示、参数自动调整、图像保存
- **适用**: 立体图像对采集

#### 2. StereoReconstruct - 3D重建
```bash
./StereoReconstruction/StereoReconstruct
```
- **功能**: 从立体图像对生成3D点云
- **输入**: 立体图像对 + 标定参数
- **输出**: 3D点云文件

#### 3. StereoMatchConsole - 立体匹配控制台
```bash
./StereoMatch/src/console/StereoMatchConsole
```
- **功能**: 命令行立体匹配
- **算法**: SGBM、BM、ELAS
- **适用**: 批量处理、自动化脚本

#### 4. test - 标定测试
```bash
./StereoCalibration/test
```
- **功能**: 立体标定测试和验证
- **用途**: 标定精度评估

### GUI程序

#### 1. StereoCalib - 立体标定GUI
```bash
./StereoCalibration/StereoCalib
```
- **功能**: 图形化立体标定
- **特性**: 
  - 实时标定预览
  - 棋盘格检测
  - 标定参数保存/加载
  - 标定精度评估

#### 2. StereoMatchGUI - 立体匹配GUI
```bash
./StereoMatch/src/GUI/StereoMatchGUI
```
- **功能**: 图形化立体匹配
- **算法支持**:
  - **SGBM**: 半全局块匹配
  - **BM**: 块匹配
  - **ELAS**: 高效大区域立体匹配
- **特性**:
  - 实时参数调整
  - 视差图可视化
  - 批量处理
  - 结果保存

#### 3. stereoRectifier - 立体校正GUI
```bash
./StereoRectifier/stereoRectifier
```
- **功能**: 立体图像校正
- **特性**:
  - 校正参数设置
  - 实时校正预览
  - 批量校正处理

## 🔧 编译选项

### 编译特定项目
```bash
# 只编译GUI程序
make StereoCalib
make StereoMatchGUI
make stereoRectifier

# 只编译控制台程序
make DualMicroscopeCamera
make StereoReconstruct
make StereoMatchConsole
make test

# 只编译算法库
make elaslib
```

### 编译配置
```bash
# Debug模式编译
cmake -DCMAKE_BUILD_TYPE=Debug ..
make build_all

# Release模式编译
cmake -DCMAKE_BUILD_TYPE=Release ..
make build_all

# 并行编译（使用4个线程）
make -j4 build_all
```

## 📖 使用流程

### 完整的立体视觉处理流程

1. **相机标定**
   ```bash
   # 启动标定程序
   ./StereoCalibration/StereoCalib
   ```
   - 准备标定板（推荐棋盘格）
   - 采集多组标定图像
   - 执行标定计算
   - 保存标定参数

2. **图像采集**
   ```bash
   # 启动双相机采集
   ./DualMicroscopeCamera/DualMicroscopeCamera
   ```
   - 连接双相机
   - 调整相机参数
   - 采集立体图像对

3. **立体匹配**
   ```bash
   # GUI版本
   ./StereoMatch/src/GUI/StereoMatchGUI
   
   # 或控制台版本
   ./StereoMatch/src/console/StereoMatchConsole
   ```
   - 选择匹配算法
   - 调整参数
   - 生成视差图

4. **3D重建**
   ```bash
   ./StereoReconstruction/StereoReconstruct
   ```
   - 输入立体图像对
   - 输入标定参数
   - 生成3D点云

5. **图像校正**
   
   ```bash
   ./StereoRectifier/stereoRectifier
   ```
   - 加载标定参数
   - 校正立体图像对

## ⚙️ 参数配置

### 立体匹配参数

#### SGBM参数
- **BlockSize**: 块大小（通常为奇数）
- **numDisp**: 视差范围
- **minDisp**: 最小视差
- **p1, p2**: 平滑参数
- **preFilterCap**: 预滤波截断值
- **uniquenessRatio**: 唯一性比率

#### BM参数
- **BlockSize**: 块大小
- **numDisp**: 视差范围
- **minDisp**: 最小视差
- **preFilterCap**: 预滤波截断值
- **preFilterSize**: 预滤波核大小

#### ELAS参数
- **support_threshold**: 支持阈值
- **texture_threshold**: 纹理阈值
- **disp_min**: 最小视差
- **disp_max**: 最大视差

## 🐛 故障排除

### 编译问题

#### 1. 找不到OpenCV
```bash
# 检查OpenCV安装
pkg-config --modversion opencv4

# 如果未安装，执行
sudo apt install libopencv-dev
```

#### 2. 找不到PCL
```bash
# 检查PCL安装
sudo apt install libpcl-dev
```

#### 3. 找不到Qt5
```bash
# 检查Qt5安装
sudo apt install qt5-default qtbase5-dev
```

#### 4. CMake版本过低
```bash
# 升级CMake
sudo apt install cmake
```

### 运行时问题

#### 1. GUI程序无法启动
- 检查X11显示环境
- 确认Qt5库正确安装
- 检查显示器设置

#### 2. 相机无法访问
```bash
# 添加用户到video组
sudo usermod -a -G video $USER
# 重新登录生效
```

#### 3. 权限问题
```bash
# 给可执行文件添加执行权限
chmod +x ./StereoCalibration/StereoCalib
chmod +x ./StereoMatch/src/GUI/StereoMatchGUI
chmod +x ./StereoRectifier/stereoRectifier
```

### 性能优化

#### 1. 编译优化
```bash
# Release模式编译
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc) build_all
```

#### 2. 运行时优化
- 使用SSD存储图像数据
- 确保足够的内存（推荐8GB+）
- 使用多核CPU进行并行处理
