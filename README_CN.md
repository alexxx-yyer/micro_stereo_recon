# 微立体重建项目 (Micro Stereo Reconstruction)

**[中文文档](README_CN.md) | [English Documentation](README.md)**

> 一个完整的立体视觉处理系统，包含立体标定、立体匹配和3D重建功能。支持Linux和Windows双平台。

## 📋 项目简介

本项目是一个完整的立体视觉处理系统，包含以下功能：

- **立体标定**：立体视觉系统的相机标定
- **立体匹配**：多种算法（SGBM、BM、ELAS），支持后处理滤波器（WLS、FBS）
- **3D重建**：从立体图像对生成点云
- **图像校正**：立体图像对校正
- **双相机采集**：同步双显微镜相机采集

项目提供**图形用户界面（GUI）**和**命令行工具**，适用于不同使用场景。

## 🌿 分支结构

本仓库包含多个分支，适用于不同平台：

- **`main`**：主分支，包含跨平台文档
- **`dev_linux`**：Linux开发分支（Ubuntu 24.04 LTS）
  - 完整的Linux支持，包含Qt5 GUI
  - 详细Linux设置请参考 [dev_linux 分支 README](https://github.com/your-repo/micro_stereo_recon/tree/dev_linux)
- **`dev_win`**：Windows开发分支
  - Windows特定配置
  - 详细Windows设置请参考 [dev_win 分支 README](https://github.com/your-repo/micro_stereo_recon/tree/dev_win)

### 我应该使用哪个分支？

- **Linux用户**：切换到 `dev_linux` 分支
  ```bash
  git checkout dev_linux
  ```
- **Windows用户**：切换到 `dev_win` 分支
  ```bash
  git checkout dev_win
  ```
- **仅查看文档**：保持在 `main` 分支

## 🖥️ 平台支持

### Linux (dev_linux 分支)

**系统要求：**
- Ubuntu 24.04 LTS (Linux 6.14.0-29-generic)
- GCC 14.2.0
- CMake 3.16+
- C++17

**依赖库：**
- OpenCV 4.6.0
- PCL 1.14.0
- Qt5 5.15.13
- Boost, FLANN, Eigen3, VTK, OpenMP

**快速开始：**
```bash
git checkout dev_linux
mkdir build && cd build
cmake ..
make build_all
```

详细的Linux设置说明，请参考 [dev_linux README](https://github.com/your-repo/micro_stereo_recon/tree/dev_linux)。

### Windows (dev_win 分支)

**系统要求：**
- Windows 10/11
- Visual Studio 2019 或更高版本
- CMake 3.16+

**依赖库：**
- OpenCV (Windows版本)
- PCL (Windows版本)
- Qt5 (Windows版本)

**快速开始：**
```bash
git checkout dev_win
# 按照Windows特定的构建说明操作
```

详细的Windows设置说明，请参考 [dev_win README](https://github.com/your-repo/micro_stereo_recon/tree/dev_win)。

## 🚀 快速开始

### 1. 克隆仓库

```bash
git clone <your-repository-url>
cd micro_stereo_recon
```

### 2. 选择平台分支

```bash
# Linux用户
git checkout dev_linux

# Windows用户
git checkout dev_win
```

### 3. 构建项目

**Linux:**
```bash
mkdir build && cd build
cmake ..
make build_all
```

**Windows:**
```bash
# 使用Visual Studio或CMake GUI
# 按照平台特定的说明操作
```

## 📁 项目结构

```
micro_stereo_recon/
├── CMakeLists.txt              # 主构建文件
├── README.md                   # 本文档（英文）
├── README_CN.md                # 本文档（中文）
├── DualMicroscopeCamera/      # 双相机采集模块
│   ├── main_linux.cpp         # Linux版本
│   └── main.cpp               # Windows版本
├── StereoCalibration/         # 立体标定模块
│   ├── panel.h/.cpp          # GUI界面
│   └── test.cpp              # 标定测试
├── StereoMatch/              # 立体匹配模块
│   ├── src/
│   │   ├── console/          # 控制台版本
│   │   ├── GUI/              # GUI版本
│   │   └── include/          # 算法头文件
│   └── 3rdparty/elas/        # ELAS算法库
├── StereoReconstruction/     # 3D重建模块
├── StereoRectifier/          # 立体校正模块
└── CustomLib-CV/            # 自定义CV库
    ├── DirTools.h/.cpp
    ├── StereoMatchingTools.h/.cpp
    ├── VisualizationTools.h/.cpp
    └── OCV_PCL.h/.cpp
```

## 🎯 功能特性

### 算法支持

- **SGBM (半全局块匹配)**：高质量立体匹配，支持后处理
  - WLS (加权最小二乘) 滤波器
  - FBS (快速双边求解器) 滤波器
  - 可配置参数
- **BM (块匹配)**：快速块匹配算法
- **ELAS (高效大规模立体)**：高效大规模立体匹配

### 应用程序

- **StereoCalib**：立体相机标定GUI
- **StereoMatchGUI**：立体匹配GUI，支持实时预览
- **StereoMatchConsole**：批量处理命令行工具
- **StereoReconstruct**：3D点云生成
- **stereoRectifier**：立体图像校正
- **DualMicroscopeCamera**：双相机同步采集

## 📖 使用流程

1. **相机标定** → 生成标定参数
2. **图像采集** → 采集立体图像对
3. **立体匹配** → 生成视差图
4. **3D重建** → 创建3D点云
5. **图像校正** → 校正立体图像对

详细的使用说明，请参考 `dev_linux` 或 `dev_win` 分支中的平台特定README。

## 🔧 编译选项

### 编译所有项目
```bash
# Linux
make build_all

# Windows
# 使用Visual Studio解决方案或CMake
```

### 编译特定模块
```bash
# GUI程序
make StereoCalib
make StereoMatchGUI
make stereoRectifier

# 控制台程序
make DualMicroscopeCamera
make StereoReconstruct
make StereoMatchConsole
```

## 📚 文档

- **主文档**：本README（中文）
- **English Documentation**: See `README.md`
- **Linux详细信息**：参考 `dev_linux` 分支 README
- **Windows详细信息**：参考 `dev_win` 分支 README

## 🤝 贡献

这是一个本科毕业论文项目。如需贡献或提问：

1. 查看相应的开发分支（`dev_linux` 或 `dev_win`）
2. 阅读平台特定的文档
3. 遵循编码规范和构建流程

## 📝 许可证

本项目是本科毕业论文的一部分。详细信息请参考许可证文件。

## 🔗 链接

- **Linux开发**：[dev_linux 分支](https://github.com/your-repo/micro_stereo_recon/tree/dev_linux)
- **Windows开发**：[dev_win 分支](https://github.com/your-repo/micro_stereo_recon/tree/dev_win)
- **English Documentation**: `README.md`

## 📧 联系方式

如有问题或疑问，请参考平台特定的分支获取详细的支持信息。

---

**注意**：这是主分支，包含跨平台概览。如需平台特定的设置和详细说明，请切换到相应的开发分支（`dev_linux` 或 `dev_win`）。

