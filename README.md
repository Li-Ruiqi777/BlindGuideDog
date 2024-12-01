# 基于宇树A1的导盲犬项目

## 项目简介

本项目为我的本科毕设——《基于图像分割的四足导盲机器人循迹软件设计》的代码。项目运行于宇树A1四足机器人的主板 Jetson Xavier NX 上，核心功能包括：

1. 使用 **PP-LiteSeg** 语义分割模型对盲道进行分割；
2. 利用 **RANSAC 算法**对盲道边缘进行直线拟合，并计算盲道中线；
3. 以“让盲道中线中点的横坐标位于图像中心”为控制目标，通过 **PID 控制算法**对机器人的速度进行调整，实现盲道循迹功能。



## 演示视频

https://www.bilibili.com/video/BV1ca4y157hs/?spm_id_from=333.999.0.0&vd_source=0a0f4621dff300eac623b735ee922ef



## 运行环境

### 硬件

- **主控板**：Jetson Xavier NX
- **机器人**：宇树 A1

### 第三方库

- **Paddle Inference**
- **OpenCV**
- **Realsense SDK**



## 快速开始

### 1. 配置依赖路径

打开项目根目录下的 `CMakeLists.txt` 文件，将其中第三方库的位置修改为你本地的实际路径。

### 2. 构建项目

1. 创建构建目录：

   ```bash
   mkdir build  
   cd build  
   ```

2. 生成 Makefile：

   ```bash
   cmake ..  
   ```

   > 如果配置正确，此步骤会成功生成 `Makefile` 文件。

3. 编译项目：

   ```bash
   make  
   ```

4. 执行生成的可执行文件 `Infer`：

   ```bash
   ./Infer  
   ```
