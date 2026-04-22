<p align="center">
  <img src="https://raw.githubusercontent.com/Geekgineer/YOLOs-CPP/main/data/cover.png" alt="ros2_yolos_cpp" width="100%"/>
</p>

<h1 align="center">ROS 2 YOLOs-CPP</h1>
<h3 align="center">面向 YOLO 推理的高性能 ROS 2 封装</h3>

<p align="center">
  <em>基于 <a href="https://github.com/Geekgineer/YOLOs-CPP">YOLOs-CPP</a> 的生产级 ROS 2 生命周期节点，支持目标检测、分割、姿态估计、OBB 与分类任务。</em>
</p>

<p align="center">
  <a href="https://github.com/Geekgineer/ros2_yolos_cpp/actions"><img src="https://img.shields.io/github/actions/workflow/status/Geekgineer/ros2_yolos_cpp/ci.yml?style=flat-square&label=CI" alt="CI"/></a>
  <a href="https://github.com/Geekgineer/ros2_yolos_cpp/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-AGPL--3.0-ef4444?style=flat-square" alt="License"/></a>
  <a href="https://index.ros.org/p/ros2_yolos_cpp/"><img src="https://img.shields.io/badge/ros-humble%20|%20jazzy-blue?style=flat-square&logo=ros" alt="ROS 2 Versions"/></a>
</p>

---

## 🚀 项目简介

**ros2_yolos_cpp** 将 [YOLOs-CPP](https://github.com/Geekgineer/YOLOs-CPP) 的高性能推理能力与统一 API 带入 ROS 2。它为整个 YOLO 系列模型（v5、v8、v11、v26 等）提供可组合、受生命周期管理的节点实现。

---

## 🎬 Rviz 演示

<table align="center" cellpadding="10">
  <tr>
    <td align="center" style="border:1px solid #ccc">
      <b>姿态估计</b><br>
      <img src="assets/pose.gif" width="400">
    </td>
    <td align="center" style="border:1px solid #ccc">
      <b>目标检测</b><br>
      <img src="assets/object.gif" width="400">
    </td>
  </tr>
  <tr>
    <td colspan="2" align="center" style="border:1px solid #ccc">
      <b>图像分割</b><br>
      <img src="assets/segment.gif" width="400">
    </td>
  </tr>
</table>

---

### 主要特性
- **⚡ 零拷贝传输**：基于 `rclcpp::Subscription` 优化高吞吐图像处理链路。
- **🔄 生命周期管理**：完整支持 `configure`、`activate`、`deactivate`、`shutdown` 状态切换。
- **🛠️ 可组合节点**：支持在同一个容器中运行多个模型，提高资源利用率。
- **📦 全任务覆盖**：支持检测、分割、姿态估计、OBB 与图像分类。
- **🏗️ 面向生产环境**：具备 CI/CD 测试、严格类型参数和标准化消息接口（`vision_msgs`）。

---

## 📥 安装

### 前置依赖
- **ROS 2**：Humble 或 Jazzy
- **OpenCV**：4.5+
- **ONNX Runtime**：1.16+（构建过程中自动下载）

### 从源码构建
```bash
# 创建工作区
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 克隆仓库
git clone https://github.com/Geekgineer/ros2_yolos_cpp.git

# 安装依赖
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -y

# 编译（建议使用 Release 模式以获得更好性能）
colcon build --packages-select ros2_yolos_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 🛠️ 使用方法

本包为每类任务都提供了对应的 launch 文件。你**必须**提供 ONNX 模型路径，并可选提供标签文件路径。

### 1. 目标检测
发布 `vision_msgs/Detection2DArray`，并基于已知相机内参与固定目标平面距离，额外输出像素偏移、平面内实际偏移和相机到目标点的估计距离。

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
    model_path:=/path/to/yolo11n.onnx \
    labels_path:=/path/to/coco.names \
    use_gpu:=true \
    image_topic:=/camera/image_raw \
    params_file:=/path/to/detector_params.yaml
```

检测节点启动后需手动切换生命周期状态：

```bash
ros2 lifecycle set /yolos_detector configure
ros2 lifecycle set /yolos_detector activate
```

`detector.launch.py` 当前采用标准 ROS 图像输入，不再直接访问 `/dev/videoX`。请先运行相机驱动发布 `sensor_msgs/msg/Image` 到 `image_topic`，再启动检测节点。

若需要估算真实距离，请在参数文件中提供一组相机内参与已知平面距离。`config/default_params.yaml` 已给出占位值，可按标定结果替换：

```yaml
/**:
  ros__parameters:
    camera_fx: 812.4
    camera_fy: 809.7
    camera_cx: 323.1
    camera_cy: 242.8
    target_plane_distance_m: 2.0
```

其中：
- `camera_fx`, `camera_fy`：相机焦距，单位为像素
- `camera_cx`, `camera_cy`：主点坐标，单位为像素
- `target_plane_distance_m`：相机到目标所在平面的已知前向距离，单位为米

节点对每个检测框中心点执行如下换算：

```text
X = Z * (u - cx) / fx
Y = Z * (v - cy) / fy
planar_distance = sqrt(X^2 + Y^2)
line_distance = sqrt(planar_distance^2 + Z^2)
```

其中 `(u, v)` 为检测框中心像素坐标，`Z = target_plane_distance_m`。

检测结果由生命周期节点 `/yolos_detector` 发布。节点激活后，可直接从以下话题读取结果：

- `/yolos_detector/detections`：检测框与类别结果
- `/yolos_detector/offset_px`：像素偏移，格式为 `[offset_x_px, offset_y_px]`
- `/yolos_detector/offset_m`：平面内实际偏移，格式为 `[offset_x_m, offset_y_m]`
- `/yolos_detector/distance_m`：距离结果，格式为 `[planar_distance_m, line_distance_m]`
- `/yolos_detector/timing`：耗时统计，格式为 `[preprocess_ms, inference_ms, postprocess_ms]`，仅在启用 `publish_timing` 时发布

常用查看命令：

```bash
ros2 topic echo /yolos_detector/offset_px
ros2 topic echo /yolos_detector/offset_m
ros2 topic echo /yolos_detector/distance_m
```

### 2. 实例分割
发布 `vision_msgs/Detection2DArray` 和同步的掩码图像。

```bash
ros2 launch ros2_yolos_cpp segmentor.launch.py \
    model_path:=/path/to/yolo11n-seg.onnx \
    labels_path:=/path/to/coco.names \
    image_topic:=/camera/image_raw
```

### 3. 姿态估计
发布带关键点信息的 `vision_msgs/Detection2DArray`。

```bash
ros2 launch ros2_yolos_cpp pose.launch.py \
    model_path:=/path/to/yolo11n-pose.onnx \
    image_topic:=/camera/image_raw
```

### 4. 旋转目标框（OBB）
发布自定义消息 `ros2_yolos_cpp/OBBDetection2DArray`，用于表示旋转边界框。

```bash
ros2 launch ros2_yolos_cpp obb.launch.py \
    model_path:=/path/to/yolo11n-obb.onnx \
    labels_path:=/path/to/dota.names \
    image_topic:=/camera/image_raw
```

### 5. 图像分类
发布 `vision_msgs/Classification`。

```bash
ros2 launch ros2_yolos_cpp classifier.launch.py \
    model_path:=/path/to/yolo11n-cls.onnx \
    labels_path:=/path/to/imagenet.names \
    image_topic:=/camera/image_raw
```

---

## 🔄 节点控制

将 `<node_name>` 替换为对应任务的节点名称：
- **Detection**：`/yolos_detector`
- **Segmentation**：`/yolos_segmentor`
- **Pose**：`/yolos_pose`
- **OBB**：`/yolos_obb`
- **Classification**：`/yolos_classifier`

### 1. 启动节点
```bash
ros2 lifecycle set <node_name> configure
ros2 lifecycle set <node_name> activate
```

### 2. 关闭节点
```bash
ros2 lifecycle set <node_name> deactivate
ros2 lifecycle set <node_name> shutdown
```

---

## ⚙️ 配置说明

节点既可以通过 launch 参数配置，也可以通过 YAML 参数文件配置。模板可参考 `config/default_params.yaml`。

| 参数 | 类型 | 默认值 | 说明 |
|-----------|------|---------|-------------|
| `model_path` | string | **必填** | `.onnx` 模型文件的绝对路径。 |
| `labels_path` | string | "" | 类别名称文本文件路径，每行一个类别。 |
| `use_gpu` | bool | `false` | 启用 CUDA 加速，要求使用 GPU 版本构建。 |
| `conf_threshold` | double | `0.4` | 输出检测结果所需的最小置信度。 |
| `nms_threshold` | double | `0.45` | 非极大值抑制（NMS）的 IoU 阈值。 |
| `image_topic` | string | `/camera/image_raw` | 输入图像订阅话题。 |
| `publish_timing` | bool | `false` | 发布推理耗时，格式为 `[preprocess_ms, inference_ms, postprocess_ms]`。 |
| `camera_fx` | double | `812.4` | 相机 x 方向焦距，单位为像素。 |
| `camera_fy` | double | `809.7` | 相机 y 方向焦距，单位为像素。 |
| `camera_cx` | double | `323.1` | 相机主点 x 坐标，单位为像素。 |
| `camera_cy` | double | `242.8` | 相机主点 y 坐标，单位为像素。 |
| `target_plane_distance_m` | double | `2.0` | 相机到目标所在平面的已知前向距离，单位为米。 |

### 话题说明

| 节点类型 | 订阅 | 发布 |
|-----------|---------------|--------------|
| **Detector** | `~/image_raw` | `~/detections` (Detection2DArray)<br>`~/offset_px` (Int32MultiArray, `[offset_x_px, offset_y_px]`)<br>`~/offset_m` (Float64MultiArray, `[offset_x_m, offset_y_m]`)<br>`~/distance_m` (Float64MultiArray, `[planar_distance_m, line_distance_m]`)<br>`~/timing` (Float64MultiArray, optional) |
| **Segmentor** | `~/image_raw` | `~/detections` (Detection2DArray)<br>`~/masks` (Image)<br>`~/debug_image` |
| **Pose** | `~/image_raw` | `~/detections` (Detection2DArray)<br>`~/debug_image` |
| **OBB** | `~/image_raw` | `~/detections` (OBBDetection2DArray)<br>`~/debug_image` |
| **Classifier** | `~/image_raw` | `~/classification` (Classification)<br>`~/debug_image` |

---

## 🐳 Docker

无需在本地安装全部依赖，即可直接运行整套环境。

```bash
# 构建 Docker 镜像
docker build -t ros2_yolos_cpp .

# 启用 GPU 运行
docker run --gpus all -it --rm \
    -v /path/to/models:/models \
    ros2_yolos_cpp \
    ros2 launch ros2_yolos_cpp detector.launch.py model_path:=/models/yolov8n.onnx
```

---

## 📄 许可证

本项目采用 **GNU Affero General Public License v3.0 (AGPL-3.0)** 许可证，详见 [LICENSE](LICENSE)。

<p align="center">
  由 <a href="https://github.com/Geekgineer/YOLOs-CPP">YOLOs-CPP Team</a> 用心打造
</p>
