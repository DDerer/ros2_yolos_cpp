# ros2_yolos_cpp

ROS 2 YOLOs-CPP 目标检测节点。

基于 [YOLOs-CPP](https://github.com/Geekgineer/YOLOs-CPP) 的高性能 YOLO 推理库，提供目标检测功能。

---

## 安装

### 前置依赖
- **ROS 2**：Humble 或 Jazzy
- **OpenCV**：4.5+
- **ONNX Runtime**：1.16+（构建过程中自动下载）

### 从源码构建
```bash
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build --packages-select ros2_yolos_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 使用方法

### 启动检测节点

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
    model_path:=/path/to/model.onnx \
    labels_path:=/path/to/labels.txt \
    image_topic:=/camera/image_raw
```

启动后需手动切换生命周期状态：

```bash
ros2 lifecycle set /yolos_detector configure
ros2 lifecycle set /yolos_detector activate
```

### 自动启动相机

`detector.launch.py` 可自动启动 `usb_cam` 相机驱动：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
    model_path:=/path/to/model.onnx \
    labels_path:=/path/to/labels.txt \
    camera_params_file:=/path/to/usb_cam_params.yaml \
    image_topic:=/camera/image_raw
```

已有相机驱动时，关闭自动启动：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
    start_camera:=false \
    image_topic:=/your/image/topic \
    model_path:=/path/to/model.onnx \
    labels_path:=/path/to/labels.txt
```

### 距离估算

提供相机内参与目标平面距离，计算检测框中心到相机的距离：

```yaml
/**:
  ros__parameters:
    camera_fx: 812.4
    camera_fy: 809.7
    camera_cx: 323.1
    camera_cy: 242.8
    target_plane_distance_m: 2.0
```

---

## 话题说明

| 话题 | 类型 | 说明 |
|------|------|------|
| `~/image_raw` | `sensor_msgs/Image` | 输入图像 |
| `~/detections` | `vision_msgs/Detection2DArray` | 检测结果 |
| `~/offset_px` | `std_msgs/Int32MultiArray` | 像素偏移 `[x, y]` |
| `~/offset_m` | `std_msgs/Float64MultiArray` | 实际偏移 `[x_m, y_m]` |
| `~/distance_m` | `std_msgs/Float64MultiArray` | 距离 `[平面距离, 直线距离]` |
| `~/timing` | `std_msgs/Float64MultiArray` | 耗时（可选） |

常用查看命令：

```bash
ros2 topic echo /yolos_detector/offset_px
ros2 topic echo /yolos_detector/offset_m
ros2 topic echo /yolos_detector/distance_m
```

---

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | string | **必填** | ONNX 模型路径 |
| `labels_path` | string | "" | 类别名称文件 |
| `use_gpu` | bool | `false` | 启用 GPU |
| `conf_threshold` | double | `0.4` | 置信度阈值 |
| `nms_threshold` | double | `0.45` | NMS IoU 阈值 |
| `image_topic` | string | `/camera/image_raw` | 输入图像话题 |
| `start_camera` | bool | `true` | 自动启动 usb_cam |
| `camera_params_file` | string | `config/usb_cam_params.yaml` | 相机参数文件 |
| `publish_timing` | bool | `false` | 发布推理耗时 |
| `camera_fx` | double | `812.4` | 焦距 x |
| `camera_fy` | double | `809.7` | 焦距 y |
| `camera_cx` | double | `323.1` | 主点 x |
| `camera_cy` | double | `242.8` | 主点 y |
| `target_plane_distance_m` | double | `2.0` | 目标平面距离 |

---

## 许可证

AGPL-3.0
