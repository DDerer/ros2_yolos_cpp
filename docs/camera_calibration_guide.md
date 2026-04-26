# Camera Calibration Guide

本文档说明如何为当前项目使用的普通 USB 摄像头完成单目标定，并将标定结果接入 `ros2_yolos_cpp` 的检测节点。

本文假设你的整体链路是：

```text
USB camera -> usb_cam -> image topic -> camera_calibration -> detector
```

标定的目标是获得相机内参：

- `camera_fx`
- `camera_fy`
- `camera_cx`
- `camera_cy`

以及畸变参数。当前 detector 节点的实际距离估计主要使用前四个参数。

## 一、标定前准备

### 1. 准备硬件

- 一台普通 USB 摄像头
- 一张棋盘格标定板

### 2. 确认棋盘格参数

你需要提前确认两项信息：

- 棋盘格内角点数量，例如 `8x6`
- 每个方格的边长，例如 `0.025` 米

注意：

- `8x6` 指的是内角点数量
- 不是黑白格总数

### 3. 固定实际运行参数

标定时相机的这些设置应尽量与后续检测保持一致：

- 分辨率
- 帧率
- 对焦状态
- 镜头位置

如果后续改了分辨率，通常需要重新标定或重新确认内参。

## 二、启动相机驱动


项目中已提供一个 `usb_cam` 参数文件模板：

[config/usb_cam_params.yaml](/home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/config/usb_cam_params.yaml:1)

如果需要，先修改以下常见参数：

- `video_device`
- `image_width`
- `image_height`
- `pixel_format`

启动命令：

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/config/usb_cam_params.yaml
```

启动后检查图像话题：

```bash
ros2 topic list
ros2 topic info /image_raw
```

如果 `usb_cam` 发布的不是 `/image_raw`，请以你的实际输出为准。

## 三、安装标定工具

ROS 2 中常用的相机标定包是 `camera_calibration`。

安装命令：

```bash
sudo apt install ros-$ROS_DISTRO-camera-calibration
```

## 四、运行单目标定

单目标定常用命令如下：

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/image_raw \
  camera:=/camera
```

参数说明：

- `--size 8x6`：棋盘格内角点数量
- `--square 0.025`：每个方格边长，单位米
- `image:=/image_raw`：图像话题
- `camera:=/camera`：相机命名空间

如果你的实际图像话题不同，例如 `/camera/image_raw`，命令需要对应修改：

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/camera/image_raw \
  camera:=/camera
```

## 五、采集标定图像的正确方式

打开标定界面后，不要只让棋盘格停在正中央。应尽量采集不同视角的有效图像，包括：

- 左右移动
- 上下移动
- 远近变化
- 倾斜角变化
- 画面中心和四周都覆盖到

建议：

- 图像要清晰
- 避免明显模糊
- 避免强反光
- 不要只采集正对镜头的一种姿态

## 六、保存标定结果

标定完成后，工具通常会输出相机矩阵和畸变参数，并允许保存结果。

你最关心的是相机矩阵 `camera_matrix`，通常形式类似：

```yaml
camera_matrix:
  rows: 3
  cols: 3
  data: [812.4, 0.0, 323.1, 0.0, 809.7, 242.8, 0.0, 0.0, 1.0]
```

其中：

- `fx = 812.4`
- `fy = 809.7`
- `cx = 323.1`
- `cy = 242.8`

也就是：

- `camera_fx = K[0]`
- `camera_fy = K[4]`
- `camera_cx = K[2]`
- `camera_cy = K[5]`

## 七、将标定结果写入 detector 参数文件

将标定结果填入 detector 的参数文件：

[config/default_params.yaml](/home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/config/default_params.yaml:1)

建议替换这一段：

```yaml
camera_fx: 812.4
camera_fy: 809.7
camera_cx: 323.1
camera_cy: 242.8
target_plane_distance_m: 2.0
```

其中：

- `camera_fx`、`camera_fy`、`camera_cx`、`camera_cy` 来自标定
- `target_plane_distance_m` 来自你对场景的实际测量

如果你不想直接改默认文件，建议复制出你自己的参数文件，例如：

```bash
cp /home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/config/default_params.yaml \
   /home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/config/my_detector_params.yaml
```

然后在自己的文件里修改内参。

## 八、运行 detector 验证结果

启动 detector：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/labels.txt \
  image_topic:=/image_raw \
  params_file:=/home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/config/default_params.yaml
```

再激活生命周期节点：

```bash
ros2 lifecycle set /yolos_detector configure
ros2 lifecycle set /yolos_detector activate
```

查看输出：

```bash
ros2 topic echo /yolos_detector/offset_m
ros2 topic echo /yolos_detector/distance_m
```

其中：

- `/yolos_detector/offset_m`：平面内偏移 `[offset_x_m, offset_y_m]`
- `/yolos_detector/distance_m`：距离结果 `[planar_distance_m, line_distance_m]`

## 九、常见问题

### 1. 改了相机分辨率，还能继续用原来的内参吗？

通常不建议。  
一旦分辨率、裁剪方式或焦距状态发生变化，内参往往也要重新确认，最好重新标定。

### 2. 为什么标定后 `cx`、`cy` 不等于图像中心？

这是正常的。  
`cx`、`cy` 表示相机主点，不一定严格等于图像几何中心。

### 3. 检测结果距离不准，应该先查什么？

优先检查：

1. `camera_fx`、`camera_fy`、`camera_cx`、`camera_cy` 是否来自真实标定
2. `target_plane_distance_m` 是否正确
3. 当前分辨率是否和标定时一致
4. 检测框中心是否接近你真正想测的目标点

## 十、推荐的完整顺序

建议按照下面的顺序进行：

1. 配置并启动 `usb_cam`
2. 确认图像话题
3. 使用 `camera_calibration` 对棋盘格做单目标定
4. 记录 `fx`、`fy`、`cx`、`cy`
5. 更新 detector 参数文件
6. 启动 detector 并查看 `/yolos_detector/distance_m`

完成后，你的 detector 节点就能基于真实相机内参进行实际偏移和距离估计。
