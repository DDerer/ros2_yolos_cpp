# Detector Parameter Guide

本文档说明 `ros2_yolos_cpp` 中检测节点 `/yolos_detector` 的关键参数应该如何修改，以及这些参数分别影响什么结果。

## 参数修改入口

当前 detector 的参数主要来自两个地方：

1. launch 启动参数
2. YAML 参数文件

对应文件：

- launch 文件：[launch/detector.launch.py](/home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/launch/detector.launch.py:1)
- 默认 YAML 文件：[config/default_params.yaml](/home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/config/default_params.yaml:1)

## 一、必须修改的参数

这些参数不改，节点通常无法正确工作，或者算出来的实际距离没有意义。

### 1. `model_path`

含义：YOLO ONNX 模型路径。  
修改位置：launch 命令传参。  
示例：

```bash
model_path:=/home/ding/models/yolo11n.onnx
```

### 2. `labels_path`

含义：类别名称文件路径。  
修改位置：launch 命令传参。  
示例：

```bash
labels_path:=/home/ding/models/coco.names
```

### 3. `camera_fx`

含义：相机 x 方向焦距，单位是像素。  
修改位置：YAML 文件。  
要求：使用真实标定值，不能长期使用占位值。

### 4. `camera_fy`

含义：相机 y 方向焦距，单位是像素。  
修改位置：YAML 文件。  
要求：使用真实标定值。

### 5. `camera_cx`

含义：相机主点的 x 坐标，单位是像素。  
修改位置：YAML 文件。  
要求：使用真实标定值。

### 6. `camera_cy`

含义：相机主点的 y 坐标，单位是像素。  
修改位置：YAML 文件。  
要求：使用真实标定值。

### 7. `target_plane_distance_m`

含义：相机到目标所在平面的已知前向距离，单位是米。  
修改位置：YAML 文件。  
要求：必须与你的真实场景一致。

这个参数直接参与实际距离计算：

```text
X = Z * (u - cx) / fx
Y = Z * (v - cy) / fy
```

这里的 `Z` 就是 `target_plane_distance_m`。

## 二、通常需要确认的参数

这些参数不是每次都要改，但在换模型、换相机话题、换部署环境时需要确认。

### 1. `image_topic`

含义：输入图像话题。  
修改位置：launch 命令传参。  
默认值：

```bash
/camera/image_raw
```

如果你的相机驱动发布的是别的话题，需要改成对应名称，例如：

```bash
image_topic:=/usb_cam/image_raw
```

### 2. `conf_threshold`

含义：置信度阈值。  
修改位置：launch 命令传参或 YAML。  
建议：

- 漏检较多：适当降低
- 误检较多：适当提高

### 3. `use_gpu`

含义：是否启用 GPU 推理。  
修改位置：launch 命令传参。  
建议：

- 已正确配置 GPU 推理环境：`true`
- 否则：`false`

## 三、一般可以先保持默认的参数

### 1. `nms_threshold`

含义：NMS 阈值。  
如果没有明显重复框问题，通常可以先保持默认。

### 2. `yolo_version`

含义：YOLO 版本选择。  
默认 `auto` 一般足够。

### 3. `publish_timing`

含义：是否发布耗时统计。  
调试性能时可开启，正常使用不是必须。

## 四、建议直接修改的 YAML 段落

你最常改的是这个部分：

文件：[config/default_params.yaml](/home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/config/default_params.yaml:4)

```yaml
/**:
  ros__parameters:
    model_path: ""
    labels_path: ""
    use_gpu: false
    yolo_version: "auto"
    conf_threshold: 0.4
    nms_threshold: 0.45
    publish_timing: false

    camera_fx: 812.4
    camera_fy: 809.7
    camera_cx: 323.1
    camera_cy: 242.8
    target_plane_distance_m: 2.0
```

其中：

- `model_path`、`labels_path` 也可以放在 launch 命令里覆盖
- `camera_fx`、`camera_fy`、`camera_cx`、`camera_cy`、`target_plane_distance_m` 最好固定写在你自己的参数文件中

## 五、推荐做法

不建议直接改默认文件作为长期方案。更推荐复制一份你自己的参数文件，例如：

```bash
cp src/ros2_yolos_cpp/config/default_params.yaml src/ros2_yolos_cpp/config/my_detector_params.yaml
```

然后只修改你自己的：

- `camera_fx`
- `camera_fy`
- `camera_cx`
- `camera_cy`
- `target_plane_distance_m`

启动时指定：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/labels.txt \
  image_topic:=/camera/image_raw \
  params_file:=/home/ding/Documents/code/ros2_ws/src/ros2_yolos_cpp/config/my_detector_params.yaml
```

## 六、修改后如何检查是否生效

启动并激活节点后，可以查看这些结果话题：

```bash
ros2 topic echo /yolos_detector/offset_px
ros2 topic echo /yolos_detector/offset_m
ros2 topic echo /yolos_detector/distance_m
```

如果你只关心最终计算结果，重点看：

```bash
ros2 topic echo /yolos_detector/distance_m
```

返回格式：

```text
[planar_distance_m, line_distance_m]
```

其中：

- 第 1 个值：目标在目标平面内相对中心的距离
- 第 2 个值：相机到目标点的直线距离

## 七、最小修改清单

在你自己的项目环境里，至少确认这 4 件事：

1. `model_path` 是否正确
2. `labels_path` 是否正确
3. `image_topic` 是否和相机驱动一致
4. `camera_fx`、`camera_fy`、`camera_cx`、`camera_cy`、`target_plane_distance_m` 是否替换成真实值

如果这几项没有改对，`/yolos_detector/distance_m` 的结果就不可信。
