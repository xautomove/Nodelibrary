# SlamToolbox 节点

使用Slam Toolbox算法进行SLAM建图的ROS2节点，支持定时保存地图功能。

## 功能特性

- 使用Slam Toolbox算法进行实时SLAM建图
- 支持自定义激光雷达话题、TF话题和里程计话题
- 定时自动保存地图文件（PGM图像和YAML配置文件）
- 可配置的地图分辨率、更新频率等参数
- 自动覆盖保存的地图文件

## 输入参数

- `scan_topic`: 激光雷达话题名称（默认: `/scan`）

## 配置参数

- `save_interval`: 保存地图间隔，单位秒（默认: 30.0）
- `save_path`: 地图保存路径（默认: `/tmp/map`）
- `map_resolution`: 地图分辨率，单位米/像素（默认: 0.05）
- `max_range`: 激光雷达最大范围，单位米（默认: 20.0）
- `update_rate`: 地图更新频率，单位Hz（默认: 5.0）

## 依赖包

- `slam_toolbox`: Slam Toolbox算法包
- `nav_msgs`: 导航消息包
- `sensor_msgs`: 传感器消息包
- `geometry_msgs`: 几何消息包
- `std_msgs`: 标准消息包

## 安装依赖

```bash
sudo apt install ros-humble-slam-toolbox
```

## 输出文件

节点运行后会在指定路径生成以下文件：
- `map.pgm`: 地图图像文件
- `map.yaml`: 地图配置文件

## 使用说明

1. 确保激光雷达数据正常发布到指定话题
2. 确保TF变换树正确配置（base_link -> odom -> map）
3. 启动节点后，Slam Toolbox会自动开始建图
4. 地图会按照设定的时间间隔自动保存
5. 保存的地图文件会自动覆盖之前的文件
6. 地图文件包含PGM图像和YAML配置文件 