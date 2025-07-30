# MapServer 地图服务器节点

## 功能描述

MapServer节点用于启动和管理nav2_map_server，加载并发布栅格地图。该节点基于ROS2官方的nav2_map_server，支持标准的ROS2地图格式（.yaml + .pgm），将地图数据发布为`nav_msgs/OccupancyGrid`类型的消息，供导航系统使用。

启动后会自动配置和激活生命周期节点，确保地图服务器正常工作。

## 主要特性

- 基于ROS2官方nav2_map_server
- 支持标准ROS2地图格式（YAML + PGM）
- 自动解析地图元数据（分辨率、原点、阈值等）
- 可配置的发布周期和话题名称
- 支持地图坐标系自定义
- 兼容nav2导航系统
- 进程管理和监控

## 配置参数

### 必需参数

- **map_file** (string): 地图YAML文件的完整路径
  - 示例: `/path/to/map.yaml`

### 可选参数

- **frame_id** (string): 地图坐标系ID，默认为"map"
- **topic_name** (string): 地图发布话题名称，默认为"map"
- **use_compressed** (boolean): 是否使用压缩地图，默认为false

## 地图文件格式

### YAML文件示例

```yaml
image: map.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.25
negate: 0
```

### 参数说明

- **image**: PGM图像文件名
- **resolution**: 地图分辨率（米/像素）
- **origin**: 地图原点在世界坐标系中的位置 [x, y, yaw]
- **occupied_thresh**: 占用阈值（0-1），超过此值认为是占用区域
- **free_thresh**: 空闲阈值（0-1），低于此值认为是空闲区域
- **negate**: 是否反转图像（0或1）

## 输出

节点发布`nav_msgs/OccupancyGrid`类型的消息到指定话题，包含：

- 地图尺寸信息（宽度、高度、分辨率）
- 地图原点位置
- 占用栅格数据（0=空闲，100=占用，-1=未知）

## 使用方法

1. 准备地图文件（.yaml和对应的.pgm文件）
2. 配置节点参数，特别是地图文件路径
3. 启动节点，系统会自动：
   - 启动nav2_map_server进程
   - 配置生命周期节点（configure）
   - 激活生命周期节点（activate）
   - 发布地图数据到指定话题


## 依赖项

- ROS2
- nav2_map_server
- nav_msgs

### 安装命令

```bash
# Ubuntu/Debian系统
sudo apt update
sudo apt install ros-humble-nav2-map-server
```

## 注意事项

- 确保地图文件路径正确且文件存在
- PGM文件应与YAML文件在同一目录下
- 地图分辨率应与实际环境匹配
- 需要安装nav2_map_server包
- 节点会自动管理nav2_map_server进程的生命周期
- 建议定期发布地图以确保导航系统能获取最新地图数据 