## Carla3d 插件

基于 CARLA 的 3D 仿真插件，提供车辆生成、键盘控制、屏幕渲染，并将 LiDAR、GNSS、IMU 等传感器数据通过 ROS2 话题发布。

### 功能特性

- **车辆生成与控制**: 同步模式仿真，WASD 键/空格/ESC 控制
- **多传感器仿真**: LiDAR、GNSS、IMU；可选多摄像头（默认关闭）
- **ROS2 话题发布**: 使用可靠 QoS，固定频率定时发布
- **屏幕渲染**: 内置第一人称相机用于窗口渲染（不发布 ROS 话题）

### 传感器与话题

| 传感器 | 消息类型 | 话题 | frame_id | 发布频率 |
|---|---|---|---|---|
| 激光雷达（LiDAR） | `sensor_msgs/PointCloud2` | `/points_raw` | `velodyne` | 20 Hz |
| GNSS | `sensor_msgs/NavSatFix` | `/carla/gnss` | `gps_link` | 20 Hz |
| IMU | `sensor_msgs/Imu` | `/imu_raw` | `imu_link` | 250 Hz（传感器采样 100 Hz） |
| 摄像头（可选，多路） | `sensor_msgs/Image` | `/carla/camera/{front,left,right}` | `base_link` | ≈20 Hz（默认关闭） |

说明：
- 仿真以同步模式运行（`fixed_delta_seconds = 0.05`），全局步进 20 Hz。
- IMU 定时发布频率配置为 250 Hz，而传感器采样周期为 0.01s（100 Hz）；发布会重复最新帧。
- 单一渲染相机仅用于窗口显示，不发布 ROS2 话题；如需 ROS 发布，请启用多摄像头（见下文）。

### 启用多摄像头发布（可选）

- 在 `spawn_vehicle_and_sensors()` 中启用已注释的多摄像头创建与 `MultiCameraManager` 初始化后，将自动发布：
  - `/carla/camera/front`
  - `/carla/camera/left`
  - `/carla/camera/right`
  - 消息类型 `sensor_msgs/Image`，`frame_id = base_link`，约 20 Hz。

### 键盘控制

- W：前进；S：倒车；A/D：左/右转
- 空格：刹车（含手刹）；ESC：退出

### 依赖与环境

- 需要已安装并运行 CARLA Server 与 Python API
- ROS2 环境（含 `rclpy`、消息定义包）
- Python 依赖（示例）：

```bash
pip install pygame numpy
```

### 数据类型与要点

- LiDAR：点云中包含 `x,y,z,ring,intensity` 字段
- GNSS：提供 `latitude, longitude, altitude`，并设置位置协方差
- IMU：发布姿态（四元数）、角速度、线加速度，含协方差

### 故障排查

- 无法连接 CARLA：检查服务器是否启动、网络和端口（默认 2000）
- 话题无数据：确认仿真运行于同步模式且未暂停、车辆与传感器已成功生成
- IMU 频率偏差：属设计如此（采样 100 Hz，定时发布 250 Hz）


