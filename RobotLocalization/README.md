# 机器人定位融合器

## 功能描述
基于robot_localization的多传感器数据融合节点，支持GNSS、IMU、里程计数据融合。该节点使用扩展卡尔曼滤波器(EKF)将多个传感器的数据融合，提供更准确的机器人定位信息。

### 主要功能
- 多传感器数据融合（GNSS、IMU、里程计）
- 基于扩展卡尔曼滤波器的状态估计
- 自动TF变换发布
- 支持2D/3D运动模式
- 灵活的传感器配置

### 应用场景
- 自动驾驶车辆定位
- 移动机器人导航
- 无人机定位系统
- 多传感器融合定位

## 节点接口

### 输入参数
- **gnss_topic** (string): GNSS数据话题名称，通过节点输入连接提供
- **imu_topic** (string): IMU数据话题名称，通过节点输入连接提供
- **odom_topic** (string): 里程计数据话题名称，通过节点输入连接提供

### 输出参数
- **fused_topic** (string): 融合后的输出话题名称，通过节点输出连接提供

## 配置参数

### 基本参数
- **frequency**: 滤波器更新频率，30Hz一般足够（默认：30.0）
- **sensor_timeout**: 超时时间，单位秒，避免死锁（默认：0.1）
- **two_d_mode**: 若是纯平面运动（无飞行、爬楼）设为true（默认：true）
- **publish_tf**: 是否自动发布TF（建议为true）（默认：true）

### 坐标系参数
- **map_frame**: 世界参考坐标系（固定）（默认：map）
- **odom_frame**: 滤波输出的中间参考系（默认：odom）
- **base_link_frame**: 机器人本体坐标系（与TF一致）（默认：base_link）
- **world_frame**: 滤波器输出参考帧（建议为map）（默认：map）

### 输出参数
- **fused_topic**: 融合后的输出话题名称（默认：/fused_odom）

### 传感器向量配置
- **imu_vector**: IMU数据向量配置，格式：[x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az]，默认为空（全false）
- **odom_vector**: 里程计数据向量配置，格式：[x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az]，默认为空（全false）
- **gnss_vector**: GNSS数据向量配置，格式：[x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az]，默认为空（全false）

## 传感器向量说明

传感器向量是一个15维的布尔数组，表示哪些状态变量可以从该传感器获得：

### 向量索引含义
- [0-2]: x, y, z 位置
- [3-5]: roll, pitch, yaw 姿态
- [6-8]: vx, vy, vz 线速度
- [9-11]: vroll, vpitch, vyaw 角速度
- [12-14]: ax, ay, az 线加速度

### 常用配置示例

#### IMU向量（典型配置）
```json
[false, false, false, false, false, false, false, false, false, true, true, true, true, true, true]
```
表示IMU提供角速度和线加速度信息。

#### 里程计向量（典型配置）
```json
[true, true, false, false, false, true, true, true, false, false, false, true, false, false, false]
```
表示里程计提供位置和线速度信息。

#### GNSS向量（典型配置）
```json
[true, true, false, false, false, false, false, false, false, false, false, false, false, false, false]
```
表示GNSS提供位置信息。

## 工作原理

1. **输入接收**: 通过节点输入接收传感器话题名称
2. **配置解析**: 解析传感器向量配置和基本参数
3. **EKF启动**: 动态生成并启动robot_localization的EKF节点
4. **数据融合**: EKF节点自动订阅传感器数据并进行融合
5. **结果输出**: 发布融合后的定位信息和TF变换
6. **进程管理**: 监控EKF进程状态，确保稳定运行

## 使用说明

节点会在客户端环境中自动启动，通过配置文件进行参数设置。传感器话题名称通过节点输入连接提供，融合话题名称通过配置项设置。

### 配置步骤
1. 通过节点输入连接提供传感器话题名称
2. 在配置中设置相应的传感器向量
3. 配置基本参数（频率、超时等）
4. 设置输出话题名称

## 输入输出

### 订阅话题
- **NavSatFix**: GNSS定位数据（话题名称通过节点输入提供）
- **Imu**: IMU传感器数据（话题名称通过节点输入提供）
- **Odometry**: 里程计数据（话题名称通过节点输入提供）

### 发布话题
- **Odometry**: 融合后的定位信息（话题名称通过配置设置）
- **TF**: 坐标系变换（如果publish_tf为true）

## 可视化

在RViz中可视化融合结果：
1. 添加Odometry显示类型
2. 设置话题为配置的fused_topic值
3. 添加TF显示类型查看坐标系变换

## 注意事项

1. **传感器配置**: 必须为每个活跃传感器配置正确的向量
2. **话题有效性**: 确保输入的话题名称在系统中存在
3. **坐标系一致性**: 确保所有传感器使用相同的坐标系
4. **数据质量**: 传感器数据质量直接影响融合效果
5. **计算资源**: EKF计算量较大，注意系统性能
6. **依赖要求**: 需要安装robot_localization包

## 故障排除

**问题1：EKF节点启动失败**
- 检查robot_localization包是否正确安装
- 确认传感器向量配置格式是否正确
- 查看控制台输出确认错误信息

**问题2：融合效果不佳**
- 检查传感器向量配置是否与实际数据匹配
- 确认传感器数据质量和坐标系一致性
- 调整frequency和sensor_timeout参数

**问题3：TF变换异常**
- 检查坐标系名称配置是否正确
- 确认publish_tf参数设置
- 验证坐标系层次结构

## 依赖

- ROS2
- robot_localization
- sensor_msgs
- nav_msgs
- geometry_msgs
- tf2_ros

## 安装依赖

```bash
# 安装robot_localization包
sudo apt update
sudo apt install ros-humble-robot-localization
``` 