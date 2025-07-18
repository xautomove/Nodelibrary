# StaticTransform 静态变换节点

## 节点描述

StaticTransform 是一个基于 ROS2 的静态坐标变换发布节点，用于在机器人系统中建立固定的坐标系变换关系。该节点可以发布一个静态的 TF（Transform）变换，将两个坐标系之间的相对位置和姿态关系广播到整个 ROS2 系统中，供其他节点使用。

### 主要功能
- 发布静态的坐标变换关系
- 支持 3D 平移（x, y, z）和旋转（roll, pitch, yaw）
- 自动将欧拉角转换为四元数
- 持续运行并维护变换关系
- 支持自定义源坐标系和目标坐标系

### 应用场景
- 机器人传感器标定
- 机械臂工具坐标系定义
- 多传感器融合中的坐标系对齐
- 机器人部件间的固定变换关系

## 节点接口

### 输入参数
- **source_frame** (string): 父坐标系名称，作为变换的参考坐标系
- **target_frame** (string): 子坐标系名称，相对于父坐标系的变换目标

### 输出参数
- 无（任务类型节点，持续发布TF变换）

## 详细使用方法

### 1. 基本配置参数

#### 输入参数（通过节点输入连接提供）
- **source_frame** (string): 父坐标系名称，作为变换的参考坐标系
- **target_frame** (string): 子坐标系名称，相对于父坐标系的变换目标

#### 配置参数
- **static_x** (number): X轴平移距离，默认值 0.0
- **static_y** (number): Y轴平移距离，默认值 0.0  
- **static_z** (number): Z轴平移距离，默认值 0.0
- **static_roll** (number): 绕X轴旋转角度（弧度），默认值 0.0
- **static_pitch** (number): 绕Y轴旋转角度（弧度），默认值 0.0
- **static_yaw** (number): 绕Z轴旋转角度（弧度），默认值 0.0

### 2. 配置示例

#### 示例1：简单的平移变换
```json
{
  "static_x": 0.1,
  "static_y": 0.0,
  "static_z": 0.5
}
```
当输入source_frame为"base_link"，target_frame为"camera_link"时，这将在 `base_link` 坐标系基础上，沿X轴平移0.1米，沿Z轴平移0.5米创建 `camera_link` 坐标系。

#### 示例2：包含旋转的变换
```json
{
  "static_x": 0.0,
  "static_y": 0.0,
  "static_z": 0.2,
  "static_roll": 0.0,
  "static_pitch": 0.0,
  "static_yaw": 1.5708
}
```
当输入source_frame为"base_link"，target_frame为"laser_link"时，这将在 `base_link` 坐标系基础上，沿Z轴平移0.2米，并绕Z轴旋转90度（π/2弧度）创建 `laser_link` 坐标系。

#### 示例3：复杂的3D变换
```json
{
  "static_x": 0.3,
  "static_y": 0.1,
  "static_z": 0.4,
  "static_roll": 0.7854,
  "static_pitch": 0.0,
  "static_yaw": 0.5236
}
```
当输入source_frame为"robot_base"，target_frame为"gripper_tool"时，这将在 `robot_base` 坐标系基础上，进行复杂的3D变换创建 `gripper_tool` 坐标系。

### 3. 运行方式

节点启动后会：
1. 通过节点输入接收source_frame和target_frame
2. 初始化 ROS2 节点和 TF 广播器
3. 根据配置参数创建静态变换
4. 持续发布变换关系
5. 在控制台显示运行状态信息

### 4. 验证变换

节点启动后会自动发布静态变换，可以通过以下方式验证：

- 在RViz中添加TF显示类型来可视化坐标系
- 通过其他节点的TF监听功能验证变换是否正确
- 查看节点控制台输出确认变换发布状态

### 5. 注意事项

- 所有角度参数使用弧度制，不是角度制
- 旋转顺序为：先绕Z轴（yaw），再绕Y轴（pitch），最后绕X轴（roll）
- 节点会持续运行直到收到停止信号
- 确保源坐标系在系统中存在，否则变换可能无效
- 建议使用有意义的坐标系名称，便于系统调试和维护
- source_frame和target_frame必须通过节点输入连接提供

### 6. 故障排除

**问题1：变换没有发布**
- 确认节点是否成功启动
- 查看控制台输出信息
- 检查配置参数是否正确
- 确认source_frame和target_frame输入是否正确提供

**问题2：变换不正确**
- 检查坐标系名称是否正确
- 确认角度单位是否为弧度
- 验证变换参数是否符合预期

**问题3：坐标系冲突**
- 确保目标坐标系名称唯一
- 检查是否与其他节点发布的变换冲突 