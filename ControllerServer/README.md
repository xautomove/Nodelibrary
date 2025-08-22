# ControllerServer 控制服务节点

## 描述
ROS2导航控制服务节点，负责执行路径跟踪和机器人控制。

## 功能
- 启动 `nav2_controller` 的 `controller_server` 节点
- 自动管理生命周期状态（配置和激活）
- 启动前检查依赖条件
- 支持DWB局部规划器

## 启动前检查
节点启动前会检查以下条件：
1. `/map_server` 生命周期节点处于激活状态
2. 里程计话题（默认 `/odom`）存在

## 配置参数
- `controller_frequency`: 控制器频率 (默认: 20.0)
- `costmap_update_timeout`: 代价地图更新超时时间 (默认: 0.3)
- `min_x_velocity_threshold`: 最小X方向速度阈值 (默认: 0.001)
- `min_y_velocity_threshold`: 最小Y方向速度阈值 (默认: 0.5)
- `min_theta_velocity_threshold`: 最小角速度阈值 (默认: 0.001)
- `failure_tolerance`: 失败容忍度 (默认: 0.3)
- `odom_topic`: 里程计话题名称 (默认: "odom")
- `required_movement_radius`: 进度检查器所需移动半径 (默认: 0.5)
- `movement_time_allowance`: 移动时间允许值 (默认: 10.0)
- `xy_goal_tolerance`: XY目标容差 (默认: 0.25)
- `yaw_goal_tolerance`: 偏航角目标容差 (默认: 0.25)

### DWB局部规划器批评器参数
- `xy_goal_tolerance_critic`: DWB旋转到目标批评器的XY目标容差 (默认: 0.25)
- `yaw_goal_tolerance_critic`: DWB旋转到目标批评器的偏航角目标容差 (默认: 0.25)
- `oscillation_reset_dist`: 振荡重置距离 (默认: 0.05)
- `forward_point_distance`: 前向点距离 (默认: 0.325)
- `threshold_to_consider`: 路径对齐考虑阈值 (默认: 0.5)
- `offset_from_furthest`: 最远点偏移量 (默认: 20)
- `trajectory_point_step`: 轨迹点步长 (默认: 4)
- `use_path_orientations`: 是否使用路径方向 (默认: false)
- `path_dist_scale`: 路径距离批评器缩放因子 (默认: 32.0)
- `goal_dist_scale`: 目标距离批评器缩放因子 (默认: 24.0)

## 插件配置
- 进度检查器: `nav2_controller::SimpleProgressChecker`
- 目标检查器: `nav2_controller::SimpleGoalChecker`
- 局部规划器: `dwb_core::DWBLocalPlanner`

### DWB批评器
- `RotateToGoalCritic`: 旋转到目标批评器
- `OscillationCritic`: 振荡批评器
- `GoalAlignCritic`: 目标对齐批评器
- `PathAlignCritic`: 路径对齐批评器
- `PathDistCritic`: 路径距离批评器
- `GoalDistCritic`: 目标距离批评器
- `ObstacleFootprintCritic`: 障碍物足迹批评器

## 依赖
- ROS2 Navigation2 包
- nav2_controller
- dwb_core 