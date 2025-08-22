import yaml


class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.odom_topic = "odom"
        self.cmd_vel_topic = "cmd_vel"
        self.sdk.debug("ControllerServer 精简模式初始化")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        if 'odom_topic' in config:
            odom_val = config['odom_topic']
            if isinstance(odom_val, str) and odom_val.strip() != "":
                self.odom_topic = odom_val
        if 'cmd_vel_topic' in config:
            cmd_val = config['cmd_vel_topic']
            if isinstance(cmd_val, str) and cmd_val.strip() != "":
                self.cmd_vel_topic = cmd_val

    def get_config_value(self, key, default=None):
        if isinstance(self.config, list):
            for item in self.config:
                if isinstance(item, dict) and item.get('name') == key:
                    value = item.get('default_value', item.get('value', default))
                    if item.get('type') == 'number':
                        try:
                            return float(value) if value is not None else default
                        except (ValueError, TypeError):
                            return default
                    elif item.get('type') == 'boolean':
                        if isinstance(value, str):
                            return value.lower() in ['true', '1', 'yes', 'on']
                        return bool(value)
                    else:
                        return value
        elif isinstance(self.config, dict):
            return self.config.get(key, default)
        return default

    def execute(self):
        try:
            controller_frequency = float(self.get_config_value("controller_frequency", 20.0))
            costmap_update_timeout = float(self.get_config_value("costmap_update_timeout", 0.3))
            min_x_velocity_threshold = float(self.get_config_value("min_x_velocity_threshold", 0.001))
            min_y_velocity_threshold = float(self.get_config_value("min_y_velocity_threshold", 0.0))
            min_theta_velocity_threshold = float(self.get_config_value("min_theta_velocity_threshold", 0.001))
            failure_tolerance = float(self.get_config_value("failure_tolerance", 0.3))
            required_movement_radius = float(self.get_config_value("required_movement_radius", 0.5))
            movement_time_allowance = float(self.get_config_value("movement_time_allowance", 10.0))
            xy_goal_tolerance = float(self.get_config_value("xy_goal_tolerance", 0.25))
            yaw_goal_tolerance = float(self.get_config_value("yaw_goal_tolerance", 0.25))

            xy_goal_tolerance_critic = float(self.get_config_value("xy_goal_tolerance_critic", 0.25))
            yaw_goal_tolerance_critic = float(self.get_config_value("yaw_goal_tolerance_critic", 0.25))
            oscillation_reset_dist = float(self.get_config_value("oscillation_reset_dist", 0.05))
            forward_point_distance = float(self.get_config_value("forward_point_distance", 0.325))
            threshold_to_consider = float(self.get_config_value("threshold_to_consider", 0.5))
            offset_from_furthest = int(self.get_config_value("offset_from_furthest", 20))
            trajectory_point_step = int(self.get_config_value("trajectory_point_step", 4))
            use_path_orientations = self.get_config_value("use_path_orientations", False)
            path_dist_scale = float(self.get_config_value("path_dist_scale", 32.0))
            goal_dist_scale = float(self.get_config_value("goal_dist_scale", 24.0))

            # 顶层附加参数
            use_realtime_priority = self.get_config_value("use_realtime_priority", False)
            speed_limit_topic = self.get_config_value("speed_limit_topic", "speed_limit")

            # 话题名配置，允许通过配置覆盖，并做空值防御
            # 优先使用节点输入的 odom_topic，不从 config 覆盖
            odom_topic_cfg = self.odom_topic if isinstance(self.odom_topic, str) and self.odom_topic.strip() != "" else "odom"
            cmd_vel_topic_cfg = self.get_config_value("cmd_vel_topic", self.cmd_vel_topic)
            if not isinstance(cmd_vel_topic_cfg, str) or cmd_vel_topic_cfg.strip() == "":
                cmd_vel_topic_cfg = "cmd_vel"

            # DWB 关键运动学与采样参数
            min_vel_x = float(self.get_config_value("min_vel_x", 0.0))
            max_vel_x = float(self.get_config_value("max_vel_x", 0.5))
            max_vel_theta = float(self.get_config_value("max_vel_theta", 1.9))
            acc_lim_x = float(self.get_config_value("acc_lim_x", 2.5))
            acc_lim_theta = float(self.get_config_value("acc_lim_theta", 3.2))
            vx_samples = int(self.get_config_value("vx_samples", 20))
            vtheta_samples = int(self.get_config_value("vtheta_samples", 20))
            sim_time = float(self.get_config_value("sim_time", 2.0))
            linear_granularity = float(self.get_config_value("linear_granularity", 0.05))
            angular_granularity = float(self.get_config_value("angular_granularity", 0.025))
            transform_tolerance = float(self.get_config_value("transform_tolerance", 0.2))

            params_config = {
                'controller_server': {
                    'ros__parameters': {
                        'controller_frequency': controller_frequency,
                        'costmap_update_timeout': costmap_update_timeout,
                        'min_x_velocity_threshold': min_x_velocity_threshold,
                        'min_y_velocity_threshold': min_y_velocity_threshold,
                        'min_theta_velocity_threshold': min_theta_velocity_threshold,
                        'failure_tolerance': failure_tolerance,
                        'odom_topic': odom_topic_cfg,
                        'cmd_vel_topic': cmd_vel_topic_cfg,
                        'use_realtime_priority': use_realtime_priority,
                        'speed_limit_topic': speed_limit_topic,
                        'progress_checker_plugins': ["progress_checker"],
                        'goal_checker_plugins': ["general_goal_checker"],
                        'controller_plugins': ["FollowPath"],
                        'progress_checker': {
                            'plugin': "nav2_controller::SimpleProgressChecker",
                            'required_movement_radius': required_movement_radius,
                            'movement_time_allowance': movement_time_allowance
                        },
                        'general_goal_checker': {
                            'plugin': "nav2_controller::SimpleGoalChecker",
                            'xy_goal_tolerance': xy_goal_tolerance,
                            'yaw_goal_tolerance': yaw_goal_tolerance,
                            'stateful': True
                        },
                        'FollowPath': {
                            'plugin': "dwb_core::DWBLocalPlanner",
                            'min_vel_x': min_vel_x,
                            'max_vel_x': max_vel_x,
                            'max_vel_theta': max_vel_theta,
                            'acc_lim_x': acc_lim_x,
                            'acc_lim_theta': acc_lim_theta,
                            'vx_samples': vx_samples,
                            'vtheta_samples': vtheta_samples,
                            'sim_time': sim_time,
                            'linear_granularity': linear_granularity,
                            'angular_granularity': angular_granularity,
                            'transform_tolerance': transform_tolerance,
                            'critics': [
                                "RotateToGoal",
                                "Oscillation",
                                "GoalAlign",
                                "PathAlign",
                                "PathDist",
                                "GoalDist",
                                "ObstacleFootprint"
                            ],
                            'RotateToGoal': {
                                'enabled': True,
                                'xy_goal_tolerance': xy_goal_tolerance_critic,
                                'yaw_goal_tolerance': yaw_goal_tolerance_critic
                            },
                            'Oscillation': {
                                'enabled': True,
                                'oscillation_reset_dist': oscillation_reset_dist
                            },
                            'GoalAlign': {
                                'enabled': True,
                                'forward_point_distance': forward_point_distance
                            },
                            'PathAlign': {
                                'enabled': True,
                                'threshold_to_consider': threshold_to_consider,
                                'offset_from_furthest': offset_from_furthest,
                                'trajectory_point_step': trajectory_point_step,
                                'use_path_orientations': use_path_orientations
                            },
                            'PathDist': {
                                'enabled': True,
                                'scale': path_dist_scale
                            },
                            'GoalDist': {
                                'enabled': True,
                                'scale': goal_dist_scale
                            },
                            'ObstacleFootprint': {
                                'enabled': True
                            }
                        }
                    }
                }
            }

            cache_key = 'nav_controller_server'
            cache_val = yaml.dump(params_config, default_flow_style=False, allow_unicode=True)
            self.cache.set(cache_key, cache_val)
            self.sdk.output({"控制配置": cache_key})
            self.sdk.finish()
        except Exception as e:
            self.sdk.debug(f"写入缓存失败: {e}")
            self.sdk.error() 