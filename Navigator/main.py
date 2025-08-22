import yaml


class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.global_frame = 'map'
        self.robot_base_frame = 'base_link'
        self.sdk.debug("Navigator 精简模式初始化")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        if 'global_frame' in config and isinstance(config['global_frame'], str) and config['global_frame'].strip() != '':
            self.global_frame = config['global_frame']
        if 'robot_base_frame' in config and isinstance(config['robot_base_frame'], str) and config['robot_base_frame'].strip() != '':
            self.robot_base_frame = config['robot_base_frame']

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
            # 上游输入优先，其次使用配置回退
            global_frame = self.global_frame if isinstance(self.global_frame, str) and self.global_frame.strip() != '' else self.get_config_value('global_frame', 'map')
            robot_base_frame = self.robot_base_frame if isinstance(self.robot_base_frame, str) and self.robot_base_frame.strip() != '' else self.get_config_value('robot_base_frame', 'base_link')
            use_sim_time = self.get_config_value('use_sim_time', False)
            transform_tolerance = float(self.get_config_value('transform_tolerance', 0.1))
            filter_duration = float(self.get_config_value('filter_duration', 0.3))
            always_reload_bt_xml = self.get_config_value('always_reload_bt_xml', False)
            introspection_mode = self.get_config_value('introspection_mode', 'disabled')

            goal_blackboard_id = self.get_config_value('goal_blackboard_id', 'goal')
            goals_blackboard_id = self.get_config_value('goals_blackboard_id', 'goals')
            path_blackboard_id = self.get_config_value('path_blackboard_id', 'path')
            waypoint_statuses_blackboard_id = self.get_config_value('waypoint_statuses_blackboard_id', 'waypoint_statuses')

            navigate_to_pose_enable_groot = self.get_config_value('navigate_to_pose_enable_groot', False)
            navigate_to_pose_groot_port = int(self.get_config_value('navigate_to_pose_groot_port', 1667))
            navigate_through_poses_enable_groot = self.get_config_value('navigate_through_poses_enable_groot', False)
            navigate_through_poses_groot_port = int(self.get_config_value('navigate_through_poses_groot_port', 1669))

            # 行为树 XML 路径（可配置，使用给定示例为默认）
            default_nav_to_pose_bt_xml = self.get_config_value(
                'default_nav_to_pose_bt_xml',
                '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
            )
            default_nav_through_poses_bt_xml = self.get_config_value(
                'default_nav_through_poses_bt_xml',
                '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml'
            )

            params_config = {
                'bt_navigator': {
                    'ros__parameters': {
                        'use_sim_time': use_sim_time,
                        'global_frame': global_frame,
                        'robot_base_frame': robot_base_frame,
                        'transform_tolerance': transform_tolerance,
                        'filter_duration': filter_duration,
                        'introspection_mode': introspection_mode,
                        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                        'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
                        'always_reload_bt_xml': always_reload_bt_xml,
                        'goal_blackboard_id': goal_blackboard_id,
                        'goals_blackboard_id': goals_blackboard_id,
                        'path_blackboard_id': path_blackboard_id,
                        'waypoint_statuses_blackboard_id': waypoint_statuses_blackboard_id,
                        'navigators': ['navigate_to_pose', 'navigate_through_poses'],
                        'navigate_to_pose': {
                            'plugin': 'nav2_bt_navigator/NavigateToPoseNavigator',
                            'enable_groot_monitoring': navigate_to_pose_enable_groot,
                            'groot_server_port': navigate_to_pose_groot_port
                        },
                        'navigate_through_poses': {
                            'plugin': 'nav2_bt_navigator/NavigateThroughPosesNavigator',
                            'enable_groot_monitoring': navigate_through_poses_enable_groot,
                            'groot_server_port': navigate_through_poses_groot_port
                        },
                        'plugin_lib_names': [
                            'nav2_compute_path_to_pose_action_bt_node',
                            'nav2_follow_path_action_bt_node',
                            'nav2_back_up_action_bt_node',
                            'nav2_spin_action_bt_node',
                            'nav2_wait_action_bt_node',
                            'nav2_clear_costmap_service_bt_node',
                            'nav2_is_stuck_condition_bt_node',
                            'nav2_goal_reached_condition_bt_node',
                            'nav2_initial_pose_received_condition_bt_node',
                            'nav2_goal_updated_condition_bt_node',
                            'nav2_reinitialize_global_localization_service_bt_node',
                            'nav2_rate_controller_bt_node',
                            'nav2_distance_controller_bt_node',
                            'nav2_speed_controller_bt_node',
                            'nav2_recovery_node_bt_node',
                            'nav2_pipeline_sequence_bt_node',
                            'nav2_round_robin_node_bt_node',
                            'nav2_transform_available_condition_bt_node',
                            'nav2_time_expired_condition_bt_node',
                            'nav2_distance_traveled_condition_bt_node',
                            'nav2_single_trigger_bt_node'
                        ],
                        'error_code_name_prefixes': [
                            'assisted_teleop',
                            'backup',
                            'compute_path',
                            'dock_robot',
                            'drive_on_heading',
                            'follow_path',
                            'nav_thru_poses',
                            'nav_to_pose',
                            'spin',
                            'route',
                            'undock_robot',
                            'wait'
                        ]
                    }
                },
                'behavior_server': {
                    'ros__parameters': {
                        'local_costmap_topic': 'local_costmap/costmap_raw',
                        'global_costmap_topic': 'global_costmap/costmap_raw',
                        'local_footprint_topic': 'local_costmap/published_footprint',
                        'global_footprint_topic': 'global_costmap/published_footprint',
                        'behavior_plugins': [
                            'spin',
                            'backup',
                            'drive_on_heading',
                            'wait'
                        ],
                        'spin': {
                            'plugin': 'nav2_behaviors/Spin'
                        },
                        'backup': {
                            'plugin': 'nav2_behaviors/BackUp'
                        },
                        'drive_on_heading': {
                            'plugin': 'nav2_behaviors/DriveOnHeading'
                        },
                        'wait': {
                            'plugin': 'nav2_behaviors/Wait'
                        },
                        'local_frame': 'odom',
                        'global_frame': global_frame,
                        'robot_base_frame': robot_base_frame,
                        'transform_tolerance': transform_tolerance,
                        'simulate_ahead_time': 2.0,
                        'max_rotational_vel': 1.0,
                        'min_rotational_vel': 0.4,
                        'rotational_acc_lim': 3.2
                    }
                },
            }

            cache_key = 'nav_bt_navigator'
            cache_val = yaml.dump(params_config, default_flow_style=False, allow_unicode=True)
            self.cache.set(cache_key, cache_val)
            self.sdk.output({"导航配置": cache_key})
            self.sdk.finish()
        except Exception as e:
            self.sdk.debug(f"写入缓存失败: {e}")
            self.sdk.error()
