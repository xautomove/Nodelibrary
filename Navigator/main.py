import subprocess
import os
import time
import yaml

class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.navigator_process = None
        self.params_file = None
        self.global_frame = "map"
        self.robot_base_frame = "base_link"
        self.sdk.debug("Navigator节点初始化")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        if 'global_frame' in config:
            self.global_frame = config['global_frame']
            self.sdk.debug(f"接收到全局坐标系: {self.global_frame}")
        if 'robot_base_frame' in config:
            self.robot_base_frame = config['robot_base_frame']
            self.sdk.debug(f"接收到机器人基座坐标系: {self.robot_base_frame}")

    def get_config_value(self, key, default=None):
        """获取用户配置参数值"""
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

    def create_params_file(self):
        """创建bt_navigator参数文件"""
        try:
            transform_tolerance = self.get_config_value("transform_tolerance", 0.1)
            filter_duration = self.get_config_value("filter_duration", 0.3)
            always_reload_bt_xml = self.get_config_value("always_reload_bt_xml", False)
            
            goal_blackboard_id = self.get_config_value("goal_blackboard_id", "goal")
            goals_blackboard_id = self.get_config_value("goals_blackboard_id", "goals")
            path_blackboard_id = self.get_config_value("path_blackboard_id", "path")
            waypoint_statuses_blackboard_id = self.get_config_value("waypoint_statuses_blackboard_id", "waypoint_statuses")
            
            navigate_to_pose_enable_groot = self.get_config_value("navigate_to_pose_enable_groot", False)
            navigate_to_pose_groot_port = self.get_config_value("navigate_to_pose_groot_port", 1667)
            navigate_through_poses_enable_groot = self.get_config_value("navigate_through_poses_enable_groot", False)
            navigate_through_poses_groot_port = self.get_config_value("navigate_through_poses_groot_port", 1669)

            params_config = {
                'bt_navigator': {
                    'ros__parameters': {
                        'global_frame': self.global_frame,
                        'robot_base_frame': self.robot_base_frame,
                        'transform_tolerance': transform_tolerance,
                        'filter_duration': filter_duration,
                        'introspection_mode': 'disabled',
                        'default_nav_to_pose_bt_xml': 'replace/with/path/to/bt.xml',
                        'default_nav_through_poses_bt_xml': 'replace/with/path/to/bt.xml',
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
                            'nav2_is_stopped_condition_bt_node',
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
                }
            }

            fixed_filename = '/tmp/navigator_params.yaml'
            
            if os.path.exists(fixed_filename):
                try:
                    os.unlink(fixed_filename)
                except Exception as e:
                    self.sdk.debug(f"删除参数文件失败: {e}")

            with open(fixed_filename, 'w') as f:
                yaml.dump(params_config, f, default_flow_style=False)
            
            self.params_file = type('obj', (object,), {'name': fixed_filename})()

            self.sdk.debug(f"参数配置: {yaml.dump(params_config, default_flow_style=False)}")

            return True

        except Exception as e:
            self.sdk.debug(f"创建navigator参数文件失败: {e}")
            return False

    def check_node_exists(self):
        """检查bt_navigator节点是否存在"""
        try:
            subprocess.run(['bash', '-c', 'ros2 daemon stop && ros2 daemon start'], capture_output=True, timeout=5)
            
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                node_list = result.stdout.strip().split('\n')
                return '/bt_navigator' in node_list
            return False
        except Exception as e:
            self.sdk.debug(f"检查节点存在性失败: {e}")
            return False

    def get_lifecycle_state(self):
        """获取bt_navigator生命周期节点状态"""
        try:
            result = subprocess.run(
                ['ros2', 'lifecycle', 'get', '/bt_navigator'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                state_output = result.stdout.strip()
                state = state_output.split()[0] if state_output else None
                return state
            return None
        except Exception as e:
            self.sdk.debug(f"获取生命周期状态失败: {e}")
            return None

    def configure_and_activate_navigator(self):
        """配置并激活bt_navigator生命周期节点"""
        try:
            time.sleep(2)
            
            current_state = self.get_lifecycle_state()
            self.sdk.debug(f"bt_navigator当前状态: {current_state}")
            
            if current_state is None:
                self.sdk.debug("无法获取bt_navigator状态，尝试直接配置")
                
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/bt_navigator', 'configure'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"配置失败: {result.stderr}")
                    return False
                
                time.sleep(3)
                
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/bt_navigator', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
                self.sdk.debug("bt_navigator配置和激活成功")
                return True
            
            if current_state == 'unconfigured':
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/bt_navigator', 'configure'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"配置失败: {result.stderr}")
                    return False
                
                time.sleep(3)
                
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/bt_navigator', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
            elif current_state == 'inactive':
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/bt_navigator', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
            elif current_state == 'active':
                self.sdk.debug("bt_navigator已经是激活状态")
                return True
                
            elif current_state == 'finalized':
                self.sdk.debug("bt_navigator节点已终结，无法激活")
                return False
            else:
                self.sdk.debug("未知状态")
                return False
            
            self.sdk.debug("bt_navigator配置和激活成功")
            return True
                
        except Exception as e:
            self.sdk.debug(f"配置和激活bt_navigator时出错: {e}")
            return False

    def start_navigator(self):
        """启动bt_navigator进程"""
        try:
            if self.check_node_exists():
                self.sdk.debug("bt_navigator节点已存在，获取当前状态")
                current_state = self.get_lifecycle_state()
                self.sdk.debug(f"当前bt_navigator状态: {current_state}")
                
                if current_state == 'active':
                    self.sdk.debug("bt_navigator已经是激活状态，无需重新启动")
                    return True
                
                if self.configure_and_activate_navigator():
                    self.sdk.debug("bt_navigator配置和激活成功")
                    return True
                else:
                    self.sdk.debug("bt_navigator配置和激活失败")
                    return False
            
            if not self.create_params_file():
                return False

            cmd = [
                'ros2', 'run', 'nav2_bt_navigator', 'bt_navigator',
                '--ros-args', '--params-file', self.params_file.name
            ]

            self.sdk.debug(f"启动bt_navigator")
            
            self.navigator_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True
            )
            
            self.sdk.save_pid(self.navigator_process.pid)

            time.sleep(3)

            if self.navigator_process.poll() is None:
                self.sdk.debug("bt_navigator进程启动成功")
                
                if not self.configure_and_activate_navigator():
                    self.sdk.debug("配置和激活bt_navigator失败")
                    return False
                
                self.sdk.debug("bt_navigator完全启动成功")
                return True
            else:
                stdout, stderr = self.navigator_process.communicate()
                self.sdk.debug(f"bt_navigator启动失败: {stderr}")
                return False

        except Exception as e:
            self.sdk.debug(f"启动bt_navigator失败: {e}")
            return False

    def stop_navigator(self):
        """停止bt_navigator进程"""
        if self.navigator_process:
            try:
                self.navigator_process.terminate()
                self.navigator_process.wait(timeout=5)
                self.sdk.debug("bt_navigator进程已停止")
            except subprocess.TimeoutExpired:
                self.navigator_process.kill()
                self.sdk.debug("强制终止bt_navigator")
            except Exception as e:
                self.sdk.debug(f"停止bt_navigator时出错: {e}")
        
        if self.params_file and os.path.exists(self.params_file.name):
            try:
                os.unlink(self.params_file.name)
                self.sdk.debug("临时参数文件已清理")
            except Exception as e:
                self.sdk.debug(f"清理临时参数文件失败: {e}")

    def execute(self):
        if not self.start_navigator():
            self.sdk.debug("启动bt_navigator失败")
            self.sdk.error()
            return

        global_frame = self.global_frame
        robot_base_frame = self.robot_base_frame
        
        self.sdk.debug(f"bt_navigator节点已启动，持续运行中...")
        self.sdk.debug(f"全局坐标系: {global_frame}")
        self.sdk.debug(f"机器人基座坐标系: {robot_base_frame}")
        
        self.sdk.output({"navigator_status": "bt_navigator"})
        
        try:
            self.sdk.bg(1)
            self.sdk.finish()
            
            while True:
                if self.navigator_process and self.navigator_process.poll() is not None:
                    self.sdk.debug("bt_navigator进程意外退出")
                    break
                
                if self.sdk.is_stop():
                    self.sdk.debug("收到停止信号，正在关闭节点...")
                    break
                
                time.sleep(1)
                
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()
        finally:
            self.stop_navigator()
            self.sdk.debug("# bt_navigator节点已停止")
            self.sdk.bg(0)
            self.sdk.finish() 