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
        self.planner_process = None
        self.params_file = None
        self.scan_topic = "/scan"
        self.map_topic = "map"
        self.sdk.debug("PlannerServer节点初始化")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        if 'scan_topic' in config:
            self.scan_topic = config['scan_topic']
            self.sdk.debug(f"接收到激光雷达话题: {self.scan_topic}")
        if 'map_topic' in config:
            self.map_topic = config['map_topic']
            self.sdk.debug(f"接收到地图话题: {self.map_topic}")


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
        """创建planner_server参数文件"""
        try:
            # 获取配置参数
            planner_frequency = self.get_config_value("planner_frequency", 20.0)
            planner_plugin = self.get_config_value("planner_plugin", "nav2_navfn_planner/NavfnPlanner")
            costmap_update_timeout = self.get_config_value("costmap_update_timeout", 1.0)
            
            global_frame = self.get_config_value("global_frame", "map")
            robot_base_frame = self.get_config_value("robot_base_frame", "base_link")
            robot_radius = self.get_config_value("robot_radius", 0.22)
            resolution = self.get_config_value("resolution", 0.05)
            update_frequency = self.get_config_value("update_frequency", 1.0)
            publish_frequency = self.get_config_value("publish_frequency", 1.0)
            
            local_frame = self.get_config_value("local_frame", "odom")
            rolling_window = self.get_config_value("rolling_window", True)
            width = self.get_config_value("width", 3)
            height = self.get_config_value("height", 3)
            local_update_frequency = self.get_config_value("local_update_frequency", 5.0)
            local_publish_frequency = self.get_config_value("local_publish_frequency", 2.0)
            
            obstacle_max_range = self.get_config_value("obstacle_max_range", 2.5)
            obstacle_min_range = self.get_config_value("obstacle_min_range", 0.0)
            raytrace_max_range = self.get_config_value("raytrace_max_range", 3.0)
            raytrace_min_range = self.get_config_value("raytrace_min_range", 0.0)
            max_obstacle_height = self.get_config_value("max_obstacle_height", 2.0)
            min_obstacle_height = self.get_config_value("min_obstacle_height", 0.0)
            
            inflation_radius = self.get_config_value("inflation_radius", 0.55)
            cost_scaling_factor = self.get_config_value("cost_scaling_factor", 1.0)

            # 构建参数配置
            params_config = {
                'planner_server': {
                    'ros__parameters': {
                        'expected_planner_frequency': planner_frequency,
                        'costmap_update_timeout': costmap_update_timeout,
                        'introspection_mode': 'disabled',
                        'planner_plugins': ['GridBased'],
                        'GridBased': {
                            'plugin': planner_plugin
                        }
                    }
                },
                'global_costmap': {
                    'global_costmap': {
                        'ros__parameters': {
                            'footprint_padding': 0.03,
                            'update_frequency': update_frequency,
                            'publish_frequency': publish_frequency,
                            'global_frame': global_frame,
                            'robot_base_frame': robot_base_frame,
                            'robot_radius': robot_radius,
                            'resolution': resolution,
                            'plugins': ["static_layer", "obstacle_layer", "inflation_layer"],
                            'obstacle_layer': {
                                'plugin': "nav2_costmap_2d/ObstacleLayer",
                                'enabled': True,
                                'observation_sources': 'scan',
                                'footprint_clearing_enabled': True,
                                'max_obstacle_height': max_obstacle_height,
                                'combination_method': 1,
                                'scan': {
                                    'topic': self.scan_topic,
                                    'obstacle_max_range': obstacle_max_range,
                                    'obstacle_min_range': obstacle_min_range,
                                    'raytrace_max_range': raytrace_max_range,
                                    'raytrace_min_range': raytrace_min_range,
                                    'max_obstacle_height': max_obstacle_height,
                                    'min_obstacle_height': min_obstacle_height,
                                    'clearing': True,
                                    'marking': True,
                                    'data_type': "LaserScan",
                                    'inf_is_valid': False
                                }
                            },
                            'static_layer': {
                                'plugin': "nav2_costmap_2d/StaticLayer",
                                'map_subscribe_transient_local': True,
                                'enabled': True,
                                'subscribe_to_updates': True,
                                'transform_tolerance': 0.1
                            },
                            'inflation_layer': {
                                'plugin': "nav2_costmap_2d/InflationLayer",
                                'enabled': True,
                                'inflation_radius': inflation_radius,
                                'cost_scaling_factor': cost_scaling_factor,
                                'inflate_unknown': False,
                                'inflate_around_unknown': True
                            },
                            'always_send_full_costmap': True,
                            'introspection_mode': 'disabled'
                        }
                    }
                },
                'local_costmap': {
                    'local_costmap': {
                        'ros__parameters': {
                            'update_frequency': local_update_frequency,
                            'publish_frequency': local_publish_frequency,
                            'global_frame': local_frame,
                            'robot_base_frame': robot_base_frame,
                            'rolling_window': rolling_window,
                            'width': width,
                            'height': height,
                            'resolution': resolution,
                            'introspection_mode': 'disabled'
                        }
                    }
                }
            }

            fixed_filename = '/tmp/planner_params.yaml'
            
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
            self.sdk.debug(f"创建planner参数文件失败: {e}")
            return False

    def check_node_exists(self):
        """检查planner_server节点是否存在"""
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
                return '/planner_server' in node_list
            return False
        except Exception as e:
            self.sdk.debug(f"检查节点存在性失败: {e}")
            return False

    def get_lifecycle_state(self):
        """获取planner_server生命周期节点状态"""
        try:
            result = subprocess.run(
                ['ros2', 'lifecycle', 'get', '/planner_server'],
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

    def configure_and_activate_planner(self):
        """配置并激活planner_server生命周期节点"""
        try:
            time.sleep(2)
            
            current_state = self.get_lifecycle_state()
            self.sdk.debug(f"planner_server当前状态: {current_state}")
            
            if current_state is None:
                self.sdk.debug("无法获取planner_server状态，尝试直接配置")
                
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/planner_server', 'configure'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"配置失败: {result.stderr}")
                    return False
                
                time.sleep(3)
                
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/planner_server', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
                self.sdk.debug("planner_server配置和激活成功")
                return True
            
            if current_state == 'unconfigured':
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/planner_server', 'configure'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"配置失败: {result.stderr}")
                    return False
                
                time.sleep(3)
                
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/planner_server', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
            elif current_state == 'inactive':
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/planner_server', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
            elif current_state == 'active':
                self.sdk.debug("planner_server已经是激活状态")
                return True
                
            elif current_state == 'finalized':
                self.sdk.debug("planner_server节点已终结，无法激活")
                return False
            else:
                self.sdk.debug("未知状态")
                return False
            
            self.sdk.debug("planner_server配置和激活成功")
            return True
                
        except Exception as e:
            self.sdk.debug(f"配置和激活planner_server时出错: {e}")
            return False

    def start_planner_server(self):
        """启动planner_server进程"""
        try:
            if self.check_node_exists():
                self.sdk.debug("planner_server节点已存在，获取当前状态")
                current_state = self.get_lifecycle_state()
                self.sdk.debug(f"当前planner_server状态: {current_state}")
                
                if current_state == 'active':
                    self.sdk.debug("planner_server已经是激活状态，无需重新启动")
                    return True
                
                if self.configure_and_activate_planner():
                    self.sdk.debug("planner_server配置和激活成功")
                    return True
                else:
                    self.sdk.debug("planner_server配置和激活失败")
                    return False
            
            if not self.create_params_file():
                return False

            cmd = [
                'ros2', 'run', 'nav2_planner', 'planner_server',
                '--ros-args', '--params-file', self.params_file.name
            ]

            self.sdk.debug(f"启动planner_server")
            
            self.planner_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True
            )
            
            self.sdk.save_pid(self.planner_process.pid)

            time.sleep(3)

            if self.planner_process.poll() is None:
                self.sdk.debug("planner_server进程启动成功")
                
                if not self.configure_and_activate_planner():
                    self.sdk.debug("配置和激活planner_server失败")
                    return False
                
                self.sdk.debug("planner_server完全启动成功")
                return True
            else:
                stdout, stderr = self.planner_process.communicate()
                self.sdk.debug(f"planner_server启动失败: {stderr}")
                return False

        except Exception as e:
            self.sdk.debug(f"启动planner_server失败: {e}")
            return False

    def stop_planner_server(self):
        """停止planner_server进程"""
        if self.planner_process:
            try:
                self.planner_process.terminate()
                self.planner_process.wait(timeout=5)
                self.sdk.debug("planner_server进程已停止")
            except subprocess.TimeoutExpired:
                self.planner_process.kill()
                self.sdk.debug("强制终止planner_server")
            except Exception as e:
                self.sdk.debug(f"停止planner_server时出错: {e}")
        
        if self.params_file and os.path.exists(self.params_file.name):
            try:
                os.unlink(self.params_file.name)
                self.sdk.debug("临时参数文件已清理")
            except Exception as e:
                self.sdk.debug(f"清理临时参数文件失败: {e}")

    def execute(self):
        if not self.start_planner_server():
            self.sdk.debug("启动planner_server失败")
            self.sdk.error()
            return

        scan_topic = self.scan_topic
        map_topic = self.map_topic
        pointcloud_topic = self.pointcloud_topic
        
        self.sdk.debug(f"planner_server节点已启动，持续运行中...")
        self.sdk.debug(f"激光雷达话题: {scan_topic}")
        self.sdk.debug(f"地图话题: {map_topic}")
        self.sdk.debug(f"点云话题: {pointcloud_topic}")
        
        self.sdk.output({"plan_topic": "/plan_navigation"})
        
        try:
            self.sdk.bg(1)
            self.sdk.finish()
            
            while True:
                if self.planner_process and self.planner_process.poll() is not None:
                    self.sdk.debug("planner_server进程意外退出")
                    break
                
                if self.sdk.is_stop():
                    self.sdk.debug("收到停止信号，正在关闭节点...")
                    break
                
                time.sleep(1)
                
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()
        finally:
            self.stop_planner_server()
            self.sdk.debug("# planner_server节点已停止")
            self.sdk.bg(0)
            self.sdk.finish() 