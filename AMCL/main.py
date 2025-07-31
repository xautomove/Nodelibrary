import subprocess
import os
import time
import yaml
import tempfile
import json

class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.amcl_process = None
        self.params_file = None
        self.scan_topic = "scan"
        self.map_topic = "map"
        self.sdk.debug("AMCL定位节点初始化")

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
        if isinstance(self.config, list):
            for item in self.config:
                if isinstance(item, dict) and item.get('name') == key:
                    value = item.get('default_value', item.get('value', default))
                    
                    if item.get('type') == 'number':
                        try:
                            if key in ['max_particles', 'min_particles']:
                                return int(value) if value is not None else default
                            else:
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
        """创建AMCL参数文件"""
        try:
            scan_topic = self.scan_topic
            map_topic = self.map_topic
            base_frame_id = self.get_config_value("base_frame_id", "base_footprint")
            odom_frame_id = self.get_config_value("odom_frame_id", "odom")
            global_frame_id = self.get_config_value("global_frame_id", "map")
            max_particles = self.get_config_value("max_particles", 2000)
            min_particles = self.get_config_value("min_particles", 500)
            alpha1 = self.get_config_value("alpha1", 0.2)
            alpha2 = self.get_config_value("alpha2", 0.2)
            alpha3 = self.get_config_value("alpha3", 0.2)
            alpha4 = self.get_config_value("alpha4", 0.2)
            alpha5 = self.get_config_value("alpha5", 0.2)
            laser_max_range = self.get_config_value("laser_max_range", 100.0)
            laser_min_range = self.get_config_value("laser_min_range", -1.0)
            set_initial_pose = self.get_config_value("set_initial_pose", False)
            initial_pose_x = self.get_config_value("initial_pose_x", 0.0)
            initial_pose_y = self.get_config_value("initial_pose_y", 0.0)
            initial_pose_z = self.get_config_value("initial_pose_z", 0.0)
            initial_pose_yaw = self.get_config_value("initial_pose_yaw", 0.0)

            # 构建AMCL参数配置
            params_config = {
                'amcl': {
                    'ros__parameters': {
                        'alpha1': alpha1,
                        'alpha2': alpha2,
                        'alpha3': alpha3,
                        'alpha4': alpha4,
                        'alpha5': alpha5,
                        'base_frame_id': base_frame_id,
                        'introspection_mode': 'disabled',
                        'beam_skip_distance': 0.5,
                        'beam_skip_error_threshold': 0.9,
                        'beam_skip_threshold': 0.3,
                        'do_beamskip': False,
                        'global_frame_id': global_frame_id,
                        'lambda_short': 0.1,
                        'laser_likelihood_max_dist': 2.0,
                        'laser_max_range': laser_max_range,
                        'laser_min_range': laser_min_range,
                        'laser_model_type': 'likelihood_field',
                        'max_beams': 60,
                        'max_particles': max_particles,
                        'min_particles': min_particles,
                        'odom_frame_id': odom_frame_id,
                        'pf_err': 0.05,
                        'pf_z': 0.99,
                        'recovery_alpha_fast': 0.0,
                        'recovery_alpha_slow': 0.0,
                        'resample_interval': 1,
                        'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                        'save_pose_rate': 0.5,
                        'sigma_hit': 0.2,
                        'tf_broadcast': True,
                        'transform_tolerance': 1.0,
                        'update_min_a': 0.2,
                        'update_min_d': 0.25,
                        'z_hit': 0.5,
                        'z_max': 0.05,
                        'z_rand': 0.5,
                        'z_short': 0.05,
                        'scan_topic': scan_topic,
                        'map_topic': map_topic,
                        'set_initial_pose': set_initial_pose,
                        'always_reset_initial_pose': False,
                        'first_map_only': False,
                        'initial_pose': {
                            'x': initial_pose_x,
                            'y': initial_pose_y,
                            'z': initial_pose_z,
                            'yaw': initial_pose_yaw
                        }
                    }
                }
            }

            fixed_filename = '/tmp/amcl_params.yaml'
            
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
            self.sdk.debug(f"创建AMCL参数文件失败: {e}")
            return False

    def start_amcl(self):
        """启动AMCL定位进程"""
        try:
            if self.check_node_exists():
                self.sdk.debug("AMCL节点已存在，获取当前状态")
                current_state = self.get_lifecycle_state()
                self.sdk.debug(f"当前AMCL状态: {current_state}")
                
                if current_state == 'active':
                    self.sdk.debug("AMCL已经是激活状态，无需重新启动")
                    return True
                
                if self.configure_and_activate_amcl():
                    self.sdk.debug("AMCL配置和激活成功")
                    return True
                else:
                    self.sdk.debug("AMCL配置和激活失败")
                    return False
            
            if not self.create_params_file():
                return False

            cmd = [
                'ros2', 'run', 'nav2_amcl', 'amcl',
                '--ros-args', '--params-file', self.params_file.name
            ]

            self.sdk.debug(f"启动AMCL定位节点")
            
            self.amcl_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True
            )
            
            self.sdk.save_pid(self.amcl_process.pid)

            time.sleep(3)

            if self.amcl_process.poll() is None:
                self.sdk.debug("AMCL定位节点启动成功")
                
                if not self.configure_and_activate_amcl():
                    self.sdk.debug("配置和激活AMCL失败")
                    return False
                
                self.sdk.debug("AMCL定位节点完全启动成功")
                return True
            else:
                stdout, stderr = self.amcl_process.communicate()
                self.sdk.debug(f"AMCL启动失败: {stderr}")
                return False

        except Exception as e:
            self.sdk.debug(f"启动AMCL失败: {e}")
            return False

    def check_node_exists(self):
        """检查AMCL节点是否存在"""
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
                return '/amcl' in node_list
            return False
        except Exception as e:
            self.sdk.debug(f"检查节点存在性失败: {e}")
            return False

    def get_lifecycle_state(self):
        """获取AMCL生命周期节点状态"""
        try:
            result = subprocess.run(
                ['ros2', 'lifecycle', 'get', '/amcl'],
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

    def configure_and_activate_amcl(self):
        """配置并激活AMCL生命周期节点"""
        try:
            time.sleep(2)
            
            current_state = self.get_lifecycle_state()
            self.sdk.debug(f"AMCL当前状态: {current_state}")
            
            if current_state is None:
                self.sdk.debug("无法获取AMCL状态，尝试直接配置")

                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/amcl', 'configure'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode != 0:
                    self.sdk.debug(f"配置失败: {result.stderr}")
                    return False
                
                time.sleep(2)
                
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/amcl', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
                self.sdk.debug("AMCL配置和激活成功")
                return True
            
            if current_state == 'unconfigured':
                result = subprocess.run(
                    ['bash', '-c', 'ros2 lifecycle set /amcl configure && ros2 lifecycle set /amcl activate'],
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                if result.returncode != 0:
                    self.sdk.debug(f"连贯操作失败: {result.stderr}")
                    return False
                
            elif current_state == 'inactive':
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/amcl', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
            elif current_state == 'active':
                self.sdk.debug("AMCL已经是激活状态")
                return True
            else:
                self.sdk.debug("未知状态")
                return False
            
            self.sdk.debug("AMCL配置和激活成功")
            return True
        except Exception as e:
            self.sdk.debug(f"配置和激活AMCL时出错: {e}")
            return False

    def stop_amcl(self):
        if self.amcl_process:
            try:
                self.amcl_process.terminate()
                self.amcl_process.wait(timeout=5)
                self.sdk.debug("AMCL进程已停止")
            except subprocess.TimeoutExpired:
                self.amcl_process.kill()
                self.sdk.debug("强制终止AMCL")
            except Exception as e:
                self.sdk.debug(f"停止AMCL时出错: {e}")
        
        if self.params_file and os.path.exists(self.params_file.name):
            try:
                os.unlink(self.params_file.name)
                self.sdk.debug("临时参数文件已清理")
            except Exception as e:
                self.sdk.debug(f"清理临时参数文件失败: {e}")

    def execute(self):
        if not self.start_amcl():
            self.sdk.debug("启动AMCL定位失败")
            self.sdk.error()
            return

        scan_topic = self.scan_topic
        map_topic = self.map_topic
        base_frame_id = self.get_config_value("base_frame_id", "base_footprint")
        
        self.sdk.debug(f"AMCL定位节点已启动，持续运行中...")
        self.sdk.debug(f"激光雷达话题: {scan_topic}")
        self.sdk.debug(f"地图话题: {map_topic}")
        self.sdk.debug(f"基座坐标系: {base_frame_id}")

        self.sdk.output({"robot_pose": "/amcl_pose"})
        
        try:
            self.sdk.bg(1)
            self.sdk.finish()
            
            while True:
                if self.amcl_process and self.amcl_process.poll() is not None:
                    self.sdk.debug("AMCL进程意外退出")
                    break
                
                if self.sdk.is_stop():
                    self.sdk.debug("收到停止信号，正在关闭节点...")
                    break
                
                time.sleep(1)
                
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()
        finally:
            self.stop_amcl()
            self.sdk.debug("# AMCL定位节点已停止")
            self.sdk.bg(0)
            self.sdk.finish()