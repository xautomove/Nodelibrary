import subprocess
import os
import time
import yaml
import tempfile

class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.map_server_process = None
        self.params_file = None
        self.sdk.debug("MapServer init")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        pass

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

    def create_params_file(self):
        """创建nav2_map_server参数文件"""
        try:
            map_file = self.get_config_value("map_file")
            frame_id = self.get_config_value("frame_id", "map")
            topic_name = self.get_config_value("topic_name", "map")
            use_compressed = self.get_config_value("use_compressed", False)

            params_config = {
                'map_server': {
                    'ros__parameters': {
                        'yaml_filename': map_file,
                        'frame_id': frame_id,
                        'topic_name': topic_name,
                        'use_sim_time': False,
                        'introspection_mode': 'disabled'
                    }
                }
            }

            if use_compressed:
                params_config['map_server']['ros__parameters']['use_compressed'] = True

            fixed_filename = '/tmp/map_server_params.yaml'
            
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
            self.sdk.debug(f"创建参数文件失败: {e}")
            return False

    def start_map_server(self):
        """启动nav2_map_server进程"""
        try:
            if self.check_node_exists():
                self.sdk.debug("map_server节点已存在，获取当前状态")
                current_state = self.get_lifecycle_state()
                self.sdk.debug(f"当前map_server状态: {current_state}")
                
                if current_state == 'active':
                    self.sdk.debug("map_server已经是激活状态，无需重新启动")
                    return True
                
                if self.configure_and_activate_map_server():
                    self.sdk.debug("map_server配置和激活成功")
                    return True
                else:
                    self.sdk.debug("map_server配置和激活失败")
                    return False
            
            map_file = self.get_config_value("map_file")
            if not map_file:
                self.sdk.debug("未指定地图文件路径")
                return False

            if not os.path.exists(map_file):
                self.sdk.debug(f"地图文件不存在: {map_file}")
                return False

            if not self.create_params_file():
                return False

            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_server',
                '--ros-args', '--params-file', self.params_file.name
            ]

            self.sdk.debug(f"启动nav2_map_server")
            
            self.map_server_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True
            )
            
            self.sdk.save_pid(self.map_server_process.pid)

            time.sleep(2)

            if self.map_server_process.poll() is None:
                self.sdk.debug("nav2_map_server进程启动成功")
                
                if not self.configure_and_activate_map_server():
                    self.sdk.debug("配置和激活map_server失败")
                    return False
                
                self.sdk.debug("nav2_map_server完全启动成功")
                return True
            else:
                stdout, stderr = self.map_server_process.communicate()
                self.sdk.debug(f"nav2_map_server启动失败: {stderr}")
                return False

        except Exception as e:
            self.sdk.debug(f"启动nav2_map_server失败: {e}")
            return False

    def check_node_exists(self):
        """检查map_server节点是否存在"""
        try:
            subprocess.run(['bash', '-c', 'ros2 daemon stop && ros2 daemon start'], capture_output=True, timeout=5)
            
            time.sleep(3)
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                node_list = result.stdout.strip().split('\n')
                return '/map_server' in node_list
            return False
        except Exception as e:
            self.sdk.debug(f"检查节点存在性失败: {e}")
            return False

    def get_lifecycle_state(self):
        """获取map_server生命周期节点状态"""
        try:
            result = subprocess.run(
                ['ros2', 'lifecycle', 'get', '/map_server'],
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

    def configure_and_activate_map_server(self):
        """配置并激活map_server生命周期节点"""
        try:
            time.sleep(3)

            current_state = self.get_lifecycle_state()
            self.sdk.debug(f"map_server当前状态: {current_state}")
            
            if current_state is None:
                self.sdk.debug("无法获取map_server状态，尝试直接配置")

                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"配置失败: {result.stderr}")
                    return False
                
                time.sleep(2)
 
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
                self.sdk.debug("map_server配置和激活成功")
                return True
            
            if current_state == 'unconfigured':
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"配置失败: {result.stderr}")
                    return False

                time.sleep(3)

                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
            elif current_state == 'inactive':
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode != 0:
                    self.sdk.debug(f"激活失败: {result.stderr}")
                    return False
                
            elif current_state == 'active':
                self.sdk.debug("map_server已经是激活状态")
                return True
            else:
                self.sdk.debug(f"未知状态 {current_state}")
                return False
            
            self.sdk.debug("map_server配置和激活成功")
            return True
                
        except Exception as e:
            self.sdk.debug(f"配置和激活map_server时出错: {e}")
            return False

    def stop_map_server(self):
        if self.map_server_process:
            try:
                self.map_server_process.terminate()
                self.map_server_process.wait(timeout=5)
                self.sdk.debug("nav2_map_server进程已停止")
            except subprocess.TimeoutExpired:
                self.map_server_process.kill()
                self.sdk.debug("强制终止nav2_map_server")
            except Exception as e:
                self.sdk.debug(f"停止nav2_map_server时出错: {e}")
        
        if self.params_file and os.path.exists(self.params_file.name):
            try:
                os.unlink(self.params_file.name)
                self.sdk.debug("临时参数文件已清理")
            except Exception as e:
                self.sdk.debug(f"清理临时参数文件失败: {e}")

    def execute(self):
        if not self.start_map_server():
            self.sdk.debug("启动nav2_map_server失败")
            self.sdk.error()
            return

        map_file = self.get_config_value("map_file")
        topic_name = self.get_config_value("topic_name", "map")
        frame_id = self.get_config_value("frame_id", "map")
        
        self.sdk.output({"map_name": topic_name})
        
        self.sdk.debug(f"地图服务器已启动，持续运行中...")
        self.sdk.debug(f"地图文件: {map_file}")
        self.sdk.debug(f"发布话题: {topic_name}")
        self.sdk.debug(f"坐标系: {frame_id}")
        
        try:
            self.sdk.bg(1)
            self.sdk.finish()
            while True:
                if self.map_server_process and self.map_server_process.poll() is not None:
                    self.sdk.debug("nav2_map_server进程意外退出")
                    break
                
                if self.sdk.is_stop():
                    self.sdk.debug("收到停止信号，正在关闭节点...")
                    break
                
                time.sleep(1)
                
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()
        finally:
            self.stop_map_server()
            self.sdk.debug("# 地图服务器节点已停止")
            self.sdk.bg(0)
            self.sdk.finish()