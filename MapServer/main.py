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

            self.params_file = tempfile.NamedTemporaryFile(
                mode='w',
                suffix='.yaml',
                prefix='map_server_params',
                delete=False
            )

            fixed_filename = '/tmp/map_server_params.yaml'
            self.params_file.close()

            with open(fixed_filename, 'w') as f:
                yaml.dump(params_config, f, default_flow_style=False)
            
            self.params_file = type('obj', (object,), {'name': fixed_filename})()

            self.sdk.debug(f"创建参数文件: {self.params_file.name}")
            self.sdk.debug(f"参数配置: {yaml.dump(params_config, default_flow_style=False)}")

            return True

        except Exception as e:
            self.sdk.debug(f"创建参数文件失败: {e}")
            return False

    def start_map_server(self):
        """启动nav2_map_server进程"""
        try:
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
                
                # 配置并激活生命周期节点
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

    def configure_and_activate_map_server(self):
        """配置并激活map_server生命周期节点"""
        try:
            cmd = [
                'bash', '-c',
                'ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate'
            ]
            self.sdk.debug(f"配置并激活map_server")
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=15,
                preexec_fn=os.setsid
            )
            
            if hasattr(result, 'pid'):
                self.sdk.save_pid(result.pid)
            
            if result.returncode == 0:
                self.sdk.debug("map_server配置和激活成功")
                return True
            else:
                self.sdk.debug(f"map_server配置和激活失败: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            self.sdk.debug("map_server配置和激活超时")
            return False
        except Exception as e:
            self.sdk.debug(f"配置和激活map_server时出错: {e}")
            return False

    def stop_map_server(self):
        """停止nav2_map_server进程"""
        try:
            cmd = ['ros2', 'lifecycle', 'set', '/map_server', 'shutdown']
            result = subprocess.run(
                cmd, 
                capture_output=True, 
                text=True, 
                timeout=5,
                preexec_fn=os.setsid
            )
            
            if hasattr(result, 'pid'):
                self.sdk.save_pid(result.pid)
                
            self.sdk.debug("map_server生命周期节点已关闭")
        except Exception as e:
            self.sdk.debug(f"关闭map_server生命周期节点时出错: {e}")
        
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