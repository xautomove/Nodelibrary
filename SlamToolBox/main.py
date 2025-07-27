import rclpy
import subprocess
import time
import os
import shutil
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String

class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.node = None
        self.slam_toolbox_process = None
        self.save_timer = None
        self.last_save_time = 0
        self.sdk.debug("SlamToolbox init")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        if isinstance(config, dict):
            self.scan_topic = str(config.get('scan_topic', '/scan'))
            self.sdk.debug(f"节点输入scan_topic: {self.scan_topic}")
        else:
            self.scan_topic = '/scan'

    def get_config_value(self, key, default=None):
        """从配置列表中获取指定键的值"""
        if isinstance(self.config, list):
            for item in self.config:
                if isinstance(item, dict) and item.get('name') == key:
                    value = item.get('default_value', item.get('value', default))
                    
                    if item.get('type') == 'number':
                        try:
                            return float(value) if value is not None else default
                        except (ValueError, TypeError):
                            return default
                    else:
                        return value
        elif isinstance(self.config, dict):
            return self.config.get(key, default)
        return default

    def init_ros_node(self):
        """初始化ROS2节点"""
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node('slam_toolbox_node')
        self.sdk.debug("# ROS2节点初始化完成")

    def start_slam_toolbox(self):
        """启动Slam Toolbox"""
        try:
            self.sdk.debug("启动Slam Toolbox")
            
            slam_cmd = [
                'ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
                '--ros-args',
                '-p', f'base_frame:=base_link',
                '-p', f'odom_frame:=odom',
                '-p', f'map_frame:=map',
                '-p', f'map_update_interval:={1.0/self.get_config_value("update_rate", 5.0)}',
                '-p', f'max_laser_range:={self.get_config_value("max_range", 20.0)}',
                '-p', f'minimum_time_interval:=0.2',
                '-p', f'transform_timeout:=0.2',
                '-p', f'update_rate:={self.get_config_value("update_rate", 5.0)}',
                '-p', f'resolution:={self.get_config_value("map_resolution", 0.05)}',
                '-p', f'max_queue_size:=10',
                '-p', f'use_interactive_mode:=false',
                '--remap', f'scan:={self.scan_topic}'
            ]

            self.sdk.debug(f"启动Slam Toolbox: {slam_cmd}")

            self.slam_toolbox_process = subprocess.Popen(
                slam_cmd,
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.sdk.save_pid(self.slam_toolbox_process.pid)
            
            time.sleep(3)
            
            self.sdk.debug("Slam Toolbox已启动")
            return True
        except Exception as e:
            self.sdk.debug(f"启动Slam Toolbox失败: {e}")
            return False

    def save_map(self):
        """保存地图"""
        if not self.node:
            self.sdk.debug("ROS2节点未初始化")
            return
        
        try:
            save_path = self.get_config_value("save_path", "/tmp/map")
            
            save_dir = os.path.dirname(save_path)
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)
                self.sdk.debug(f"创建保存目录: {save_dir}")
            
            timestamp = int(time.time())
            map_name = f"{os.path.basename(save_path)}_{timestamp}"
            
            success = self._call_save_map_service(map_name)
            if success:
                self._rename_map_files(map_name, save_path)
                
        except Exception as e:
            self.sdk.debug(f"保存地图时出错: {e}")
    
    def _call_save_map_service(self, map_name):
        """调用保存地图服务"""
        client = self.node.create_client(SaveMap, '/slam_toolbox/save_map')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.sdk.debug("保存地图服务不可用")
            return False
        
        request = SaveMap.Request()
        request.name = String(data=map_name)
        
        self.sdk.debug(f"调用保存地图服务: {map_name}")
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        if not future.done():
            self.sdk.debug("保存地图服务超时")
            return False
        
        response = future.result()
        if not response:
            self.sdk.debug("保存地图服务调用失败")
            return False
        
        self.sdk.debug(f"地图已保存: {map_name}")
        return True
    
    def _rename_map_files(self, map_name, save_path):
        """重命名地图文件"""
        try:
            if os.path.exists(f"{map_name}.pgm"):
                shutil.move(f"{map_name}.pgm", f"{save_path}.pgm")
            if os.path.exists(f"{map_name}.yaml"):
                shutil.move(f"{map_name}.yaml", f"{save_path}.yaml")
            self.sdk.debug(f"地图文件已覆盖: {save_path}")
        except Exception as e:
            self.sdk.debug(f"移动地图文件时出错: {e}")
            # 如果移动失败，尝试复制然后删除
            try:
                if os.path.exists(f"{map_name}.pgm"):
                    shutil.copy2(f"{map_name}.pgm", f"{save_path}.pgm")
                    os.remove(f"{map_name}.pgm")
                if os.path.exists(f"{map_name}.yaml"):
                    shutil.copy2(f"{map_name}.yaml", f"{save_path}.yaml")
                    os.remove(f"{map_name}.yaml")
                self.sdk.debug(f"地图文件已复制覆盖: {save_path}")
            except Exception as e2:
                self.sdk.debug(f"复制地图文件时出错: {e2}")

    def save_map_periodic(self):
        """定时保存地图"""
        save_interval = self.get_config_value("save_interval", 30.0)
        current_time = time.time()
        
        if current_time - self.last_save_time >= save_interval:
            self.save_map()
            self.last_save_time = current_time

    def execute(self):
        # 初始化ROS2节点
        self.init_ros_node()
        
        # 启动Slam Toolbox
        if not self.start_slam_toolbox():
            self.sdk.error()
            return

        
        self.sdk.debug(f"Slam Toolbox节点已启动")
        self.sdk.debug(f"激光雷达话题: {self.scan_topic}")
        self.sdk.debug(f"保存间隔: {self.get_config_value('save_interval', 30.0)}秒")
        self.sdk.debug(f"保存路径: {self.get_config_value('save_path', '/tmp/map')}")
        
        # 持续运行
        try:
            self.sdk.bg(1)
            self.sdk.finish()
            while True:
                # 处理ROS2事件
                if self.node:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                
                # 定时保存地图
                self.save_map_periodic()
                
                if self.sdk.is_stop():
                    self.sdk.debug("收到停止信号，正在关闭节点...")
                    break
                    
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()
        finally:
            # 清理进程
            if self.slam_toolbox_process:
                self.slam_toolbox_process.terminate()
                self.slam_toolbox_process.wait()
                self.sdk.debug("Slam Toolbox进程已停止")

            # 清理ROS2资源
            if self.node:
                self.node.destroy_node()
            
            self.sdk.debug("# Slam Toolbox节点已停止")
            self.sdk.bg(0)
            self.sdk.finish() 