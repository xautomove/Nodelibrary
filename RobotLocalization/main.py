import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
import subprocess
import time
import os

class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.node = None
        self.ekf_node = None
        self.ekf_process = None
        
        # 输入话题
        self.gnss_topic = ""
        self.imu_topic = ""
        self.odom_topic = ""
        
        # 配置参数
        self.frequency = 30.0
        self.sensor_timeout = 0.1
        self.two_d_mode = True
        self.publish_tf = True
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_link_frame = "base_link"
        self.world_frame = "map"
        self.fused_topic = "/fused_odom"
        
        # 传感器向量配置
        self.imu_vector = []
        self.odom_vector = []
        self.gnss_vector = []
        
        self.sdk.debug("RobotLocalization init")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        # 期望config为字典，包含gnss_topic, imu_topic, odom_topic
        if isinstance(config, dict):
            self.gnss_topic = str(config.get('gnss_topic', ''))
            self.imu_topic = str(config.get('imu_topic', ''))
            self.odom_topic = str(config.get('odom_topic', ''))
            self.sdk.debug(f"节点输入gnss_topic: {self.gnss_topic}")
            self.sdk.debug(f"节点输入imu_topic: {self.imu_topic}")
            self.sdk.debug(f"节点输入odom_topic: {self.odom_topic}")
        else:
            self.gnss_topic = ""
            self.imu_topic = ""
            self.odom_topic = ""

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
                    elif item.get('type') == 'boolean':
                        try:
                            return bool(value) if value is not None else default
                        except (ValueError, TypeError):
                            return default
                    else:
                        return str(value) if value is not None else default
        elif isinstance(self.config, dict):
            return self.config.get(key, default)
        return default

    def parse_vector_string(self, vector_str):
        """解析向量字符串为布尔列表"""
        if not vector_str or vector_str.strip() == "":
            # 如果向量为空，返回15个False
            return [False] * 15
        try:
            # 移除方括号并分割
            vector_str = vector_str.strip()
            if vector_str.startswith('[') and vector_str.endswith(']'):
                vector_str = vector_str[1:-1]
            
            # 分割并转换为布尔值
            parts = vector_str.split(',')
            result = [part.strip().lower() == 'true' for part in parts]
            
            # 确保返回15维向量
            if len(result) < 15:
                result.extend([False] * (15 - len(result)))
            elif len(result) > 15:
                result = result[:15]
                
            return result
        except Exception as e:
            self.sdk.debug(f"解析向量字符串失败: {e}")
            # 解析失败时也返回15个False
            return [False] * 15

    def generate_ekf_config(self):
        """生成EKF配置文件"""
        config = {
            "frequency": self.frequency,
            "sensor_timeout": self.sensor_timeout,
            "two_d_mode": self.two_d_mode,
            "publish_tf": self.publish_tf,
            "map_frame": self.map_frame,
            "odom_frame": self.odom_frame,
            "base_link_frame": self.base_link_frame,
            "world_frame": self.world_frame
        }
        return config



    def start_ekf_node(self):
        """启动EKF节点"""
        try:
            # 构建参数字典
            parameters = {
                "frequency": self.frequency,
                "sensor_timeout": self.sensor_timeout,
                "two_d_mode": self.two_d_mode,
                "publish_tf": self.publish_tf,
                "map_frame": self.map_frame,
                "odom_frame": self.odom_frame,
                "base_link_frame": self.base_link_frame,
                "world_frame": self.world_frame,
            }
            
            # 添加传感器配置
            if self.odom_topic and self.odom_vector:
                parameters["odom0"] = self.odom_topic
                parameters["odom0_config"] = self.odom_vector
            
            if self.imu_topic and self.imu_vector:
                parameters["imu0"] = self.imu_topic
                parameters["imu0_config"] = self.imu_vector
            
            if self.gnss_topic and self.gnss_vector:
                parameters["gps0"] = self.gnss_topic
                parameters["gps0_config"] = self.gnss_vector
            
            # 构建参数列表
            param_args = []
            for key, value in parameters.items():
                if isinstance(value, list):
                    # 处理向量参数 - 使用YAML格式
                    vector_str = "[" + ", ".join(str(v) for v in value) + "]"
                    param_args.extend([f"--ros-args", "-p", f"{key}:={vector_str}"])
                else:
                    param_args.extend([f"--ros-args", "-p", f"{key}:={value}"])
            
            # 添加remapping
            param_args.extend(["--ros-args", "-r", f"odometry/filtered:={self.fused_topic}"])
            
            self.sdk.bg(1)
            # 启动ros2 run
            cmd = ["ros2", "run", "robot_localization", "ekf_node"] + param_args
            self.sdk.debug(f"启动命令: {cmd}")
            self.ekf_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
            
            if self.ekf_process:
                self.sdk.debug(f"EKF节点已启动，PID: {self.ekf_process.pid}")
                # 保存进程ID
                self.sdk.save_pid(str(self.ekf_process.pid))
            else:
                self.sdk.debug("EKF进程启动失败")
                return False
            
            # 等待一小段时间让进程启动
            time.sleep(1)
            
            # 检查进程是否还在运行
            if self.ekf_process.poll() is not None:
                # 进程已退出，读取错误输出
                stdout, stderr = self.ekf_process.communicate()
                self.sdk.debug(f"EKF进程启动后立即退出")
                self.sdk.debug(f"标准输出: {stdout.decode('utf-8', errors='ignore')}")
                self.sdk.debug(f"错误输出: {stderr.decode('utf-8', errors='ignore')}")
                return False
            
            return True
            
        except Exception as e:
            self.sdk.debug(f"启动EKF节点失败: {e}")
            return False

    def stop_ekf_node(self):
        """停止EKF节点"""
        if self.ekf_process:
            try:
                # 使用进程组ID杀死整个进程组
                os.killpg(os.getpgid(self.ekf_process.pid), 15)  # SIGTERM
                self.ekf_process.wait(timeout=5)
                self.sdk.debug("EKF节点已停止")
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.ekf_process.pid), 9)  # SIGKILL
                self.sdk.debug("强制停止EKF节点")
            except Exception as e:
                self.sdk.debug(f"停止EKF节点时出错: {e}")

    def init_ros_node(self):
        """初始化ROS2节点"""
        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('robot_localization_fusion')
        
        # 获取配置参数
        self.frequency = self.get_config_value("frequency", 30.0)
        self.sensor_timeout = self.get_config_value("sensor_timeout", 0.1)
        self.two_d_mode = self.get_config_value("two_d_mode", True)
        self.publish_tf = self.get_config_value("publish_tf", True)
        self.map_frame = str(self.get_config_value("map_frame", "map"))
        self.odom_frame = str(self.get_config_value("odom_frame", "odom"))
        self.base_link_frame = str(self.get_config_value("base_link_frame", "base_link"))
        self.world_frame = str(self.get_config_value("world_frame", "map"))
        self.fused_topic = str(self.get_config_value("fused_topic", "/fused_odom"))
        
        # 解析传感器向量
        imu_vector_str = self.get_config_value("imu_vector", "")
        odom_vector_str = self.get_config_value("odom_vector", "")
        gnss_vector_str = self.get_config_value("gnss_vector", "")
        
        self.imu_vector = self.parse_vector_string(imu_vector_str)
        self.odom_vector = self.parse_vector_string(odom_vector_str)
        self.gnss_vector = self.parse_vector_string(gnss_vector_str)
        
        self.sdk.debug(f"ROS2节点初始化完成")
        self.sdk.debug(f"频率: {self.frequency}Hz")
        self.sdk.debug(f"传感器超时: {self.sensor_timeout}s")
        self.sdk.debug(f"2D模式: {self.two_d_mode}")
        self.sdk.debug(f"发布TF: {self.publish_tf}")
        self.sdk.debug(f"融合话题: {self.fused_topic}")

    def execute(self):
        """执行节点主逻辑"""
        # 初始化ROS2节点
        self.init_ros_node()
        
        # 检查是否有有效的传感器输入
        active_sensors = []
        if self.gnss_topic and self.gnss_vector:
            active_sensors.append(f"GNSS({self.gnss_topic})")
        if self.imu_topic and self.imu_vector:
            active_sensors.append(f"IMU({self.imu_topic})")
        if self.odom_topic and self.odom_vector:
            active_sensors.append(f"里程计({self.odom_topic})")
        
        if not active_sensors:
            self.sdk.debug("警告: 没有配置有效的传感器输入")
            self.sdk.error()
            return
        
        self.sdk.debug(f"机器人定位融合器已启动")
        self.sdk.debug(f"活跃传感器: {', '.join(active_sensors)}")
        self.sdk.debug(f"输出话题: {self.fused_topic}")
        
        # 启动EKF节点
        if not self.start_ekf_node():
            self.sdk.debug("EKF进程启动失败")
            self.sdk.bg(0)
            self.sdk.error()
            return
        
        # 等待一小段时间让EKF进程稳定启动
        time.sleep(2)
        
        # 检查EKF进程是否还在运行
        if self.ekf_process.poll() is not None:
            self.sdk.debug("EKF进程启动后立即退出")
            self.sdk.error()
            return
        
        # 输出融合话题名称
        self.sdk.output({"fused_topic": self.fused_topic})
        
        try:
            self.sdk.finish()
            
            # 持续运行以保持EKF进程活跃
            while True:
                if self.sdk.is_stop():
                    self.sdk.debug("收到停止信号，正在关闭节点...")
                    break
                
                # 检查EKF进程是否还在运行
                if self.ekf_process.poll() is not None:
                    # 进程已退出，读取输出
                    stdout, stderr = self.ekf_process.communicate()
                    self.sdk.debug("EKF进程已退出")
                    self.sdk.debug(f"标准输出: {stdout.decode('utf-8', errors='ignore')}")
                    self.sdk.debug(f"错误输出: {stderr.decode('utf-8', errors='ignore')}")
                    break
                
                # 尝试读取非阻塞输出
                try:
                    # 检查stdout
                    if self.ekf_process.stdout:
                        stdout_line = self.ekf_process.stdout.readline()
                        if stdout_line:
                            self.sdk.debug(f"EKF输出: {stdout_line.decode('utf-8', errors='ignore').strip()}")
                    
                    # 检查stderr
                    if self.ekf_process.stderr:
                        stderr_line = self.ekf_process.stderr.readline()
                        if stderr_line:
                            self.sdk.debug(f"EKF错误: {stderr_line.decode('utf-8', errors='ignore').strip()}")
                except:
                    pass
                
                time.sleep(1)
                
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()
        finally:
            # 清理资源
            self.stop_ekf_node()
            if self.node:
                self.node.destroy_node()
            self.sdk.bg(0)
            self.sdk.debug("机器人定位融合器已停止")