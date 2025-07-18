import rclpy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class MainNode:
    def __init__(self, cache, uuid,sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.static_broadcaster = None
        self.node = None
        self.sdk.debug("StaticTransform init")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        if isinstance(config, dict):
            self.source_frame = str(config.get('source_frame', 'base_link'))
            self.target_frame = str(config.get('target_frame', 'target_frame'))
            self.sdk.debug(f"节点输入source_frame: {self.source_frame}")
            self.sdk.debug(f"节点输入target_frame: {self.target_frame}")
        else:
            self.source_frame = 'base_link'
            self.target_frame = 'target_frame'

    def get_config_value(self, key, default=None):
        """从配置列表中获取指定键的值"""
        if isinstance(self.config, list):
            for item in self.config:
                if isinstance(item, dict) and item.get('name') == key:
                    # 优先返回 default_value，如果没有则返回 value
                    value = item.get('default_value', item.get('value', default))
                    
                    # 根据配置项类型进行类型转换
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
        """初始化ROS2节点和TF相关组件"""
        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('static_transform_node')
        self.static_broadcaster = StaticTransformBroadcaster(self.node)
        
        self.sdk.debug("# ROS2节点和TF组件初始化完成")

    def setup_static_transform(self):
        """设置静态变换"""
        if self.node is not None:
            transform = TransformStamped()
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.header.frame_id = getattr(self, 'source_frame', 'base_link')
            transform.child_frame_id = getattr(self, 'target_frame', 'target_frame')
            
            # 设置平移
            transform.transform.translation.x = self.get_config_value("static_x", 0.0)
            transform.transform.translation.y = self.get_config_value("static_y", 0.0)
            transform.transform.translation.z = self.get_config_value("static_z", 0.0)
            
            # 设置旋转（欧拉角转四元数）
            roll = self.get_config_value("static_roll", 0.0) or 0.0
            pitch = self.get_config_value("static_pitch", 0.0) or 0.0
            yaw = self.get_config_value("static_yaw", 0.0) or 0.0
            
            # 欧拉角转四元数
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            
            transform.transform.rotation.w = cr * cp * cy + sr * sp * sy
            transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
            transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
            transform.transform.rotation.z = cr * cp * sy - sr * sp * cy
            
            if self.static_broadcaster is not None:
                self.static_broadcaster.sendTransform(transform)
                self.sdk.debug(f"发布静态变换: {transform.header.frame_id} -> {transform.child_frame_id}")

    def execute(self):
        # 初始化ROS2节点
        self.init_ros_node()
        
        # 设置静态变换
        self.setup_static_transform()
        
        self.sdk.debug(f"静态TF节点已启动，持续运行中...")
        self.sdk.debug(f"源坐标系: {getattr(self, 'source_frame', 'base_link')}")
        self.sdk.debug(f"目标坐标系: {getattr(self, 'target_frame', 'target_frame')}")
        
        # 持续运行
        try:
            self.sdk.finish()
            while True:
                # 处理ROS2事件
                if self.node:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                if self.sdk.is_stop():
                    self.sdk.debug("收到停止信号，正在关闭节点...")
                    break
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()
        finally:
            # 清理ROS2资源
            if self.node:
                self.node.destroy_node()
            self.sdk.debug("# 静态TF节点已停止")
            self.sdk.finish()