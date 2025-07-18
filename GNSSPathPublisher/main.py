import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class MainNode:
    def __init__(self, cache, uuid,sdk):
        self.config = {}
        self.cache = cache
        self.node = None
        self.sdk = sdk
        self.uuid = uuid
        self.gnss_subscriber = None
        self.path_publisher = None
        self.path_points = []
        self.last_point = None
        self.frame_id = "map"
        self.max_points = 1000
        self.min_distance = 1.0
        self.distance_offset = 0.0
        self.reference_lat = None
        self.reference_lon = None
        self.running = False
        self.iswaiting = True
        self.sdk.debug("GNSSPathPublisher init")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        if isinstance(config, dict) and 'gnss_topic' in config:
            self.gnss_topic = str(config['gnss_topic'])
            self.sdk.debug(f"节点输入gnss_topic: {self.gnss_topic}")
        else:
            self.gnss_topic = None

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
                        return str(value) if value is not None else default
        elif isinstance(self.config, dict):
            return self.config.get(key, default)
        return default

    def init_ros_node(self):
        """初始化ROS2节点和订阅发布器"""
        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('gnss_path_publisher')
        
        # 获取配置参数
        gnss_topic = getattr(self, 'gnss_topic', None)
        if not gnss_topic:
            self.sdk.debug("gnss_topic未通过节点输入提供！")
            self.sdk.error()
            return
        path_topic = str(self.get_config_value("path_topic", "/gnss_path"))
        self.frame_id = str(self.get_config_value("frame_id", "map"))
        max_points_val = self.get_config_value("max_points", 1000)
        self.max_points = int(max_points_val) if max_points_val is not None else 1000
        min_distance_val = self.get_config_value("min_distance", 1.0)
        self.min_distance = float(min_distance_val) if min_distance_val is not None else 1.0
        distance_offset_val = self.get_config_value("distance_offset", 0.0)
        self.distance_offset = float(distance_offset_val) if distance_offset_val is not None else 0.0
        
        self.sdk.debug(f"等待GNSS数据...")
        self.gnss_subscriber = self.node.create_subscription(
            NavSatFix,
            gnss_topic,
            self.gnss_callback,
            10
        )
        
        self.path_publisher = self.node.create_publisher(
            Path,
            path_topic,
            10
        )
        
        self.sdk.debug(f"ROS2节点初始化完成")
        self.sdk.debug(f"订阅GNSS话题: {gnss_topic}")
        self.sdk.debug(f"发布路径话题: {path_topic}")
        self.sdk.debug(f"坐标系: {self.frame_id}")

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """计算两个GPS坐标点之间的距离（米）"""
        R = 6371000  # 地球半径（米）
        
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c

    def latlon_to_xy(self, lat, lon):
        """将经纬度坐标转换为相对XY坐标（米）"""
        if self.reference_lat is None or self.reference_lon is None:
            self.reference_lat = lat
            self.reference_lon = lon
            return 0.0, 0.0
        
        R = 6371000  # 地球半径（米）
        
        x = R * math.cos(math.radians(self.reference_lat)) * math.radians(lon - self.reference_lon)
        
        y = R * math.radians(lat - self.reference_lat)
        
        return x, y

    def gnss_callback(self, msg):
        """GNSS数据回调函数"""
        if msg.status.status < 0:  # 无效的GNSS数据
            return
            
        current_point = (msg.latitude, msg.longitude)

        if current_point and self.iswaiting:
            self.sdk.debug("GNSS数据已收到")
            self.sdk.bg(1)
            self.iswaiting = False
        
        should_add = True
        if self.last_point is not None:
            distance = self.calculate_distance(
                self.last_point[0], self.last_point[1],
                current_point[0], current_point[1]
            )
            adjusted_threshold = self.min_distance + self.distance_offset
            should_add = distance >= adjusted_threshold
        
        if should_add:
            x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
            
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = msg.header.stamp
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.pose.position.x = x  # 使用转换后的X坐标（米）
            pose_stamped.pose.position.y = y  # 使用转换后的Y坐标（米）
            pose_stamped.pose.position.z = msg.altitude if msg.altitude > -1000 else 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            self.path_points.append(pose_stamped)
            self.last_point = current_point
            
            if len(self.path_points) > self.max_points:
                self.path_points.pop(0)
            
            self.publish_path()

    def publish_path(self):
        """发布路径消息"""
        if not self.path_points or self.node is None or self.path_publisher is None:
            return

        if not self.running:
            self.sdk.finish()
            self.running = True
            
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        path_msg.poses = self.path_points
        
        self.path_publisher.publish(path_msg)

    def execute(self):
        """执行节点主逻辑"""
        self.init_ros_node()
        self.sdk.debug(f"GNSS路径发布器已启动")
        self.sdk.debug(f"最大路径点数量: {self.max_points}")
        self.sdk.debug(f"最小距离阈值: {self.min_distance} 米")
        self.sdk.debug(f"距离偏移值: {self.distance_offset} 米")
        self.sdk.debug(f"实际距离阈值: {self.min_distance + self.distance_offset} 米")
        self.sdk.output({"path_topic": str(self.get_config_value("path_topic", "/gnss_path"))})
        try:
            while True:
                if self.sdk.is_stop():
                    self.sdk.debug("收到停止信号，正在关闭节点...")
                    break
                if self.node:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()
        finally:
            if self.node:
                self.node.destroy_node()
            self.sdk.debug("GNSS路径发布器已停止")
            self.sdk.bg(0)
            self.sdk.finish()