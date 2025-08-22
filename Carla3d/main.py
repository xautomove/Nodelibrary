import carla
import pygame
import numpy as np
import time,threading,random,weakref,os
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, NavSatFix, Imu
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sensor_msgs.msg import PointField
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

exit_requested = threading.Event()

def get_timestamp():
    """获取毫秒级时间戳"""
    return int(time.time() * 1000)

class SensorManager:
    def __init__(self, sensor, node, topic_name, msg_type, publish_frequency):
        self.sensor = sensor
        self.node = node
        self._lock = threading.Lock()
        self._running = True
        self._latest_data = None
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher = self.node.create_publisher(msg_type, topic_name, qos_profile)
        
        self.timer = self.node.create_timer(1.0/publish_frequency, self.publish_data)
        
        self._thread = threading.Thread(target=self._run_sensor_listener)
        self._thread.daemon = True
        self._thread.start()
    
    def _run_sensor_listener(self):
        weak_self = weakref.ref(self)
        try:
            self.sensor.listen(lambda data: self._on_sensor_data(weak_self, data))
        except Exception as e:
            print(f"启动传感器监听失败: {e}")
            time.sleep(1)
    
    def _on_sensor_data(self, weak_self, data):
        raise NotImplementedError
    
    def publish_data(self):
        raise NotImplementedError
    
    def destroy(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.sensor:
            self.sensor.stop()
            self.sensor.destroy()
        if hasattr(self, 'timer'):
            self.timer.destroy()

class LidarManager(SensorManager):
    def __init__(self, lidar_sensor, node):
        super().__init__(lidar_sensor, node, '/points_raw', PointCloud2, 20.0)  # 20Hz
    
    def _on_sensor_data(self, weak_self, data):
        self = weak_self()
        if not self:
            return
            
        try:
            points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
            points = points.reshape((-1, 4))
            
            header = Header()
            timestamp = get_timestamp()
            header.stamp.sec = timestamp // 1000
            header.stamp.nanosec = (timestamp % 1000) * 1000000
            header.frame_id = 'velodyne'
            
            cloud_points = []
            for i, point in enumerate(points):
                ring = i % 32
                intensity = 1.0
                cloud_points.append([point[0], point[1], point[2], ring, intensity])
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='ring', offset=12, datatype=PointField.UINT16, count=1),
                PointField(name='intensity', offset=14, datatype=PointField.FLOAT32, count=1)
            ]
            
            cloud = pc2.create_cloud(header, fields, cloud_points)
            
            with self._lock:
                self._latest_data = cloud
            
        except Exception as e:
            print(f"处理激光雷达数据时出错: {e}")
    
    def publish_data(self):
        with self._lock:
            if self._latest_data is not None:
                timestamp = get_timestamp()
                self._latest_data.header.stamp.sec = timestamp // 1000
                self._latest_data.header.stamp.nanosec = (timestamp % 1000) * 1000000
                self.publisher.publish(self._latest_data)

class CameraManager:
    def __init__(self, camera):
        self.sensor = camera
        self._image_queue = deque(maxlen=1)
        self._lock = threading.Lock()
        self._surface = None
        weak_self = weakref.ref(self)
        camera.listen(lambda image: CameraManager._on_image(weak_self, image))

    @staticmethod
    def _on_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3].swapaxes(0, 1)
        with self._lock:
            self._image_queue.append(array)

    def render(self, display):
        with self._lock:
            if self._image_queue:
                frame = self._image_queue[-1]
                self._surface = pygame.surfarray.make_surface(frame)
            if self._surface:
                display.blit(self._surface, (0, 0))

    def destroy(self):
        if self.sensor:
            self.sensor.stop()
            self.sensor.destroy()

class GNSSManager(SensorManager):
    def __init__(self, gnss_sensor, node):
        super().__init__(gnss_sensor, node, '/carla/gnss', NavSatFix, 20.0)  # 20Hz
    
    def _on_sensor_data(self, weak_self, data):
        self = weak_self()
        if not self:
            return
            
        try:
            msg = NavSatFix()
            msg.header = Header()
            timestamp = get_timestamp()
            msg.header.stamp.sec = timestamp // 1000
            msg.header.stamp.nanosec = (timestamp % 1000) * 1000000
            msg.header.frame_id = 'gps_link'
            
            msg.latitude = data.latitude
            msg.longitude = data.longitude
            msg.altitude = data.altitude

            # 设置协方差，3x3矩阵展开成列表，单位为 m^2
            # 这里示例给出0.5米方差，适当调节
            covariance_value = 0.5 * 0.5
            msg.position_covariance = [
                covariance_value, 0.0, 0.0,
                0.0, covariance_value, 0.0,
                0.0, 0.0, covariance_value
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED  # 2
            
            with self._lock:
                self._latest_data = msg
            
        except Exception as e:
            print(f"处理GNSS数据时出错: {e}")
    
    def publish_data(self):
        with self._lock:
            if self._latest_data is not None:
                # 更新时间戳
                timestamp = get_timestamp()
                self._latest_data.header.stamp.sec = timestamp // 1000
                self._latest_data.header.stamp.nanosec = (timestamp % 1000) * 1000000  # 转换为纳秒
                self.publisher.publish(self._latest_data)

class IMUManager(SensorManager):
    def __init__(self, imu_sensor, node):
        super().__init__(imu_sensor, node, '/imu_raw', Imu, 250.0)  # 100Hz
    
    def _on_sensor_data(self, weak_self, data):
        self = weak_self()
        if not self:
            return
            
        try:
            msg = Imu()
            msg.header = Header()
            timestamp = get_timestamp()
            msg.header.stamp.sec = timestamp // 1000
            msg.header.stamp.nanosec = (timestamp % 1000) * 1000000  # 转换为纳秒
            msg.header.frame_id = 'imu_link'
            
            # Carla坐标系 → ROS (REP-105)
            msg.angular_velocity.x = data.gyroscope.x
            msg.angular_velocity.y = -data.gyroscope.y
            msg.angular_velocity.z = data.gyroscope.z
            
            msg.linear_acceleration.x = data.accelerometer.x
            msg.linear_acceleration.y = -data.accelerometer.y
            msg.linear_acceleration.z = data.accelerometer.z

            # 从Carla获取姿态信息
            transform = data.transform
            roll = transform.rotation.roll * np.pi / 180.0
            pitch = transform.rotation.pitch * np.pi / 180.0
            yaw = transform.rotation.yaw * np.pi / 180.0
            
            # 计算四元数
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            
            msg.orientation.w = cr * cp * cy + sr * sp * sy
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy
            
            # 设置协方差
            msg.orientation_covariance = [0.01] * 9
            msg.angular_velocity_covariance = [1e-3] * 9
            msg.linear_acceleration_covariance = [1e-3] * 9
            
            with self._lock:
                self._latest_data = msg
            
        except Exception as e:
            print(f"处理IMU数据时出错: {e}")
    
    def publish_data(self):
        with self._lock:
            if self._latest_data is not None:
                # 更新时间戳
                timestamp = get_timestamp()
                self._latest_data.header.stamp.sec = timestamp // 1000
                self._latest_data.header.stamp.nanosec = (timestamp % 1000) * 1000000  # 转换为纳秒
                self.publisher.publish(self._latest_data)

class MultiCameraManager:
    def __init__(self, cameras, node):
        self.sensors = cameras
        self.node = node
        self._lock = threading.Lock()
        self._running = True
        self.bridge = CvBridge()
        
        # 创建QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 为每个摄像头创建发布者
        self.publishers = {
            'front': self.node.create_publisher(Image, '/carla/camera/front', qos_profile),
            'left': self.node.create_publisher(Image, '/carla/camera/left', qos_profile),
            'right': self.node.create_publisher(Image, '/carla/camera/right', qos_profile)
        }
        
        # 为每个摄像头创建线程
        self._threads = {}
        for name, sensor in self.sensors.items():
            self._threads[name] = threading.Thread(
                target=self._run_camera_listener,
                args=(name, sensor)
            )
            self._threads[name].daemon = True
            self._threads[name].start()
    
    def _run_camera_listener(self, name, sensor):
        weak_self = weakref.ref(self)
        try:
            sensor.listen(lambda data: MultiCameraManager._on_camera_data(weak_self, name, data))
        except Exception as e:
            print(f"启动{name}摄像头监听失败: {e}")
            time.sleep(1)
    
    @staticmethod
    def _on_camera_data(weak_self, name, data):
        self = weak_self()
        if not self:
            return
            
        try:
            array = np.frombuffer(data.raw_data, dtype=np.uint8)
            array = array.reshape((data.height, data.width, 4))
            array = array[:, :, :3]  # 只保留RGB通道
            
            msg = self.bridge.cv2_to_imgmsg(array, encoding='rgb8')
            msg.header = Header()
            timestamp = get_timestamp()
            msg.header.stamp.sec = timestamp // 1000
            msg.header.stamp.nanosec = (timestamp % 1000) * 1000000  # 转换为纳秒
            msg.header.frame_id = 'base_link'
            
            self.publishers[name].publish(msg)
            
        except Exception as e:
            print(f"处理{name}摄像头数据时出错: {e}")
    
    def destroy(self):
        self._running = False
        for thread in self._threads.values():
            if thread.is_alive():
                thread.join(timeout=1.0)
        for sensor in self.sensors.values():
            if sensor:
                sensor.stop()
                sensor.destroy()

class KeyboardController(threading.Thread):
    def __init__(self, carla_controller):
        super().__init__()
        self.carla_controller = carla_controller
        self.running = True
        self.daemon = True  # 设置为守护线程，这样主程序退出时线程会自动结束

    def run(self):
        while self.running:
            self._process_events()
            time.sleep(0.01)  # 控制循环频率，约100Hz

    def _process_events(self):
        keys = pygame.key.get_pressed()
        
        # 控制油门、倒车和刹车
        if keys[pygame.K_w]:
            self.carla_controller.control.reverse = False
            self.carla_controller.control.throttle = 1.0
            self.carla_controller.control.brake = 0.0
        elif keys[pygame.K_s]:
            self.carla_controller.control.reverse = True  # 设置倒车模式
            self.carla_controller.control.throttle = 1.0  # 倒车时油门值仍为正
            self.carla_controller.control.brake = 0.0
        elif keys[pygame.K_SPACE]:  # 空格键为刹车和手刹
            self.carla_controller.control.throttle = 0.0
            self.carla_controller.control.brake = 1.0  # 最大刹车力
            self.carla_controller.control.hand_brake = True  # 启用手刹
        else:
            # 松开按键后快速减速
            if self.carla_controller.control.throttle > 0:
                self.carla_controller.control.throttle = max(0.0, self.carla_controller.control.throttle - 0.5)  # 增加减速速率
            elif self.carla_controller.control.throttle < 0:
                self.carla_controller.control.throttle = min(0.0, self.carla_controller.control.throttle + 0.5)  # 增加减速速率
            # 当速度接近0时，使用更强的刹车确保快速停止
            if abs(self.carla_controller.control.throttle) < 0.3:  # 扩大触发刹车的速度范围
                self.carla_controller.control.brake = 1.0  # 增加刹车力度
            else:
                self.carla_controller.control.brake = 0.5  # 增加基础刹车力
            
            # 松开空格键时关闭手刹
            self.carla_controller.control.hand_brake = False

        # 控制转向
        self.carla_controller.control.steer = 0.0
        if keys[pygame.K_a]:
            self.carla_controller.control.steer = -0.5
        elif keys[pygame.K_d]:
            self.carla_controller.control.steer = 0.5

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                self.carla_controller.running = False
                self.running = False

class WorldUpdateThread(threading.Thread):
    def __init__(self, world):
        super().__init__()
        self.world = world
        self.running = True
        self.daemon = True

    def run(self):
        while self.running:
            try:
                self.world.tick()
            except Exception as e:
                break
            time.sleep(0.005)
    def stop(self):
        self.running = False

class ROS2SpinThread(threading.Thread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True
        self.daemon = True

    def run(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.001)
            time.sleep(0.001)

    def stop(self):
        self.running = False

class Carla3dController(Node):
    def __init__(self, host='127.0.0.1', port=2000):
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"]="1"

        super().__init__('carla_controller')
        self.is_connected = False
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
        except Exception as e:
            self.get_logger().error(f"连接Carla Server失败: {e}")
            return
        
        self.is_connected = True

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20Hz
        self.world.apply_settings(settings)

        self.vehicle = None
        self.camera_manager = None
        self.lidar_manager = None
        self.gnss_manager = None
        self.imu_manager = None
        self.multi_camera_manager = None
        self.control = carla.VehicleControl()
        self.running = True
        self.width = 1280
        self.height = 720

        pygame.init()
        self.display = pygame.display.set_mode((self.width, self.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("Carla Controller")
        self.clock = pygame.time.Clock()
        
        self.keyboard_controller = KeyboardController(self)
        self.keyboard_controller.start()

        self.world_update_thread = WorldUpdateThread(self.world)
        self.world_update_thread.start()

        self.ros2_spin_thread = ROS2SpinThread(self)
        self.ros2_spin_thread.start()

    def spawn_vehicle_and_sensors(self):
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')[0]  # 选择第一个车辆
        spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = random.choice(spawn_points)
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)
        if not self.vehicle:
            raise RuntimeError("车辆生成失败")

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.width))
        camera_bp.set_attribute('image_size_y', str(self.height))
        camera_bp.set_attribute('fov', '90')
        camera_bp.set_attribute('sensor_tick', '0.05')
        vehicle_bb = self.vehicle.bounding_box
        cam_x = -vehicle_bb.extent.x * 3.0
        cam_z = vehicle_bb.extent.z + 3.0
        camera_transform = carla.Transform(
            carla.Location(x=cam_x, y=0.0, z=cam_z),
            carla.Rotation(pitch=-15.0, yaw=0.0, roll=0.0)
        )
        camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        self.camera_manager = CameraManager(camera)

        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '200.0')
        lidar_bp.set_attribute('points_per_second', '100000')
        lidar_bp.set_attribute('rotation_frequency', '20.0')
        lidar_bp.set_attribute('upper_fov', '10.0')
        lidar_bp.set_attribute('lower_fov', '-30.0')
        lidar_bp.set_attribute('horizontal_fov', '360.0')
        lidar_bp.set_attribute('sensor_tick', '0.05')
        lidar_transform = carla.Transform(carla.Location(x=1.0, z=1.8))
        lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        # lidar.enable_for_ros()
        self.lidar_manager = LidarManager(lidar, self)

        gnss_bp = blueprint_library.find('sensor.other.gnss')
        gnss_bp.set_attribute('sensor_tick', '0.05')
        gnss_transform = carla.Transform(carla.Location(x=1.0, z=1.8))
        gnss = self.world.spawn_actor(gnss_bp, gnss_transform, attach_to=self.vehicle)
        # gnss.enable_for_ros()
        self.gnss_manager = GNSSManager(gnss, self)

        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_bp.set_attribute('sensor_tick', '0.01')  # 100Hz
        imu_transform = carla.Transform(carla.Location(x=1.0, z=1.8))
        imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
        # imu.enable_for_ros()
        self.imu_manager = IMUManager(imu, self)

        # 生成多摄像头（用于ROS2话题发布）
        # camera_bp = blueprint_library.find('sensor.camera.rgb')
        # camera_bp.set_attribute('image_size_x', str(self.width))
        # camera_bp.set_attribute('image_size_y', str(self.height))
        # camera_bp.set_attribute('fov', '90')
        # camera_bp.set_attribute('sensor_tick', '0.05')

        # # 前摄像头
        # front_transform = carla.Transform(
        #     carla.Location(x=2.0, z=1.0),
        #     carla.Rotation(pitch=0.0)
        # )
        # front_camera = self.world.spawn_actor(camera_bp, front_transform, attach_to=self.vehicle)

        # # 左摄像头
        # left_transform = carla.Transform(
        #     carla.Location(x=0.0, y=-1.0, z=1.0),
        #     carla.Rotation(yaw=-90.0)
        # )
        # left_camera = self.world.spawn_actor(camera_bp, left_transform, attach_to=self.vehicle)

        # # 右摄像头
        # right_transform = carla.Transform(
        #     carla.Location(x=0.0, y=1.0, z=1.0),
        #     carla.Rotation(yaw=90.0)
        # )
        # right_camera = self.world.spawn_actor(camera_bp, right_transform, attach_to=self.vehicle)

        # cameras = {
        #     'front': front_camera,
        #     'left': left_camera,
        #     'right': right_camera
        # }
        # self.multi_camera_manager = MultiCameraManager(cameras, self)

    def run(self):
        if not self.is_connected:
            return
        self.spawn_vehicle_and_sensors()
        try:
            while self.running:
                # self.clock.tick(60)
                self.vehicle.apply_control(self.control)
                self.camera_manager.render(self.display)
                pygame.display.flip()
        except Exception as e:
            self.get_logger().error(f"运行时出错: {e}")
            self.cleanup()

    def cleanup(self):
        self.get_logger().info("清理资源...")
        self.running = False
        try:
            if hasattr(self, 'keyboard_controller') and self.keyboard_controller:
                self.keyboard_controller.running = False
                self.keyboard_controller.join(timeout=1.0)
            if hasattr(self, 'world_update_thread') and self.world_update_thread:
                self.world_update_thread.stop()
                self.world_update_thread.join(timeout=1.0)
            if hasattr(self, 'ros2_spin_thread') and self.ros2_spin_thread:
                self.ros2_spin_thread.stop()
                self.ros2_spin_thread.join(timeout=1.0)
            if hasattr(self, 'camera_manager') and self.camera_manager:
                self.camera_manager.destroy()
            if hasattr(self, 'lidar_manager') and self.lidar_manager:
                self.lidar_manager.destroy()
            if hasattr(self, 'gnss_manager') and self.gnss_manager:
                self.gnss_manager.destroy()
            if hasattr(self, 'imu_manager') and self.imu_manager:
                self.imu_manager.destroy()
            if hasattr(self, 'multi_camera_manager') and self.multi_camera_manager:
                self.multi_camera_manager.destroy()
            if hasattr(self, 'vehicle') and self.vehicle:
                self.vehicle.destroy()
        except Exception as e:
            self.get_logger().error(f"清理资源时出错: {e}")
        finally:
            pygame.quit()
            self.destroy_node()

if __name__ == "__main__":
    rclpy.init()
    controller = Carla3dController()
    controller.run()
    rclpy.shutdown()