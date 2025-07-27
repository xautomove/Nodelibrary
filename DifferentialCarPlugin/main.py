import os
import subprocess
import time
import yaml
import xml.etree.ElementTree as ET
import threading
import sys
import termios
import rclpy
from geometry_msgs.msg import Twist

try:
    import pygame
    from pygame.locals import *
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("提示：未安装pygame库，将使用控制台模式。安装方法：pip install pygame")


def load_config():
    config_dir = os.path.join(os.path.dirname(__file__), 'config')
    
    with open(os.path.join(config_dir, 'imu.yaml'), 'r', encoding='utf-8') as f:
        imu_config = yaml.safe_load(f)
    
    with open(os.path.join(config_dir, 'lidar.yaml'), 'r', encoding='utf-8') as f:
        lidar_config = yaml.safe_load(f)
    
    return imu_config, lidar_config

def process_xacro_to_urdf(xacro_path, urdf_path):
    """将XACRO文件转换为URDF文件"""
    try:
        result = subprocess.run(['xacro', xacro_path, '-o', urdf_path], 
                              capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            print(f"✓ XACRO文件成功转换为URDF: {urdf_path}")
            return urdf_path
        else:
            print(f"✗ XACRO转换失败: {result.stderr}")
            return None
    except Exception as e:
        print(f"✗ XACRO处理失败: {e}")
        return None

def update_urdf_with_config(urdf_path, imu_config, lidar_config):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    for gazebo in root.findall('.//gazebo[@reference="lidar_link"]'):
        sensor = gazebo.find('.//sensor[@name="lidar"]')
        if sensor is not None:
            update_rate = sensor.find('update_rate')
            if update_rate is not None:
                update_rate.text = str(lidar_config['lidar']['update_rate'])
            
            plugin = sensor.find('.//plugin')
            if plugin is not None:
                remapping = plugin.find('.//remapping')
                if remapping is not None:
                    topic = lidar_config['lidar']['topic'].lstrip('/')
                    remapping.text = f"~/out:={topic}"
                frame_name_new = plugin.find('frame_name')
                if frame_name_new is not None:
                    frame_name_new.text = lidar_config['lidar']['frame_id']
            
            ray = sensor.find('.//ray')
            if ray is not None:
                min_angle = ray.find('.//min_angle')
                if min_angle is not None:
                    min_angle.text = str(lidar_config['lidar']['min_angle'])
                max_angle = ray.find('.//max_angle')
                if max_angle is not None:
                    max_angle.text = str(lidar_config['lidar']['max_angle'])
                samples = ray.find('.//samples')
                if samples is not None:
                    samples.text = str(lidar_config['lidar']['samples'])
    
    for gazebo in root.findall('.//gazebo[@reference="imu_link"]'):
        sensor = gazebo.find('.//sensor[@name="imu_sensor"]')
        if sensor is not None:
            update_rate = sensor.find('update_rate')
            if update_rate is not None:
                update_rate.text = str(imu_config['imu']['update_rate'])
            
            plugin = sensor.find('.//plugin')
            if plugin is not None:
                remapping = plugin.find('.//remapping')
                if remapping is not None:
                    topic = imu_config['imu']['topic'].lstrip('/')
                    remapping.text = f"~/out:={topic}"
                frame_name_new = plugin.find('frame_name')
                if frame_name_new is not None:
                    frame_name_new.text = imu_config['imu']['frame_id']
    
    temp_urdf_path = urdf_path.replace('.urdf', '_temp.urdf')
    tree.write(temp_urdf_path, encoding='utf-8', xml_declaration=False)
    return temp_urdf_path

class DifferentialKeyboardController:
    def __init__(self):
        self.cmd_vel_publisher = None
        self.running = False
        
        if not PYGAME_AVAILABLE:
            self.settings = termios.tcgetattr(sys.stdin)
        
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        
        self.pressed_keys = {
            'w': False, 's': False, 'a': False, 'd': False
        }

    def publish_commands(self):
        twist = Twist()
        
        if self.pressed_keys['w']:
            twist.linear.x = self.linear_speed
        elif self.pressed_keys['s']:
            twist.linear.x = -self.linear_speed
        
        if self.pressed_keys['a']:
            twist.angular.z = -self.angular_speed  
        elif self.pressed_keys['d']:
            twist.angular.z = self.angular_speed   
        
        if self.cmd_vel_publisher:
            self.cmd_vel_publisher.publish(twist)
        
        if any(self.pressed_keys.values()):
            keys = [k for k, v in self.pressed_keys.items() if v]
            print(f"控制: {'+'.join(keys).upper()} | 线速度: {twist.linear.x:.1f} m/s | 角速度: {twist.angular.z:.1f} rad/s")

    def start_pygame_keyboard_control(self):
        rclpy.init()
        node = rclpy.create_node('differential_keyboard_controller')
        
        self.cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        
        pygame.init()
        
        window_size = (400, 300)
        screen = pygame.display.set_mode(window_size)
        pygame.display.set_caption("Differential Car Control")
        
        try:
            font = pygame.font.SysFont('arial', 20)
            title_font = pygame.font.SysFont('arial', 28)
        except:
            font = pygame.font.Font(None, 24)
            title_font = pygame.font.Font(None, 32)
        
        BLACK = (0, 0, 0)
        WHITE = (255, 255, 255)
        GREEN = (0, 255, 0)
        RED = (255, 0, 0)
        BLUE = (0, 100, 255)
        GRAY = (128, 128, 128)
        
        self.running = True
        clock = pygame.time.Clock()
        
        print("\n" + "="*50)
        print("Pygame keyboard control started!")
        print("Use W/S to control speed, A/D to control turning")
        print("Press ESC or close window to exit")
        print("="*50)
        
        try:
            while self.running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                
                keys = pygame.key.get_pressed()
                
                self.pressed_keys['w'] = keys[pygame.K_w]
                self.pressed_keys['s'] = keys[pygame.K_s]
                self.pressed_keys['a'] = keys[pygame.K_a]
                self.pressed_keys['d'] = keys[pygame.K_d]
                
                self.publish_commands()
                
                screen.fill(BLACK)
                
                title_text = title_font.render("Differential Car", True, WHITE)
                screen.blit(title_text, (80, 20))
                
                y_offset = 70
                
                key_states = [
                    ("W - Forward", self.pressed_keys['w']),
                    ("S - Backward", self.pressed_keys['s']),
                    ("A - Turn Left", self.pressed_keys['a']),
                    ("D - Turn Right", self.pressed_keys['d'])
                ]
                
                for i, (text, pressed) in enumerate(key_states):
                    color = GREEN if pressed else GRAY
                    key_text = font.render(text, True, color)
                    screen.blit(key_text, (50, y_offset + i * 30))
                
                speed = 0.0
                angular = 0.0
                if self.pressed_keys['w']:
                    speed = self.linear_speed
                elif self.pressed_keys['s']:
                    speed = -self.linear_speed
                
                if self.pressed_keys['a']:
                    angular = self.angular_speed
                elif self.pressed_keys['d']:
                    angular = -self.angular_speed
                
                speed_text = font.render(f"Linear Speed: {speed:.1f} m/s", True, WHITE)
                angular_text = font.render(f"Angular Speed: {angular:.1f} rad/s", True, WHITE)
                
                screen.blit(speed_text, (50, y_offset + 140))
                screen.blit(angular_text, (50, y_offset + 170))
                
                info_text = font.render("Front: Caster Wheel", True, BLUE)
                info_text2 = font.render("Rear: Differential Drive", True, BLUE)
                screen.blit(info_text, (50, y_offset + 200))
                screen.blit(info_text2, (50, y_offset + 230))
                
                exit_text = font.render("Press ESC or close window to exit", True, RED)
                screen.blit(exit_text, (50, 260))
                
                pygame.display.flip()
                clock.tick(30)  
                
        except KeyboardInterrupt:
            pass
        finally:
            for k in self.pressed_keys:
                self.pressed_keys[k] = False
            self.publish_commands()
            
            pygame.quit()
            print("\nPygame keyboard control stopped")
            
            node.destroy_node()
            rclpy.shutdown()
            self.running = False

    def start_keyboard_control(self):
        if PYGAME_AVAILABLE:
            self.start_pygame_keyboard_control()

    def stop(self):
        self.running = False

def launch_gazebo_and_spawn():
    imu_config, lidar_config = load_config()
    print(f"Loaded IMU config: {imu_config}")
    print(f"Loaded LiDAR config: {lidar_config}")
    
    xacro_path = os.path.join(os.path.dirname(__file__), 'urdf', 'differential_car.urdf.xacro')
    urdf_path = os.path.join(os.path.dirname(__file__), 'urdf', 'differential_car.urdf')
    
    xacro_time = os.path.getmtime(xacro_path) if os.path.exists(xacro_path) else 0
    urdf_time = os.path.getmtime(urdf_path) if os.path.exists(urdf_path) else 0
    
    if xacro_time > urdf_time:
        print("检测到XACRO文件更新，重新生成URDF...")
        if not process_xacro_to_urdf(xacro_path, urdf_path):
            print("XACRO处理失败，退出程序")
            return
    else:
        print("URDF文件是最新的，跳过XACRO处理")
    
    temp_urdf_path = update_urdf_with_config(urdf_path, imu_config, lidar_config)
    
    world_path = os.path.join(os.path.dirname(__file__), 'worlds', 'empty.world')
    
    gazebo_cmd = [
        'gazebo', '--verbose', world_path,
        '-s', 'libgazebo_ros_factory.so'
    ]
    print(f"Starting Gazebo: {' '.join(gazebo_cmd)}")
    gazebo_proc = subprocess.Popen(gazebo_cmd)

    time.sleep(8)  

    spawn_cmd = [
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', 'differential_car',
        '-file', temp_urdf_path,
        '-x', '0', '-y', '0', '-z', '0.1'
    ]
    print(f"Spawning model: {' '.join(spawn_cmd)}")
    subprocess.run(spawn_cmd)

    try:
        os.remove(temp_urdf_path)
    except:
        pass

    keyboard_controller = DifferentialKeyboardController()
    keyboard_thread = threading.Thread(target=keyboard_controller.start_keyboard_control)
    keyboard_thread.daemon = True
    keyboard_thread.start()

    try:
        gazebo_proc.wait()
    except KeyboardInterrupt:
        print("\nExiting...")
        keyboard_controller.stop()
        gazebo_proc.terminate()



if __name__ == '__main__':
    launch_gazebo_and_spawn() 