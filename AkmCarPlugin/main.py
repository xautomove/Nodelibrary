import os
import subprocess
import time
import yaml
import xml.etree.ElementTree as ET


def load_config():
    config_dir = os.path.join(os.path.dirname(__file__), 'config')
    
    with open(os.path.join(config_dir, 'imu.yaml'), 'r', encoding='utf-8') as f:
        imu_config = yaml.safe_load(f)
    
    with open(os.path.join(config_dir, 'lidar.yaml'), 'r', encoding='utf-8') as f:
        lidar_config = yaml.safe_load(f)
    
    return imu_config, lidar_config

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
                topic_name = plugin.find('topicName')
                if topic_name is not None:
                    topic_name.text = lidar_config['lidar']['topic']
                frame_name = plugin.find('frameName')
                if frame_name is not None:
                    frame_name.text = lidar_config['lidar']['frame_id']
            
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
                topic_name = plugin.find('topicName')
                if topic_name is not None:
                    topic_name.text = imu_config['imu']['topic']
                frame_name = plugin.find('frameName')
                if frame_name is not None:
                    frame_name.text = imu_config['imu']['frame_id']
    
    temp_urdf_path = urdf_path.replace('.urdf', '_temp.urdf')
    tree.write(temp_urdf_path, encoding='utf-8', xml_declaration=False)
    return temp_urdf_path

def launch_gazebo_and_spawn():
    imu_config, lidar_config = load_config()
    print(f"加载IMU配置: {imu_config}")
    print(f"加载激光雷达配置: {lidar_config}")
    
    urdf_path = os.path.join(os.path.dirname(__file__), 'urdf', 'akm_car.urdf')
    temp_urdf_path = update_urdf_with_config(urdf_path, imu_config, lidar_config)
    
    world_path = os.path.join(os.path.dirname(__file__), 'worlds', 'empty.world')
    
    gazebo_cmd = [
        'gazebo', '--verbose', world_path,
        '-s', 'libgazebo_ros_factory.so'
    ]
    print(f"启动Gazebo: {' '.join(gazebo_cmd)}")
    gazebo_proc = subprocess.Popen(gazebo_cmd)

    time.sleep(5)

    spawn_cmd = [
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', 'akm_car',
        '-file', temp_urdf_path,
        '-x', '0', '-y', '0', '-z', '0.1'
    ]
    print(f"加载模型: {' '.join(spawn_cmd)}")
    subprocess.run(spawn_cmd)

    try:
        os.remove(temp_urdf_path)
    except:
        pass

    gazebo_proc.wait()

if __name__ == '__main__':
    launch_gazebo_and_spawn() 