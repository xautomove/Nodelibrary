import yaml


class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.scan_topic = "scan"
        self.map_topic = "map"
        self.sdk.debug("AMCL 精简模式初始化")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        if 'scan_topic' in config:
            self.scan_topic = config['scan_topic']
        if 'map_topic' in config:
            self.map_topic = config['map_topic']

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

    def execute(self):
        try:
            scan_topic = self.scan_topic
            map_topic = self.map_topic
            base_frame_id = self.get_config_value("base_frame_id", "base_footprint")
            odom_frame_id = self.get_config_value("odom_frame_id", "odom")
            global_frame_id = self.get_config_value("global_frame_id", "map")
            max_particles = int(self.get_config_value("max_particles", 2000))
            min_particles = int(self.get_config_value("min_particles", 500))
            alpha1 = float(self.get_config_value("alpha1", 0.2))
            alpha2 = float(self.get_config_value("alpha2", 0.2))
            alpha3 = float(self.get_config_value("alpha3", 0.2))
            alpha4 = float(self.get_config_value("alpha4", 0.2))
            alpha5 = float(self.get_config_value("alpha5", 0.2))
            laser_max_range = float(self.get_config_value("laser_max_range", 100.0))
            laser_min_range = float(self.get_config_value("laser_min_range", -1.0))
            set_initial_pose = bool(self.get_config_value("set_initial_pose", False))
            initial_pose_x = float(self.get_config_value("initial_pose_x", 0.0))
            initial_pose_y = float(self.get_config_value("initial_pose_y", 0.0))
            initial_pose_z = float(self.get_config_value("initial_pose_z", 0.0))
            initial_pose_yaw = float(self.get_config_value("initial_pose_yaw", 0.0))

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

            cache_key = 'nav_amcl'
            cache_val = yaml.dump(params_config, default_flow_style=False, allow_unicode=True)
            self.cache.set(cache_key, cache_val)
            self.sdk.output({"定位配置": cache_key})
            self.sdk.finish()
        except Exception as e:
            self.sdk.debug(f"写入缓存失败: {e}")
            self.sdk.error()
