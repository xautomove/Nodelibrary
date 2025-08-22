import yaml


class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.scan_topic = "/scan"
        self.map_topic = "map"
        self.sdk.debug("PlannerServer 精简模式初始化")

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
            planner_frequency = self.get_config_value("planner_frequency", 20.0)
            planner_plugin = self.get_config_value("planner_plugin", "nav2_navfn_planner/NavfnPlanner")
            costmap_update_timeout = self.get_config_value("costmap_update_timeout", 1.0)

            global_frame = self.get_config_value("global_frame", "map")
            robot_base_frame = self.get_config_value("robot_base_frame", "base_link")
            robot_radius = float(self.get_config_value("robot_radius", 0.22))
            resolution = float(self.get_config_value("resolution", 0.05))
            update_frequency = float(self.get_config_value("update_frequency", 1.0))
            publish_frequency = float(self.get_config_value("publish_frequency", 1.0))

            local_frame = self.get_config_value("local_frame", "odom")
            rolling_window = self.get_config_value("rolling_window", True)
            # width/height 在 Nav2 中声明为整数参数
            width = int(self.get_config_value("width", 3))
            height = int(self.get_config_value("height", 3))
            local_update_frequency = float(self.get_config_value("local_update_frequency", 5.0))
            local_publish_frequency = float(self.get_config_value("local_publish_frequency", 2.0))

            obstacle_max_range = float(self.get_config_value("obstacle_max_range", 2.5))
            obstacle_min_range = float(self.get_config_value("obstacle_min_range", 0.0))
            raytrace_max_range = float(self.get_config_value("raytrace_max_range", 3.0))
            raytrace_min_range = float(self.get_config_value("raytrace_min_range", 0.0))
            max_obstacle_height = float(self.get_config_value("max_obstacle_height", 2.0))
            min_obstacle_height = float(self.get_config_value("min_obstacle_height", 0.0))

            inflation_radius = float(self.get_config_value("inflation_radius", 0.55))
            cost_scaling_factor = float(self.get_config_value("cost_scaling_factor", 1.0))

            params_config = {
                'planner_server': {
                    'ros__parameters': {
                        'expected_planner_frequency': planner_frequency,
                        'costmap_update_timeout': costmap_update_timeout,
                        'introspection_mode': 'disabled',
                        'planner_plugins': ['GridBased'],
                        'GridBased': {
                            'plugin': planner_plugin
                        }
                    }
                },
                'global_costmap': {
                    'ros__parameters': {
                        'footprint_padding': 0.03,
                        'update_frequency': update_frequency,
                        'publish_frequency': publish_frequency,
                        'global_frame': global_frame,
                        'robot_base_frame': robot_base_frame,
                        'robot_radius': robot_radius,
                        'resolution': resolution,
                        'plugins': ["static_layer", "obstacle_layer", "inflation_layer"],
                        'obstacle_layer': {
                            'plugin': "nav2_costmap_2d/ObstacleLayer",
                            'enabled': True,
                            'observation_sources': 'scan',
                            'footprint_clearing_enabled': True,
                            'max_obstacle_height': max_obstacle_height,
                            'combination_method': 1,
                            'scan': {
                                'topic': self.scan_topic,
                                'obstacle_max_range': obstacle_max_range,
                                'obstacle_min_range': obstacle_min_range,
                                'raytrace_max_range': raytrace_max_range,
                                'raytrace_min_range': raytrace_min_range,
                                'max_obstacle_height': max_obstacle_height,
                                'min_obstacle_height': min_obstacle_height,
                                'clearing': True,
                                'marking': True,
                                'data_type': "LaserScan",
                                'inf_is_valid': False
                            }
                        },
                        'static_layer': {
                            'plugin': "nav2_costmap_2d/StaticLayer",
                            'map_subscribe_transient_local': True,
                            'enabled': True,
                            'subscribe_to_updates': True,
                            'transform_tolerance': 0.1
                        },
                        'inflation_layer': {
                            'plugin': "nav2_costmap_2d/InflationLayer",
                            'enabled': True,
                            'inflation_radius': inflation_radius,
                            'cost_scaling_factor': cost_scaling_factor,
                            'inflate_unknown': False,
                            'inflate_around_unknown': True
                        },
                        'always_send_full_costmap': True,
                        'introspection_mode': 'disabled'
                    }
                },
                'local_costmap': {
                    'local_costmap': {
                        'ros__parameters': {
                            'update_frequency': local_update_frequency,
                            'publish_frequency': local_publish_frequency,
                            'global_frame': local_frame,
                            'robot_base_frame': robot_base_frame,
                            'rolling_window': rolling_window,
                            'width': width,
                            'height': height,
                            'resolution': resolution,
                            'introspection_mode': 'disabled'
                        }
                    }
                }
            }

            cache_key = 'nav_planner_server'
            cache_val = yaml.dump(params_config, default_flow_style=False, allow_unicode=True)
            self.cache.set(cache_key, cache_val)
            self.sdk.output({"规划配置": cache_key})
            self.sdk.finish()
        except Exception as e:
            self.sdk.debug(f"写入缓存失败: {e}")
            self.sdk.error() 