import yaml


class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.sdk.debug("MapServer 精简模式初始化")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        # 无上游依赖
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

    def execute(self):
        try:
            map_file = self.get_config_value("map_file")
            frame_id = self.get_config_value("frame_id", "map")
            topic_name = self.get_config_value("topic_name", "map")
            use_compressed = self.get_config_value("use_compressed", False)

            params_config = {
                'map_server': {
                    'ros__parameters': {
                        'yaml_filename': str(map_file),
                        'frame_id': frame_id,
                        'topic_name': topic_name,
                        'use_sim_time': False,
                        'introspection_mode': 'disabled'
                    }
                }
            }
            if use_compressed:
                params_config['map_server']['ros__parameters']['use_compressed'] = True

            cache_key = 'nav_map_server'
            cache_val = yaml.dump(params_config, default_flow_style=False, allow_unicode=True)
            self.cache.set(cache_key, cache_val)
            self.sdk.output({"地图配置": cache_key})
            self.sdk.finish()
        except Exception as e:
            self.sdk.debug(f"写入缓存失败: {e}")
            self.sdk.error()