import os
import yaml
import subprocess
import threading
import time


class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.config = {}
        self.cache = cache
        self.sdk = sdk
        self.uuid = uuid
        self.nav2_process = None
        self.output_thread = None
        self._stop_output_monitor = False
        self.params_path = "/tmp/nav2_params.yaml"
        self.input_keys = {}
        self.sdk.debug("Nav2Launcher 初始化")

    def get_user_input(self, config):
        self.config = config
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        # 上游传入的就是缓存key
        self.input_keys = config or {}

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

    def _load_yaml_from_cache(self, key):
        if not key:
            return None
        val = self.cache.get(key)
        if not val:
            return None
        try:
            return yaml.safe_load(val)
        except Exception as e:
            self.sdk.debug(f"解析缓存键 {key} 失败: {e}")
            return None

    def _deep_merge(self, base, override):
        if not isinstance(base, dict) or not isinstance(override, dict):
            return override
        result = dict(base)
        for k, v in override.items():
            if k in result:
                result[k] = self._deep_merge(result[k], v)
            else:
                result[k] = v
        return result

    def build_merged_params(self):
        merged = {}
        key_map = {
            '地图配置': None,
            '导航配置': None,
            '定位配置': None,
            '规划配置': None,
            '控制配置': None,
        }
        key_map.update(self.input_keys)

        cache_keys = [
            key_map.get('地图配置'),
            key_map.get('导航配置'),
            key_map.get('定位配置'),
            key_map.get('控制配置'),
            key_map.get('规划配置'),
        ]

        for key in cache_keys:
            cfg = self._load_yaml_from_cache(key)
            if cfg:
                merged = self._deep_merge(merged, cfg)
            else:
                self.sdk.debug(f"缓存中未找到 {key}，跳过")

        use_sim_time = self.get_config_value('use_sim_time', False)
        for section in merged.values():
            if isinstance(section, dict) and 'ros__parameters' in section:
                section['ros__parameters'].setdefault('use_sim_time', use_sim_time)

        return merged

    def write_params_file(self, data):
        try:
            if os.path.exists(self.params_path):
                os.unlink(self.params_path)
        except Exception:
            pass
        with open(self.params_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
        self.sdk.debug(f"写出合并后的Nav2参数到: {self.params_path}")

    def get_map_yaml_path(self):
        key = (self.input_keys or {}).get('地图配置')
        if not key:
            return None
        cfg = self._load_yaml_from_cache(key)
        if not isinstance(cfg, dict):
            return None
        try:
            return cfg.get('map_server', {}).get('ros__parameters', {}).get('yaml_filename')
        except Exception:
            return None

    def monitor_process_output_thread(self):
        try:
            while not self._stop_output_monitor and self.nav2_process and self.nav2_process.poll() is None:
                if self.nav2_process.stdout:
                    line = self.nav2_process.stdout.readline()
                    if line:
                        self.sdk.debug(f"nav2 stdout: {line.strip()}")
                if self.nav2_process.stderr:
                    line = self.nav2_process.stderr.readline()
                    if line:
                        self.sdk.debug(f"nav2 stderr: {line.strip()}")
                time.sleep(0.01)
        except Exception as e:
            self.sdk.debug(f"输出监控线程出错: {e}")

    def start_output_monitor(self):
        if self.nav2_process and self.nav2_process.poll() is None:
            self._stop_output_monitor = False
            self.output_thread = threading.Thread(target=self.monitor_process_output_thread, daemon=True)
            self.output_thread.start()

    def stop_output_monitor(self):
        self._stop_output_monitor = True
        if self.output_thread and self.output_thread.is_alive():
            self.output_thread.join(timeout=2)

    def start_nav2(self):
        try:
            cmd = [
                'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                f'params_file:={self.params_path}',
                f'use_sim_time:={str(self.get_config_value("use_sim_time", False)).lower()}'
            ]
            map_yaml = self.get_map_yaml_path()
            if map_yaml:
                cmd.append(f'map:={map_yaml}')
            self.sdk.debug("启动 Nav2 bringup")
            self.nav2_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            self.sdk.save_pid(self.nav2_process.pid)
            self.start_output_monitor()
            return True
        except Exception as e:
            self.sdk.debug(f"启动Nav2失败: {e}")
            return False

    def stop_nav2(self):
        if self.nav2_process:
            try:
                self.nav2_process.terminate()
                self.nav2_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.nav2_process.kill()
            except Exception:
                pass

    def execute(self):
        try:
            merged = self.build_merged_params()
            self.write_params_file(merged)
            if not self.start_nav2():
                self.sdk.error()
                return
            self.sdk.output({"nav2_status": "running"})
            self.sdk.bg(1)
            self.sdk.finish()
            while True:
                if self.nav2_process and self.nav2_process.poll() is not None:
                    self.sdk.debug("Nav2 进程退出")
                    break
                if self.sdk.is_stop():
                    self.sdk.debug("收到停止信号，正在关闭Nav2...")
                    break
                time.sleep(1)
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()
        finally:
            self.stop_output_monitor()
            self.stop_nav2()
            self.sdk.bg(0)
            self.sdk.finish()


