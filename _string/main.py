import sys

class MainNode:
    def __init__(self, cache, uuid,sdk):
        self.input_data = {}
        self.output_data = {}
        self.config = {}
        self.sdk = sdk
        self.uuid = uuid
        self.cache = cache

    def get_node_input(self, config):
        # 初始值节点没有输入
        pass

    def get_user_input(self, config):
        self.config = {item['name']: item['default_value'] for item in config}

    def execute(self):
        # 获取配置的初始值
        value = str(self.config.get('初始值', ''))

        self.sdk.output({"字符串": value})
        self.sdk.finish()
