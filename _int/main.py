import sys

class MainNode:
    def __init__(self):
        self.input_data = {}
        self.output_data = {}
        self.config = {}

    def get_node_input(self, config):
        # 初始值节点没有输入
        pass

    def get_user_input(self, config):
        self.config = {item['name']: item['default_value'] for item in config}

    def execute(self):
        # 获取配置的初始值
        value = float(self.config.get('初始值', 0))
        
        # 返回结果
        result = {
            "outputs": {
                "数值": value
            }
        }
        
        return result
