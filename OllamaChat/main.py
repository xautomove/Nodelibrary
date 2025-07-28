import requests
import json
import time

class MainNode:
    def __init__(self, cache, uuid, sdk):
        self.input_data = {}
        self.output_data = {}
        self.config = {}
        self.sdk = sdk
        self.uuid = uuid
        self.cache = cache
        self.sdk.debug("OllamaChat init")

    def get_user_input(self, config):
        self.config = {item['name']: item['default_value'] for item in config}
        self.sdk.debug(f"用户配置: {self.config}")

    def get_node_input(self, config):
        if isinstance(config, dict):
            self.user_prompt = str(config.get('user_prompt', ''))
            self.sdk.debug(f"节点输入user_prompt: {self.user_prompt}")
        else:
            self.user_prompt = ''

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
                        return value
        elif isinstance(self.config, dict):
            return self.config.get(key, default)
        return default



    def call_ollama_api(self, prompt):
        """调用Ollama API"""
        try:
            host = self.get_config_value("ollama_host", "localhost")
            port = self.get_config_value("ollama_port", 11434)
            model = self.get_config_value("model_name", "llama2")
            system_prompt = self.get_config_value("system_prompt", "你是一个有用的AI助手。")
            timeout = self.get_config_value("timeout", 120)
            
            url = f"http://{host}:{port}/api/generate"
            
            payload = {
                "model": model,
                "prompt": prompt,
                "system": system_prompt,
                "stream": False
            }
            
            self.sdk.debug(f"调用Ollama API: {url}")
            self.sdk.debug(f"模型: {model}")
            self.sdk.debug(f"用户提示词: {prompt}")
            
            response = requests.post(url, json=payload, timeout=int(timeout))
            
            if response.status_code == 200:
                result = response.json()
                if 'response' in result:
                    ai_response = result['response']
                    self.sdk.debug(f"AI回复: {ai_response}")
                    return ai_response
                else:
                    self.sdk.debug(f"API响应格式错误: {result}")
                    return None
            else:
                self.sdk.debug(f"API请求失败，状态码: {response.status_code}")
                self.sdk.debug(f"错误信息: {response.text}")
                return None
                
        except requests.exceptions.Timeout:
            self.sdk.debug("请求超时")
            return None
        except requests.exceptions.ConnectionError:
            self.sdk.debug("连接失败，请检查Ollama服务是否运行")
            return None
        except Exception as e:
            self.sdk.debug(f"调用Ollama API时出错: {e}")
            return None

    def execute(self):
        if not self.user_prompt:
            self.sdk.debug("用户提示词为空")
            self.sdk.error()
            return
        
        self.sdk.debug(f"OllamaChat节点已启动")
        self.sdk.debug(f"用户提示词: {self.user_prompt}")
        self.sdk.debug(f"Ollama地址: {self.get_config_value('ollama_host', 'localhost')}:{self.get_config_value('ollama_port', 11434)}")
        self.sdk.debug(f"模型名称: {self.get_config_value('model_name', 'llama2')}")
        
        try:
            response = self.call_ollama_api(self.user_prompt)
            
            if response:
                self.sdk.debug("成功获取AI回复")
                self.sdk.output({"ai_response": response})
                self.sdk.finish()
            else:
                self.sdk.debug("获取AI回复失败")
                self.sdk.error()
        except Exception as e:
            self.sdk.debug(f"节点运行出错: {e}")
            self.sdk.error()   
        self.sdk.debug("# OllamaChat节点已停止") 