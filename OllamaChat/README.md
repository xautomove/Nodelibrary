# OllamaChat 节点

与本地Ollama大语言模型交互的聊天节点，支持自定义模型和系统提示词。

## 功能特性

- 🤖 与本地Ollama服务进行交互
- 💬 支持自定义用户提示词
- ⚙️ 可配置模型名称、系统提示词等参数
- 🌐 支持自定义Ollama服务器地址和端口
- ⏱️ 可设置请求超时时间
- 📝 将AI回复存储到缓存中供其他节点使用

## 输入参数

- `user_prompt`: 用户输入的提示词

## 配置参数

- `ollama_host`: Ollama服务器地址（默认: `localhost`）
- `ollama_port`: Ollama服务器端口（默认: `11434`）
- `model_name`: 使用的模型名称（默认: `llama2`）
- `system_prompt`: 系统提示词（默认: `你是一个有用的AI助手。`）
- `timeout`: 请求超时时间，单位秒（默认: `120`）

## 依赖包

- `requests`: HTTP请求库

## 安装依赖

```bash
pip install requests
```

## 前置条件

1. 确保已安装并启动Ollama服务
2. 确保已下载所需的模型（如llama2）

## 使用说明

1. 启动Ollama服务：`ollama serve`
2. 下载模型：`ollama pull llama2`
3. 配置节点参数（服务器地址、端口、模型名称等）
4. 输入用户提示词
5. 节点会自动调用Ollama API获取回复
6. AI回复会存储在缓存中，键名为`ollama_response`

## API说明

节点使用Ollama的HTTP API `/api/generate`端点，支持以下功能：
- 非流式响应
- 自定义系统提示词
- 模型选择
- 超时控制

## 错误处理

- 连接失败：检查Ollama服务是否运行
- 请求超时：调整timeout参数
- 模型不存在：确保已下载指定模型 