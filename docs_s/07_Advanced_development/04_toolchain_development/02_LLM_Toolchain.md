---
sidebar_position: 1
---

# 7.4.2 LLM工具链

## RDK S100 1.0.0 大模型工具链

在 S100/S100P 平台上，D-Robotics_LLM_S100 目前支持以下模型和功能： 

**LLM:**
1. DeepSeek-R1-Distill-Qwen：支持 DeepSeek-R1-Distill-Qwen-1.5B 和 DeepSeek-R1-Distill-Qwen-7B，提供模型量化、简单会话、多轮对话、PPL评估功能。 

2. InternLM2：支持 InternLM2-1.8B，提供模型量化、简单会话、PPL评估功能。 

3. Qwen2.5：支持 Qwen2.5-1.5B、Qwen2.5-7B、Qwen2.5-1.5B-Instruct 和Qwen2.5-7B-Instruct，提供模型量化、简单会话、多轮对话（仅 Instruct）、PPL 评估功能。 

**多模态**
1. Qwen2.5-Omni：支持 Qwen2.5-Omni-3B，提供模型量化、离线运行、在线运行功能。

## 下载方式
**D-Robotics_LLM_S100 开发工具包**

```bash 
wget https://d-robotics-aitoolchain.oss-cn-beijing.aliyuncs.com/llm_s100/1.0.0/D-Robotics_LLM_S100_1.0.0_SDK.tar.gz
```

**D-Robotics_LLM_S100 用户手册**

```bash
wget https://d-robotics-aitoolchain.oss-cn-beijing.aliyuncs.com/llm_s100/1.0.0/D-Robotics_LLM_S100_1.0.0_Doc.zip
```

**D-Robotics_LLM_S100 已编译模型**

下载开发工具包后，查看 oellm_runtime/model/resolve_model_nash-m.txt 获取下载链接。
