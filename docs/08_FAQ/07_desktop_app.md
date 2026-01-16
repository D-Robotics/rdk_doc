---
sidebar_position: 7
---
# 8.7 桌面应用

本节主要解答在桌面使用第三方应用遇到的问题。

<!-- ```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
``` -->

### Q1: 下载 Visual Studio Code 应用打不开？

<!-- <Tabs groupId="accessory">
<TabItem value="rdk_s600" label="rdk_s600"> -->

**A:**
* **使用命令行打开：** Visual Studio Code 使用的 Electron shell 在处理某些 GPU（图形处理单元）硬件加速时存在问题，您可以尝试在启动 VS Code 时通过添加 Electron --disable-gpu 命令行开关来禁用 GPU 加速（https://code.visualstudio.com/docs/supporting/faq#_vs-code-is-blank）：
  
```bash
    code --disable-gpu
```
<!-- </TabItem>
</Tabs> -->
